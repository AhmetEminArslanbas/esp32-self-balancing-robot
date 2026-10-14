// MPU6050 register-level driver and tilt / heading estimation.
//
// The DMP is deliberately not used: it outputs fused data at 100 Hz with extra latency.
// Reading raw gyro + accel at 500 Hz and fusing here gives a lower-latency, fully
// deterministic angle, and the raw gyro rate feeds the D term without differentiation.
//
// Axis convention (proven on the Kuasar build): sensor X points along the direction of
// travel. The tilt angle is atan2(ax, sqrt(ay^2 + az^2)), the same quantity Kuasar used.
// The body "up" direction is learned from the gravity component in the Y-Z plane, so
// the board may sit with either Y or Z vertical (or anything in between).

#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_CONFIG2 0x1D  // MPU6500 family only
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75

static const float GYRO_LSB_PER_DPS = 65.5f;   // +-500 deg/s range
static const float ACCEL_LSB_PER_G  = 8192.0f; // +-4 g range
static const float VERTICAL_TAU_S   = 0.5f;
static const uint16_t CALIB_SAMPLES = 500;     // 1 s of stillness at 500 Hz
static const float CALIB_MAX_STD_DPS = 1.0f;
static const uint32_t CALIB_GIVE_UP_MS = 6000;

ImuReading imuLast = {};
float imuAngle = 0, imuAccAngle = 0, imuRate = 0, imuYawRate = 0;
float imuGyroBias[3] = {0, 0, 0};
uint8_t imuAddress = 0x68, imuWhoAmI = 0;

static float gravityY = 0, gravityZ = 1;  // low-passed gravity projection on the Y-Z plane
static float upY = 0, upZ = 1;            // unit "up" direction of the body in that plane
static bool filterReady = false;

static uint16_t calibCount = 0;
static double calibSum[3], calibSumSq[3];
static uint32_t calibBeginMs = 0;

static bool imuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool imuReadRegs(uint8_t reg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddress, length) != length) return false;
  for (uint8_t i = 0; i < length; i++) buffer[i] = (uint8_t)Wire.read();
  return true;
}

bool imuBegin() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setTimeOut(5);

  const uint8_t candidates[2] = {0x68, 0x69};
  bool found = false;
  for (uint8_t addr : candidates) {
    imuAddress = addr;
    uint8_t who = 0;
    if (imuReadRegs(MPU_REG_WHO_AM_I, &who, 1)) {
      imuWhoAmI = who;  // 0x68 on genuine parts, clones report other values but work the same
      found = true;
      break;
    }
  }
  if (!found) return false;

  bool ok = imuWriteReg(MPU_REG_PWR_MGMT_1, 0x80);  // device reset
  delay(100);
  ok &= imuWriteReg(MPU_REG_PWR_MGMT_1, 0x01);      // wake up, clock from gyro X PLL
  delay(10);
  ok &= imuWriteReg(MPU_REG_SMPLRT_DIV, 0x00);      // 1 kHz internal sample rate
  ok &= imuWriteReg(MPU_REG_CONFIG, 0x02);          // DLPF ~98 Hz: kills motor vibration, ~3 ms lag
  ok &= imuWriteReg(MPU_REG_GYRO_CONFIG, 0x08);     // +-500 deg/s
  ok &= imuWriteReg(MPU_REG_ACCEL_CONFIG, 0x08);    // +-4 g, bumps do not clip
  // MPU6500-family chips (WHO_AM_I 0x70, sold as "MPU6050" too) have a separate accel
  // filter that defaults to 460 Hz and would pass motor vibration into the tilt angle.
  if (imuWhoAmI != 0x68) ok &= imuWriteReg(MPU_REG_ACCEL_CONFIG2, 0x02);  // ~99 Hz

  uint8_t check = 0;
  ok &= imuReadRegs(MPU_REG_GYRO_CONFIG, &check, 1) && check == 0x08;
  filterReady = false;
  return ok;
}

bool imuRead(ImuReading& out) {
  uint8_t b[14];
  // Motor switching noise occasionally corrupts one I2C transfer; one retry costs ~0.4 ms.
  if (!imuReadRegs(MPU_REG_ACCEL_XOUT_H, b, 14) && !imuReadRegs(MPU_REG_ACCEL_XOUT_H, b, 14)) {
    return false;
  }
  const int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  const int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  const int16_t az = (int16_t)((b[4] << 8) | b[5]);
  const int16_t gx = (int16_t)((b[8] << 8) | b[9]);
  const int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  const int16_t gz = (int16_t)((b[12] << 8) | b[13]);

  // An all-zero frame means the sensor lost power or was reset.
  if (ax == 0 && ay == 0 && az == 0) return false;

  out.ax = ax / ACCEL_LSB_PER_G;
  out.ay = ay / ACCEL_LSB_PER_G;
  out.az = az / ACCEL_LSB_PER_G;
  out.gx = gx / GYRO_LSB_PER_DPS - imuGyroBias[0];
  out.gy = gy / GYRO_LSB_PER_DPS - imuGyroBias[1];
  out.gz = gz / GYRO_LSB_PER_DPS - imuGyroBias[2];
  imuLast = out;
  return true;
}

void imuResetFilter() {
  filterReady = false;
}

void imuUpdate(const ImuReading& r, float dt) {
  if (!filterReady) {
    gravityY = r.ay;
    gravityZ = r.az;
  } else {
    const float a = dt / (VERTICAL_TAU_S + dt);
    gravityY += a * (r.ay - gravityY);
    gravityZ += a * (r.az - gravityZ);
  }
  // Tilting about the axle does not change the direction of gravity inside the Y-Z
  // plane, so this gives the body's own "up" axis regardless of how far it leans.
  const float n = sqrtf(gravityY * gravityY + gravityZ * gravityZ);
  if (n > 0.3f) {  // below that the robot lies on its face and the direction is noise
    upY = gravityY / n;
    upZ = gravityZ / n;
  }

  // d(tilt)/dt = gyro . (X x up);  yaw rate = gyro . up
  imuRate    = -r.gy * upZ + r.gz * upY;
  imuYawRate =  r.gy * upY + r.gz * upZ;
  imuAccAngle = atan2f(r.ax, sqrtf(r.ay * r.ay + r.az * r.az)) * RAD_TO_DEG;

  if (!filterReady) {
    imuAngle = imuAccAngle;
    filterReady = true;
    return;
  }
  // Complementary filter: gyro for fast motion, accelerometer to remove long-term drift.
  const float k = P.tau / (P.tau + dt);
  imuAngle = k * (imuAngle + imuRate * dt) + (1.0f - k) * imuAccAngle;
}

void imuCalibStart() {
  calibCount = 0;
  for (int i = 0; i < 3; i++) calibSum[i] = calibSumSq[i] = 0;
  calibBeginMs = millis();
}

// Returns 0 while collecting, 1 when a new bias was measured, 2 when the robot never
// stayed still long enough and the stored bias is kept.
int imuCalibStep(const ImuReading& r) {
  const float raw[3] = {r.gx + imuGyroBias[0], r.gy + imuGyroBias[1], r.gz + imuGyroBias[2]};
  for (int i = 0; i < 3; i++) {
    calibSum[i] += raw[i];
    calibSumSq[i] += (double)raw[i] * raw[i];
  }
  if (++calibCount < CALIB_SAMPLES) {
    return millis() - calibBeginMs > CALIB_GIVE_UP_MS ? 2 : 0;
  }

  float mean[3];
  float worstStd = 0;
  for (int i = 0; i < 3; i++) {
    mean[i] = (float)(calibSum[i] / calibCount);
    const double var = calibSumSq[i] / calibCount - (double)mean[i] * mean[i];
    worstStd = fmaxf(worstStd, (float)sqrt(var > 0 ? var : 0));
  }

  if (worstStd <= CALIB_MAX_STD_DPS) {
    for (int i = 0; i < 3; i++) imuGyroBias[i] = mean[i];
    pendingBiasSave = true;
    filterReady = false;
    return 1;
  }
  if (millis() - calibBeginMs > CALIB_GIVE_UP_MS) {
    filterReady = false;
    return 2;
  }
  // Moved during the window: start a fresh one.
  calibCount = 0;
  for (int i = 0; i < 3; i++) calibSum[i] = calibSumSq[i] = 0;
  return 0;
}

void imuBiasLoad() {
  prefs.begin("denge", true);
  if (prefs.getBytesLength("gbias") == sizeof(imuGyroBias)) {
    prefs.getBytes("gbias", imuGyroBias, sizeof(imuGyroBias));
  }
  prefs.end();
}

void imuBiasSave() {
  prefs.begin("denge", false);
  prefs.putBytes("gbias", imuGyroBias, sizeof(imuGyroBias));
  prefs.end();
}
