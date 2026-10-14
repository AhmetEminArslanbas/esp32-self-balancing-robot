// 500 Hz control task: state machine + cascaded balance controller.
//
//   position/speed loop (outer)  ->  target tilt angle
//   angle loop (inner, PID)       ->  common wheel command u   [%]
//   heading loop                  ->  differential command     [%]
//
// Why the angle loop needs an I term: a PWM-driven gear motor behaves like a speed
// source (speed ~ duty), but catching a falling body needs wheel *acceleration*
// proportional to the tilt. With PD only, speed ~ tilt, so acceleration ~ tilt rate and
// the -g*tilt term of the pendulum is never cancelled: the robot slowly runs away.
// Integrating the angle error turns the command into "speed = integral of tilt", i.e.
// acceleration ~ tilt, which is exactly what balancing needs. (Kuasar's large KI_roll
// was doing the same job.)
//
// There are no wheel encoders, so wheel speed is estimated from what the motors were
// told to do: a DC gear motor under slow-decay PWM settles to a speed roughly
// proportional to its duty, with a short lag (vel_tau). Integrating that estimate gives
// the distance travelled, which is what lets the robot return to where it started.
//
// Sign conventions: tilt > 0 means the robot leans toward the side a positive wheel
// command drives to, so u > 0 moves the wheels under the falling body.

static uint32_t stateSinceMs = 0;
// After power-up (and after trimcal) the robot must be tipped over once before it may
// start by itself, so it never drives off while it is being handled upright.
static bool bootLatch = true;
// "stop" holds the robot down (no auto start, no self-righting) until "start".
static bool stopHold = false;
static uint8_t imuFailCount = 0;

static uint32_t restSinceMs = 0;    // lying still since (0 = not resting)
static uint8_t riseFailures = 0;
static int8_t riseSide = 0;         // +1 lying toward positive tilt, -1 toward negative
static bool riseKicking = false;    // false = wind-up phase, true = kick phase
static uint32_t risePhaseStartMs = 0;

static float speedEstimate = 0;     // v: estimated wheel speed      [% duty]
static float positionEstimate = 0;  // x: integral of v              [% duty * s]
static float lastCommand = 0;       // u actually sent to the motors [%]
static float targetAngle = 0;
static float angleIntegral = 0;     // I term of the angle loop, in % (acts as a speed command)
static float rateFiltered = 0;
static float yawAngle = 0, yawTarget = 0, yawCommand = 0;
static bool yawHoldActive = true;
static uint32_t saturatedSinceMs = 0;
static uint32_t balanceStartMs = 0;
static float trimAtBalanceStart = 0;

static uint32_t kickUntilMs = 0;
static float kickPct = 0;
static uint32_t testUntilMs = 0;
static float testLeft = 0, testRight = 0;

// Balance-point calibration ("trimcal"): the robot is held by hand at the point where it
// neither falls forward nor backward; the mean tilt over 2 s becomes the trim.
static const uint16_t LEVEL_CAL_SAMPLES = 1000;
static const float LEVEL_CAL_MAX_RATE_STD = 4.0f;   // deg/s, a steady hand is ~1-2
static const float BIAS_FROM_HAND_MAX_STD = 1.0f;   // only a really still hold also updates bias
static const uint32_t LEVEL_CAL_GIVE_UP_MS = 10000;
static uint16_t levelCount = 0;
static double levelAngleSum = 0, levelGyroSum[3], levelGyroSq[3];
static uint32_t levelBeginMs = 0;

// Battery: sampled at 20 Hz and low-passed (~1 s) so motor current spikes do not trip the
// cutoff; compensation in Motors.ino uses the same filtered value.
float batteryVoltage = 0;
bool batteryLow = false;
static const uint32_t BATTERY_SAMPLE_EVERY = 25;   // loops -> 20 Hz
static const uint32_t BATTERY_LOW_HOLD_MS = 2000;
static const float BATTERY_RECOVER_V = 0.15f;      // hysteresis above vbat_min
static uint32_t batteryLoopCounter = 0;
static uint32_t batteryBelowSinceMs = 0;

static void updateBattery(uint32_t nowMs) {
  if (P.vbat_div <= 0) {
    batteryVoltage = 0;
    batteryLow = false;
    return;
  }
  if (++batteryLoopCounter % BATTERY_SAMPLE_EVERY) return;
  const float v = analogReadMilliVolts(PIN_VBAT) / 1000.0f * P.vbat_div;
  batteryVoltage = batteryVoltage <= 0 ? v : batteryVoltage + 0.05f * (v - batteryVoltage);

  if (batteryVoltage < P.vbat_min) {
    if (!batteryBelowSinceMs) batteryBelowSinceMs = nowMs;
    if (nowMs - batteryBelowSinceMs > BATTERY_LOW_HOLD_MS) batteryLow = true;
  } else {
    batteryBelowSinceMs = 0;
    if (batteryVoltage > P.vbat_min + BATTERY_RECOVER_V) batteryLow = false;
  }
}

static float pwmHzApplied = 0;
static uint32_t statWindowStartMs = 0, statLoops = 0, statDtMaxUs = 0;
static uint32_t telemetryCounter = 0;

const char* stateName(RobotState s) {
  switch (s) {
    case RobotState::Calib:     return "CALIB";
    case RobotState::Idle:      return "IDLE";
    case RobotState::Arming:    return "ARMING";
    case RobotState::Balance:   return "BALANCE";
    case RobotState::Fallen:    return "FALLEN";
    case RobotState::Fault:     return "FAULT";
    case RobotState::MotorTest: return "MTEST";
    case RobotState::LevelCal:  return "TRIMCAL";
    case RobotState::Rising:    return "RISING";
  }
  return "?";
}

static void enterState(RobotState s) {
  robotState = s;
  stateSinceMs = millis();
}

static bool timeReached(uint32_t nowMs, uint32_t deadlineMs) {
  return (int32_t)(nowMs - deadlineMs) >= 0;
}

// initialSpeed: the wheels may already be turning (after a self-righting kick), and the
// speed estimate must know that or the position loop would start from a wrong velocity.
static void resetBalanceState(float initialSpeed) {
  speedEstimate = initialSpeed;
  positionEstimate = 0;
  lastCommand = initialSpeed;
  angleIntegral = 0;
  rateFiltered = imuRate;
  yawAngle = 0;
  yawTarget = 0;
  yawCommand = 0;
  yawHoldActive = true;
  saturatedSinceMs = 0;
  kickUntilMs = 0;
}

static void leaveBalance(RobotState next) {
  motorsCoast();
  lastCommand = 0;
  yawCommand = 0;
  const bool longRun = millis() - balanceStartMs > TRIM_AUTOSAVE_MIN_BALANCE_MS;
  if (longRun && fabsf(P.trim - trimAtBalanceStart) > 0.2f) pendingTrimSave = true;
  if (next == RobotState::Fallen) controlStats.falls++;
  enterState(next);
}

static void handleRequests(uint32_t nowMs) {
  if (requestStop) {
    requestStop = false;
    stopHold = true;
    if (robotState == RobotState::Balance) leaveBalance(RobotState::Idle);
    else if (robotState == RobotState::Arming || robotState == RobotState::MotorTest ||
             robotState == RobotState::Rising) {
      motorsCoast();
      enterState(RobotState::Idle);
    }
  }
  if (requestStart) {
    requestStart = false;
    stopHold = false;
    bootLatch = false;
    riseFailures = 0;
  }
  if (requestZero) {
    requestZero = false;
    positionEstimate = 0;
    yawTarget = yawAngle;
  }
  if (requestCalib) {
    requestCalib = false;
    if (robotState != RobotState::Balance && robotState != RobotState::Fault) {
      motorsCoast();
      imuCalibStart();
      enterState(RobotState::Calib);
    }
  }
  if (requestLevelCal) {
    requestLevelCal = false;
    if (robotState == RobotState::Idle || robotState == RobotState::Arming ||
        robotState == RobotState::Fallen) {
      motorsCoast();
      levelCalReset();
      levelBeginMs = nowMs;
      enterState(RobotState::LevelCal);
    }
  }
  if (requestKickMs) {
    if (robotState == RobotState::Balance) {
      kickPct = requestKickPct;
      kickUntilMs = nowMs + requestKickMs;
    }
    requestKickMs = 0;
  }
  if (requestTestMs) {
    if (robotState == RobotState::Idle) {
      testLeft = requestTestLeft;
      testRight = requestTestRight;
      testUntilMs = nowMs + requestTestMs;
      enterState(RobotState::MotorTest);
    }
    requestTestMs = 0;
  }
}

static void levelCalReset() {
  levelCount = 0;
  levelAngleSum = 0;
  for (int i = 0; i < 3; i++) levelGyroSum[i] = levelGyroSq[i] = 0;
}

static void runLevelCal(const ImuReading& r, uint32_t nowMs) {
  motorsCoast();
  const float raw[3] = {r.gx + imuGyroBias[0], r.gy + imuGyroBias[1], r.gz + imuGyroBias[2]};
  levelAngleSum += imuAccAngle;
  for (int i = 0; i < 3; i++) {
    levelGyroSum[i] += raw[i];
    levelGyroSq[i] += (double)raw[i] * raw[i];
  }
  if (++levelCount < LEVEL_CAL_SAMPLES) {
    if (nowMs - levelBeginMs > LEVEL_CAL_GIVE_UP_MS) {
      levelCalResult = 2;
      bootLatch = true;
      enterState(RobotState::Idle);
    }
    return;
  }

  float mean[3], worstStd = 0;
  for (int i = 0; i < 3; i++) {
    mean[i] = (float)(levelGyroSum[i] / levelCount);
    const double var = levelGyroSq[i] / levelCount - (double)mean[i] * mean[i];
    worstStd = fmaxf(worstStd, (float)sqrt(var > 0 ? var : 0));
  }
  if (worstStd > LEVEL_CAL_MAX_RATE_STD) {  // too shaky: start a new window
    levelCalReset();
    return;
  }
  P.trim = constrain((float)(levelAngleSum / levelCount), -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);
  if (worstStd <= BIAS_FROM_HAND_MAX_STD) {
    for (int i = 0; i < 3; i++) imuGyroBias[i] = mean[i];
    pendingBiasSave = true;
  }
  pendingParamSave = true;
  levelCalResult = 1;
  // The robot is being held upright right now: it must not start driving in the hand.
  bootLatch = true;
  enterState(RobotState::Idle);
}

static bool uprightAndCalm(float tilt) {
  return fabsf(tilt) < P.start_angle && fabsf(imuRate) < ARM_MAX_RATE_DPS;
}

static void startBalance(uint32_t nowMs, float initialSpeed) {
  resetBalanceState(initialSpeed);
  balanceStartMs = nowMs;
  trimAtBalanceStart = P.trim;
  enterState(RobotState::Balance);
}

static void runIdle(uint32_t nowMs, float tilt) {
  motorsCoast();
  if (bootLatch && fabsf(tilt) > 20.0f) bootLatch = false;  // tipped over once: allowed
  const bool mayStart = P.auto_start > 0.5f && !bootLatch && !stopHold && !batteryLow;

  if (uprightAndCalm(tilt)) {
    riseFailures = 0;  // someone stood it up by hand
    if (mayStart) enterState(RobotState::Arming);
    return;
  }

  // "Lying" is judged against fall_angle, so a robot with lower bumpers only needs a
  // smaller fall_angle and self-righting follows automatically.
  const bool lyingStill = fabsf(tilt) > P.fall_angle - 5.0f && fabsf(tilt) < RISE_ABORT_TILT_DEG &&
                          fabsf(imuRate) < RISE_REST_MAX_RATE_DPS;
  if (!lyingStill) {
    restSinceMs = 0;
    return;
  }
  if (!restSinceMs) restSinceMs = nowMs;
  const bool mayRise = mayStart && P.self_right > 0.5f && riseFailures < (uint8_t)P.rise_tries;
  if (mayRise && nowMs - restSinceMs >= RISE_REST_MS) {
    restSinceMs = 0;
    riseSide = tilt > 0 ? 1 : -1;
    riseKicking = P.rise_back_ms <= 0 || P.rise_back <= 0;
    risePhaseStartMs = nowMs;
    speedEstimate = 0;
    yawAngle = 0;
    enterState(RobotState::Rising);
  }
}

// Self-righting: a short drive away from the lean spins the wheels up, then full power
// toward the lean. Reversing a spinning motor briefly gives up to ~2x its stall torque,
// which is what lifts the body off the bumper. The balance loop takes over mid-swing.
static void runRise(float dt, uint32_t nowMs, float tilt) {
  if (fabsf(tilt) > RISE_ABORT_TILT_DEG) {
    motorsCoast();
    riseFailures++;
    enterState(RobotState::Idle);
    return;
  }
  float command;
  if (!riseKicking) {
    command = -riseSide * P.rise_back;
    if (nowMs - risePhaseStartMs >= (uint32_t)P.rise_back_ms) {
      riseKicking = true;
      risePhaseStartMs = nowMs;
    }
  } else {
    if (fabsf(tilt) < P.rise_catch) {
      startBalance(nowMs, speedEstimate);
      runBalance(dt, nowMs, tilt);
      return;
    }
    if (nowMs - risePhaseStartMs > RISE_KICK_MAX_MS) {
      motorsCoast();
      riseFailures++;
      enterState(RobotState::Idle);
      return;
    }
    command = riseSide * P.rise_kick;
  }
  // Keep the wheel-speed model running so the balance loop starts with the right speed.
  speedEstimate += (command - speedEstimate) * dt / fmaxf(P.vel_tau, dt);
  lastCommand = command;

  // The two motors are never exactly equal; at full power that turns the robot, so the
  // heading loop runs here as well (the saturated side cannot go up, the other goes down).
  yawAngle += imuYawRate * dt;
  yawCommand = P.yaw_dir * (-P.kyaw * yawAngle - P.kyaw_d * imuYawRate);
  yawCommand = constrain(yawCommand, -YAW_MAX_OUT_PCT, YAW_MAX_OUT_PCT);
  motorsWrite(command - yawCommand, command + yawCommand);
}

static void runBalance(float dt, uint32_t nowMs, float tilt) {
  if (fabsf(tilt) > P.fall_angle) {
    leaveBalance(RobotState::Fallen);
    return;
  }
  if (batteryLow) {  // an empty cell resets the ESP mid-balance; better to let go on purpose
    leaveBalance(RobotState::Idle);
    return;
  }
  if (riseFailures && nowMs - balanceStartMs > RISE_SUCCESS_MS) riseFailures = 0;

  // --- speed and position estimate from the previous command -----------------
  const float velTau = fmaxf(P.vel_tau, dt);
  speedEstimate += (lastCommand - speedEstimate) * dt / velTau;
  positionEstimate += speedEstimate * dt;

  // --- outer loop: where should the body lean to stop and come back? ----------
  // Drifting toward +x needs a lean toward -x, hence the minus signs.
  const float lean = P.max_lean;
  const float positionTerm = constrain(P.kx * positionEstimate, -0.7f * lean, 0.7f * lean);
  targetAngle = P.trim - positionTerm - P.kv * speedEstimate;
  targetAngle = constrain(targetAngle, P.trim - lean, P.trim + lean);

  // A steady position offset means the stored balance angle is wrong: slowly move it.
  // This is the integral part of the position loop, and it is what gets saved to flash.
  const float xForLearning = constrain(positionEstimate, -TRIM_LEARN_X_CLAMP, TRIM_LEARN_X_CLAMP);
  P.trim = constrain(P.trim - P.kxi * xForLearning * dt, -TRIM_LIMIT_DEG, TRIM_LIMIT_DEG);

  // --- inner loop: PID on the angle, D taken straight from the gyro -------------
  if (P.d_lpf > 0) {
    const float rc = 1.0f / (2.0f * PI * P.d_lpf);
    rateFiltered += (imuRate - rateFiltered) * dt / (rc + dt);
  } else {
    rateFiltered = imuRate;
  }
  const float angleError = imuAngle - targetAngle;
  // Clamped to the motor range so it can never wind up beyond what the wheels can do.
  angleIntegral = constrain(angleIntegral + P.ki * angleError * dt, -P.max_out, P.max_out);
  float u = P.kp * angleError + angleIntegral + P.kd * rateFiltered;

  if (kickUntilMs && !timeReached(nowMs, kickUntilMs)) u += kickPct;
  else kickUntilMs = 0;

  u = constrain(u, -P.max_out, P.max_out);

  // Full power for a whole second means the wheels are in the air or blocked.
  if (fabsf(u) >= 0.98f * P.max_out) {
    if (!saturatedSinceMs) saturatedSinceMs = nowMs;
    else if (nowMs - saturatedSinceMs > SATURATION_FALL_MS) {
      leaveBalance(RobotState::Fallen);
      return;
    }
  } else {
    saturatedSinceMs = 0;
  }

  // --- heading hold ------------------------------------------------------------
  yawAngle += imuYawRate * dt;
  yawCommand = 0;
  if (yawHoldActive && P.yaw_dir != 0) {
    const float yawError = yawTarget - yawAngle;
    if (fabsf(yawError) > YAW_RUNAWAY_DEG) {
      // Either the sign is wrong (it spins itself away) or someone turned it by hand.
      // Holding on would only make it spin, so heading hold rests until the next start.
      yawHoldActive = false;
      controlStats.yawRunaways++;
    } else {
      yawCommand = P.yaw_dir * (P.kyaw * yawError - P.kyaw_d * imuYawRate);
      yawCommand = constrain(yawCommand, -YAW_MAX_OUT_PCT, YAW_MAX_OUT_PCT);
    }
  }

  motorsWrite(u - yawCommand, u + yawCommand);
  lastCommand = u;
}

static void updateLoopStats(uint32_t dtUs, uint32_t nowMs) {
  statLoops++;
  if (dtUs > statDtMaxUs) statDtMaxUs = dtUs;
  if (nowMs - statWindowStartMs >= 1000) {
    controlStats.loopHz = statLoops * 1000.0f / (nowMs - statWindowStartMs);
    controlStats.dtMaxUs = statDtMaxUs;
    statLoops = 0;
    statDtMaxUs = 0;
    statWindowStartMs = nowMs;
  }
}

static void publishTelemetry(uint32_t nowMs, uint32_t dtUs) {
  const bool balancing = robotState == RobotState::Balance;
  const bool driving = balancing || robotState == RobotState::Rising;
  Telemetry t;
  t.tMs = nowMs;
  t.state = (uint8_t)robotState;
  t.angle = imuAngle;
  t.accAngle = imuAccAngle;
  t.rate = imuRate;
  t.target = balancing ? targetAngle : P.trim;
  t.u = driving ? lastCommand : 0;
  t.x = positionEstimate;
  t.v = speedEstimate;
  t.trim = P.trim;
  t.yaw = yawAngle;
  t.uDiff = yawCommand;
  t.dtUs = (uint16_t)min<uint32_t>(dtUs, 65535);
  t.vbat = batteryVoltage;

  portENTER_CRITICAL(&telemetryMux);
  latestTelemetry = t;
  portEXIT_CRITICAL(&telemetryMux);

  const uint32_t div = (uint32_t)max(1.0f, P.tel_div);
  if ((telemetryStreaming || serialStreaming) && (++telemetryCounter % div) == 0) {
    xQueueSend(telemetryQueue, &t, 0);  // never wait: dropping a sample beats a late update
  }
}

static void updateLed(uint32_t nowMs) {
  bool on;
  switch (robotState) {
    case RobotState::Balance:
    case RobotState::Rising:
    case RobotState::MotorTest: on = true; break;
    case RobotState::Calib:     on = (nowMs / 100) % 2; break;
    case RobotState::Arming:    on = (nowMs / 200) % 2; break;
    case RobotState::LevelCal:  on = (nowMs / 100) % 2; break;
    case RobotState::Fault:     on = (nowMs / 50) % 2; break;
    case RobotState::Idle:      // low battery: double blink; otherwise one short blink
      on = batteryLow ? (nowMs % 1000) < 80 || ((nowMs % 1000) >= 200 && (nowMs % 1000) < 280)
                      : (nowMs % 1000) < 80;
      break;
    default:                    on = false; break;
  }
  digitalWrite(PIN_LED, on ? LOW : HIGH);
}

void controlTask(void* arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t prevUs = micros();
  statWindowStartMs = millis();

  if (robotState != RobotState::Fault) {
    imuCalibStart();
    enterState(RobotState::Calib);
  } else {
    enterState(RobotState::Fault);
  }

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    const uint32_t nowUs = micros();
    const uint32_t dtUs = nowUs - prevUs;
    prevUs = nowUs;
    const float dt = constrain(dtUs * 1e-6f, 0.0005f, 0.02f);
    const uint32_t nowMs = millis();
    controlHeartbeatMs = nowMs;
    updateLoopStats(dtUs, nowMs);
    updateBattery(nowMs);

    if (P.pwm_hz != pwmHzApplied) {
      motorsSetFrequency(P.pwm_hz);
      pwmHzApplied = P.pwm_hz;
    }
    handleRequests(nowMs);

    if (otaActive) {  // firmware is being replaced: stay still until the reboot
      motorsCoast();
      if (robotState == RobotState::Balance || robotState == RobotState::Rising) {
        enterState(RobotState::Idle);
      }
      updateLed(nowMs);
      continue;
    }

    if (robotState == RobotState::Fault) {
      motorsCoast();
      if (nowMs - stateSinceMs > 500) {
        if (imuBegin()) {
          imuFailCount = 0;
          imuCalibStart();
          enterState(RobotState::Calib);
        } else {
          stateSinceMs = millis();
        }
      }
      updateLed(nowMs);
      continue;
    }

    ImuReading r;
    if (!imuRead(r)) {
      controlStats.imuErrors++;
      if (++imuFailCount >= 5) {
        if (robotState == RobotState::Balance) leaveBalance(RobotState::Fault);
        else enterState(RobotState::Fault);
        motorsCoast();
      }
      continue;  // a single missed sample: keep the last motor command
    }
    imuFailCount = 0;
    imuUpdate(r, dt);

    const float tilt = imuAngle - P.trim;

    switch (robotState) {
      case RobotState::Calib: {
        motorsCoast();
        const int result = imuCalibStep(r);
        if (result != 0) enterState(RobotState::Idle);
        break;
      }
      case RobotState::Idle:
        runIdle(nowMs, tilt);
        break;
      case RobotState::Arming:
        motorsCoast();
        if (!uprightAndCalm(tilt) || stopHold || P.auto_start < 0.5f) {
          enterState(RobotState::Idle);
        } else if (nowMs - stateSinceMs >= ARM_HOLD_MS) {
          startBalance(nowMs, 0);
        }
        break;
      case RobotState::Rising:
        runRise(dt, nowMs, tilt);
        break;
      case RobotState::Balance:
        runBalance(dt, nowMs, tilt);
        break;
      case RobotState::Fallen:
        motorsCoast();
        if (nowMs - stateSinceMs > 300) enterState(RobotState::Idle);
        break;
      case RobotState::MotorTest:
        if (timeReached(nowMs, testUntilMs)) {
          motorsCoast();
          enterState(RobotState::Idle);
        } else {
          motorsWrite(testLeft, testRight);
        }
        break;
      case RobotState::LevelCal:
        runLevelCal(r, nowMs);
        break;
      case RobotState::Fault:
        break;
    }

    publishTelemetry(nowMs, dtUs);
    updateLed(nowMs);
  }
}
