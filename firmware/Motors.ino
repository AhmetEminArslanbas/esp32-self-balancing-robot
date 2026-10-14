// Motor output for an H-bridge with two inputs per motor (DRV8833 / MX1508 / L9110 style;
// also an L298N with its enable jumpers fitted).
//
// Slow decay (default): one input held HIGH, the other PWMed. During the off-time the
// motor is shorted (brake) instead of freewheeling, so its speed follows the duty almost
// linearly. That linearity is what makes the encoder-less position estimate usable.
// Fast decay: one input PWMed, the other LOW (what Kuasar did with analogWrite).

static const uint8_t PWM_BITS = 10;
static const uint32_t PWM_FULL = (1u << PWM_BITS) - 1;  // the core maps this to 100 %
static const float DEADBAND_RAMP_PCT = 2.0f;           // commands below this fade in linearly
static const uint8_t MOTOR_PINS[4] = {PIN_LEFT_A, PIN_LEFT_B, PIN_RIGHT_A, PIN_RIGHT_B};

static uint32_t clampFrequency(float hz) {
  return (uint32_t)constrain(hz, 100.0f, 40000.0f);
}

void motorsBegin() {
  const uint32_t freq = clampFrequency(P.pwm_hz);
  for (uint8_t pin : MOTOR_PINS) {
    ledcAttach(pin, freq, PWM_BITS);
    ledcWrite(pin, 0);
  }
}

void motorsSetFrequency(float hz) {
  const uint32_t freq = clampFrequency(hz);
  for (uint8_t pin : MOTOR_PINS) ledcChangeFrequency(pin, freq, PWM_BITS);
}

// Maps a command magnitude (0..100 %) to the fraction of time the motor is driven.
// A DC motor does nothing below a certain duty (friction), so that dead band is added
// on top; very small commands ramp in so the output stays continuous through zero.
static float driveFraction(float magnitudePct) {
  const float mag = fminf(magnitudePct, 100.0f);
  const float deadband = constrain(P.deadband, 0.0f, 60.0f);
  const float span = (100.0f - deadband) / 100.0f;
  if (mag < DEADBAND_RAMP_PCT) {
    return (mag / DEADBAND_RAMP_PCT) * (deadband + DEADBAND_RAMP_PCT * span) / 100.0f;
  }
  return (deadband + mag * span) / 100.0f;
}

static void writeWheel(uint8_t pinA, uint8_t pinB, float commandPct, bool inverted) {
  if (inverted) commandPct = -commandPct;
  const uint32_t duty = (uint32_t)lroundf(driveFraction(fabsf(commandPct)) * PWM_FULL);
  const uint8_t drivePin = commandPct >= 0 ? pinA : pinB;
  const uint8_t otherPin = commandPct >= 0 ? pinB : pinA;

  if (P.slow_decay > 0.5f) {
    ledcWrite(drivePin, PWM_FULL);
    ledcWrite(otherPin, PWM_FULL - duty);  // HIGH/HIGH = brake, HIGH/LOW = drive
  } else {
    ledcWrite(drivePin, duty);
    ledcWrite(otherPin, 0);
  }
}

// Motor speed is proportional to duty x battery voltage. Scaling the duty by
// vbat_ref / vbat keeps the same command meaning the same wheel speed as the cell drains,
// so the tuning and the encoder-less position estimate stay valid.
static float batteryScale() {
  if (P.vbat_div <= 0 || batteryVoltage < 2.5f) return 1.0f;  // not fitted / implausible
  return constrain(P.vbat_ref / batteryVoltage, 0.8f, 1.4f);
}

void motorsWrite(float leftPct, float rightPct) {
  const float k = batteryScale();
  writeWheel(PIN_LEFT_A, PIN_LEFT_B, constrain(leftPct * k, -100.0f, 100.0f), P.inv_l > 0.5f);
  writeWheel(PIN_RIGHT_A, PIN_RIGHT_B, constrain(rightPct * k, -100.0f, 100.0f), P.inv_r > 0.5f);
}

// All inputs LOW: the motors freewheel. Used whenever the robot is not balancing.
void motorsCoast() {
  for (uint8_t pin : MOTOR_PINS) ledcWrite(pin, 0);
}
