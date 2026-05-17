#include "motors.h"

Motors motors;

void Motors::begin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(M1A_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_BITS, M1A_PWM_CH);
  ledcAttachChannel(M1B_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_BITS, M1B_PWM_CH);
  ledcAttachChannel(M2A_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_BITS, M2A_PWM_CH);
  ledcAttachChannel(M2B_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_BITS, M2B_PWM_CH);
#else
  ledcSetup(M1A_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcSetup(M1B_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcSetup(M2A_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcSetup(M2B_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcAttachPin(M1A_PIN, M1A_PWM_CH);
  ledcAttachPin(M1B_PIN, M1B_PWM_CH);
  ledcAttachPin(M2A_PIN, M2A_PWM_CH);
  ledcAttachPin(M2B_PIN, M2B_PWM_CH);
#endif
  stop();
}

void Motors::stop() {
  _leftOut = 0;
  _rightOut = 0;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(M1A_PWM_CH, 0);
  ledcWriteChannel(M1B_PWM_CH, 0);
  ledcWriteChannel(M2A_PWM_CH, 0);
  ledcWriteChannel(M2B_PWM_CH, 0);
#else
  ledcWrite(M1A_PWM_CH, 0);
  ledcWrite(M1B_PWM_CH, 0);
  ledcWrite(M2A_PWM_CH, 0);
  ledcWrite(M2B_PWM_CH, 0);
#endif
}

void Motors::emergencyStop() {
  _eStop = true;
  stop();
}

void Motors::clearEmergency() {
  _eStop = false;
}

bool Motors::isEmergencyStopped() const {
  return _eStop;
}

void Motors::set(int left, int right) {
  if (_eStop) {
    stop();
    return;
  }
  int leftTarget = constrain(left, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  int rightTarget = constrain(right, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  _leftOut = rampToward(_leftOut, leftTarget);
  _rightOut = rampToward(_rightOut, rightTarget);
  writeOne(M1A_PWM_CH, M1B_PWM_CH, _leftOut);
  writeOne(M2A_PWM_CH, M2B_PWM_CH, _rightOut);
}

int Motors::rampToward(int current, int target) const {
  if (current < target) return min(current + MOTOR_RAMP_STEP, target);
  if (current > target) return max(current - MOTOR_RAMP_STEP, target);
  return current;
}

void Motors::writeOne(uint8_t forwardChannel, uint8_t reverseChannel, int value) {
  uint8_t forwardPwm = value >= 0 ? abs(value) : 0;
  uint8_t reversePwm = value < 0 ? abs(value) : 0;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(forwardChannel, forwardPwm);
  ledcWriteChannel(reverseChannel, reversePwm);
#else
  ledcWrite(forwardChannel, forwardPwm);
  ledcWrite(reverseChannel, reversePwm);
#endif
}
