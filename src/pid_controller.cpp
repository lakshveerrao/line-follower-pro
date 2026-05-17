#include "pid_controller.h"

void PidController::setTunings(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void PidController::reset() {
  _lastError = 0;
  _integral = 0;
  _output = 0;
}

float PidController::compute(float error, float dtSeconds) {
  if (dtSeconds <= 0.0f) return _output;
  _integral += error * dtSeconds;
  _integral = constrain(_integral, -1000.0f, 1000.0f);
  float derivative = (error - _lastError) / dtSeconds;
  _lastError = error;
  _output = (_kp * error) + (_ki * _integral) + (_kd * derivative);
  return _output;
}

