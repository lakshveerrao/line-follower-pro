#pragma once

#include <Arduino.h>

class PidController {
public:
  void setTunings(float kp, float ki, float kd);
  void reset();
  float compute(float error, float dtSeconds);
  float kp() const { return _kp; }
  float ki() const { return _ki; }
  float kd() const { return _kd; }
  float output() const { return _output; }

private:
  float _kp = 0;
  float _ki = 0;
  float _kd = 0;
  float _lastError = 0;
  float _integral = 0;
  float _output = 0;
};

