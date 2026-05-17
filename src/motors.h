#pragma once

#include <Arduino.h>
#include "config.h"

struct MotorSettings {
  int leftSpeed = DEFAULT_LEFT_SPEED;
  int rightSpeed = DEFAULT_RIGHT_SPEED;
  int baseSpeed = DEFAULT_BASE_SPEED;
  int turnSpeed = DEFAULT_TURN_SPEED;
};

class Motors {
public:
  void begin();
  void stop();
  void emergencyStop();
  void clearEmergency();
  bool isEmergencyStopped() const;
  void set(int left, int right);
  int leftOutput() const { return _leftOut; }
  int rightOutput() const { return _rightOut; }

private:
  void writeOne(uint8_t forwardChannel, uint8_t reverseChannel, int value);
  int rampToward(int current, int target) const;
  int _leftOut = 0;
  int _rightOut = 0;
  bool _eStop = false;
};

extern Motors motors;
