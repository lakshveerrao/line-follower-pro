#pragma once

#include <Arduino.h>
#include "config.h"

enum class JoyEvent : uint8_t {
  None,
  Up,
  Down,
  Left,
  Right,
  Press,
  LongPress
};

class Joystick {
public:
  void begin();
  JoyEvent update();
  bool pressed() const;
  bool bootHeldLongEnough(unsigned long bootStart) const;
  int rawX() const { return _rawX; }
  int rawY() const { return _rawY; }
  JoyEvent lastAxisEvent() const { return axisEvent(_rawX, _rawY); }

private:
  JoyEvent axisEvent(int x, int y) const;
  int _rawX = 0;
  int _rawY = 0;
  int _filteredX = 0;
  int _filteredY = 0;
  int _centerX = 2048;
  int _centerY = 2048;
  bool _lastButton = false;
  bool _longSent = false;
  unsigned long _pressStart = 0;
  unsigned long _lastAxisEvent = 0;
  unsigned long _lastDebug = 0;
};

extern Joystick joystick;
