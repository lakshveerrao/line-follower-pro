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
  void recalibrateCenter();
  bool pressed() const;
  bool bootHeldLongEnough(unsigned long bootStart) const;
  int rawX() const { return _rawX; }
  int rawY() const { return _rawY; }
  int centerX() const { return _centerX; }
  int centerY() const { return _centerY; }
  JoyEvent lastAxisEvent() const { return axisEvent(_rawX, _rawY); }

private:
  int readAveraged(uint8_t pin) const;
  JoyEvent axisEvent(int x, int y) const;
  void updateIdleCenter(JoyEvent axis);
  bool readButtonRaw() const;
  void updateButton(unsigned long now);
  int _rawX = 0;
  int _rawY = 0;
  int _filteredX = 0;
  int _filteredY = 0;
  int _centerX = 2048;
  int _centerY = 2048;
  JoyEvent _heldAxis = JoyEvent::None;
  JoyEvent _candidateAxis = JoyEvent::None;
  unsigned long _candidateSince = 0;
  unsigned long _axisHeldSince = 0;
  bool _lastButton = false;
  bool _stableButton = false;
  bool _rawButton = false;
  bool _longSent = false;
  unsigned long _pressStart = 0;
  unsigned long _lastButtonChange = 0;
  unsigned long _lastAxisEvent = 0;
  unsigned long _lastDebug = 0;
};

extern Joystick joystick;
