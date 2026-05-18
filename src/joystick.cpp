#include "joystick.h"

Joystick joystick;

void Joystick::begin() {
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(JOY_X_PIN, ADC_11db);
  analogSetPinAttenuation(JOY_Y_PIN, ADC_11db);
  unsigned long settleStart = millis();
  while (millis() - settleStart < JOY_BOOT_CALIBRATION_WAIT_MS) {
    delay(5);
  }
  long sumX = 0;
  long sumY = 0;
  uint16_t samples = 0;
  unsigned long sampleStart = millis();
  while (millis() - sampleStart < JOY_BOOT_CALIBRATION_SAMPLE_MS) {
    sumX += readAveraged(JOY_X_PIN);
    sumY += readAveraged(JOY_Y_PIN);
    samples++;
    delay(5);
  }
  if (samples == 0) samples = 1;
  _centerX = sumX / samples;
  _centerY = sumY / samples;
  _rawX = _filteredX = _centerX;
  _rawY = _filteredY = _centerY;
  _rawButton = _stableButton = readButtonRaw();
  _lastButton = _stableButton;
  _heldAxis = JoyEvent::None;
  _candidateAxis = JoyEvent::None;
  _candidateSince = 0;
}

void Joystick::recalibrateCenter() {
  long sumX = 0;
  long sumY = 0;
  constexpr uint8_t samples = 24;
  for (uint8_t i = 0; i < samples; i++) {
    sumX += readAveraged(JOY_X_PIN);
    sumY += readAveraged(JOY_Y_PIN);
    delay(2);
  }
  _centerX = sumX / samples;
  _centerY = sumY / samples;
  _filteredX = _rawX = _centerX;
  _filteredY = _rawY = _centerY;
  _heldAxis = JoyEvent::None;
  _candidateAxis = JoyEvent::None;
  _candidateSince = 0;
  _axisHeldSince = 0;
  _lastAxisEvent = millis();
}

JoyEvent Joystick::update() {
  unsigned long now = millis();
  updateButton(now);
  bool nowPressed = _stableButton;
  _rawX = readAveraged(JOY_X_PIN);
  _rawY = readAveraged(JOY_Y_PIN);
  _filteredX += (_rawX - _filteredX) >> JOY_FILTER_SHIFT;
  _filteredY += (_rawY - _filteredY) >> JOY_FILTER_SHIFT;
  JoyEvent axis = axisEvent(_filteredX, _filteredY);
  updateIdleCenter(axis);

  if (JOYSTICK_SERIAL_DEBUG && now - _lastDebug >= JOYSTICK_DEBUG_MS) {
    _lastDebug = now;
    Serial.print("JOY x=");
    Serial.print(_rawX);
    Serial.print(" y=");
    Serial.print(_rawY);
    Serial.print(" cx=");
    Serial.print(_centerX);
    Serial.print(" cy=");
    Serial.print(_centerY);
    Serial.print(" sw=");
    Serial.print(nowPressed ? "PRESSED" : "open");
    Serial.print(" dir=");
    switch (axis) {
      case JoyEvent::Up: Serial.print("UP"); break;
      case JoyEvent::Down: Serial.print("DOWN"); break;
      case JoyEvent::Left: Serial.print("LEFT"); break;
      case JoyEvent::Right: Serial.print("RIGHT"); break;
      default: Serial.print("CENTER"); break;
    }
    Serial.print(" pins X/Y/SW=");
    Serial.print(JOY_X_PIN);
    Serial.print("/");
    Serial.print(JOY_Y_PIN);
    Serial.print("/");
    Serial.println(JOY_BTN_PIN);
  }

  if (nowPressed && !_lastButton) {
    _pressStart = now;
    _longSent = false;
  }
  if (nowPressed && !_longSent && now - _pressStart >= JOY_LONG_PRESS_MS) {
    _longSent = true;
    _lastButton = nowPressed;
    return JoyEvent::LongPress;
  }
  if (!nowPressed && _lastButton && !_longSent) {
    _lastButton = nowPressed;
    return JoyEvent::Press;
  }
  _lastButton = nowPressed;

  if (axis == JoyEvent::None) {
    _heldAxis = JoyEvent::None;
    _candidateAxis = JoyEvent::None;
    _candidateSince = 0;
    _axisHeldSince = 0;
    return JoyEvent::None;
  }

  if (axis != _candidateAxis) {
    _candidateAxis = axis;
    _candidateSince = now;
    return JoyEvent::None;
  }

  if (now - _candidateSince < JOY_AXIS_CONFIRM_MS) {
    return JoyEvent::None;
  }

  if (_heldAxis == JoyEvent::None) {
    _heldAxis = axis;
    _axisHeldSince = now;
    _lastAxisEvent = now;
    return axis;
  }

  if (!JOY_HOLD_REPEAT_ENABLED) {
    return JoyEvent::None;
  }

  unsigned long repeatMs = now - _axisHeldSince < JOY_FIRST_REPEAT_MS ? JOY_FIRST_REPEAT_MS : JOY_REPEAT_MS;
  if (now - _lastAxisEvent >= repeatMs) {
      _lastAxisEvent = now;
      return axis;
  }
  return JoyEvent::None;
}

bool Joystick::pressed() const {
  return _stableButton;
}

bool Joystick::bootHeldLongEnough(unsigned long bootStart) const {
  return pressed() && millis() - bootStart >= BOOT_SAFE_HOLD_MS;
}

int Joystick::readAveraged(uint8_t pin) const {
  uint16_t total = 0;
  constexpr uint8_t samples = 2;
  for (uint8_t i = 0; i < samples; i++) {
    total += analogRead(pin);
  }
  return total / samples;
}

bool Joystick::readButtonRaw() const {
  return digitalRead(JOY_BTN_PIN) == LOW;
}

void Joystick::updateButton(unsigned long now) {
  bool raw = readButtonRaw();
  if (raw != _rawButton) {
    _rawButton = raw;
    _lastButtonChange = now;
  }
  if (now - _lastButtonChange >= JOY_DEBOUNCE_MS) {
    _stableButton = _rawButton;
  }
}

JoyEvent Joystick::axisEvent(int x, int y) const {
  int dx = x - _centerX;
  int dy = y - _centerY;
  int ax = abs(dx);
  int ay = abs(dy);
  if (_heldAxis != JoyEvent::None) {
    if (ax < JOY_AXIS_RELEASE_ZONE && ay < JOY_Y_AXIS_RELEASE_ZONE) return JoyEvent::None;
    if ((_heldAxis == JoyEvent::Left || _heldAxis == JoyEvent::Right) && ax >= JOY_AXIS_RELEASE_ZONE && ax >= ay) {
      return dx < 0 ? JoyEvent::Left : JoyEvent::Right;
    }
    if ((_heldAxis == JoyEvent::Up || _heldAxis == JoyEvent::Down) && ay >= JOY_Y_AXIS_RELEASE_ZONE) {
      return dy < 0 ? JoyEvent::Up : JoyEvent::Down;
    }
  }

  bool xActive = ax >= JOY_AXIS_DEADZONE;
  bool yActive = ay >= JOY_Y_AXIS_DEADZONE;
  if (!xActive && !yActive) return JoyEvent::None;

  // The Y axis on many joystick modules has a much smaller ADC span. Compare
  // movement after scaling by each axis threshold so UP/DOWN is still usable.
  long xScore = xActive ? (long)ax * 100L / JOY_AXIS_DEADZONE : 0;
  long yScore = yActive ? (long)ay * 100L / JOY_Y_AXIS_DEADZONE : 0;

  // Prefer Y unless X is clearly stronger. The measured Y range on this
  // joystick is small, so equal scoring still needs to mean UP/DOWN.
  if (xScore > yScore + 45) {
    return dx < 0 ? JoyEvent::Left : JoyEvent::Right;
  }
  if (yActive) return dy < 0 ? JoyEvent::Up : JoyEvent::Down;
  return dx < 0 ? JoyEvent::Left : JoyEvent::Right;
}

void Joystick::updateIdleCenter(JoyEvent axis) {
  int dx = _rawX - _centerX;
  int dy = _rawY - _centerY;
  if (axis != JoyEvent::None) return;
  if (abs(dx) > 18 || abs(dy) > 18) return;

  // Joystick modules often rest off-center. Track tiny idle drift slowly, but
  // never follow real stick movement.
  if (dx > 0) _centerX++;
  if (dx < 0) _centerX--;
  if (dy > 0) _centerY++;
  if (dy < 0) _centerY--;
}
