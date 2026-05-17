#include "joystick.h"

Joystick joystick;

void Joystick::begin() {
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  long sumX = 0;
  long sumY = 0;
  constexpr uint8_t samples = 16;
  for (uint8_t i = 0; i < samples; i++) {
    sumX += analogRead(JOY_X_PIN);
    sumY += analogRead(JOY_Y_PIN);
    delay(2);
  }
  _centerX = sumX / samples;
  _centerY = sumY / samples;
  _rawX = _filteredX = _centerX;
  _rawY = _filteredY = _centerY;
}

JoyEvent Joystick::update() {
  bool nowPressed = pressed();
  unsigned long now = millis();
  _rawX = analogRead(JOY_X_PIN);
  _rawY = analogRead(JOY_Y_PIN);
  _filteredX += (_rawX - _filteredX) >> JOY_FILTER_SHIFT;
  _filteredY += (_rawY - _filteredY) >> JOY_FILTER_SHIFT;

  if (JOYSTICK_SERIAL_DEBUG && now - _lastDebug >= JOYSTICK_DEBUG_MS) {
    _lastDebug = now;
    Serial.print("JOY x=");
    Serial.print(_rawX);
    Serial.print(" y=");
    Serial.print(_rawY);
    Serial.print(" sw=");
    Serial.print(nowPressed ? "PRESSED" : "open");
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

  if (now - _lastAxisEvent >= JOY_REPEAT_MS) {
    JoyEvent e = axisEvent(_filteredX, _filteredY);
    if (e != JoyEvent::None) {
      _lastAxisEvent = now;
      return e;
    }
  }
  return JoyEvent::None;
}

bool Joystick::pressed() const {
  return digitalRead(JOY_BTN_PIN) == LOW;
}

bool Joystick::bootHeldLongEnough(unsigned long bootStart) const {
  return pressed() && millis() - bootStart >= BOOT_SAFE_HOLD_MS;
}

JoyEvent Joystick::axisEvent(int x, int y) const {
  int dx = x - _centerX;
  int dy = y - _centerY;
  if (abs(dx) < JOY_AXIS_DEADZONE && abs(dy) < JOY_AXIS_DEADZONE) return JoyEvent::None;

  // Choose the axis moved farthest from its boot-calibrated center. This keeps
  // diagonal noise from making menu navigation feel random.
  if (abs(dx) > abs(dy)) {
    return dx < 0 ? JoyEvent::Left : JoyEvent::Right;
  }
  return dy < 0 ? JoyEvent::Up : JoyEvent::Down;
}
