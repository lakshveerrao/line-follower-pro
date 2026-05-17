#include "sensors.h"

Sensors sensors;

void Sensors::begin() {
  pinMode(SENSOR_LEDON_PIN, OUTPUT);
  digitalWrite(SENSOR_LEDON_PIN, HIGH);
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    _cal.black[i] = 3500;
    _cal.white[i] = 500;
  }
  update();
}

void Sensors::update() {
  unsigned long now = millis();
  if (now - _lastRead < SENSOR_PERIOD_MS) return;
  _lastRead = now;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    _values[i] = analogRead(SENSOR_PINS[i]);
  }
}

void Sensors::calibrateBlack() {
  update();
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) _cal.black[i] = _values[i];
}

void Sensors::calibrateWhite() {
  update();
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) _cal.white[i] = _values[i];
}

bool Sensors::valid() const {
  bool anyMovingValue = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (_values[i] > SENSOR_INVALID_LOW && _values[i] < SENSOR_INVALID_HIGH) anyMovingValue = true;
  }
  return anyMovingValue;
}

bool Sensors::lineDetected() const {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (lineValue(i) >= _cal.threshold) return true;
  }
  return false;
}

int Sensors::position() const {
  long weighted = 0;
  long total = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t v = lineValue(i);
    if (v < _cal.threshold) v = 0;
    v = (static_cast<uint32_t>(v) * v) / 1000U; // Favor the strongest sensors.
    weighted += static_cast<long>(v) * i * 1000L;
    total += v;
  }
  if (total == 0) return ((SENSOR_COUNT - 1) * 1000) / 2;
  return weighted / total;
}

int Sensors::error() const {
  int center = ((SENSOR_COUNT - 1) * 1000) / 2;
  return position() - center;
}

bool Sensors::junction() const {
  bool leftEdge = lineValue(0) >= _cal.threshold || lineValue(1) >= _cal.threshold;
  bool rightEdge = lineValue(SENSOR_COUNT - 2) >= _cal.threshold || lineValue(SENSOR_COUNT - 1) >= _cal.threshold;
  return activeCount() >= 6 && leftEdge && rightEdge;
}

uint16_t Sensors::value(uint8_t index) const {
  return index < SENSOR_COUNT ? _values[index] : 0;
}

uint16_t Sensors::lineValue(uint8_t index) const {
  if (index >= SENSOR_COUNT) return 0;
  uint16_t minRaw = _values[0];
  uint16_t maxRaw = _values[0];
  for (uint8_t i = 1; i < SENSOR_COUNT; i++) {
    minRaw = min(minRaw, _values[i]);
    maxRaw = max(maxRaw, _values[i]);
  }

  if (maxRaw - minRaw >= SENSOR_MIN_CONTRAST) {
    long adaptive = (static_cast<long>(_values[index]) - minRaw) * 1000L / (maxRaw - minRaw);
    return constrain(static_cast<int>(adaptive), 0, 1000);
  }

  int black = _cal.black[index];
  int white = _cal.white[index];
  int span = black - white;
  if (abs(span) < 20) return 0;

  long normalized = (static_cast<long>(_values[index]) - white) * 1000L / span;
  return constrain(static_cast<int>(normalized), 0, 1000);
}

uint8_t Sensors::activeCount() const {
  uint8_t active = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (lineValue(i) >= _cal.threshold) active++;
  }
  return active;
}

void Sensors::setCalibration(const SensorCalibration &cal) {
  _cal = cal;
}

SensorCalibration Sensors::calibration() const {
  return _cal;
}
