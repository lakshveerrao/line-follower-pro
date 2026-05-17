#include "battery.h"

Battery battery;

void Battery::begin() {
  if (BATTERY_SENSE_ENABLED) pinMode(BATTERY_PIN, INPUT);
}

void Battery::update() {
  unsigned long now = millis();
  if (now - _lastRead < BATTERY_PERIOD_MS) return;
  _lastRead = now;
  if (!BATTERY_SENSE_ENABLED) {
    _voltage = 0;
    return;
  }
  uint16_t raw = analogRead(BATTERY_PIN);
  float adcVoltage = (raw / 4095.0f) * ADC_REF_VOLTAGE;
  _voltage = adcVoltage * BATTERY_DIVIDER_RATIO;
}

float Battery::voltage() const {
  return _voltage;
}

uint8_t Battery::percent() const {
  if (!BATTERY_SENSE_ENABLED) return 0;
  float pct = ((_voltage - BATTERY_CRITICAL_V) / (BATTERY_FULL_V - BATTERY_CRITICAL_V)) * 100.0f;
  return constrain(static_cast<int>(pct), 0, 100);
}

const char *Battery::status() const {
  if (!BATTERY_SENSE_ENABLED) return "N/A";
  if (_voltage <= BATTERY_CRITICAL_V) return "CRITICAL";
  if (_voltage <= BATTERY_LOW_V) return "LOW";
  return "GOOD";
}

