#pragma once

#include <Arduino.h>
#include "config.h"

struct SensorCalibration {
  uint16_t black[SENSOR_COUNT]{};
  uint16_t white[SENSOR_COUNT]{};
  uint16_t threshold = DEFAULT_SENSOR_THRESHOLD;
};

class Sensors {
public:
  void begin();
  void update();
  void calibrateBlack();
  void calibrateWhite();
  bool valid() const;
  bool lineDetected() const;
  int position() const;
  int error() const;
  bool junction() const;
  uint16_t value(uint8_t index) const;
  uint16_t lineValue(uint8_t index) const;
  uint8_t activeCount() const;
  void setCalibration(const SensorCalibration &cal);
  SensorCalibration calibration() const;

private:
  uint16_t _values[SENSOR_COUNT]{};
  SensorCalibration _cal;
  unsigned long _lastRead = 0;
};

extern Sensors sensors;
