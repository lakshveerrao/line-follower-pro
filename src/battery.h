#pragma once

#include <Arduino.h>
#include "config.h"

class Battery {
public:
  void begin();
  void update();
  float voltage() const;
  uint8_t percent() const;
  const char *status() const;

private:
  float _voltage = 0;
  unsigned long _lastRead = 0;
};

extern Battery battery;

