#pragma once

#include <Arduino.h>
#include "config.h"

class Logger {
public:
  void begin();
  void log(const String &message);
  uint8_t count() const;
  String line(uint8_t indexFromNewest) const;

private:
  char lines[LOG_LINE_COUNT][LOG_LINE_LEN]{};
  uint8_t head = 0;
  uint8_t used = 0;
};

extern Logger logger;

