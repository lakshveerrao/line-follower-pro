#include "logger.h"

Logger logger;

void Logger::begin() {
  head = 0;
  used = 0;
}

void Logger::log(const String &message) {
  message.substring(0, LOG_LINE_LEN - 1).toCharArray(lines[head], LOG_LINE_LEN);
  head = (head + 1) % LOG_LINE_COUNT;
  if (used < LOG_LINE_COUNT) used++;
  Serial.println(message);
}

uint8_t Logger::count() const {
  return used;
}

String Logger::line(uint8_t indexFromNewest) const {
  if (indexFromNewest >= used) return "";
  int idx = static_cast<int>(head) - 1 - indexFromNewest;
  while (idx < 0) idx += LOG_LINE_COUNT;
  return String(lines[idx]);
}

