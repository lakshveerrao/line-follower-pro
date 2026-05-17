#pragma once

#include <Arduino.h>
#include <WiFi.h>

class WifiManager {
public:
  void begin(bool enabled, const char *ssid, const char *password);
  void update();
  void setEnabled(bool enabled);
  bool enabled() const;
  bool connected() const;
  int scan();
  String ssidAt(uint8_t index) const;
  int32_t rssiAt(uint8_t index) const;
  void connectTo(const char *ssid, const char *password);
  void disconnect();
  String ip() const;
  int32_t rssi() const;

private:
  bool _enabled = false;
  unsigned long _lastUpdate = 0;
};

extern WifiManager wifiManager;

