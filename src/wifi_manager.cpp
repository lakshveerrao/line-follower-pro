#include "wifi_manager.h"
#include "config.h"

WifiManager wifiManager;

void WifiManager::begin(bool enabled, const char *ssid, const char *password) {
  _enabled = enabled;
  WiFi.mode(_enabled ? WIFI_STA : WIFI_OFF);
  if (_enabled && strlen(ssid) > 0) {
    WiFi.begin(ssid, password);
  }
}

void WifiManager::update() {
  if (millis() - _lastUpdate < WIFI_PERIOD_MS) return;
  _lastUpdate = millis();
}

void WifiManager::setEnabled(bool enabled) {
  _enabled = enabled;
  if (!_enabled) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  } else {
    WiFi.mode(WIFI_STA);
  }
}

bool WifiManager::enabled() const {
  return _enabled;
}

bool WifiManager::connected() const {
  return _enabled && WiFi.status() == WL_CONNECTED;
}

int WifiManager::scan() {
  if (!_enabled) setEnabled(true);
  return WiFi.scanNetworks();
}

String WifiManager::ssidAt(uint8_t index) const {
  return WiFi.SSID(index);
}

int32_t WifiManager::rssiAt(uint8_t index) const {
  return WiFi.RSSI(index);
}

void WifiManager::connectTo(const char *ssid, const char *password) {
  if (!_enabled) setEnabled(true);
  WiFi.begin(ssid, password);
}

void WifiManager::disconnect() {
  WiFi.disconnect();
}

String WifiManager::ip() const {
  return connected() ? WiFi.localIP().toString() : "0.0.0.0";
}

int32_t WifiManager::rssi() const {
  return connected() ? WiFi.RSSI() : 0;
}

