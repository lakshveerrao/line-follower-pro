#include "settings_store.h"

SettingsStore settingsStore;

void SettingsStore::begin() {
  _prefs.begin("lf-os", false);
}

void SettingsStore::load(RobotSettings &s) {
  s.motors.leftSpeed = _prefs.getInt("leftSpd", DEFAULT_LEFT_SPEED);
  s.motors.rightSpeed = _prefs.getInt("rightSpd", DEFAULT_RIGHT_SPEED);
  s.motors.baseSpeed = _prefs.getInt("baseSpd", DEFAULT_BASE_SPEED);
  s.motors.turnSpeed = _prefs.getInt("turnSpd", DEFAULT_TURN_SPEED);
  s.kp = _prefs.getFloat("kp", DEFAULT_KP);
  s.ki = _prefs.getFloat("ki", DEFAULT_KI);
  s.kd = _prefs.getFloat("kd", DEFAULT_KD);
  s.sensors.threshold = _prefs.getUShort("thr", DEFAULT_SENSOR_THRESHOLD);
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    s.sensors.black[i] = _prefs.getUShort(("blk" + String(i)).c_str(), 3500);
    s.sensors.white[i] = _prefs.getUShort(("wht" + String(i)).c_str(), 500);
  }
  s.algorithm = static_cast<AlgorithmMode>(_prefs.getUChar("alg", 0));
  s.screenSleepIndex = _prefs.getUChar("sleep", 6);
  s.autoLockIndex = _prefs.getUChar("lock", 2);
  s.pinEnabled = _prefs.getBool("pinEn", false);
  _prefs.getString("pin", s.pin, sizeof(s.pin));
  s.wifiEnabled = _prefs.getBool("wifiEn", false);
  _prefs.getString("ssid", s.wifiSsid, sizeof(s.wifiSsid));
  _prefs.getString("pass", s.wifiPass, sizeof(s.wifiPass));
}

void SettingsStore::save(const RobotSettings &s) {
  _prefs.putInt("leftSpd", s.motors.leftSpeed);
  _prefs.putInt("rightSpd", s.motors.rightSpeed);
  _prefs.putInt("baseSpd", s.motors.baseSpeed);
  _prefs.putInt("turnSpd", s.motors.turnSpeed);
  _prefs.putFloat("kp", s.kp);
  _prefs.putFloat("ki", s.ki);
  _prefs.putFloat("kd", s.kd);
  _prefs.putUShort("thr", s.sensors.threshold);
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    _prefs.putUShort(("blk" + String(i)).c_str(), s.sensors.black[i]);
    _prefs.putUShort(("wht" + String(i)).c_str(), s.sensors.white[i]);
  }
  _prefs.putUChar("alg", static_cast<uint8_t>(s.algorithm));
  _prefs.putUChar("sleep", s.screenSleepIndex);
  _prefs.putUChar("lock", s.autoLockIndex);
  _prefs.putBool("pinEn", s.pinEnabled);
  _prefs.putString("pin", s.pin);
  _prefs.putBool("wifiEn", s.wifiEnabled);
  _prefs.putString("ssid", s.wifiSsid);
  _prefs.putString("pass", s.wifiPass);
}

void SettingsStore::savePid(float kp, float ki, float kd) {
  _prefs.putFloat("kp", kp);
  _prefs.putFloat("ki", ki);
  _prefs.putFloat("kd", kd);
}

void SettingsStore::factoryReset() {
  _prefs.clear();
}

void SettingsStore::pairingReset() {
  _prefs.remove("ssid");
  _prefs.remove("pass");
  _prefs.putBool("wifiEn", false);
}

