#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "motors.h"
#include "sensors.h"
#include "algorithms.h"

struct RobotSettings {
  MotorSettings motors;
  float kp = DEFAULT_KP;
  float ki = DEFAULT_KI;
  float kd = DEFAULT_KD;
  SensorCalibration sensors;
  AlgorithmMode algorithm = AlgorithmMode::NormalFollow;
  uint8_t screenSleepIndex = 6; // Never
  uint8_t autoLockIndex = 2;    // 30 sec
  bool pinEnabled = false;
  char pin[5] = "0000";
  bool wifiEnabled = false;
  char wifiSsid[33] = "";
  char wifiPass[65] = "";
};

class SettingsStore {
public:
  void begin();
  void load(RobotSettings &settings);
  void save(const RobotSettings &settings);
  void savePid(float kp, float ki, float kd);
  void factoryReset();
  void pairingReset();

private:
  Preferences _prefs;
};

extern SettingsStore settingsStore;

