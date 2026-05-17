#include "oled_ui.h"
#include "battery.h"
#include "wifi_manager.h"
#include "sensors.h"
#include "motors.h"
#include "algorithms.h"
#include "logger.h"
#include "pid_controller.h"

OledUi oledUi;
extern PidController pid;

static const char *const MAIN_ITEMS[] = {
  "Start / Stop", "Emergency Stop", "Motor Settings", "PID Settings",
  "Sensor Settings", "Algorithm Select", "Junction Memory", "Route Memory",
  "Debug Mode", "Battery Status", "Wi-Fi Settings", "Device PIN",
  "Screen & Lock", "Reset Options", "About Device"
};
static const char *const MOTOR_ITEMS[] = {"Manual motor test", "Left motor speed", "Right motor speed", "Base speed", "Turn speed"};
static const char *const PID_ITEMS[] = {"Kp", "Ki", "Kd", "Save PID", "Reset PID"};
static const char *const SENSOR_ITEMS[] = {"Live sensor values", "Calibrate black", "Calibrate white", "Sensor threshold"};
static const char *const ALG_ITEMS[] = {"Normal line follow", "Left hand rule", "Right hand rule", "Path memory", "Flood fill", "A*", "Dijkstra"};
static const char *const DEBUG_ITEMS[] = {"Sensor live values", "Motor output", "Error value", "PID output", "Run logs"};
static const char *const WIFI_ITEMS[] = {"Wi-Fi on/off", "Scan networks", "Select network", "Password keypad", "Connect", "Disconnect", "IP / Signal"};
static const char *const PIN_ITEMS[] = {"Setup PIN safety?", "Set 4-digit PIN", "Change PIN", "Remove PIN"};
static const char *const SCREEN_ITEMS[] = {"Screen sleep", "Auto lock"};
static const char *const RESET_ITEMS[] = {"Reset robot", "Factory reset", "Pairing reset"};
static const char KEYMAP[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789.-_@#* ";
static const uint8_t SLEEP_SECONDS[] = {5, 10, 15, 20, 25, 30, 0};
static const uint8_t LOCK_SECONDS[] = {10, 20, 30};

void OledUi::begin(RobotSettings *settings, SystemState *state) {
  _settings = settings;
  _state = state;
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!_display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    _display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  }
  _display.clearDisplay();
  _display.setTextColor(SSD1306_WHITE);
  _screen = state->askPinSetup ? UiScreen::DevicePin : UiScreen::MainMenu;
  _lastActivity = millis();
}

void OledUi::update(JoyEvent event) {
  if (event != JoyEvent::None) touch();
  if (_state->locked && (event == JoyEvent::Press || event == JoyEvent::LongPress)) {
    _state->locked = false;
    _pinUnlockMode = true;
    strcpy(_pinEdit, "0000");
    _pinPos = 0;
    _screen = UiScreen::PinEntry;
    return;
  }
  if (_state->locked) return;

  switch (_screen) {
    case UiScreen::MainMenu: handleMain(event); break;
    case UiScreen::MotorMenu: handleMotor(event); break;
    case UiScreen::PidMenu: handlePid(event); break;
    case UiScreen::SensorMenu: handleSensor(event); break;
    case UiScreen::AlgorithmMenu: handleAlgorithm(event); break;
    case UiScreen::RouteMemory: handleRouteMemory(event); break;
    case UiScreen::DebugMenu:
      if (event == JoyEvent::LongPress) backToMain(); else handleValueMenu(event, 5);
      break;
    case UiScreen::WifiMenu: handleWifi(event); break;
    case UiScreen::WifiScan: handleWifiScan(event); break;
    case UiScreen::WifiKeypad: handleKeypad(event); break;
    case UiScreen::DevicePin: handlePin(event); break;
    case UiScreen::PinEntry: handlePinEntry(event); break;
    case UiScreen::ScreenLock: handleScreenLock(event); break;
    case UiScreen::ResetOptions: handleReset(event); break;
    case UiScreen::Confirm: handleConfirm(event); break;
    default:
      if (event == JoyEvent::LongPress || event == JoyEvent::Press) backToMain();
      break;
  }

  uint8_t lockSecs = LOCK_SECONDS[_settings->autoLockIndex];
  if (_settings->pinEnabled && millis() - _lastActivity > lockSecs * 1000UL) _state->locked = true;
}

void OledUi::render(bool force) {
  if (!force && millis() - _lastRender < UI_PERIOD_MS) return;
  _lastRender = millis();
  if (isSleeping()) {
    _display.clearDisplay();
    _display.display();
    return;
  }
  _display.clearDisplay();
  _display.setTextSize(1);
  drawStatusBar();
  if (_state->locked) {
    _display.setCursor(20, 28);
    _display.print("LOCKED");
    _display.setCursor(8, 44);
    _display.print("Press for PIN");
    _display.display();
    return;
  }
  switch (_screen) {
    case UiScreen::MainMenu: drawMain(); break;
    case UiScreen::MotorMenu: drawMotor(); break;
    case UiScreen::PidMenu: drawPid(); break;
    case UiScreen::SensorMenu: drawSensor(); break;
    case UiScreen::AlgorithmMenu: drawAlgorithm(); break;
    case UiScreen::JunctionMemory:
      _display.setCursor(0, 14); _display.print("Junction count");
      _display.setCursor(0, 30); _display.print(algorithms.junctionCount());
      break;
    case UiScreen::RouteMemory:
      _display.setCursor(0, 14); _display.print("Route:");
      _display.setCursor(0, 28); _display.print(algorithms.route().substring(0, 21));
      _display.setCursor(0, 42);
      _display.print(algorithms.replayActive() ? "Replay " : "Replay off ");
      _display.print(algorithms.replayIndex());
      _display.setCursor(0, 54); _display.print("R play L stop P clr");
      break;
    case UiScreen::DebugMenu: drawDebug(); break;
    case UiScreen::BatteryStatus:
      _display.setCursor(0, 14); _display.printf("Voltage: %.2fV", battery.voltage());
      _display.setCursor(0, 28); _display.printf("Percent: %u%%", battery.percent());
      _display.setCursor(0, 42); _display.print("Status: "); _display.print(battery.status());
      break;
    case UiScreen::WifiMenu: drawWifi(); break;
    case UiScreen::WifiScan: drawWifiScan(); break;
    case UiScreen::WifiKeypad: drawKeypad(); break;
    case UiScreen::DevicePin: drawPin(); break;
    case UiScreen::PinEntry: drawPinEntry(); break;
    case UiScreen::ScreenLock: drawScreenLock(); break;
    case UiScreen::ResetOptions: drawReset(); break;
    case UiScreen::Confirm: drawConfirm(); break;
    case UiScreen::About: drawAbout(); break;
    case UiScreen::Message:
      _display.setCursor(0, 20); _display.print(_message1);
      _display.setCursor(0, 36); _display.print(_message2);
      break;
    default: break;
  }
  _display.display();
}

void OledUi::showMessage(const String &line1, const String &line2) {
  _message1 = line1;
  _message2 = line2;
  _screen = UiScreen::Message;
  render(true);
}

void OledUi::showJoystickDiagnostics() {
  _display.clearDisplay();
  _display.setTextColor(SSD1306_WHITE);
  _display.setTextSize(1);
  _display.setCursor(0, 0);
  _display.print("JOYSTICK DEBUG");
  _display.setCursor(0, 14);
  _display.print("X GPIO ");
  _display.print(JOY_X_PIN);
  _display.print(": ");
  _display.print(joystick.rawX());
  _display.setCursor(0, 26);
  _display.print("Y GPIO ");
  _display.print(JOY_Y_PIN);
  _display.print(": ");
  _display.print(joystick.rawY());
  _display.setCursor(0, 38);
  _display.print("SW GPIO ");
  _display.print(JOY_BTN_PIN);
  _display.print(": ");
  _display.print(joystick.pressed() ? "LOW" : "HIGH");
  _display.setCursor(0, 52);
  _display.print("DIR: ");
  switch (joystick.lastAxisEvent()) {
    case JoyEvent::Up: _display.print("UP"); break;
    case JoyEvent::Down: _display.print("DOWN"); break;
    case JoyEvent::Left: _display.print("LEFT"); break;
    case JoyEvent::Right: _display.print("RIGHT"); break;
    default: _display.print("CENTER"); break;
  }
  _display.display();
}

bool OledUi::isSleeping() const {
  uint8_t sec = SLEEP_SECONDS[_settings->screenSleepIndex];
  return sec > 0 && millis() - _lastActivity > sec * 1000UL;
}

void OledUi::drawStatusBar() {
  _display.setCursor(0, 0);
  _display.print(_state->running ? "RUN" : "STOP");
  if (_state->safeMode) _display.print(" SAFE");
  _display.setCursor(64, 0);
  _display.print(wifiManager.connected() ? "WiFi" : "----");
  _display.setCursor(104, 0);
  if (_settings->pinEnabled) _display.print(_state->locked ? "L" : "U");
  _display.setCursor(112, 0);
  _display.print(BATTERY_SENSE_ENABLED ? String(battery.percent()) + "%" : "USB");
  _display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
}

void OledUi::drawMenu(const char *title, const char *const *items, uint8_t count, uint8_t selected, uint8_t top) {
  _display.setCursor(0, 12);
  _display.print(title);
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t idx = top + row;
    if (idx >= count) break;
    uint8_t y = 24 + row * 10;
    if (idx == selected) {
      _display.fillRect(0, y - 1, 128, 9, SSD1306_WHITE);
      _display.setTextColor(SSD1306_BLACK);
    }
    _display.setCursor(2, y);
    _display.print(items[idx]);
    _display.setTextColor(SSD1306_WHITE);
  }
}

void OledUi::drawMain() { drawMenu("Line Follower OS", MAIN_ITEMS, 15, _selected, _top); }

void OledUi::drawMotor() {
  drawMenu("Motor Settings", MOTOR_ITEMS, 5, _selected, _top);
  _display.setCursor(86, 24);
  if (_selected == 0) _display.print(_state->manualMotorTest ? "ON" : "OFF");
  if (_selected == 1) _display.print(_settings->motors.leftSpeed);
  if (_selected == 2) _display.print(_settings->motors.rightSpeed);
  if (_selected == 3) _display.print(_settings->motors.baseSpeed);
  if (_selected == 4) _display.print(_settings->motors.turnSpeed);
}

void OledUi::drawPid() {
  drawMenu("PID Settings", PID_ITEMS, 5, _selected, _top);
  _display.setCursor(82, 24);
  if (_selected == 0) _display.print(_settings->kp, 2);
  if (_selected == 1) _display.print(_settings->ki, 2);
  if (_selected == 2) _display.print(_settings->kd, 2);
}

void OledUi::drawSensor() {
  if (_selected == 0) {
    _display.setCursor(0, 14); _display.print("Live sensor values");
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      _display.setCursor((i % 4) * 32, 26 + (i / 4) * 12);
      _display.print(sensors.value(i));
    }
    _display.setCursor(0, 52); _display.print("Err "); _display.print(sensors.error());
    return;
  }
  drawMenu("Sensor Settings", SENSOR_ITEMS, 4, _selected, _top);
  if (_selected == 3) {
    _display.setCursor(86, 54);
    _display.print(_settings->sensors.threshold);
  }
}

void OledUi::drawAlgorithm() {
  drawMenu("Algorithm", ALG_ITEMS, 7, _selected, _top);
  if (_selected >= 4) {
    _display.setCursor(0, 54);
    _display.print("Requires grid map");
  }
}

void OledUi::drawDebug() {
  drawMenu("Debug", DEBUG_ITEMS, 5, _selected, _top);
  _display.setCursor(0, 54);
  if (_selected == 0) _display.print("S0 "); 
  if (_selected == 0) _display.print(sensors.value(0));
  if (_selected == 1) _display.printf("L%d R%d", motors.leftOutput(), motors.rightOutput());
  if (_selected == 2) _display.printf("Err %d", sensors.error());
  if (_selected == 3) _display.printf("PID %.1f", pid.output());
  if (_selected == 4) _display.print(logger.line(0));
}

void OledUi::drawWifi() { drawMenu("Wi-Fi", WIFI_ITEMS, 7, _selected, _top); }

void OledUi::drawWifiScan() {
  _display.setCursor(0, 12); _display.print("Networks");
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t idx = _top + row;
    if (idx >= _scanCount) break;
    uint8_t y = 24 + row * 10;
    if (idx == _selected) {
      _display.fillRect(0, y - 1, 128, 9, SSD1306_WHITE);
      _display.setTextColor(SSD1306_BLACK);
    }
    _display.setCursor(2, y);
    _display.print(wifiManager.ssidAt(idx).substring(0, 14));
    _display.setCursor(96, y);
    _display.print(wifiManager.rssiAt(idx));
    _display.setTextColor(SSD1306_WHITE);
  }
}

void OledUi::drawKeypad() {
  _display.setCursor(0, 12); _display.print("Password:");
  _display.setCursor(0, 22); _display.print(_state->wifiPasswordEdit.substring(max(0, (int)_state->wifiPasswordEdit.length() - 18)));
  for (uint8_t y = 0; y < 3; y++) {
    for (uint8_t x = 0; x < 10; x++) {
      uint8_t idx = y * 10 + x;
      if (idx >= strlen(KEYMAP)) break;
      uint8_t px = x * 12;
      uint8_t py = 36 + y * 9;
      if (x == _keyX && y == _keyY) _display.drawRect(px, py - 1, 11, 9, SSD1306_WHITE);
      _display.setCursor(px + 2, py);
      _display.print(KEYMAP[idx]);
    }
  }
  if (_keyY == 3 && _keyX < 5) _display.drawRect(0, 55, 45, 9, SSD1306_WHITE);
  if (_keyY == 3 && _keyX >= 5) _display.drawRect(72, 55, 38, 9, SSD1306_WHITE);
  _display.setCursor(3, 56); _display.print("DEL");
  _display.setCursor(76, 56); _display.print("SAVE");
}

void OledUi::drawPin() { drawMenu("Device PIN", PIN_ITEMS, 4, _selected, _top); }

void OledUi::drawPinEntry() {
  _display.setCursor(0, 14);
  _display.print(_pinUnlockMode ? "Enter PIN" : "Set 4-digit PIN");
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t x = 28 + i * 18;
    if (i == _pinPos) _display.drawRect(x - 3, 29, 14, 15, SSD1306_WHITE);
    _display.setCursor(x, 33);
    _display.print(_pinEdit[i]);
  }
  _display.setCursor(0, 54);
  _display.print("Press save");
}

void OledUi::drawScreenLock() {
  drawMenu("Screen & Lock", SCREEN_ITEMS, 2, _selected, _top);
  _display.setCursor(86, 24);
  _display.print(_selected == 0 ? sleepLabel() : lockLabel());
}

void OledUi::drawReset() { drawMenu("Reset Options", RESET_ITEMS, 3, _selected, _top); }

void OledUi::drawConfirm() {
  _display.setCursor(0, 16); _display.print(_confirmText);
  _display.setCursor(0, 34); _display.print("Press OK");
  _display.setCursor(0, 48); _display.print("Long press back");
}

void OledUi::drawAbout() {
  _display.setCursor(0, 14); _display.print("Line Follower OS");
  _display.setCursor(0, 28); _display.print("ESP32-S3 OLED");
  _display.setCursor(0, 42); _display.print("Motors safe on boot");
  _display.setCursor(0, 54); _display.print("v1.0");
}

void OledUi::handleMain(JoyEvent event) {
  handleValueMenu(event, 15);
  if (event != JoyEvent::Press) return;
  switch (_selected) {
    case 0:
      if (_state->safeMode || _state->emergency || !sensors.valid() || !sensors.lineDetected()) {
        _state->running = false;
        showMessage("Cannot start", "Check safe/sensors");
      } else {
        _state->running = !_state->running;
        logger.log(_state->running ? "Robot started" : "Robot stopped");
      }
      break;
    case 1:
      if (_state->emergency) {
        _state->emergency = false;
        motors.clearEmergency();
        logger.log("Emergency cleared");
        showMessage("Emergency clear", "Motors still off");
      } else {
        _state->emergency = true;
        _state->running = false;
        motors.emergencyStop();
        logger.log("Emergency stop");
      }
      break;
    case 2: _screen = UiScreen::MotorMenu; _selected = _top = 0; break;
    case 3: _screen = UiScreen::PidMenu; _selected = _top = 0; break;
    case 4: _screen = UiScreen::SensorMenu; _selected = _top = 0; break;
    case 5: _screen = UiScreen::AlgorithmMenu; _selected = static_cast<uint8_t>(_settings->algorithm); _top = 0; break;
    case 6: _screen = UiScreen::JunctionMemory; break;
    case 7: _screen = UiScreen::RouteMemory; break;
    case 8: _screen = UiScreen::DebugMenu; _selected = _top = 0; break;
    case 9: _screen = UiScreen::BatteryStatus; break;
    case 10: _screen = UiScreen::WifiMenu; _selected = _top = 0; break;
    case 11: _screen = UiScreen::DevicePin; _selected = _top = 0; break;
    case 12: _screen = UiScreen::ScreenLock; _selected = _top = 0; break;
    case 13: _screen = UiScreen::ResetOptions; _selected = _top = 0; break;
    case 14: _screen = UiScreen::About; break;
  }
}

void OledUi::handleValueMenu(JoyEvent event, uint8_t count) {
  if (event == JoyEvent::Up && _selected > 0) _selected--;
  if (event == JoyEvent::Down && _selected + 1 < count) _selected++;
  if (_selected < _top) _top = _selected;
  if (_selected > _top + 3) _top = _selected - 3;
}

void OledUi::handleMotor(JoyEvent event) {
  if (event == JoyEvent::LongPress) { _state->manualMotorTest = false; motors.stop(); saveAll(); backToMain(); return; }
  handleValueMenu(event, 5);
  int delta = event == JoyEvent::Right ? 5 : event == JoyEvent::Left ? -5 : 0;
  if (_selected == 0 && event == JoyEvent::Press) _state->manualMotorTest = !_state->manualMotorTest;
  if (_selected == 1) _settings->motors.leftSpeed = constrain(_settings->motors.leftSpeed + delta, 0, 255);
  if (_selected == 2) _settings->motors.rightSpeed = constrain(_settings->motors.rightSpeed + delta, 0, 255);
  if (_selected == 3) _settings->motors.baseSpeed = constrain(_settings->motors.baseSpeed + delta, 0, 255);
  if (_selected == 4) _settings->motors.turnSpeed = constrain(_settings->motors.turnSpeed + delta, 0, 255);
}

void OledUi::handlePid(JoyEvent event) {
  if (event == JoyEvent::LongPress) { saveAll(); backToMain(); return; }
  handleValueMenu(event, 5);
  float delta = event == JoyEvent::Right ? 0.05f : event == JoyEvent::Left ? -0.05f : 0;
  if (_selected == 0) _settings->kp = max(0.0f, _settings->kp + delta);
  if (_selected == 1) _settings->ki = max(0.0f, _settings->ki + delta);
  if (_selected == 2) _settings->kd = max(0.0f, _settings->kd + delta);
  if (_selected == 3 && event == JoyEvent::Press) { settingsStore.savePid(_settings->kp, _settings->ki, _settings->kd); showMessage("PID saved"); }
  if (_selected == 4 && event == JoyEvent::Press) { _settings->kp = DEFAULT_KP; _settings->ki = DEFAULT_KI; _settings->kd = DEFAULT_KD; }
}

void OledUi::handleSensor(JoyEvent event) {
  if (event == JoyEvent::LongPress) { saveAll(); backToMain(); return; }
  handleValueMenu(event, 4);
  if (_selected == 1 && event == JoyEvent::Press) { sensors.calibrateBlack(); _settings->sensors = sensors.calibration(); showMessage("Black saved"); }
  if (_selected == 2 && event == JoyEvent::Press) { sensors.calibrateWhite(); _settings->sensors = sensors.calibration(); showMessage("White saved"); }
  int delta = event == JoyEvent::Right ? 25 : event == JoyEvent::Left ? -25 : 0;
  if (_selected == 3 && delta != 0) {
    _settings->sensors.threshold = constrain((int)_settings->sensors.threshold + delta, 0, 4095);
    sensors.setCalibration(_settings->sensors);
  }
}

void OledUi::handleAlgorithm(JoyEvent event) {
  if (event == JoyEvent::LongPress) { saveAll(); backToMain(); return; }
  handleValueMenu(event, 7);
  if (event == JoyEvent::Press) {
    _settings->algorithm = static_cast<AlgorithmMode>(_selected);
    algorithms.setMode(_settings->algorithm);
    if (_selected >= 4) showMessage("Requires grid map");
  }
}

void OledUi::handleWifi(JoyEvent event) {
  if (event == JoyEvent::LongPress) { saveAll(); backToMain(); return; }
  handleValueMenu(event, 7);
  if (event != JoyEvent::Press) return;
  if (_selected == 0) { _settings->wifiEnabled = !_settings->wifiEnabled; wifiManager.setEnabled(_settings->wifiEnabled); }
  if (_selected == 1) { _scanCount = wifiManager.scan(); _screen = UiScreen::WifiScan; _selected = _top = 0; }
  if (_selected == 2) {
    if (_scanCount <= 0) _scanCount = wifiManager.scan();
    _screen = UiScreen::WifiScan;
    _selected = _top = 0;
  }
  if (_selected == 3) { _screen = UiScreen::WifiKeypad; _keyX = _keyY = 0; _state->wifiPasswordEdit = _settings->wifiPass; }
  if (_selected == 4) { saveAll(); wifiManager.connectTo(_settings->wifiSsid, _settings->wifiPass); }
  if (_selected == 5) { wifiManager.disconnect(); saveAll(); }
  if (_selected == 6) showMessage(wifiManager.ip(), String(wifiManager.rssi()) + " dBm");
}

void OledUi::handleWifiScan(JoyEvent event) {
  if (event == JoyEvent::LongPress) { _screen = UiScreen::WifiMenu; _selected = _top = 0; return; }
  handleValueMenu(event, max(1, _scanCount));
  if (event == JoyEvent::Press && _scanCount > 0) {
    wifiManager.ssidAt(_selected).toCharArray(_settings->wifiSsid, sizeof(_settings->wifiSsid));
    _state->selectedNetwork = _selected;
    saveAll();
    showMessage("Selected SSID", _settings->wifiSsid);
  }
}

void OledUi::handleKeypad(JoyEvent event) {
  if (event == JoyEvent::LongPress) {
    _state->wifiPasswordEdit.toCharArray(_settings->wifiPass, sizeof(_settings->wifiPass));
    saveAll();
    _screen = UiScreen::WifiMenu;
    return;
  }
  if (event == JoyEvent::Left && _keyX > 0) _keyX--;
  if (event == JoyEvent::Right && _keyX < 9) _keyX++;
  if (event == JoyEvent::Up && _keyY > 0) _keyY--;
  if (event == JoyEvent::Down && _keyY < 3) _keyY++;
  if (event == JoyEvent::Press) {
    if (_keyY == 3) {
      if (_keyX < 5 && _state->wifiPasswordEdit.length() > 0) {
        _state->wifiPasswordEdit.remove(_state->wifiPasswordEdit.length() - 1);
      } else if (_keyX >= 5) {
        _state->wifiPasswordEdit.toCharArray(_settings->wifiPass, sizeof(_settings->wifiPass));
        saveAll();
        _screen = UiScreen::WifiMenu;
      }
      return;
    }
    uint8_t idx = _keyY * 10 + _keyX;
    if (idx < strlen(KEYMAP) && _state->wifiPasswordEdit.length() < 64) _state->wifiPasswordEdit += KEYMAP[idx];
  }
}

void OledUi::handlePin(JoyEvent event) {
  if (event == JoyEvent::LongPress) { saveAll(); backToMain(); return; }
  handleValueMenu(event, 4);
  if (event != JoyEvent::Press) return;
  if (_selected == 0 || _selected == 1 || _selected == 2) {
    strncpy(_pinEdit, _settings->pin, sizeof(_pinEdit));
    _pinEdit[4] = '\0';
    _pinPos = 0;
    _pinUnlockMode = false;
    _screen = UiScreen::PinEntry;
  }
  if (_selected == 3) { _settings->pinEnabled = false; showMessage("PIN removed"); }
}

void OledUi::handlePinEntry(JoyEvent event) {
  if (event == JoyEvent::LongPress) {
    if (_pinUnlockMode) _state->locked = true;
    _pinUnlockMode = false;
    _screen = UiScreen::DevicePin;
    return;
  }
  if (event == JoyEvent::Up && _pinPos > 0) _pinPos--;
  if (event == JoyEvent::Down && _pinPos < 3) _pinPos++;
  if (event == JoyEvent::Left && _pinEdit[_pinPos] > '0') _pinEdit[_pinPos]--;
  if (event == JoyEvent::Right && _pinEdit[_pinPos] < '9') _pinEdit[_pinPos]++;
  if (event == JoyEvent::Press) {
    if (_pinUnlockMode) {
      if (strncmp(_pinEdit, _settings->pin, 4) == 0) {
        _pinUnlockMode = false;
        showMessage("Unlocked");
      } else {
        _state->locked = true;
        _pinUnlockMode = false;
      }
      return;
    }
    strcpy(_settings->pin, _pinEdit);
    _settings->pinEnabled = true;
    saveAll();
    showMessage("PIN saved", _settings->pin);
  }
}

void OledUi::handleRouteMemory(JoyEvent event) {
  if (event == JoyEvent::LongPress) { backToMain(); return; }
  if (event == JoyEvent::Right) {
    algorithms.startReplay();
    showMessage("Path replay", algorithms.replayActive() ? "Enabled" : "No route saved");
  }
  if (event == JoyEvent::Left) {
    algorithms.stopReplay();
    showMessage("Path replay", "Stopped");
  }
  if (event == JoyEvent::Press) {
    algorithms.clearRoute();
    showMessage("Route cleared");
  }
}

void OledUi::handleScreenLock(JoyEvent event) {
  if (event == JoyEvent::LongPress) { saveAll(); backToMain(); return; }
  handleValueMenu(event, 2);
  int delta = event == JoyEvent::Right ? 1 : event == JoyEvent::Left ? -1 : 0;
  if (_selected == 0 && delta != 0) _settings->screenSleepIndex = constrain((int)_settings->screenSleepIndex + delta, 0, 6);
  if (_selected == 1 && delta != 0) _settings->autoLockIndex = constrain((int)_settings->autoLockIndex + delta, 0, 2);
}

void OledUi::handleReset(JoyEvent event) {
  if (event == JoyEvent::LongPress) { backToMain(); return; }
  handleValueMenu(event, 3);
  if (event == JoyEvent::Press) {
    _confirmReturn = UiScreen::ResetOptions;
    _confirmText = RESET_ITEMS[_selected];
    _screen = UiScreen::Confirm;
  }
}

void OledUi::handleConfirm(JoyEvent event) {
  if (event == JoyEvent::LongPress) { _screen = _confirmReturn; return; }
  if (event != JoyEvent::Press) return;
  if (_confirmText == "Reset robot") ESP.restart();
  if (_confirmText == "Factory reset") { settingsStore.factoryReset(); ESP.restart(); }
  if (_confirmText == "Pairing reset") { settingsStore.pairingReset(); strcpy(_settings->wifiSsid, ""); strcpy(_settings->wifiPass, ""); showMessage("Pairing reset"); }
}

void OledUi::backToMain() {
  _screen = UiScreen::MainMenu;
  _selected = 0;
  _top = 0;
}

void OledUi::touch() {
  _lastActivity = millis();
}

void OledUi::saveAll() {
  settingsStore.save(*_settings);
}

const char *OledUi::sleepLabel() const {
  static char buf[8];
  uint8_t sec = SLEEP_SECONDS[_settings->screenSleepIndex];
  if (sec == 0) return "Never";
  snprintf(buf, sizeof(buf), "%us", sec);
  return buf;
}

const char *OledUi::lockLabel() const {
  static char buf[8];
  snprintf(buf, sizeof(buf), "%us", LOCK_SECONDS[_settings->autoLockIndex]);
  return buf;
}
