#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "settings_store.h"
#include "joystick.h"

struct SystemState {
  bool running = false;
  bool safeMode = false;
  bool emergency = false;
  bool locked = false;
  bool manualMotorTest = false;
  int manualLeft = 0;
  int manualRight = 0;
  bool askPinSetup = false;
  String wifiPasswordEdit;
  uint8_t selectedNetwork = 0;
};

enum class UiScreen : uint8_t {
  Boot,
  MainMenu,
  MotorMenu,
  PidMenu,
  SensorMenu,
  AlgorithmMenu,
  JunctionMemory,
  RouteMemory,
  DebugMenu,
  BatteryStatus,
  WifiMenu,
  WifiScan,
  WifiKeypad,
  DevicePin,
  PinEntry,
  ScreenLock,
  ResetOptions,
  Confirm,
  About,
  Message
};

class OledUi {
public:
  void begin(RobotSettings *settings, SystemState *state);
  void update(JoyEvent event);
  void render(bool force = false);
  void showMessage(const String &line1, const String &line2 = "");
  void showHome();
  void showJoystickDiagnostics();
  bool isSleeping() const;

private:
  void drawStatusBar();
  void drawMenu(const char *title, const char *const *items, uint8_t count, uint8_t selected, uint8_t top);
  void drawMain();
  void drawMotor();
  void drawPid();
  void drawSensor();
  void drawAlgorithm();
  void drawDebug();
  void drawWifi();
  void drawWifiScan();
  void drawKeypad();
  void drawPin();
  void drawPinEntry();
  void drawScreenLock();
  void drawReset();
  void drawConfirm();
  void drawAbout();
  void handleMain(JoyEvent event);
  void handleValueMenu(JoyEvent event, uint8_t count);
  void handleMotor(JoyEvent event);
  void handlePid(JoyEvent event);
  void handleSensor(JoyEvent event);
  void handleAlgorithm(JoyEvent event);
  void handleWifi(JoyEvent event);
  void handleWifiScan(JoyEvent event);
  void handleKeypad(JoyEvent event);
  void handlePin(JoyEvent event);
  void handlePinEntry(JoyEvent event);
  void handleRouteMemory(JoyEvent event);
  void handleScreenLock(JoyEvent event);
  void handleReset(JoyEvent event);
  void handleConfirm(JoyEvent event);
  void backToMain();
  void touch();
  void saveAll();
  const char *sleepLabel() const;
  const char *lockLabel() const;

  Adafruit_SH1106G _display{OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN};
  RobotSettings *_settings = nullptr;
  SystemState *_state = nullptr;
  UiScreen _screen = UiScreen::Boot;
  UiScreen _confirmReturn = UiScreen::MainMenu;
  uint8_t _selected = 0;
  uint8_t _top = 0;
  uint8_t _keyX = 0;
  uint8_t _keyY = 0;
  uint8_t _pinPos = 0;
  char _pinEdit[5] = "0000";
  bool _pinUnlockMode = false;
  int _scanCount = 0;
  String _confirmText;
  String _message1;
  String _message2;
  unsigned long _lastRender = 0;
  unsigned long _lastActivity = 0;
};

extern OledUi oledUi;
