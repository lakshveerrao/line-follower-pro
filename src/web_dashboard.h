#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include "settings_store.h"
#include "oled_ui.h"
#include "pid_controller.h"

class WebDashboard {
public:
  void begin(RobotSettings *settings, SystemState *state, PidController *pid);
  void update();

private:
  void handleRoot();
  void handleStatus();
  void handleControl();
  void handleSettings();
  void sendJson(const String &json);
  String htmlPage() const;

  WebServer _server{80};
  RobotSettings *_settings = nullptr;
  SystemState *_state = nullptr;
  PidController *_pid = nullptr;
  bool _started = false;
};

extern WebDashboard webDashboard;
