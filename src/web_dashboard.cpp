#include "web_dashboard.h"
#include <WiFi.h>
#include "algorithms.h"
#include "battery.h"
#include "logger.h"
#include "motors.h"
#include "sensors.h"
#include "settings_store.h"
#include "wifi_manager.h"

WebDashboard webDashboard;

void WebDashboard::begin(RobotSettings *settings, SystemState *state, PidController *pid) {
  _settings = settings;
  _state = state;
  _pid = pid;
  if (!WEB_DASHBOARD_ENABLED || _started) return;

  WiFi.softAP(WEB_AP_SSID, WEB_AP_PASS);
  _server.on("/", HTTP_GET, [this]() { handleRoot(); });
  _server.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
  _server.on("/api/control", HTTP_POST, [this]() { handleControl(); });
  _server.on("/api/settings", HTTP_POST, [this]() { handleSettings(); });
  _server.begin();
  _started = true;
  logger.log(String("Web AP ") + WEB_AP_SSID + " " + WiFi.softAPIP().toString());
}

void WebDashboard::update() {
  if (_started) _server.handleClient();
}

void WebDashboard::handleRoot() {
  _server.send(200, "text/html", htmlPage());
}

void WebDashboard::handleStatus() {
  String sensorJson = "[";
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (i) sensorJson += ",";
    sensorJson += String(sensors.value(i));
  }
  sensorJson += "]";

  String json = "{";
  json += "\"running\":" + String(_state->running ? "true" : "false") + ",";
  json += "\"emergency\":" + String(_state->emergency ? "true" : "false") + ",";
  json += "\"safeMode\":" + String(_state->safeMode ? "true" : "false") + ",";
  json += "\"manualMotorTest\":" + String(_state->manualMotorTest ? "true" : "false") + ",";
  json += "\"manualLeft\":" + String(_state->manualLeft) + ",";
  json += "\"manualRight\":" + String(_state->manualRight) + ",";
  json += "\"autoEnabled\":" + String(AUTONOMOUS_LINE_FOLLOW_ENABLED ? "true" : "false") + ",";
  json += "\"autoMax\":" + String(AUTONOMOUS_MAX_PWM) + ",";
  json += "\"manualMax\":" + String(MANUAL_TEST_MAX_PWM) + ",";
  json += "\"leftOut\":" + String(motors.leftOutput()) + ",";
  json += "\"rightOut\":" + String(motors.rightOutput()) + ",";
  json += "\"error\":" + String(sensors.error()) + ",";
  json += "\"pidOut\":" + String(_pid->output(), 2) + ",";
  json += "\"threshold\":" + String(_settings->sensors.threshold) + ",";
  json += "\"kp\":" + String(_settings->kp, 3) + ",";
  json += "\"ki\":" + String(_settings->ki, 3) + ",";
  json += "\"kd\":" + String(_settings->kd, 3) + ",";
  json += "\"baseSpeed\":" + String(_settings->motors.baseSpeed) + ",";
  json += "\"turnSpeed\":" + String(_settings->motors.turnSpeed) + ",";
  json += "\"algorithm\":\"" + String(algorithms.modeName()) + "\",";
  json += "\"junctions\":" + String(algorithms.junctionCount()) + ",";
  json += "\"route\":\"" + algorithms.route() + "\",";
  json += "\"battery\":\"" + String(battery.status()) + "\",";
  json += "\"apIp\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"staIp\":\"" + wifiManager.ip() + "\",";
  json += "\"sensors\":" + sensorJson;
  json += "}";
  sendJson(json);
}

void WebDashboard::handleControl() {
  String action = _server.arg("action");
  if (action == "start") {
    if (AUTONOMOUS_LINE_FOLLOW_ENABLED && !_state->safeMode && !_state->emergency && sensors.valid() && sensors.lineDetected()) {
      _state->running = true;
      _state->manualMotorTest = false;
      _state->manualLeft = 0;
      _state->manualRight = 0;
      logger.log("Web start");
    }
  } else if (action == "stop") {
    _state->running = false;
    _state->manualMotorTest = false;
    _state->manualLeft = 0;
    _state->manualRight = 0;
    motors.stop();
    logger.log("Web stop");
  } else if (action == "estop") {
    _state->running = false;
    _state->manualMotorTest = false;
    _state->manualLeft = 0;
    _state->manualRight = 0;
    _state->emergency = true;
    motors.emergencyStop();
    logger.log("Web emergency stop");
  } else if (action == "clear_estop") {
    _state->emergency = false;
    motors.clearEmergency();
    motors.stop();
    logger.log("Web emergency clear");
  } else if (action == "manual_on") {
    _state->running = false;
    _state->manualMotorTest = true;
    _state->manualLeft = constrain(_settings->motors.leftSpeed, 0, MANUAL_TEST_MAX_PWM);
    _state->manualRight = constrain(_settings->motors.rightSpeed, 0, MANUAL_TEST_MAX_PWM);
  } else if (action == "manual_off") {
    _state->manualMotorTest = false;
    _state->manualLeft = 0;
    _state->manualRight = 0;
    motors.stop();
  } else if (action == "manual_stop") {
    _state->running = false;
    _state->manualMotorTest = true;
    _state->manualLeft = 0;
    _state->manualRight = 0;
    motors.stop();
  } else if (action == "manual_forward") {
    _state->running = false;
    _state->manualMotorTest = true;
    _state->manualLeft = MANUAL_TEST_MAX_PWM;
    _state->manualRight = MANUAL_TEST_MAX_PWM;
  } else if (action == "manual_back") {
    _state->running = false;
    _state->manualMotorTest = true;
    _state->manualLeft = -MANUAL_TEST_MAX_PWM;
    _state->manualRight = -MANUAL_TEST_MAX_PWM;
  } else if (action == "manual_left") {
    _state->running = false;
    _state->manualMotorTest = true;
    _state->manualLeft = -MANUAL_TEST_MAX_PWM;
    _state->manualRight = MANUAL_TEST_MAX_PWM;
  } else if (action == "manual_right") {
    _state->running = false;
    _state->manualMotorTest = true;
    _state->manualLeft = MANUAL_TEST_MAX_PWM;
    _state->manualRight = -MANUAL_TEST_MAX_PWM;
  } else if (action == "cal_black") {
    sensors.calibrateBlack();
    _settings->sensors = sensors.calibration();
    settingsStore.save(*_settings);
  } else if (action == "cal_white") {
    sensors.calibrateWhite();
    _settings->sensors = sensors.calibration();
    settingsStore.save(*_settings);
  } else if (action == "clear_route") {
    algorithms.clearRoute();
  } else if (action == "factory_reset") {
    settingsStore.factoryReset();
    ESP.restart();
  }
  handleStatus();
}

void WebDashboard::handleSettings() {
  if (_server.hasArg("kp")) _settings->kp = max(0.0f, _server.arg("kp").toFloat());
  if (_server.hasArg("ki")) _settings->ki = max(0.0f, _server.arg("ki").toFloat());
  if (_server.hasArg("kd")) _settings->kd = max(0.0f, _server.arg("kd").toFloat());
  if (_server.hasArg("base")) _settings->motors.baseSpeed = constrain(_server.arg("base").toInt(), 0, AUTONOMOUS_MAX_PWM);
  if (_server.hasArg("turn")) _settings->motors.turnSpeed = constrain(_server.arg("turn").toInt(), 0, AUTONOMOUS_MAX_PWM);
  if (_server.hasArg("left")) _settings->motors.leftSpeed = constrain(_server.arg("left").toInt(), 0, MANUAL_TEST_MAX_PWM);
  if (_server.hasArg("right")) _settings->motors.rightSpeed = constrain(_server.arg("right").toInt(), 0, MANUAL_TEST_MAX_PWM);
  if (_server.hasArg("threshold")) {
    _settings->sensors.threshold = constrain(_server.arg("threshold").toInt(), 0, 4095);
    sensors.setCalibration(_settings->sensors);
  }
  if (_server.hasArg("algorithm")) {
    uint8_t alg = constrain(_server.arg("algorithm").toInt(), 0, 6);
    _settings->algorithm = static_cast<AlgorithmMode>(alg);
    algorithms.setMode(_settings->algorithm);
  }
  _pid->setTunings(_settings->kp, _settings->ki, _settings->kd);
  settingsStore.save(*_settings);
  handleStatus();
}

void WebDashboard::sendJson(const String &json) {
  _server.sendHeader("Cache-Control", "no-store");
  _server.send(200, "application/json", json);
}

String WebDashboard::htmlPage() const {
  return R"rawliteral(
<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>Line Follower OS</title>
<style>
body{font-family:Arial,sans-serif;margin:0;background:#111;color:#eee}header{padding:14px;background:#20242b;position:sticky;top:0}
main{padding:12px;display:grid;gap:12px}.panel{border:1px solid #333;padding:12px;border-radius:6px;background:#181b20}
button,input,select{font-size:16px;margin:4px;padding:8px;border-radius:5px;border:1px solid #444}button{background:#2b6cb0;color:white}
.danger{background:#b3261e}.ok{background:#2f855a}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px}
.sensors{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}.bar{background:#333;height:18px}.fill{background:#48bb78;height:18px}
label{display:block;margin-top:8px}.small{color:#aaa;font-size:13px}
</style></head><body><header><b>Line Follower OS</b><div class=small id=net></div></header><main>
<section class=panel><h3>Control</h3>
<button class=ok onclick="cmd('start')">Start</button><button onclick="cmd('stop')">Stop</button>
<button class=danger onclick="cmd('estop')">Emergency Stop</button><button onclick="cmd('clear_estop')">Clear E-Stop</button>
<button onclick="cmd('manual_on')">Manual Motor Test</button><button onclick="cmd('manual_off')">Manual Off</button>
<div class=grid>
<button onclick="cmd('manual_forward')">Forward</button><button onclick="cmd('manual_back')">Back</button>
<button onclick="cmd('manual_left')">Left</button><button onclick="cmd('manual_right')">Right</button>
<button class=danger onclick="cmd('manual_stop')">Motor Stop</button>
</div></section>
<section class=panel><h3>Status</h3><div id=status></div></section>
<section class=panel><h3>Sensors</h3><div class=sensors id=sensors></div>
<button onclick="cmd('cal_white')">Calibrate White</button><button onclick="cmd('cal_black')">Calibrate Black</button></section>
<section class=panel><h3>Settings</h3><div class=grid>
<label>Kp<input id=kp type=number step=.05></label><label>Ki<input id=ki type=number step=.01></label><label>Kd<input id=kd type=number step=.05></label>
<label>Base<input id=base type=number></label><label>Turn<input id=turn type=number></label><label>Threshold<input id=threshold type=number></label>
<label>Algorithm<select id=algorithm><option value=0>Normal</option><option value=1>Left hand</option><option value=2>Right hand</option><option value=3>Path memory</option><option value=4>Flood fill</option><option value=5>A*</option><option value=6>Dijkstra</option></select></label>
</div><button onclick="save()">Save Settings</button></section>
<section class=panel><h3>Route</h3><div id=route></div><button onclick="cmd('clear_route')">Clear Route</button></section>
</main><script>
let first=true;
async function api(path,opts){let r=await fetch(path,opts);return r.json()}
async function cmd(a){await api('/api/control?action='+a,{method:'POST'});refresh()}
async function save(){let q=new URLSearchParams({kp:kp.value,ki:ki.value,kd:kd.value,base:base.value,turn:turn.value,threshold:threshold.value,algorithm:algorithm.value});await api('/api/settings?'+q,{method:'POST'});refresh()}
function setInputs(d){if(!first)return;kp.value=d.kp;ki.value=d.ki;kd.value=d.kd;base.value=d.baseSpeed;turn.value=d.turnSpeed;threshold.value=d.threshold;first=false}
async function refresh(){let d=await api('/api/status');setInputs(d);net.textContent='AP '+d.apIp+' | STA '+d.staIp;
status.innerHTML=`Run: ${d.running} | Manual: ${d.manualMotorTest} | Safe: ${d.safeMode} | EStop: ${d.emergency}<br>Motor L/R: ${d.leftOut}/${d.rightOut} | Manual target: ${d.manualLeft}/${d.manualRight}<br>Auto: ${d.autoEnabled} max ${d.autoMax} | Manual max ${d.manualMax}<br>Error: ${d.error} PID: ${d.pidOut}<br>Algorithm: ${d.algorithm} Battery: ${d.battery}`;
route.textContent=`Junctions ${d.junctions} | ${d.route}`;
sensors.innerHTML=d.sensors.map((v,i)=>`<div>S${i+1} ${v}<div class=bar><div class=fill style="width:${Math.min(100,v/40.95)}%"></div></div></div>`).join('')}
setInterval(refresh,500);refresh();
</script></body></html>
)rawliteral";
}
