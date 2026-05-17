#include <Arduino.h>
#include "config.h"
#include "logger.h"
#include "settings_store.h"
#include "joystick.h"
#include "motors.h"
#include "sensors.h"
#include "pid_controller.h"
#include "algorithms.h"
#include "battery.h"
#include "wifi_manager.h"
#include "oled_ui.h"
#include "web_dashboard.h"

RobotSettings settings;
SystemState systemState;
PidController pid;

static unsigned long lastControl = 0;
static unsigned long lastJunction = 0;
static unsigned long lastLineSeen = 0;
static unsigned long lastLineDebug = 0;
static unsigned long junctionTurnUntil = 0;
static int lastError = 0;
static MoveChoice activeJunctionMove = MoveChoice::None;
static bool placeholderShown = false;

static bool isPlaceholderAlgorithm() {
  return settings.algorithm == AlgorithmMode::FloodFill ||
         settings.algorithm == AlgorithmMode::AStar ||
         settings.algorithm == AlgorithmMode::Dijkstra;
}

static void enterSafeMode(const char *reason) {
  systemState.safeMode = true;
  systemState.running = false;
  motors.stop();
  logger.log(String("Safe mode: ") + reason);
}

static MoveChoice applyJunctionDecision() {
  if (!sensors.junction() || millis() - lastJunction < 350) return MoveChoice::None;
  lastJunction = millis();

  bool left = false;
  bool straight = false;
  bool right = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    bool active = sensors.lineValue(i) >= settings.sensors.threshold;
    if (i < SENSOR_COUNT / 3) left |= active;
    else if (i < (SENSOR_COUNT * 2) / 3) straight |= active;
    else right |= active;
  }
  MoveChoice move = algorithms.onJunction(left, straight, right);

  logger.log(String("Junction ") + static_cast<char>(move));
  return move;
}

static bool applyJunctionTurnOverride(unsigned long now) {
  if (now >= junctionTurnUntil) return false;
  int turn = constrain(settings.motors.turnSpeed, 45, MOTOR_MAX_PWM);
  if (activeJunctionMove == MoveChoice::Left) {
    motors.set(-turn, turn);
    return true;
  }
  if (activeJunctionMove == MoveChoice::Right) {
    motors.set(turn, -turn);
    return true;
  }
  if (activeJunctionMove == MoveChoice::Back) {
    motors.set(turn, -turn);
    return true;
  }
  return false;
}

static void recoverLostLine(unsigned long now) {
  int search = lastError < 0 ? -LINE_SEARCH_PWM : LINE_SEARCH_PWM;
  if (now - lastLineSeen < LOST_LINE_RECOVERY_MS) {
    motors.set(search, -search);
    return;
  }
  enterSafeMode("line lost");
}

static void printLineDebug(unsigned long now) {
  if (!LINE_SERIAL_DEBUG || now - lastLineDebug < LINE_DEBUG_MS) return;
  lastLineDebug = now;

  Serial.print("LF raw=");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (i) Serial.print(',');
    Serial.print(sensors.value(i));
  }
  Serial.print(" norm=");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (i) Serial.print(',');
    Serial.print(sensors.lineValue(i));
  }
  Serial.print(" act=");
  Serial.print(sensors.activeCount());
  Serial.print(" pos=");
  Serial.print(sensors.position());
  Serial.print(" err=");
  Serial.print(sensors.error());
  Serial.print(" pid=");
  Serial.print(pid.output(), 2);
  Serial.print(" motor=");
  Serial.print(motors.leftOutput());
  Serial.print('/');
  Serial.print(motors.rightOutput());
  Serial.print(" run=");
  Serial.print(systemState.running ? 1 : 0);
  Serial.print(" safe=");
  Serial.print(systemState.safeMode ? 1 : 0);
  Serial.print(" estop=");
  Serial.println(systemState.emergency ? 1 : 0);
}

static void runLineFollower() {
  if (!systemState.running || systemState.safeMode || systemState.emergency || motors.isEmergencyStopped()) {
    motors.stop();
    return;
  }

  if (!sensors.valid()) {
    enterSafeMode("sensor invalid");
    return;
  }

  if (isPlaceholderAlgorithm()) {
    motors.stop();
    systemState.running = false;
    if (!placeholderShown) {
      oledUi.showMessage("Requires grid map", algorithms.modeName());
      placeholderShown = true;
    }
    return;
  }

  placeholderShown = false;
  unsigned long now = millis();
  if (now - lastControl < CONTROL_PERIOD_MS) return;
  float dt = (now - lastControl) / 1000.0f;
  lastControl = now;
  printLineDebug(now);

  if (applyJunctionTurnOverride(now)) return;

  if (!sensors.lineDetected()) {
    recoverLostLine(now);
    return;
  }

  lastLineSeen = now;

  MoveChoice move = applyJunctionDecision();
  if (move == MoveChoice::Left || move == MoveChoice::Right || move == MoveChoice::Back) {
    activeJunctionMove = move;
    junctionTurnUntil = now + (move == MoveChoice::Back ? 360UL : 180UL);
    return;
  }

  pid.setTunings(settings.kp, settings.ki, settings.kd);
  lastError = sensors.error();
  float errorNorm = constrain(lastError / (((SENSOR_COUNT - 1) * 1000.0f) / 2.0f), -1.0f, 1.0f);
  float correction = pid.compute(errorNorm, dt) * PID_OUTPUT_SCALE;
  int left = settings.motors.baseSpeed + static_cast<int>(correction);
  int right = settings.motors.baseSpeed - static_cast<int>(correction);

  left = constrain(left, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  right = constrain(right, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  motors.set(left, right);
}

static void runManualMotorTest() {
  if (!systemState.manualMotorTest) return;
  if (systemState.emergency || systemState.safeMode) {
    motors.stop();
    systemState.manualMotorTest = false;
    return;
  }
  motors.set(settings.motors.leftSpeed, settings.motors.rightSpeed);
}

void setup() {
  Serial.begin(115200);

  if (JOYSTICK_SERIAL_DEBUG) {
    unsigned long serialStart = millis();
    while (!Serial && millis() - serialStart < 4000) {
      yield();
    }
    joystick.begin();
    Serial.println();
    Serial.println("JOYSTICK ONLY DEBUG");
    Serial.println("Move joystick. Expect X/Y near 0..4095 and SW LOW when pressed.");
    return;
  }

  logger.begin();
  joystick.begin();
  motors.begin(); // Motors are explicitly off before any other subsystem can start.

  unsigned long bootStart = millis();
  while (millis() - bootStart < BOOT_SAFE_HOLD_MS) {
    if (!joystick.pressed()) break;
  }
  systemState.safeMode = joystick.bootHeldLongEnough(bootStart);

  settingsStore.begin();
  settingsStore.load(settings);
  settings.sensors.threshold = constrain(settings.sensors.threshold, 0, 4095);
  sensors.begin();
  sensors.setCalibration(settings.sensors);
  if (settings.sensors.threshold > 1000) settings.sensors.threshold = DEFAULT_SENSOR_THRESHOLD;
  if (settings.sensors.threshold < 750) settings.sensors.threshold = DEFAULT_SENSOR_THRESHOLD;
  if (settings.kd > 1.0f) {
    settings.kp = DEFAULT_KP;
    settings.ki = DEFAULT_KI;
    settings.kd = DEFAULT_KD;
  }
  sensors.setCalibration(settings.sensors);
  algorithms.begin();
  algorithms.setMode(settings.algorithm);
  battery.begin();
  wifiManager.begin(settings.wifiEnabled, settings.wifiSsid, settings.wifiPass);
  systemState.askPinSetup = !settings.pinEnabled;
  oledUi.begin(&settings, &systemState);
  webDashboard.begin(&settings, &systemState, &pid);
  pid.setTunings(settings.kp, settings.ki, settings.kd);

  if (systemState.safeMode) logger.log("Boot joystick hold detected");
  logger.log("Line Follower OS ready");
}

void loop() {
  JoyEvent event = joystick.update();

  if (JOYSTICK_SERIAL_DEBUG) {
    (void)event;
    return;
  }

  sensors.update();
  battery.update();
  wifiManager.update();
  webDashboard.update();

  if (systemState.emergency) {
    systemState.running = false;
    systemState.manualMotorTest = false;
    motors.emergencyStop();
  }

  oledUi.update(event);

  if (!systemState.manualMotorTest) {
    runLineFollower();
  } else {
    systemState.running = false;
    runManualMotorTest();
  }

  oledUi.render();
}
