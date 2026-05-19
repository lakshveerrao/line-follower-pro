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
#include <Wire.h>
#include <U8g2lib.h>

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

static Adafruit_SH1106G testDisplay(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);
static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2Test(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);
static bool testDisplayOk = false;
static bool u8g2TestOk = false;
static Adafruit_SH1106G bootDisplay(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);
static bool bootDisplayOk = false;
static Adafruit_SH1106G autoDebugDisplay(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);
static bool autoDebugDisplayOk = false;
static uint8_t autoDebugI2cCount = 0;
static uint8_t autoDebugOledAddress = 0;
static unsigned long autoDebugCenterSavedUntil = 0;
static U8G2_SH1106_128X64_NONAME_F_HW_I2C compatDisplay(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);
static bool compatDisplayOk = false;
static uint8_t compatSelected = 0;
static uint8_t compatTop = 0;
static unsigned long compatLastRender = 0;

static const char *const COMPAT_ITEMS[] = {
  "Start / Stop", "Emergency Stop", "Motor Settings", "PID Settings",
  "Sensor Settings", "Algorithm", "Debug", "Battery", "WiFi", "About"
};
static constexpr uint8_t COMPAT_ITEM_COUNT = sizeof(COMPAT_ITEMS) / sizeof(COMPAT_ITEMS[0]);

static void bootPause(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    yield();
  }
}

static void drawBootScreen(const __FlashStringHelper *line1,
                           const __FlashStringHelper *line2 = nullptr,
                           const __FlashStringHelper *line3 = nullptr,
                           const __FlashStringHelper *line4 = nullptr) {
  if (!bootDisplayOk) return;
  bootDisplay.clearDisplay();
  bootDisplay.setTextColor(SH110X_WHITE);
  bootDisplay.setTextSize(1);
  bootDisplay.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SH110X_WHITE);
  bootDisplay.setCursor(6, 8);
  bootDisplay.print(line1);
  if (line2) {
    bootDisplay.setCursor(6, 22);
    bootDisplay.print(line2);
  }
  if (line3) {
    bootDisplay.setCursor(6, 36);
    bootDisplay.print(line3);
  }
  if (line4) {
    bootDisplay.setCursor(6, 50);
    bootDisplay.print(line4);
  }
  bootDisplay.display();
}

static void beginBootDisplay() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  bootDisplayOk = bootDisplay.begin(OLED_I2C_ADDR, true);
  if (!bootDisplayOk) bootDisplayOk = bootDisplay.begin(0x3D, true);
  if (bootDisplayOk) bootDisplay.setContrast(255);
}

static void scanI2cBus() {
  Serial.println("I2C scan start");
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C found 0x");
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("I2C found none");
  }
  Serial.println("I2C scan done");
}

static uint8_t scanI2cBusCompact(uint8_t *firstAddress) {
  uint8_t found = 0;
  if (firstAddress) *firstAddress = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      if (found == 0 && firstAddress) *firstAddress = address;
      found++;
    }
  }
  return found;
}

static void rawOledCommand(uint8_t address, uint8_t command) {
  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.write(command);
  Wire.endTransmission();
}

static void rawOledData(uint8_t address, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(0x40);
  for (uint8_t i = 0; i < 16; i++) {
    Wire.write(data);
  }
  Wire.endTransmission();
}

static void rawOledInitAndFill(uint8_t address, bool sh1106Mode) {
  const uint8_t init[] = {
    0xAE,       // display off
    0xD5, 0x80, // clock
    0xA8, 0x3F, // multiplex
    0xD3, 0x00, // offset
    0x40,       // start line
    0xA1,       // segment remap
    0xC8,       // COM scan direction
    0xDA, 0x12, // COM pins
    0x81, 0xFF, // contrast
    0xD9, 0xF1, // precharge
    0xDB, 0x40, // vcomh
    0xA4,       // resume RAM display
    0xA6,       // normal display
    0xAF        // display on
  };
  for (uint8_t i = 0; i < sizeof(init); i++) {
    rawOledCommand(address, init[i]);
    delay(2);
  }

  // SSD1306 charge pump. SH1106 ignores unsupported commands on most modules.
  rawOledCommand(address, 0x8D);
  rawOledCommand(address, 0x14);
  rawOledCommand(address, 0xAF);

  for (uint8_t page = 0; page < 8; page++) {
    rawOledCommand(address, 0xB0 | page);
    uint8_t col = sh1106Mode ? 2 : 0;
    rawOledCommand(address, 0x00 | (col & 0x0F));
    rawOledCommand(address, 0x10 | (col >> 4));
    for (uint8_t chunk = 0; chunk < 8; chunk++) {
      rawOledData(address, 0xFF);
    }
  }
}

static const char *joyName(JoyEvent event) {
  switch (event) {
    case JoyEvent::Up: return "UP";
    case JoyEvent::Down: return "DOWN";
    case JoyEvent::Left: return "LEFT";
    case JoyEvent::Right: return "RIGHT";
    case JoyEvent::Press: return "PRESS";
    case JoyEvent::LongPress: return "LONG";
    default: return "CENTER";
  }
}

static void compatShowMessage(const char *line1, const char *line2 = "") {
  if (!compatDisplayOk) return;
  compatDisplay.clearBuffer();
  compatDisplay.setFont(u8g2_font_6x10_tf);
  compatDisplay.drawFrame(0, 0, 128, 64);
  compatDisplay.drawStr(6, 18, line1);
  compatDisplay.drawStr(6, 36, line2);
  compatDisplay.sendBuffer();
}

static void beginCompatUi() {
  if (!U8G2_COMPAT_UI) return;
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  compatDisplay.setI2CAddress(OLED_I2C_ADDR << 1);
  compatDisplayOk = compatDisplay.begin();
  if (!compatDisplayOk) return;
  compatDisplay.setContrast(255);
  compatDisplay.clearBuffer();
  compatDisplay.drawBox(0, 0, 128, 64);
  compatDisplay.sendBuffer();
  delay(250);
  compatShowMessage("Line Follower OS", "U8G2 display OK");
  delay(700);
}

static void renderCompatUi(bool force = false) {
  if (!U8G2_COMPAT_UI || !compatDisplayOk) return;
  unsigned long now = millis();
  if (!force && now - compatLastRender < UI_PERIOD_MS) return;
  compatLastRender = now;

  compatDisplay.clearBuffer();
  compatDisplay.setFont(u8g2_font_6x10_tf);
  compatDisplay.drawStr(0, 8, systemState.running ? "RUN" : "STOP");
  compatDisplay.drawStr(36, 8, systemState.safeMode ? "SAFE" : "OS");
  compatDisplay.drawStr(88, 8, "USB");
  compatDisplay.drawLine(0, 10, 127, 10);

  for (uint8_t row = 0; row < 4; row++) {
    uint8_t idx = compatTop + row;
    if (idx >= COMPAT_ITEM_COUNT) break;
    uint8_t y = 22 + row * 10;
    if (idx == compatSelected) {
      compatDisplay.drawBox(0, y - 8, 128, 10);
      compatDisplay.setDrawColor(0);
    }
    compatDisplay.drawStr(2, y, COMPAT_ITEMS[idx]);
    compatDisplay.setDrawColor(1);
  }

  char line[32];
  snprintf(line, sizeof(line), "S:%u E:%d", sensors.activeCount(), sensors.error());
  compatDisplay.drawStr(0, 63, line);
  compatDisplay.sendBuffer();
}

static void updateCompatUi(JoyEvent event) {
  if (!U8G2_COMPAT_UI || !compatDisplayOk) return;
  if (event == JoyEvent::Up && compatSelected > 0) compatSelected--;
  if (event == JoyEvent::Down && compatSelected + 1 < COMPAT_ITEM_COUNT) compatSelected++;
  if (compatSelected < compatTop) compatTop = compatSelected;
  if (compatSelected > compatTop + 3) compatTop = compatSelected - 3;

  if (event == JoyEvent::Press) {
    switch (compatSelected) {
      case 0:
        if (systemState.safeMode || systemState.emergency || !sensors.valid() || !sensors.lineDetected()) {
          systemState.running = false;
          compatShowMessage("Cannot start", "Check sensors");
        } else {
          systemState.running = !systemState.running;
          compatShowMessage(systemState.running ? "Robot started" : "Robot stopped");
        }
        delay(450);
        break;
      case 1:
        systemState.emergency = true;
        systemState.running = false;
        motors.emergencyStop();
        compatShowMessage("EMERGENCY STOP", "Motors OFF");
        delay(650);
        break;
      case 2:
        compatShowMessage("Motor Settings", "Use web dashboard");
        delay(650);
        break;
      case 3:
        compatShowMessage("PID Settings", "Use web dashboard");
        delay(650);
        break;
      case 4:
        compatShowMessage("Sensor Values", "See bottom line");
        delay(650);
        break;
      case 5:
        compatShowMessage("Algorithm", algorithms.modeName());
        delay(650);
        break;
      case 6:
        compatShowMessage("Debug", joyName(joystick.lastAxisEvent()));
        delay(650);
        break;
      case 8:
        compatShowMessage("WiFi LineFollowerOS", "Pass line12345");
        delay(850);
        break;
      case 9:
        compatShowMessage("ESP32-S3", "Line Follower OS");
        delay(850);
        break;
      default:
        break;
    }
  }
  renderCompatUi(true);
}

static void beginJoystickOledTest() {
  joystick.begin();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  scanI2cBus();
  testDisplayOk = testDisplay.begin(OLED_I2C_ADDR, true);
  if (!testDisplayOk) testDisplayOk = testDisplay.begin(0x3D, true);
  u8g2Test.setI2CAddress(OLED_I2C_ADDR << 1);
  u8g2TestOk = u8g2Test.begin();
  u8g2Test.setContrast(255);

  Serial.println();
  Serial.println("JOYSTICK + OLED TEST");
  Serial.print("Adafruit OLED ");
  Serial.println(testDisplayOk ? "OK" : "FAILED");
  Serial.print("U8G2 OLED ");
  Serial.println(u8g2TestOk ? "OK" : "FAILED");
  Serial.println("Pins: VRX=1 VRY=2 SW=4 SDA=8 SCL=9");

  if (u8g2TestOk) {
    u8g2Test.clearBuffer();
    u8g2Test.setFont(u8g2_font_6x10_tf);
    u8g2Test.drawFrame(0, 0, 128, 64);
    u8g2Test.drawStr(8, 14, "U8G2 SH1106 OK");
    u8g2Test.drawStr(8, 30, "ADDR 0x3C");
    u8g2Test.drawStr(8, 46, "SDA8 SCL9");
    u8g2Test.sendBuffer();
  } else if (testDisplayOk) {
    testDisplay.setContrast(255);
    testDisplay.clearDisplay();
    testDisplay.setTextColor(SH110X_WHITE);
    testDisplay.setTextSize(1);
    testDisplay.setCursor(0, 0);
    testDisplay.print("JOYSTICK OLED TEST");
    testDisplay.setCursor(0, 16);
    testDisplay.print("Move stick / press");
    testDisplay.display();
  }
}

static void runJoystickOledTest(JoyEvent event) {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 120) return;
  last = now;

  Serial.print("JOY x=");
  Serial.print(joystick.rawX());
  Serial.print(" y=");
  Serial.print(joystick.rawY());
  Serial.print(" sw=");
  Serial.print(joystick.pressed() ? "PRESSED" : "open");
  Serial.print(" event=");
  Serial.println(joyName(event == JoyEvent::None ? joystick.lastAxisEvent() : event));

  if (!testDisplayOk && !u8g2TestOk) return;
  if (u8g2TestOk) {
    char line[24];
    u8g2Test.clearBuffer();
    u8g2Test.setFont(u8g2_font_6x10_tf);
    u8g2Test.drawFrame(0, 0, 128, 64);
    u8g2Test.drawStr(5, 12, "JOYSTICK TEST U8G2");
    snprintf(line, sizeof(line), "X:%d Y:%d", joystick.rawX(), joystick.rawY());
    u8g2Test.drawStr(5, 28, line);
    snprintf(line, sizeof(line), "SW:%s", joystick.pressed() ? "PRESSED" : "open");
    u8g2Test.drawStr(5, 42, line);
    snprintf(line, sizeof(line), "DIR:%s", joyName(event == JoyEvent::None ? joystick.lastAxisEvent() : event));
    u8g2Test.drawStr(5, 56, line);
    u8g2Test.sendBuffer();
  } else if (testDisplayOk) {
    testDisplay.clearDisplay();
    testDisplay.setTextColor(SH110X_WHITE);
    testDisplay.setTextSize(1);
    testDisplay.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SH110X_WHITE);
    testDisplay.setCursor(5, 5);
    testDisplay.print("JOYSTICK TEST");
    testDisplay.setCursor(5, 18);
    testDisplay.print("X:");
    testDisplay.print(joystick.rawX());
    testDisplay.setCursor(68, 18);
    testDisplay.print("Y:");
    testDisplay.print(joystick.rawY());
    testDisplay.setCursor(5, 31);
    testDisplay.print("SW:");
    testDisplay.print(joystick.pressed() ? "PRESSED" : "open");
    testDisplay.setCursor(5, 44);
    testDisplay.print("DIR:");
    testDisplay.print(joyName(event == JoyEvent::None ? joystick.lastAxisEvent() : event));
    testDisplay.display();
  }
}

static void runSensorSerialDebug() {
  static unsigned long lastPrint = 0;
  sensors.update();
  unsigned long now = millis();
  if (now - lastPrint < 250) return;
  lastPrint = now;

  Serial.print("SENSOR raw=");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (i) Serial.print(',');
    Serial.print(sensors.value(i));
  }
  Serial.print(" norm=");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (i) Serial.print(',');
    Serial.print(sensors.lineValue(i));
  }
  Serial.print(" active=");
  Serial.print(sensors.activeCount());
  Serial.print(" pos=");
  Serial.print(sensors.position());
  Serial.print(" err=");
  Serial.println(sensors.error());
}

static void beginAutoDebugMode() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  autoDebugI2cCount = scanI2cBusCompact(&autoDebugOledAddress);
  autoDebugDisplayOk = autoDebugDisplay.begin(OLED_I2C_ADDR, true);
  if (!autoDebugDisplayOk) autoDebugDisplayOk = autoDebugDisplay.begin(0x3D, true);
  if (autoDebugDisplayOk) {
    autoDebugDisplay.setContrast(255);
    autoDebugDisplay.clearDisplay();
    autoDebugDisplay.setTextColor(SH110X_WHITE);
    autoDebugDisplay.setTextSize(1);
    autoDebugDisplay.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SH110X_WHITE);
    autoDebugDisplay.setCursor(6, 8);
    autoDebugDisplay.print("AUTO DEBUG MODE");
    autoDebugDisplay.setCursor(6, 24);
    autoDebugDisplay.print("Keep stick center");
    autoDebugDisplay.setCursor(6, 40);
    autoDebugDisplay.print("Motors OFF");
    autoDebugDisplay.display();
  }

  motors.begin();
  motors.stop();
  joystick.begin();
  sensors.begin();
  Serial.println();
  Serial.println("AUTO DEBUG MODE");
  Serial.print("I2C devices=");
  Serial.print(autoDebugI2cCount);
  Serial.print(" first=0x");
  if (autoDebugOledAddress < 16) Serial.print('0');
  Serial.println(autoDebugOledAddress, HEX);
  Serial.print("OLED=");
  Serial.println(autoDebugDisplayOk ? "OK" : "FAILED");
  Serial.println("Move joystick and place sensors on/off line. Motors stay OFF.");
}

static void runAutoDebugMode() {
  static unsigned long last = 0;
  unsigned long now = millis();
  JoyEvent event = joystick.update();
  if (event == JoyEvent::Press) {
    joystick.recalibrateCenter();
    autoDebugCenterSavedUntil = millis() + 1200;
    Serial.println("DBG joystick center saved");
  }
  sensors.update();
  motors.stop();

  if (now - last < 200) return;
  last = now;

  JoyEvent dir = event == JoyEvent::None ? joystick.lastAxisEvent() : event;
  Serial.print("DBG i2c=");
  Serial.print(autoDebugI2cCount);
  Serial.print(" oled=");
  Serial.print(autoDebugDisplayOk ? 1 : 0);
  Serial.print(" joy=");
  Serial.print(joystick.rawX());
  Serial.print('/');
  Serial.print(joystick.rawY());
  Serial.print(" center=");
  Serial.print(joystick.centerX());
  Serial.print('/');
  Serial.print(joystick.centerY());
  Serial.print(" sw=");
  Serial.print(joystick.pressed() ? "PRESS" : "open");
  Serial.print(" dir=");
  Serial.print(joyName(dir));
  Serial.print(" sensor=");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (i) Serial.print(',');
    Serial.print(sensors.value(i));
  }
  Serial.print(" active=");
  Serial.print(sensors.activeCount());
  Serial.print(" err=");
  Serial.println(sensors.error());

  if (!autoDebugDisplayOk) return;
  autoDebugDisplay.clearDisplay();
  autoDebugDisplay.setTextColor(SH110X_WHITE);
  autoDebugDisplay.setTextSize(1);
  autoDebugDisplay.setCursor(0, 0);
  autoDebugDisplay.print("AUTO DEBUG");
  autoDebugDisplay.setCursor(78, 0);
  autoDebugDisplay.print(autoDebugDisplayOk ? "OLED OK" : "OLED BAD");
  autoDebugDisplay.setCursor(0, 12);
  autoDebugDisplay.print("I2C:");
  autoDebugDisplay.print(autoDebugI2cCount);
  autoDebugDisplay.print(" 0x");
  if (autoDebugOledAddress < 16) autoDebugDisplay.print('0');
  autoDebugDisplay.print(autoDebugOledAddress, HEX);
  autoDebugDisplay.setCursor(0, 24);
  autoDebugDisplay.print("X:");
  autoDebugDisplay.print(joystick.rawX());
  autoDebugDisplay.print("/");
  autoDebugDisplay.print(joystick.centerX());
  autoDebugDisplay.setCursor(68, 24);
  autoDebugDisplay.print("Y:");
  autoDebugDisplay.print(joystick.rawY());
  autoDebugDisplay.setCursor(0, 36);
  if (millis() < autoDebugCenterSavedUntil) {
    autoDebugDisplay.print("CENTER SAVED");
    autoDebugDisplay.display();
    return;
  }
  autoDebugDisplay.print("SW:");
  autoDebugDisplay.print(joystick.pressed() ? "PRESS" : "open");
  autoDebugDisplay.print(" DIR:");
  autoDebugDisplay.print(joyName(dir));
  autoDebugDisplay.setCursor(0, 48);
  autoDebugDisplay.print("S:");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    autoDebugDisplay.print(sensors.value(i) / 100);
    if (i < SENSOR_COUNT - 1) autoDebugDisplay.print(',');
  }
  autoDebugDisplay.setCursor(0, 58);
  autoDebugDisplay.print("A:");
  autoDebugDisplay.print(sensors.activeCount());
  autoDebugDisplay.print(" E:");
  autoDebugDisplay.print(sensors.error());
  autoDebugDisplay.display();
}

static void runBootDiagnostics() {
  if (!BOOT_DIAGNOSTICS) return;

  uint8_t firstAddress = 0;
  uint8_t i2cCount = scanI2cBusCompact(&firstAddress);
  for (uint8_t i = 0; i < 8; i++) {
    sensors.update();
    bootPause(20);
  }

  bool joyOk = joystick.rawX() > 30 && joystick.rawX() < 4065 &&
               joystick.rawY() > 30 && joystick.rawY() < 4065;
  bool sensorOk = sensors.valid();

  Serial.println("BOOT DIAGNOSTICS");
  Serial.print("I2C devices=");
  Serial.print(i2cCount);
  Serial.print(" first=0x");
  if (firstAddress < 16) Serial.print('0');
  Serial.println(firstAddress, HEX);
  Serial.print("Joystick center=");
  Serial.print(joystick.centerX());
  Serial.print('/');
  Serial.print(joystick.centerY());
  Serial.print(" ok=");
  Serial.println(joyOk ? 1 : 0);
  Serial.print("Sensors valid=");
  Serial.print(sensorOk ? 1 : 0);
  Serial.print(" active=");
  Serial.print(sensors.activeCount());
  Serial.print(" err=");
  Serial.println(sensors.error());

  oledUi.showMessage("Auto diagnostics", "OLED/I2C OK");
  bootPause(900);

  String joyLine = joyOk ? "Joystick OK " : "Joystick CHECK ";
  joyLine += String(joystick.centerX()) + "/" + String(joystick.centerY());
  oledUi.showMessage(joyLine, joystick.pressed() ? "Button pressed" : "Button open");
  bootPause(900);

  String sensorLine = sensorOk ? "Sensors OK " : "Sensors CHECK ";
  sensorLine += "A:" + String(sensors.activeCount()) + " E:" + String(sensors.error());
  oledUi.showMessage(sensorLine, "Motors OFF on boot");
  bootPause(900);

  if (!joyOk || !sensorOk || i2cCount == 0) {
    logger.log("Boot diagnostics warning");
  } else {
    logger.log("Boot diagnostics OK");
  }
}

static void runOledI2cDebug() {
  static bool ran = false;
  if (ran) {
    delay(1500);
    return;
  }
  ran = true;
  Serial.println();
  Serial.println("OLED I2C DEBUG");
  Serial.print("SDA=");
  Serial.print(I2C_SDA_PIN);
  Serial.print(" SCL=");
  Serial.println(I2C_SCL_PIN);
  Serial.flush();

  uint8_t pairs[2][2] = {{I2C_SDA_PIN, I2C_SCL_PIN}, {I2C_SCL_PIN, I2C_SDA_PIN}};
  uint8_t found = 0;
  uint8_t workingSda = I2C_SDA_PIN;
  uint8_t workingScl = I2C_SCL_PIN;
  for (uint8_t pair = 0; pair < 2; pair++) {
    uint8_t sda = pairs[pair][0];
    uint8_t scl = pairs[pair][1];
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
    Wire.end();
    Wire.begin(sda, scl);
    Wire.setTimeOut(50);
    Wire.setClock(50000);
    Serial.print("Scan pair SDA=");
    Serial.print(sda);
    Serial.print(" SCL=");
    Serial.println(scl);
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      if (err == 0) {
        Serial.print("I2C found 0x");
        if (addr < 16) Serial.print('0');
        Serial.print(addr, HEX);
        Serial.print(" on SDA=");
        Serial.print(sda);
        Serial.print(" SCL=");
        Serial.println(scl);
        found++;
        workingSda = sda;
        workingScl = scl;
      }
    }
  }
  if (found == 0) {
    Serial.println("No I2C devices found on normal or swapped pins.");
    Serial.println("Check OLED VCC=3.3V, GND, SDA, SCL, and solder/header.");
  }
  if (found > 0) {
    Wire.end();
    Wire.begin(workingSda, workingScl);
    Wire.setClock(100000);
    Adafruit_SH1106G testDisplay(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);
    bool ok = testDisplay.begin(OLED_I2C_ADDR, true);
    Serial.print("SH1106 begin ");
    Serial.println(ok ? "OK" : "FAILED");
    if (ok) {
      testDisplay.setContrast(255);
      testDisplay.clearDisplay();
      testDisplay.fillScreen(SH110X_WHITE);
      testDisplay.display();
      Serial.println("Adafruit SH1106 full white");
      delay(1200);
      testDisplay.invertDisplay(true);
      testDisplay.display();
      Serial.println("Adafruit SH1106 invert");
      delay(1200);
      testDisplay.invertDisplay(false);
      testDisplay.clearDisplay();
      testDisplay.setTextColor(SH110X_WHITE);
      testDisplay.setTextSize(1);
      testDisplay.drawRect(0, 0, OLED_WIDTH, OLED_HEIGHT, SH110X_WHITE);
      testDisplay.setCursor(8, 12);
      testDisplay.print("SH1106 OK");
      testDisplay.setCursor(8, 28);
      testDisplay.print("ADDR 0x3C");
      testDisplay.setCursor(8, 44);
      testDisplay.print("SDA");
      testDisplay.print(workingSda);
      testDisplay.print(" SCL");
      testDisplay.print(workingScl);
      testDisplay.display();
      delay(1200);
    }

    Serial.println("Trying U8G2 SH1106");
    U8G2_SH1106_128X64_NONAME_F_HW_I2C sh1106(U8G2_R0, U8X8_PIN_NONE, workingScl, workingSda);
    sh1106.setI2CAddress(OLED_I2C_ADDR << 1);
    if (sh1106.begin()) {
      sh1106.setContrast(255);
      sh1106.clearBuffer();
      sh1106.drawBox(0, 0, 128, 64);
      sh1106.sendBuffer();
      delay(1200);
      sh1106.clearBuffer();
      sh1106.setFont(u8g2_font_6x10_tf);
      sh1106.drawFrame(0, 0, 128, 64);
      sh1106.drawStr(6, 14, "U8G2 SH1106");
      sh1106.drawStr(6, 30, "ADDR 0x3C");
      sh1106.drawStr(6, 46, "SDA9 SCL8");
      sh1106.sendBuffer();
      delay(1200);
    } else {
      Serial.println("U8G2 SH1106 begin FAILED");
    }

    Serial.println("Trying U8G2 SSD1306");
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C ssd1306(U8G2_R0, U8X8_PIN_NONE, workingScl, workingSda);
    ssd1306.setI2CAddress(OLED_I2C_ADDR << 1);
    if (ssd1306.begin()) {
      ssd1306.setContrast(255);
      ssd1306.clearBuffer();
      ssd1306.drawBox(0, 0, 128, 64);
      ssd1306.sendBuffer();
      delay(1200);
      ssd1306.clearBuffer();
      ssd1306.setFont(u8g2_font_6x10_tf);
      ssd1306.drawFrame(0, 0, 128, 64);
      ssd1306.drawStr(6, 14, "U8G2 SSD1306");
      ssd1306.drawStr(6, 30, "ADDR 0x3C");
      ssd1306.drawStr(6, 46, "SDA9 SCL8");
      ssd1306.sendBuffer();
      delay(1200);
    } else {
      Serial.println("U8G2 SSD1306 begin FAILED");
    }

    Serial.println("Trying RAW SH1106 full white");
    rawOledInitAndFill(OLED_I2C_ADDR, true);
    delay(1800);
    Serial.println("Trying RAW SSD1306 full white");
    rawOledInitAndFill(OLED_I2C_ADDR, false);
    delay(1800);
  }
  Serial.println("Debug scan complete.");
}

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
  if (!AUTONOMOUS_LINE_FOLLOW_ENABLED) {
    systemState.running = false;
    motors.stop();
    return;
  }

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

  left = constrain(left, -AUTONOMOUS_MAX_PWM, AUTONOMOUS_MAX_PWM);
  right = constrain(right, -AUTONOMOUS_MAX_PWM, AUTONOMOUS_MAX_PWM);
  motors.set(left, right);
}

static void runManualMotorTest() {
  if (!systemState.manualMotorTest) return;
  if (systemState.emergency || systemState.safeMode) {
    motors.stop();
    systemState.manualMotorTest = false;
    return;
  }
  int left = systemState.manualLeft;
  int right = systemState.manualRight;
  if (left == 0 && right == 0) {
    left = settings.motors.leftSpeed;
    right = settings.motors.rightSpeed;
  }
  left = constrain(left, -MANUAL_TEST_MAX_PWM, MANUAL_TEST_MAX_PWM);
  right = constrain(right, -MANUAL_TEST_MAX_PWM, MANUAL_TEST_MAX_PWM);
  motors.set(left, right);
}

void setup() {
  Serial.begin(115200);

  if (AUTO_DEBUG_MODE) {
    beginAutoDebugMode();
    return;
  }

  if (JOYSTICK_OLED_TEST) {
    beginJoystickOledTest();
    return;
  }

  if (OLED_I2C_DEBUG) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setTimeOut(50);
    Wire.setClock(100000);
    return;
  }

  if (SENSOR_SERIAL_DEBUG) {
    sensors.begin();
    Serial.println();
    Serial.println("SENSOR DEBUG");
    Serial.print("LEDON=");
    Serial.println(SENSOR_LEDON_PIN);
    Serial.print("PINS=");
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      if (i) Serial.print(',');
      Serial.print(SENSOR_PINS[i]);
    }
    Serial.println();
    return;
  }

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
  beginBootDisplay();
  drawBootScreen(F("Welcome to"), F("Line Follower OS"), F("Keep joystick"), F("centered"));
  bootPause(1200);
  drawBootScreen(F("Joystick"), F("calibrating..."), F("Do not move stick"));
  joystick.begin();
  drawBootScreen(F("Joystick ready"), F("Center saved"), F("X/Y calibrated"));
  bootPause(700);
  motors.begin(); // Motors are explicitly off before any other subsystem can start.

  unsigned long bootStart = millis();
  while (millis() - bootStart < BOOT_SAFE_HOLD_MS) {
    if (!joystick.pressed()) break;
  }
  systemState.safeMode = joystick.bootHeldLongEnough(bootStart);

  settingsStore.begin();
  settingsStore.load(settings);
  settings.screenSleepIndex = 6; // Keep OLED awake by default during hardware bring-up.
  settings.autoLockIndex = constrain((int)settings.autoLockIndex, 0, 2);
  settings.motors.baseSpeed = constrain(settings.motors.baseSpeed, 0, AUTONOMOUS_MAX_PWM);
  settings.motors.turnSpeed = constrain(settings.motors.turnSpeed, 0, AUTONOMOUS_MAX_PWM);
  settings.motors.leftSpeed = constrain(settings.motors.leftSpeed, 0, MANUAL_TEST_MAX_PWM);
  settings.motors.rightSpeed = constrain(settings.motors.rightSpeed, 0, MANUAL_TEST_MAX_PWM);
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
  if (U8G2_COMPAT_UI) {
    beginCompatUi();
    compatShowMessage("WiFi LineFollowerOS", "Pass line12345");
    delay(900);
    renderCompatUi(true);
  } else {
    oledUi.begin(&settings, &systemState);
    runBootDiagnostics();
    oledUi.showMessage("WiFi LineFollowerOS", "Password line12345");
    bootPause(1600);
    oledUi.showHome();
  }
  webDashboard.begin(&settings, &systemState, &pid);
  pid.setTunings(settings.kp, settings.ki, settings.kd);

  if (systemState.safeMode) logger.log("Boot joystick hold detected");
  logger.log("Line Follower OS ready");
  systemState.running = false;
  systemState.manualMotorTest = false;
  motors.stop();
}

void loop() {
  if (AUTO_DEBUG_MODE) {
    runAutoDebugMode();
    return;
  }

  if (JOYSTICK_OLED_TEST) {
    JoyEvent event = joystick.update();
    runJoystickOledTest(event);
    return;
  }

  if (OLED_I2C_DEBUG) {
    runOledI2cDebug();
    delay(1500);
    return;
  }

  if (SENSOR_SERIAL_DEBUG) {
    runSensorSerialDebug();
    return;
  }

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

  if (U8G2_COMPAT_UI) {
    updateCompatUi(event);
  } else {
    oledUi.update(event);
  }

  if (!systemState.manualMotorTest) {
    runLineFollower();
  } else {
    systemState.running = false;
    runManualMotorTest();
  }

  if (U8G2_COMPAT_UI) {
    renderCompatUi();
  } else {
    oledUi.render();
  }
}
