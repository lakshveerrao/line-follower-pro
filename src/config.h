#pragma once

#include <Arduino.h>

// ===== Board and display =====
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr int OLED_RESET_PIN = -1;
constexpr uint8_t OLED_I2C_ADDR = 0x3C;
constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;

// ===== Joystick =====
constexpr uint8_t JOY_X_PIN = 1;
constexpr uint8_t JOY_Y_PIN = 2;
constexpr uint8_t JOY_BTN_PIN = 4;
constexpr int JOY_AXIS_DEADZONE = 130;
constexpr uint8_t JOY_FILTER_SHIFT = 3; // 1/8 smoothing, no floating point.
constexpr unsigned long JOY_REPEAT_MS = 180;
constexpr unsigned long JOY_LONG_PRESS_MS = 750;
constexpr unsigned long BOOT_SAFE_HOLD_MS = 2000;
constexpr bool JOYSTICK_SERIAL_DEBUG = false;
constexpr unsigned long JOYSTICK_DEBUG_MS = 250;

// ===== IR sensor array =====
// Keep these separate from OLED, joystick, and motor pins.
constexpr uint8_t SENSOR_COUNT = 8;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {3, 10, 11, 12, 13, 14, 15, 16};
constexpr uint8_t SENSOR_LEDON_PIN = 5;
constexpr uint16_t SENSOR_INVALID_LOW = 20;
constexpr uint16_t SENSOR_INVALID_HIGH = 4070;

// ===== MDD3A motor driver pins =====
// MDD3A uses two input pins per motor. Forward drives A with PWM and B low.
// Reverse drives A low and B with PWM.
constexpr uint8_t M1A_PIN = 6;
constexpr uint8_t M1B_PIN = 7;
constexpr uint8_t M2A_PIN = 17;
constexpr uint8_t M2B_PIN = 18;
constexpr uint32_t MOTOR_PWM_FREQ = 20000;
constexpr uint8_t MOTOR_PWM_BITS = 8;
constexpr uint8_t M1A_PWM_CH = 0;
constexpr uint8_t M1B_PWM_CH = 1;
constexpr uint8_t M2A_PWM_CH = 2;
constexpr uint8_t M2B_PWM_CH = 3;
constexpr int MOTOR_MAX_PWM = 255;
constexpr uint8_t MOTOR_RAMP_STEP = 8;
constexpr unsigned long LOST_LINE_RECOVERY_MS = 900;
constexpr int LINE_SEARCH_PWM = 75;
constexpr float PID_OUTPUT_SCALE = 95.0f;
constexpr bool LINE_SERIAL_DEBUG = true;
constexpr unsigned long LINE_DEBUG_MS = 250;

// ===== Web dashboard =====
constexpr bool WEB_DASHBOARD_ENABLED = true;
constexpr char WEB_AP_SSID[] = "LineFollowerOS";
constexpr char WEB_AP_PASS[] = "line12345";

// ===== Battery sensing =====
constexpr bool BATTERY_SENSE_ENABLED = false;
constexpr uint8_t BATTERY_PIN = 18;
constexpr float ADC_REF_VOLTAGE = 3.30f;
constexpr float BATTERY_DIVIDER_RATIO = 2.0f; // Vbat = Vadc * ratio
constexpr float BATTERY_FULL_V = 8.40f;       // Change for your battery pack.
constexpr float BATTERY_LOW_V = 7.20f;
constexpr float BATTERY_CRITICAL_V = 6.80f;

// ===== Runtime timings =====
constexpr unsigned long CONTROL_PERIOD_MS = 10;
constexpr unsigned long UI_PERIOD_MS = 80;
constexpr unsigned long SENSOR_PERIOD_MS = 5;
constexpr unsigned long BATTERY_PERIOD_MS = 1000;
constexpr unsigned long WIFI_PERIOD_MS = 1000;

// ===== Defaults =====
constexpr int DEFAULT_BASE_SPEED = 110;
constexpr int DEFAULT_TURN_SPEED = 90;
constexpr int DEFAULT_LEFT_SPEED = 120;
constexpr int DEFAULT_RIGHT_SPEED = 120;
constexpr float DEFAULT_KP = 0.90f;
constexpr float DEFAULT_KI = 0.00f;
constexpr float DEFAULT_KD = 0.08f;
constexpr uint16_t DEFAULT_SENSOR_THRESHOLD = 800; // Normalized 0..1000 line strength.
constexpr uint16_t SENSOR_MIN_CONTRAST = 70;
constexpr uint8_t ROUTE_MEMORY_SIZE = 120;
constexpr uint8_t LOG_LINE_COUNT = 8;
constexpr uint8_t LOG_LINE_LEN = 32;
