# Line Follower OS for ESP32-S3

OLED-first line follower firmware for ESP32-S3 with joystick-only control. Motors remain off at boot, and the robot can run without phone, Wi-Fi, or laptop after setup.

## Build Status

Verified with Arduino CLI:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32s3 C:\Users\PBL\line_follower_os
```

Result:

- Program storage: 954200 bytes, 72%
- RAM globals: 46868 bytes, 14%

## Required Libraries

- ESP32 Arduino core
- Adafruit SSD1306
- Adafruit GFX Library
- Wire, WiFi, Preferences from the ESP32 core

## Upload

Arduino CLI:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32s3 C:\Users\PBL\line_follower_os
arduino-cli upload -p COM_PORT --fqbn esp32:esp32:esp32s3 C:\Users\PBL\line_follower_os
```

PlatformIO:

```powershell
pio run
pio run -t upload
```

## Pin Map

All pins are in `src/config.h`.

| Hardware | Default |
| --- | --- |
| SSD1306 SDA | GPIO 8 |
| SSD1306 SCL | GPIO 9 |
| Joystick X | GPIO 1 |
| Joystick Y | GPIO 2 |
| Joystick press | GPIO 4 |
| IR array | GPIO 3, 10, 11, 12, 13, 14, 15, 16 |
| IR LEDON | GPIO 5 |
| MDD3A M1A | GPIO 6 |
| MDD3A M1B | GPIO 7 |
| MDD3A M2A | GPIO 17 |
| MDD3A M2B | GPIO 18 |
| Battery ADC | GPIO 18 |

## Modules

- `main.cpp`: boot safety, event loop, PID control loop, junction handling.
- `oled_ui.*`: OLED menus, value editing, PIN editor, Wi-Fi screens, reset confirmations.
- `joystick.*`: analog joystick directions, press, long press.
- `motors.*`: MDD3A motor output with emergency stop.
- `sensors.*`: IR sensor reads, calibration, threshold, position, error, junction detection.
- `pid_controller.*`: simple PID controller.
- `algorithms.*`: normal follow, left/right hand rule, path memory, grid-map placeholders.
- `settings_store.*`: NVS/Preferences saved settings.
- `wifi_manager.*`: Wi-Fi enable, scan, connect, IP/RSSI.
- `battery.*`: optional battery voltage and status.
- `logger.*`: small rolling log for debug screen and serial output.

## Safety Behavior

- Motors are off during boot.
- Press and hold joystick during boot for 2 seconds to enter safe mode.
- Robot will not start if safe mode, emergency stop, invalid sensors, or no line detected.
- Emergency stop immediately cuts motor output.
- Press Emergency Stop again to clear emergency state; motors stay off until Start is selected.
- Flood Fill, A*, and Dijkstra stop the robot and display `Requires grid map`.
- Route Memory screen: `RIGHT` starts saved-path replay, `LEFT` stops replay, `PRESS` clears the saved route.

## First Run

1. Check `src/config.h` pin mapping.
2. Upload firmware.
3. Open Sensor Settings.
4. Calibrate white on the background.
5. Calibrate black on the line.
6. Adjust threshold until line detection is stable.
7. Tune base speed, turn speed, Kp, Ki, and Kd.
8. Select algorithm and start.
