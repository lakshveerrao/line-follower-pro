# Line Follower OS Setup

## Required Libraries

PlatformIO installs these from `platformio.ini`:

- Adafruit SSD1306
- Adafruit GFX Library

Arduino core libraries used by ESP32-S3:

- Wire
- WiFi
- Preferences

## Pin Map

Edit `src/config.h` to match your wiring.

| Part | Default pin |
| --- | --- |
| OLED SDA | GPIO 8 |
| OLED SCL | GPIO 9 |
| Joystick X | GPIO 1 |
| Joystick Y | GPIO 2 |
| Joystick button | GPIO 4 |
| IR sensors | GPIO 3, 10, 11, 12, 13, 14, 15, 16 |
| IR LEDON | GPIO 5 |
| MDD3A M1A | GPIO 6 |
| MDD3A M1B | GPIO 7 |
| MDD3A M2A | GPIO 17 |
| MDD3A M2B | GPIO 18 |
| Battery ADC | GPIO 18 |

## Build and Upload

From `C:\Users\PBL\line_follower_os`:

```powershell
pio run
pio run -t upload
pio device monitor
```

If using Arduino IDE, copy the `src` files into a sketch folder and install:

- ESP32 board package
- Adafruit SSD1306
- Adafruit GFX Library

## First Boot

Motors stay off on boot. Hold the joystick button while powering on for 2 seconds to enter safe mode.

Use OLED menu:

- UP / DOWN moves menu selection
- LEFT / RIGHT changes values
- PRESS selects or saves
- LONG PRESS goes back
- Route Memory uses RIGHT to replay the saved path, LEFT to stop replay, and PRESS to clear the route.

Before running:

1. Open Sensor Settings.
2. Calibrate white above the track background.
3. Calibrate black above the line.
4. Adjust threshold until live values are stable.
5. Tune base speed and PID.

## Safety Notes

Emergency Stop cuts motor output and prevents running. Press Emergency Stop again to clear the emergency state; motors remain off until Start is selected.

Flood Fill, A*, and Dijkstra are placeholders because they require a grid map. The OLED displays `Requires grid map` and motors do not run in those modes.
