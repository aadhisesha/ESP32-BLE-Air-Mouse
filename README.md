# ESP32 BLE Air Mouse

A motion-controlled wireless mouse built with an **ESP32 LOLIN32 Lite**, an **MPU6050-compatible gyroscope/accelerometer**, three buttons, and a small SPI OLED display. The ESP32 presents itself as a **Bluetooth Low Energy (BLE) HID mouse**, so a compatible computer or phone can receive cursor, click, and scroll events directly.

> **No Wi-Fi network or Python receiver is required.** The firmware sends mouse events over BLE.

## Features

- **Gyroscope-based cursor movement** using hand rotation.
- **Left and right click** buttons.
- **Hold-to-scroll** mode: hold the scroll button and tilt the device to scroll.
- **Three sensitivity levels:** LOW, MED, and HIGH.
- **Gyroscope bias calibration** with calibration values saved in ESP32 non-volatile storage (NVS) after successful calibration.
- **OLED interface** with a startup animation, connection/waiting state, sensitivity, IMU status, and motion indicator.
- Motion filtering, a dead zone to reduce unwanted cursor drift, and gentle acceleration for faster wrist movements.
- IMU recovery attempts if the sensor becomes unavailable.

## Hardware

- ESP32 LOLIN32 Lite
- MPU6050 IMU module (the firmware also probes the alternate I²C address used by some compatible modules)
- 1.3-inch, 128 × 64 SPI OLED using the SH1106 driver
- 3 momentary push buttons
- Jumper wires and a suitable power source

## Wiring

### MPU6050 to ESP32

| MPU6050 pin | ESP32 LOLIN32 Lite |
|---|---|
| VCC | 3V3 (check your module's specifications) |
| GND | GND |
| SDA | GPIO 23 |
| SCL | GPIO 22 |

### 7-pin SPI OLED to ESP32

| OLED pin/label | ESP32 LOLIN32 Lite |
|---|---|
| VCC | 3V3 (check your display module's specifications) |
| GND | GND |
| D0 / SCLK / CLK | GPIO 18 |
| D1 / MOSI / DIN | GPIO 19 |
| RES / RESET | GPIO 4 |
| DC | GPIO 16 |
| CS | GPIO 17 |

### Buttons

The button inputs use `INPUT_PULLUP`, so connect **one terminal of each normally-open button to its GPIO pin and the other terminal to GND**. No external pull-up resistor is required for this wiring.

| Button | ESP32 GPIO | Action |
|---|---:|---|
| Left click | GPIO 32 | Left mouse button |
| Right click | GPIO 27 | Right mouse button |
| Scroll | GPIO 26 | Hold while tilting to scroll |

Check the pin assignments in `config.h` before wiring if you change hardware.

## Repository structure

Keep the Arduino sketch and all its header files together in the same sketch folder:

```text
ESP32-BLE-Air-Mouse/
├── README.md
└── air_mouse/
    ├── air_mouse.ino
    ├── buttons.h
    ├── config.h
    ├── motion.h
    └── ui.h
```

The folder containing the sketch should have the same name as the `.ino` file (for example, `air_mouse/air_mouse.ino`). The old Wi-Fi/TCP modules (`net.h` and `protocol.h`) are not needed for this BLE version.

## Software requirements

1. [Arduino IDE](https://www.arduino.cc/en/software)
2. Espressif ESP32 board support installed through the Arduino Boards Manager
3. The following Arduino libraries:

   - **ESP32 BLE Mouse** (`BleMouse.h`)
   - **Adafruit MPU6050**
   - **Adafruit Unified Sensor**
   - **Adafruit GFX Library**
   - **Adafruit SH110X**

`Wire`, `SPI`, and `Preferences` are provided by the ESP32 Arduino core. Install any additional dependencies requested by the Arduino Library Manager.

## Build and upload

1. Wire the IMU, OLED, and buttons as shown above.
2. Open `air_mouse.ino` in Arduino IDE.
3. Install the required libraries listed above.
4. Select the ESP32 board matching your LOLIN32 Lite and choose the correct serial port.
5. Upload the sketch.
6. Open Serial Monitor at **115200 baud** to view startup messages and debug output.
7. Turn on Bluetooth on your host device, find **AirMouse**, and connect/pair with it.

After connecting, the host should treat the ESP32 as a BLE mouse. Exact Bluetooth menu wording varies by operating system.

## Controls

| Control | Behavior |
|---|---|
| Move/rotate the device | Move the cursor using hand rotation |
| GPIO 32 button | Left click |
| GPIO 27 button | Right click |
| Hold GPIO 26 button and tilt | Scroll up/down |
| Send `s` in Serial Monitor | Cycle sensitivity: LOW → MED → HIGH |
| Send `c` in Serial Monitor | Request gyro recalibration; keep the device still |
| Send `t` in Serial Monitor | Toggle the horizontal tilt direction |

The default sensitivity is **MED**. Calibration works best when the device is resting completely still. A successful calibration saves the gyro bias values to ESP32 NVS for later startups.

## How cursor movement works

The firmware uses **gyroscope rotation rate**, rather than absolute pointing direction, to calculate cursor movement. The configured axes are:

- **Yaw (gyro Z):** primary horizontal cursor movement.
- **Roll (gyro Y):** secondary horizontal movement at lower sensitivity.
- **Pitch (gyro X):** vertical cursor movement.

When scroll mode is active, cursor movement is suspended and vertical tilting is converted into scroll events. Sensitivity, filtering, dead-zone, acceleration, and axis direction settings can be adjusted in `config.h`.

## OLED information

The display includes a startup animation and a live status screen. Depending on the device state, it shows the host connection/waiting state, current sensitivity, cursor or scroll mode, IMU status, and a ball that reflects detected motion. `NO IMU!` indicates that the sensor is offline; check its power, I²C wiring, and address.

## Troubleshooting

- **AirMouse does not appear in Bluetooth settings:** restart the ESP32, make sure BLE is enabled on the host, and check Serial Monitor for the BLE startup message.
- **The host connects but the cursor does not move:** check that the IMU is detected and the display does not show `NO IMU!`; then try recalibration while the device is still.
- **Cursor drifts or feels too sensitive:** recalibrate on a stable surface and try another sensitivity level. You can also tune the motion constants in `config.h`.
- **OLED remains blank:** check the SPI pin mapping, display controller (SH1106), power, and ground.
- **Buttons do not respond:** confirm each button connects its GPIO to GND when pressed and that the pins match `config.h`.

**Calibration note:** the current left-plus-right long-press shortcut has a timer issue that may retrigger calibration while both buttons remain held. For now, use the Serial Monitor command `c` and keep the device still.

## Project notes

- Firmware version: **1.0.0**
- BLE device name: **AirMouse**
- Serial baud rate: **115200**
- The firmware is designed for BLE HID-compatible hosts, including supported Windows, Android, macOS, and Linux devices. Compatibility can vary with the host's Bluetooth adapter and operating system.

---

Built with an ESP32, motion sensing, and BLE HID to make everyday cursor control wireless and gesture-driven.
