// =============================================================================
//  config.h  -  All user-adjustable settings for the ESP32 Wi-Fi Air Mouse
// =============================================================================
#pragma once
#include <Arduino.h>

#define FW_VERSION       "1.0.0"
#define PROTOCOL_VERSION 1          // must match receiver/windows_receiver.py

// I2C bus for MPU6500 / MPU6050
constexpr uint8_t PIN_I2C_SDA    = 23;   // MPU SDA
constexpr uint8_t PIN_I2C_SCL    = 22;   // MPU SCL

// 7-Pin SPI OLED pins (SSD1306)
constexpr int8_t PIN_OLED_MOSI   = 19;   // D1 on OLED
constexpr int8_t PIN_OLED_CLK    = 18;   // D0 on OLED
constexpr int8_t PIN_OLED_DC     = 16;   // DC on OLED
constexpr int8_t PIN_OLED_RESET  = 4;    // RES on OLED
constexpr int8_t PIN_OLED_CS     = 17;   // CS on OLED

constexpr uint8_t PIN_BTN_LEFT   = 32;   // Left Click (GPIO 32)
constexpr uint8_t PIN_BTN_RIGHT  = 27;   // Right Click (GPIO 27)
constexpr uint8_t PIN_BTN_SCROLL = 26;   // Hold-to-scroll gesture (GPIO 26)
constexpr int8_t  PIN_BTN_SENS   = -1;   // Disabled (3-button setup)

constexpr uint32_t I2C_CLOCK_HZ  = 100000;   // 100 kHz standard mode for breadboard jumper stability
constexpr uint8_t  MPU_ADDR      = 0x68;  // AD0 low / unconnected
constexpr uint8_t  MPU_ADDR_ALT  = 0x69;  // fallback probe only (AD0 high)
constexpr uint8_t  OLED_W        = 128;
constexpr uint8_t  OLED_H        = 64;

// -----------------------------------------------------------------------------
// 2. WI-FI ACCESS POINT / TCP SERVER  (settings section)
// -----------------------------------------------------------------------------
#define AP_SSID          "AirMouse"
#define AP_PASSWORD      "airmouse123"   // 8..63 chars, or "" for an open network
constexpr uint8_t  AP_CHANNEL        = 6;
constexpr uint8_t  AP_MAX_CLIENTS    = 2;          // Wi-Fi stations (PC + spare)
constexpr uint16_t TCP_PORT          = 4242;       // ESP32 = server, PC = client
// The AP IP is the ESP32 default 192.168.4.1 (verified at runtime and shown on OLED).

constexpr uint32_t CLIENT_TIMEOUT_MS = 5000;   // drop client if no valid message from PC
constexpr uint32_t HEARTBEAT_MS      = 1000;   // device -> PC heartbeat period
constexpr uint32_t SEND_PERIOD_MS    = 20;     // movement/scroll batching (50 Hz)
constexpr int      MAX_MOVE_PER_MSG  = 127;    // px, sanity clamp per message
constexpr int      MAX_SCROLL_PER_MSG= 20;     // lines

// -----------------------------------------------------------------------------
// 3. BUTTONS
// -----------------------------------------------------------------------------
constexpr uint32_t DEBOUNCE_MS = 25;

// -----------------------------------------------------------------------------
// 4. MOTION PROCESSING
// -----------------------------------------------------------------------------
constexpr uint32_t SAMPLE_PERIOD_US = 10000;   // 100 Hz IMU sampling

// Sensitivity levels: pixels of cursor travel per degree of rotation.
constexpr uint8_t SENS_LEVELS = 3;
constexpr float   SENS_PX_PER_DEG[SENS_LEVELS] = { 12.0f, 20.0f, 32.0f };
constexpr const char* SENS_NAMES[SENS_LEVELS]  = { "LOW", "MED", "HIGH" };
constexpr uint8_t DEFAULT_SENS_INDEX = 1;      // MED (20.0 px/deg) - smooth & controllable

// Gentle mouse acceleration (fine precision at normal speeds, modest boost on fast flicks)
constexpr float   ACCEL_THRESHOLD_DPS = 25.0f; // kicks in only on fast intentional swipes
constexpr float   ACCEL_GAIN          = 0.015f; // gentle acceleration slope
constexpr float   MAX_ACCEL_FACTOR    = 1.8f;  // capped at 1.8x

// Filtering / dead zone
constexpr float GYRO_LPF_ALPHA   = 0.28f;  // smoother filtering to eliminate trembling and jitter
constexpr float DEADZONE_DPS     = 1.6f;   // cursor dead zone (1.6 dps silences hand tremor completely)
constexpr float MAX_RATE_DPS     = 400.0f; // clamp per-axis rate (prevents jumps)
constexpr int   MAX_STEP_PX      = 60;     // clamp per-sample cursor step

// Scroll mode
constexpr float DEADZONE_SCROLL_DPS  = 4.0f;
constexpr float SCROLL_LINES_PER_DEG = 0.15f;  // lines per degree of tilt
constexpr bool  SCROLL_INVERT        = false;  // true = tilt-up scrolls down
constexpr int   MAX_SCROLL_STEP      = 10;

// Axis mapping:
//   yaw   (turn left/right)  = gyro Z  -> horizontal sweep (fast cursor motion)
//   roll  (tilt left/right)  = gyro Y  -> horizontal tilt (low-sensitivity fine aiming)
//   pitch (nose up/down)     = gyro X  -> vertical cursor
constexpr uint8_t HORIZ_AXIS = 2;    // Gyro Z (Yaw sweep)
constexpr int8_t  HORIZ_SIGN = -1;   // +1 / -1
constexpr uint8_t TILT_AXIS  = 1;    // Gyro Y (Roll tilt left/right)
constexpr int8_t  TILT_SIGN  = 1;    // +1 (tilting right moves cursor right)
constexpr float   TILT_RATIO = 0.35f; // Tilting has lower sensitivity than movements

constexpr uint8_t VERT_AXIS  = 0;    // Gyro X (Pitch nose up/down)
constexpr int8_t  VERT_SIGN  = -1;

// Start-up / manual calibration
constexpr uint16_t CAL_SETTLE_SAMPLES = 10;    // ignore first 0.1 s
constexpr uint16_t CAL_SAMPLES        = 80;    // 0.8 s of samples
constexpr float    CAL_MAX_STD_DPS    = 6.0f;  // generous tolerance so handheld calibration passes
constexpr float    CAL_MAX_BIAS_DPS   = 250.0f; // accepts MPU6500 hardware DC bias (up to +-250 dps)
constexpr uint8_t  CAL_MAX_ATTEMPTS   = 3;     // max attempts before falling back to safe bias

// Continuous drift compensation ("zero-velocity update")
constexpr float    STILL_THRESHOLD_DPS = 1.0f;
constexpr uint16_t STILL_SAMPLES       = 75;      // 0.75 s still before adapting
constexpr float    BIAS_ADAPT_RATE     = 0.005f;  // per sample

// Sensor sanity limits
constexpr float    GYRO_VALID_MAX_DPS  = 2000.0f;
constexpr float    ACCEL_VALID_MAX     = 200.0f;  // m/s^2 (range is +-4 g = 39)
constexpr uint16_t STUCK_LIMIT         = 100;     // identical samples => dead sensor
constexpr uint8_t  BAD_LIMIT           = 25;      // consecutive bad samples => offline
constexpr uint32_t IMU_RETRY_MS        = 3000;

// Ball animation
constexpr float BALL_FULL_SCALE_DPS = 50.0f;      // rate that pushes ball to the ring (50 dps = ring boundary)

// -----------------------------------------------------------------------------
// 5. DEBUG
// -----------------------------------------------------------------------------
// 0 = quiet, 1 = print processed rates/deltas at 5 Hz (Stage 3 tuning)
#define SERIAL_DEBUG_LEVEL 0
