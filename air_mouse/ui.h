// =============================================================================
//  ui.h - 1.3" 7-Pin Hardware SPI SH1106 Portrait (64x128) OLED UI
//         Includes startup animation: Air Mouse logo + Team Creators
// =============================================================================
#pragma once
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <math.h>
#include "config.h"
#include "motion.h"

struct UiModel {
  bool     apOk = false;
  bool     clientConnected = false;
  uint8_t  sensIdx = DEFAULT_SENS_INDEX;
  ImuState imu = ImuState::Offline;
  uint8_t  calPercent = 0;
  bool     calWarn = false;
  bool     scrollMode = false;
  float    ballX = 0, ballY = 0;
  const char* ip = "";
};

class Ui {
 public:
  Ui() : d_(OLED_W, OLED_H, &SPI, PIN_OLED_DC, PIN_OLED_RESET, PIN_OLED_CS, 1000000UL) {}

  bool ok() const { return ok_; }

  bool begin() {
    SPI.begin(PIN_OLED_CLK, -1, PIN_OLED_MOSI, PIN_OLED_CS);
    ok_ = d_.begin(0, true);
    if (!ok_) {
      Serial.println("[OLED] SPI SH1106 init failed - continuing without display");
      return false;
    }
    Serial.println("[OLED] SPI SH1106 ready (Portrait Mode 64x128)");
    d_.setRotation(3); // Portrait: left side = top, right side = bottom
    playStartupAnimation();
    requestRedraw();
    return true;
  }

  // Force a redraw on the next update() regardless of the frame limiter.
  void requestRedraw() { force_ = true; }

  void update(const UiModel& m, uint32_t nowMs) {
    if (!ok_) return;

    Snap s = snapshot(m, nowMs);
    if (!force_ && same(s, last_)) return;                       // nothing visible changed
    if (!force_ && (uint32_t)(nowMs - lastRenderMs_) < MIN_FRAME_MS) return;

    render(m, s);
    d_.display();
    last_ = s;
    lastRenderMs_ = nowMs;
    force_ = false;
  }

 private:
  static constexpr uint32_t MIN_FRAME_MS = 50;      // max ~20 fps
  static constexpr int      CX = 32, CY = 86, RING_R = 15, BALL_R = 3, BALL_TRAVEL = 11;

  struct Snap {
    bool ap, client, warn, scroll, blink;
    uint8_t sens, imu, cal;
    int8_t bx, by;
  };

  Adafruit_SH1106G d_;
  bool     ok_ = false, force_ = true;
  uint32_t lastRenderMs_ = 0;
  Snap     last_ = {};

  static bool same(const Snap& a, const Snap& b) {
    return a.ap == b.ap && a.client == b.client && a.warn == b.warn &&
           a.scroll == b.scroll && a.blink == b.blink && a.sens == b.sens &&
           a.imu == b.imu && a.cal == b.cal && a.bx == b.bx && a.by == b.by;
  }

  Snap snapshot(const UiModel& m, uint32_t nowMs) const {
    float x = m.ballX, y = m.ballY;
    float mag = sqrtf(x * x + y * y);
    if (mag > 1.0f) { x /= mag; y /= mag; }          // keep ball inside the ring
    Snap s;
    s.ap = m.apOk;  s.client = m.clientConnected;  s.warn = m.calWarn;
    s.scroll = m.scrollMode;
    s.blink = (!m.clientConnected) ? ((nowMs / 500) & 1) : false;
    s.sens = m.sensIdx;  s.imu = (uint8_t)m.imu;
    s.cal = (m.imu == ImuState::Calibrating) ? (m.calPercent / 5) : 0;   // 5% steps
    s.bx = (int8_t)lroundf(x * BALL_TRAVEL);
    s.by = (int8_t)lroundf(y * BALL_TRAVEL);
    return s;
  }

  void drawMouseIcon(int x, int y, int waveRadius) {
    // Mouse outer body (smooth rounded rectangle 20x28, radius 6)
    d_.drawRoundRect(x, y, 20, 28, 6, SH110X_WHITE);
    // Button divider line
    d_.drawFastVLine(x + 10, y + 2, 10, SH110X_WHITE);
    // Scroll wheel (filled 4x7)
    d_.fillRoundRect(x + 8, y + 4, 5, 7, 2, SH110X_WHITE);
    // Palm divider line
    d_.drawFastHLine(x + 3, y + 13, 14, SH110X_WHITE);

    // Wireless radiation waves above mouse
    if (waveRadius >= 4) {
      d_.drawCircle(x + 10, y - 3, 4, SH110X_WHITE);
      d_.fillRect(x, y - 3, 20, 5, SH110X_BLACK); // mask lower half
    }
    if (waveRadius >= 8) {
      d_.drawCircle(x + 10, y - 3, 8, SH110X_WHITE);
      d_.fillRect(x, y - 3, 20, 9, SH110X_BLACK); // mask lower half
    }
    if (waveRadius >= 12) {
      d_.drawCircle(x + 10, y - 3, 12, SH110X_WHITE);
      d_.fillRect(x, y - 3, 20, 13, SH110X_BLACK); // mask lower half
    }
  }

  void showNameBadge(const char* name) {
    d_.clearDisplay();
    d_.setTextSize(1);
    d_.setTextColor(SH110X_WHITE);
    d_.setCursor(11, 20);
    d_.print("CREATOR");
    d_.drawFastHLine(8, 32, 48, SH110X_WHITE);

    // Center badge with name
    d_.drawRoundRect(2, 48, 60, 32, 5, SH110X_WHITE);
    int len = strlen(name);
    int textX = (64 - len * 6) / 2;
    d_.setCursor(textX, 60);
    d_.print(name);
    d_.display();
  }

  void playStartupAnimation() {
    d_.setRotation(3); // Portrait: left side = top, right side = bottom

    // 1. Air Mouse Title + Mouse Icon with expanding wireless signal waves
    for (int wave = 4; wave <= 14; wave += 2) {
      d_.clearDisplay();
      d_.setTextSize(1);
      d_.setTextColor(SH110X_WHITE);
      d_.setCursor(5, 14);
      d_.print("AIR MOUSE");
      d_.drawFastHLine(4, 26, 56, SH110X_WHITE);
      drawMouseIcon(22, 52, wave);
      d_.display();
      delay(90);
    }
    delay(1000);

    // 2. Name: Roshan
    showNameBadge("Roshan");
    delay(1000);

    // 3. Name: Aadhi
    showNameBadge("Aadhi");
    delay(1000);

    // 4. Name: Muhammed
    showNameBadge("Muhammed");
    delay(1000);

    // 5. Team Summary
    d_.clearDisplay();
    d_.setTextSize(1);
    d_.setTextColor(SH110X_WHITE);
    d_.setCursor(5, 8);   d_.print("AIR MOUSE");
    d_.drawFastHLine(0, 20, 64, SH110X_WHITE);
    d_.setCursor(6, 32);  d_.print("- Roshan");
    d_.setCursor(6, 52);  d_.print("- Aadhi");
    d_.setCursor(6, 72);  d_.print("- Muhammed");
    d_.drawFastHLine(0, 92, 64, SH110X_WHITE);
    d_.setCursor(11, 106); d_.print("READY!");
    d_.display();
    delay(1200);
  }

  void render(const UiModel& m, const Snap& s) {
    d_.clearDisplay();
    d_.setTextSize(1);
    d_.setTextColor(SH110X_WHITE);

    // Header
    d_.setCursor(5, 2);   d_.print("AIR MOUSE");
    d_.drawFastHLine(0, 12, 64, SH110X_WHITE);

    // BLE status
    d_.setCursor(2, 16);  d_.print("BLE:");
    d_.setCursor(28, 16); d_.print("ON");

    // Host status
    d_.setCursor(2, 26);  d_.print("HOST:");
    d_.setCursor(2, 36);
    if (m.clientConnected) {
      d_.print("PAIRED");
    } else {
      d_.print(s.blink ? "WAITING." : "WAITING");
    }

    // Speed setting
    d_.setCursor(2, 48);  d_.print("SPD:");
    d_.setCursor(28, 48); d_.print(SENS_NAMES[m.sensIdx % SENS_LEVELS]);

    // Mode
    d_.setCursor(2, 58);
    if (m.imu == ImuState::Offline) {
      d_.print("NO IMU!");
    } else if (m.imu == ImuState::Calibrating) {
      d_.print("STILL..");
    } else {
      d_.print(m.scrollMode ? "SCROLL" : "CURSOR");
      if (m.calWarn) d_.print("!");
    }

    // Motion target box and ring (Center CX=32, CY=86)
    d_.drawRect(CX - 20, CY - 18, 40, 36, SH110X_WHITE);
    d_.drawCircle(CX, CY, RING_R, SH110X_WHITE);
    d_.drawPixel(CX, CY, SH110X_WHITE);
    d_.fillCircle(CX + s.bx, CY + s.by, BALL_R, SH110X_WHITE);

    // Bottom bar
    if (m.imu == ImuState::Calibrating) {
      d_.drawRect(2, 115, 60, 9, SH110X_WHITE);
      d_.fillRect(4, 117, (56 * m.calPercent) / 100, 5, SH110X_WHITE);
    } else {
      d_.drawFastHLine(0, 114, 64, SH110X_WHITE);
      d_.setCursor(4, 118);
      d_.print("AirMouse");
    }
  }
};
