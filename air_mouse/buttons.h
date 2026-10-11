// =============================================================================
//  buttons.h - Time-based debounce with one-shot press/release edge events
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"

class DebouncedButton {
 public:
  explicit DebouncedButton(int8_t pin) : pin_(pin) {}

  void begin() {
    if (pin_ < 0) return;
    pinMode(pin_, INPUT_PULLUP);          // pressed = LOW
    raw_ = stable_ = (digitalRead(pin_) == LOW);
    lastChangeMs_ = millis();
  }

  // Call frequently (every loop pass).
  void update(uint32_t nowMs) {
    if (pin_ < 0) return;
    bool r = (digitalRead(pin_) == LOW);
    if (r != raw_) {                      // any raw change restarts the timer
      raw_ = r;
      lastChangeMs_ = nowMs;
    }
    if (raw_ != stable_ && (uint32_t)(nowMs - lastChangeMs_) >= DEBOUNCE_MS) {
      stable_ = raw_;                     // accepted: held steady for DEBOUNCE_MS
      if (stable_) pressEdge_ = true; else releaseEdge_ = true;
    }
  }

  bool isDown() const { return stable_; }

  // One-shot events: return true once per physical press / release.
  bool takePress()   { bool e = pressEdge_;   pressEdge_   = false; return e; }
  bool takeRelease() { bool e = releaseEdge_; releaseEdge_ = false; return e; }

 private:
  uint8_t  pin_;
  bool     raw_ = false, stable_ = false;
  bool     pressEdge_ = false, releaseEdge_ = false;
  uint32_t lastChangeMs_ = 0;
};
