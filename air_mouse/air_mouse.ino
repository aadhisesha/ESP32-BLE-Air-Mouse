// =============================================================================
//  ESP32 LOLIN32 Lite  -  Native Bluetooth Low Energy (BLE) HID Air Mouse
//  MPU6050/6500 on I2C (SDA=GPIO23, SCL=GPIO22)
//  1.3" SPI OLED (SH1106: D0=18, D1=19, RES=4, DC=16, CS=17)
//  3 Buttons (internal pull-ups, pressed = LOW):
//    GPIO 32: Left Click
//    GPIO 27: Right Click
//    GPIO 26: Hold-to-scroll gesture (tilt up/down in air while holding)
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <BleMouse.h>

#include "config.h"
#include "buttons.h"
#include "motion.h"
#include "ui.h"
#include <esp_mac.h>

// BLE Mouse device configuration (No python receiver or WiFi network needed!)
static BleMouse bleMouse("AirMouse", "Espressif", 100);

// Buttons
static DebouncedButton btnLeft(PIN_BTN_LEFT);     // 32
static DebouncedButton btnRight(PIN_BTN_RIGHT);   // 27
static DebouncedButton btnScroll(PIN_BTN_SCROLL); // 26

static MotionProcessor motion;
static Ui              ui;

static uint8_t sensIdx = DEFAULT_SENS_INDEX;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP32 BLE HID Air Mouse 1.0.0 ===");

  // Set clean unique Bluetooth MAC to force macOS to invalidate any broken GATT cache
  uint8_t baseMac[6];
  if (esp_read_mac(baseMac, ESP_MAC_WIFI_STA) == ESP_OK) {
    baseMac[5] ^= 0x09;
    esp_base_mac_addr_set(baseMac);
  }

  btnLeft.begin();
  btnRight.begin();
  btnScroll.begin();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(100);

  motion.setSensitivity(sensIdx);
  motion.begin();

  ui.begin();

  Serial.println("[BLE] Starting BLE HID Mouse advertising...");
  bleMouse.begin();
}

void loop() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();

  // ---- 1. Buttons Update ----
  btnLeft.update(nowMs);
  btnRight.update(nowMs);
  btnScroll.update(nowMs);

  const bool connected = bleMouse.isConnected();
  static bool lastConn = false;
  if (connected != lastConn) {
    lastConn = connected;
    Serial.printf("[BLE] Connection state changed: %s\n", connected ? "CONNECTED/PAIRED!" : "DISCONNECTED");
  }

  // Recalibrate chord: Hold Left + Right together for 2 seconds
  static uint32_t bothHeldStart = 0;
  if (btnLeft.isDown() && btnRight.isDown()) {
    if (bothHeldStart == 0) bothHeldStart = nowMs;
    if (nowMs - bothHeldStart >= 2000) {
      Serial.println("[CAL] Chord triggered (Left+Right held for 2s) -> CALIBRATING!");
      motion.requestCalibration();
      bothHeldStart = nowMs + 5000; // debounce re-trigger
    }
  } else {
    bothHeldStart = 0;
  }

  // ---- 2. Clicks (Direct HID Events) ----
  if (connected) {
    if (btnLeft.takePress()) {
      Serial.println("[BLE] Left Press -> HID");
      bleMouse.press(MOUSE_LEFT);
    }
    if (btnLeft.takeRelease()) {
      Serial.println("[BLE] Left Release -> HID");
      bleMouse.release(MOUSE_LEFT);
    }

    if (btnRight.takePress()) {
      Serial.println("[BLE] Right Press -> HID");
      bleMouse.press(MOUSE_RIGHT);
    }
    if (btnRight.takeRelease()) {
      Serial.println("[BLE] Right Release -> HID");
      bleMouse.release(MOUSE_RIGHT);
    }
  } else {
    btnLeft.takePress();   btnLeft.takeRelease();
    btnRight.takePress();  btnRight.takeRelease();
  }

  // ---- 3. Motion & Scroll Gesture Processing ----
  // Scroll gesture is active whenever the scroll button (GPIO 26) is held DOWN
  const bool scrollMode = btnScroll.isDown() && motion.running();
  motion.update(nowUs, scrollMode);

  int dx = 0, dy = 0, sc = 0;
  if (connected) {
    if (motion.takeDelta(dx, dy)) {
      // Normal cursor movement
      bleMouse.move((signed char)dx, (signed char)dy, 0);
    }
    if (motion.takeScroll(sc)) {
      // Gesture scroll: moving up/down in air while holding scroll button
      bleMouse.move(0, 0, (signed char)sc);
    }
  } else {
    motion.discardOutputs();
  }

  // Serial commands
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'c' || c == 'C') motion.requestCalibration();
    if (c == 's' || c == 'S') {
      sensIdx = (sensIdx + 1) % SENS_LEVELS;
      motion.setSensitivity(sensIdx);
      Serial.printf("[SENS] Changed to %s (%.1f px/deg)\n", SENS_NAMES[sensIdx], SENS_PX_PER_DEG[sensIdx]);
    }
    if (c == 't' || c == 'T') motion.toggleTiltSign();
  }

  static uint32_t lastPrintMs = 0;
  if (nowMs - lastPrintMs >= 200) {
    lastPrintMs = nowMs;
    Serial.printf("[DBG] H_rate:%+.2f V_rate:%+.2f | tiltSign:%+d | bias: X=%.1f Y=%.1f Z=%.1f | dx:%d dy:%d\n",
                  motion.dbgH(), motion.dbgV(), motion.tiltSign(),
                  motion.bias(0), motion.bias(1), motion.bias(2),
                  dx, dy);
  }

  // ---- 4. OLED Display Update ----
  UiModel m;
  m.apOk = true;
  m.clientConnected = connected;
  m.sensIdx = sensIdx;
  m.imu = motion.state();
  m.calPercent = motion.calPercent();
  m.calWarn = motion.calWarn();
  m.scrollMode = scrollMode;
  m.ballX = motion.ballX();
  m.ballY = motion.ballY();
  ui.update(m, nowMs);

  delay(1);
}
