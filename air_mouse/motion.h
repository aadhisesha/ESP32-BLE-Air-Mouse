// =============================================================================
//  motion.h - MPU6500 / MPU6050 acquisition, zero-point NVS calibration,
//             filtering, and conversion of hand rotation into cursor / scroll deltas.
//
//  Approach (gyro-rate pointing):
//    cursor velocity  ~  angular rate (deg/s) after bias removal, low-pass
//    filtering and a dead zone. Pixels = rate * dt * px_per_degree.
//  Zero-point calibration:
//    Stored persistently in ESP32 NVS (non-volatile flash) so the device
//    never starts up with corrupted or uncalibrated offsets.
// =============================================================================
#pragma once
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <math.h>
#include "config.h"

enum class ImuState : uint8_t { Offline = 0, Calibrating = 1, Running = 2 };

class MotionProcessor {
 public:
  bool begin() {
    lastSampleUs_ = micros();
    loadBiasFromNvs();
    recoverI2C();
    if (initSensor()) {
      state_ = ImuState::Running;
      return true;
    }
    state_ = ImuState::Offline;
    lastRetryMs_ = millis();
    return false;
  }

  void setSensitivity(uint8_t idx) { sens_ = (idx < SENS_LEVELS) ? idx : 0; }

  // Start (or restart) gyro bias calibration. Device must be held still.
  void requestCalibration() {
    if (state_ == ImuState::Offline) return;
    startCalibration();
  }

  // Call every loop pass; internally rate-limited to SAMPLE_PERIOD_US.
  void update(uint32_t nowUs, bool scrollMode) {
    if (state_ == ImuState::Offline) {
      maybeRetry();
      relaxBall();
      return;
    }
    if ((uint32_t)(nowUs - lastSampleUs_) < SAMPLE_PERIOD_US) return;
    float dt = (float)(uint32_t)(nowUs - lastSampleUs_) * 1e-6f;
    lastSampleUs_ = nowUs;
    if (dt > 0.05f) dt = 0.05f;

    float g[3], a[3];
    if (!readSample(g, a)) {
      if (++badCount_ >= BAD_LIMIT) goOffline();
      return;
    }
    badCount_ = 0;

    if (state_ == ImuState::Calibrating) accumulateCal(g);
    else process(g, dt, scrollMode);
  }

  // ---- outputs (integers; fractional remainders stay in the accumulators) ----
  bool takeDelta(int& dx, int& dy) {
    dx = (int)dxAcc_;  dy = (int)dyAcc_;
    dxAcc_ -= dx;      dyAcc_ -= dy;
    dx = constrain(dx, -MAX_STEP_PX, MAX_STEP_PX);
    dy = constrain(dy, -MAX_STEP_PX, MAX_STEP_PX);
    return dx != 0 || dy != 0;
  }
  bool takeScroll(int& d) {
    d = (int)scrollAcc_;
    scrollAcc_ -= d;
    d = constrain(d, -MAX_SCROLL_STEP, MAX_SCROLL_STEP);
    return d != 0;
  }
  void discardOutputs() { dxAcc_ = dyAcc_ = scrollAcc_ = 0; }

  // ---- state for UI / debug ----
  ImuState state() const { return state_; }
  bool     running() const { return state_ == ImuState::Running; }
  uint8_t  calPercent() const { return (uint8_t)((uint32_t)calN_ * 100 / CAL_SAMPLES); }
  bool     calWarn() const { return calWarn_; }
  float    ballX() const { return ballX_; }   // -1..1, +right
  float    ballY() const { return ballY_; }   // -1..1, +down
  float    dbgH() const { return dbgH_; }
  float    dbgV() const { return dbgV_; }
  float    bias(uint8_t i) const { return (i < 3) ? bias_[i] : 0.0f; }
  int8_t   tiltSign() const { return tiltSign_; }
  void     toggleTiltSign() {
    tiltSign_ = -tiltSign_;
    Serial.printf("[TILT] Sign toggled to %+d\n", tiltSign_);
  }

  // Manually force and save zero-point bias values (e.g. from serial command)
  void setBias(float bx, float by, float bz) {
    bias_[0] = bx; bias_[1] = by; bias_[2] = bz;
    for (uint8_t i = 0; i < 3; i++) filt_[i] = 0;
    saveBiasToNvs();
  }

 private:
  static constexpr float RAD2DEG = 57.29577951f;

  int8_t   tiltSign_ = TILT_SIGN;

  Adafruit_MPU6050 mpu_;
  ImuState state_ = ImuState::Offline;
  uint8_t  sens_  = DEFAULT_SENS_INDEX;
  uint32_t lastSampleUs_ = 0, lastRetryMs_ = 0;

  float bias_[3] = {-1.26f, 4.78f, 2.40f};
  float filt_[3] = {0, 0, 0};
  float dxAcc_ = 0, dyAcc_ = 0, scrollAcc_ = 0;
  float ballX_ = 0, ballY_ = 0, dbgH_ = 0, dbgV_ = 0;

  // calibration accumulators (double precision to prevent cancellation)
  double   calSum_[3] = {0, 0, 0}, calSumSq_[3] = {0, 0, 0};
  uint16_t calN_ = 0, calSkip_ = 0;
  uint8_t  calAttempt_ = 0;
  bool     calWarn_ = false;

  // health & drift tracking
  uint16_t stillCount_ = 0;
  uint8_t  badCount_ = 0;

  // ---------------------------------------------------------------- NVS --
  void loadBiasFromNvs() {
    Preferences prefs;
    prefs.begin("airmouse", true);
    if (prefs.getBool("cal_ok", false)) {
      bias_[0] = prefs.getFloat("bx", -1.26f);
      bias_[1] = prefs.getFloat("by", 4.78f);
      bias_[2] = prefs.getFloat("bz", 2.40f);
      Serial.printf("[NVS] Loaded saved zero-point bias: X=%.2f Y=%.2f Z=%.2f\n",
                    bias_[0], bias_[1], bias_[2]);
    } else {
      bias_[0] = -1.26f;
      bias_[1] = 4.78f;
      bias_[2] = 2.40f;
      Serial.printf("[NVS] Default hardware zero-point applied: X=%.2f Y=%.2f Z=%.2f\n",
                    bias_[0], bias_[1], bias_[2]);
    }
    prefs.end();
  }

  void saveBiasToNvs() {
    Preferences prefs;
    prefs.begin("airmouse", false);
    prefs.putFloat("bx", bias_[0]);
    prefs.putFloat("by", bias_[1]);
    prefs.putFloat("bz", bias_[2]);
    prefs.putBool("cal_ok", true);
    prefs.end();
    Serial.printf("[NVS] Saved zero-point to NVS flash: X=%.2f Y=%.2f Z=%.2f\n",
                  bias_[0], bias_[1], bias_[2]);
  }

  // ---------------------------------------------------------------- bus recovery --
  void recoverI2C() {
    Wire.end();
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
    for (int i = 0; i < 9; i++) {
      digitalWrite(PIN_I2C_SCL, LOW);
      delayMicroseconds(10);
      digitalWrite(PIN_I2C_SCL, HIGH);
      delayMicroseconds(10);
    }
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_CLOCK_HZ);
  }

  // ---------------------------------------------------------------- sensor --
  bool initSensor() {
    const uint8_t addrs[2] = { MPU_ADDR, MPU_ADDR_ALT };
    for (uint8_t addr : addrs) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() != 0) continue;
      if (mpu_.begin(addr, &Wire)) {
        mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
        mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.printf("[IMU] MPU6050 ready at 0x%02X\n", addr);
        return true;
      }
      Serial.printf("[IMU] 0x%02X ACKs but not recognised\n", addr);
    }
    Serial.println("[IMU] MPU6050 NOT FOUND - check wiring: SDA=GPIO23 SCL=GPIO22 3V3 GND");
    return false;
  }

  void maybeRetry() {
    uint32_t now = millis();
    if (now - lastRetryMs_ < IMU_RETRY_MS) return;
    lastRetryMs_ = now;
    recoverI2C();
    if (initSensor()) {
      lastSampleUs_ = micros();
      state_ = ImuState::Running;
      badCount_ = 0;
      dbgH_ = dbgV_ = 0.0f;
    }
  }

  void goOffline() {
    Serial.println("[IMU] sensor stopped responding / invalid data -> OFFLINE");
    state_ = ImuState::Offline;
    dbgH_ = dbgV_ = 0.0f;
    for (uint8_t i = 0; i < 3; i++) filt_[i] = 0.0f;
    discardOutputs();
    lastRetryMs_ = millis();
    recoverI2C();
  }

  // Reads one sample. Returns false ONLY on actual read failure or non-finite values.
  bool readSample(float g[3], float a[3]) {
    sensors_event_t ea, eg, et;
    if (!mpu_.getEvent(&ea, &eg, &et)) return false;
    g[0] = eg.gyro.x * RAD2DEG;  g[1] = eg.gyro.y * RAD2DEG;  g[2] = eg.gyro.z * RAD2DEG;
    a[0] = ea.acceleration.x;    a[1] = ea.acceleration.y;    a[2] = ea.acceleration.z;

    for (uint8_t i = 0; i < 3; i++) {
      if (!isfinite(g[i]) || !isfinite(a[i])) return false;
      if (fabsf(g[i]) > GYRO_VALID_MAX_DPS || fabsf(a[i]) > ACCEL_VALID_MAX) return false;
    }
    return true;
  }

  // ----------------------------------------------------------- calibration --
  void startCalibration() {
    state_ = ImuState::Calibrating;
    resetCalAccumulators();
    calAttempt_ = 0;
    calWarn_ = false;
    discardOutputs();
    Serial.println("[CAL] Keep the device STILL on a surface...");
  }

  void resetCalAccumulators() {
    for (uint8_t i = 0; i < 3; i++) calSum_[i] = calSumSq_[i] = 0.0;
    calN_ = 0;
    calSkip_ = CAL_SETTLE_SAMPLES;
  }

  void accumulateCal(const float g[3]) {
    if (calSkip_ > 0) { calSkip_--; return; }
    for (uint8_t i = 0; i < 3; i++) {
      calSum_[i] += (double)g[i];
      calSumSq_[i] += (double)g[i] * (double)g[i];
    }
    if (++calN_ < CAL_SAMPLES) return;

    double mean[3];
    bool still = true, plausible = true;
    for (uint8_t i = 0; i < 3; i++) {
      mean[i] = calSum_[i] / (double)CAL_SAMPLES;
      double var = (calSumSq_[i] / (double)CAL_SAMPLES) - (mean[i] * mean[i]);
      double sd = sqrt(var > 0.0 ? var : 0.0);
      if (sd > (double)CAL_MAX_STD_DPS) still = false;
      if (fabs(mean[i]) > (double)CAL_MAX_BIAS_DPS) plausible = false;
    }

    Serial.printf("[CAL] X: mean=%.1f | Y: mean=%.1f | Z: mean=%.1f (still=%d, plausible=%d)\n",
                  mean[0], mean[1], mean[2], still, plausible);

    resetCalAccumulators();

    if (still && plausible) {
      float fMean[3] = { (float)mean[0], (float)mean[1], (float)mean[2] };
      finishCal(fMean, false);
      saveBiasToNvs();
    } else if (++calAttempt_ >= CAL_MAX_ATTEMPTS) {
      // NEVER overwrite the bias with hand-motion data! Keep previous known good bias.
      state_ = ImuState::Running;
      calWarn_ = true;
      discardOutputs();
      Serial.println("[CAL] Motion detected: calibration aborted to preserve valid zero-point!");
    } else {
      Serial.printf("[CAL] Motion detected (attempt %d/%d) - retrying, keep STILL...\n",
                    calAttempt_, CAL_MAX_ATTEMPTS);
    }
  }

  void finishCal(const float mean[3], bool warn) {
    for (uint8_t i = 0; i < 3; i++) { bias_[i] = mean[i]; filt_[i] = 0; }
    calWarn_ = warn;
    stillCount_ = 0;
    discardOutputs();
    state_ = ImuState::Running;
    Serial.printf("[CAL] DONE! Zero-point locked: X=%.2f Y=%.2f Z=%.2f\n",
                  bias_[0], bias_[1], bias_[2]);
  }

  // ------------------------------------------------------------ processing --
  static float deadzone(float v, float dz) {
    float a = fabsf(v);
    return (a <= dz) ? 0.0f : copysignf(a - dz, v);
  }

  void process(const float g[3], float dt, bool scrollMode) {
    float r[3];
    for (uint8_t i = 0; i < 3; i++) {
      r[i] = g[i] - bias_[i];
      r[i] = constrain(r[i], -MAX_RATE_DPS, MAX_RATE_DPS);
      filt_[i] += GYRO_LPF_ALPHA * (r[i] - filt_[i]);
    }

    // Continuous zero-velocity update: tracks slow thermal drift when device is motionless
    bool isStill = (fabsf(r[0]) < STILL_THRESHOLD_DPS) &&
                   (fabsf(r[1]) < STILL_THRESHOLD_DPS) &&
                   (fabsf(r[2]) < STILL_THRESHOLD_DPS);
    if (isStill) {
      if (++stillCount_ >= STILL_SAMPLES) {
        for (uint8_t i = 0; i < 3; i++) {
          bias_[i] += BIAS_ADAPT_RATE * (g[i] - bias_[i]);
        }
      }
    } else {
      stillCount_ = 0;
    }

    float h_move = filt_[HORIZ_AXIS] * HORIZ_SIGN;    // +: cursor right (yaw sweep)
    float h_tilt = filt_[TILT_AXIS]  * tiltSign_;     // +: cursor right (roll tilt)
    float v      = filt_[VERT_AXIS]  * VERT_SIGN;     // +: cursor down (pitch tilt)
    float up     = -v * (SCROLL_INVERT ? -1.0f : 1.0f);  // +: scroll up

    float dz_move = deadzone(h_move, DEADZONE_DPS);
    float dz_tilt = deadzone(h_tilt, DEADZONE_DPS);
    float dz_v    = deadzone(v, DEADZONE_DPS);

    // Horizontal combined rate: sweep movements at full speed, tilting at lower sensitivity
    float h_rate = dz_move + dz_tilt * TILT_RATIO;
    dbgH_ = h_rate; dbgV_ = dz_v;

    float tx, ty;
    if (scrollMode) {
      dxAcc_ = dyAcc_ = 0;                        // cursor frozen while scrolling
      scrollAcc_ += deadzone(up, DEADZONE_SCROLL_DPS) * SCROLL_LINES_PER_DEG * dt;
      tx = 0;
      ty = -up / BALL_FULL_SCALE_DPS;
    } else {
      scrollAcc_ = 0;

      // Anti-creep leak: if resting inside deadzone, decay any sub-pixel fractional remainders
      if (fabsf(h_rate) < 0.001f) dxAcc_ *= 0.75f;
      if (fabsf(dz_v)   < 0.001f) dyAcc_ *= 0.75f;

      // Dynamic mouse acceleration: enables fast wrist flicks to reach screen edges easily
      float speed = sqrtf(h_rate * h_rate + dz_v * dz_v);
      float accel = 1.0f;
      if (speed > ACCEL_THRESHOLD_DPS) {
        accel = 1.0f + (speed - ACCEL_THRESHOLD_DPS) * ACCEL_GAIN;
        if (accel > MAX_ACCEL_FACTOR) accel = MAX_ACCEL_FACTOR;
      }

      const float k = SENS_PX_PER_DEG[sens_] * accel * dt;
      dxAcc_ += h_rate * k;
      dyAcc_ += dz_v * k;

      // Ball on OLED reflects both movements and tilting directly so user gets immediate visual feedback
      tx = (dz_move + dz_tilt) / BALL_FULL_SCALE_DPS;
      ty = dz_v / BALL_FULL_SCALE_DPS;
    }
    smoothBall(tx, ty);
  }

  void smoothBall(float tx, float ty) {
    ballX_ += 0.25f * (constrain(tx, -1.0f, 1.0f) - ballX_);
    ballY_ += 0.25f * (constrain(ty, -1.0f, 1.0f) - ballY_);
  }
  void relaxBall() { smoothBall(0, 0); }
};
