#pragma once
#include <Wire.h>
#include "config.h"
#include "control.h"

class Mpu6050 {
public:
    bool begin() {
        if (!writeReg(0x6B, 0x00)) return false;   // wake
        delay(50);
        writeReg(0x19, 0x01);   // 500 Hz
        writeReg(0x1A, 0x02);   // DLPF ~98 Hz
        writeReg(0x1B, 0x10);   // gyro ±1000 dps
        writeReg(0x1C, 0x08);   // accel ±4 g
        present_ = true;
        return true;
    }

    void setRetryEnabled(bool on) { retryEnabled_ = on; }
    void setCoastEnabled(bool on) { coastEnabled_ = on; }
    void setDFilterAlpha(float a) { dFilterAlpha_ = constrain(a, 0.05f, 1.0f); }
    void setKalmanR(float r)      { filter_.setNoise(KF_Q_ANGLE, KF_Q_BIAS, r); }

    bool isPresent() const { return present_; }
    void markLost()        { present_ = false; }

    float pitchDeg()    const { return filter_.angle(); }
    float gyroRateDps() const { return gyroRateFilt_; }
    float gyroBiasDps() const { return filter_.bias(); }

    uint32_t totalReads() const { return totalReads_; }
    uint32_t errorReads() const { return errorReads_; }
    uint32_t retriesWon() const { return retriesWon_; }

    bool update(float dt) {
        int16_t ax, ay, az, gx, gy, gz;
        if (!readRaw(ax, ay, az, gx, gy, gz)) {
            if (++consecutiveFails_ > IMU_FAIL_THRESHOLD) present_ = false;
            if (coastEnabled_ && consecutiveFails_ <= IMU_COAST_CYCLES) {
                filter_.coastBy(gyroRateFilt_, dt);
                return true;
            }
            return false;
        }
        consecutiveFails_ = 0;

        const float axg = ax / IMU_ACCEL_LSB_PER_G;
        const float ayg = ay / IMU_ACCEL_LSB_PER_G;
        const float azg = az / IMU_ACCEL_LSB_PER_G;
        const float gyDps = (gy - gyroOffY_) / IMU_GYRO_LSB_PER_DPS;

        const float rate = gyDps - filter_.bias();
        gyroRateFilt_ += dFilterAlpha_ * (rate - gyroRateFilt_);

        const float accelAngle = atan2f(axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;
        const float accelMagSq = axg*axg + ayg*ayg + azg*azg;

        filter_.update(rate, accelAngle, dt, accelMagSq);
        return true;
    }

    void seedAngle() {
        int16_t ax, ay, az, gx, gy, gz;
        if (!readRaw(ax, ay, az, gx, gy, gz)) return;
        const float axg = ax / IMU_ACCEL_LSB_PER_G;
        const float ayg = ay / IMU_ACCEL_LSB_PER_G;
        const float azg = az / IMU_ACCEL_LSB_PER_G;
        const float p = atan2f(axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;
        filter_.reset(p);
        gyroRateFilt_ = 0.0f;
    }

    void calibrateGyro() {
        Serial.println("Calibrating gyro (keep still)...");
        for (int attempt = 0; attempt < 3; ++attempt) {
            long sx = 0, sy = 0, sz = 0;
            int16_t minGy = 32767, maxGy = -32768;
            int good = 0;
            for (int i = 0; i < IMU_CAL_SAMPLES; ++i) {
                int16_t ax, ay, az, gx, gy, gz;
                if (readRaw(ax, ay, az, gx, gy, gz)) {
                    sx += gx; sy += gy; sz += gz;
                    if (gy < minGy) minGy = gy;
                    if (gy > maxGy) maxGy = gy;
                    ++good;
                }
                delay(3);
            }
            if (good > IMU_CAL_SAMPLES / 2) {
                gyroOffX_ = float(sx) / good;
                gyroOffY_ = float(sy) / good;
                gyroOffZ_ = float(sz) / good;
                if ((maxGy - minGy) < int16_t(IMU_CAL_MOTION_LIMIT * IMU_GYRO_LSB_PER_DPS)) {
                    Serial.printf("Gyro offsets: %.1f %.1f %.1f (%d samples)\n",
                                  gyroOffX_, gyroOffY_, gyroOffZ_, good);
                    return;
                }
            }
        }
        Serial.printf("Gyro offsets (fallback): %.1f %.1f %.1f\n",
                      gyroOffX_, gyroOffY_, gyroOffZ_);
    }

    bool reinit() {
        recoverBus();
        consecutiveFails_ = 0;
        if (begin()) { seedAngle(); return true; }
        return false;
    }

    void recoverBus() {
        Wire.end();
        pinMode(PIN_SDA, INPUT_PULLUP);
        pinMode(PIN_SCL, OUTPUT);
        for (int i = 0; i < 9; ++i) {
            digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);
            digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
            if (digitalRead(PIN_SDA) == HIGH) break;
        }
        pinMode(PIN_SDA, OUTPUT);
        digitalWrite(PIN_SDA, LOW);  delayMicroseconds(5);
        digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
        digitalWrite(PIN_SDA, HIGH); delayMicroseconds(5);
        Wire.begin(PIN_SDA, PIN_SCL);
        Wire.setClock(IMU_I2C_CLOCK_HZ);
        Wire.setTimeOut(IMU_I2C_TIMEOUT_MS);
    }

private:
    bool  present_          = false;
    int   consecutiveFails_ = 0;
    float gyroOffX_ = 0, gyroOffY_ = 0, gyroOffZ_ = 0;
    float gyroRateFilt_     = 0.0f;
    float dFilterAlpha_     = DEF_D_FILTER;
    bool  retryEnabled_     = true;
    bool  coastEnabled_     = true;
    KalmanFilter1D filter_;
    uint32_t totalReads_ = 0, errorReads_ = 0, retriesWon_ = 0;

    bool writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(IMU_I2C_ADDR);
        Wire.write(reg); Wire.write(val);
        return Wire.endTransmission() == 0;
    }

    bool readRawOnce(int16_t& ax, int16_t& ay, int16_t& az,
                     int16_t& gx, int16_t& gy, int16_t& gz) {
        Wire.beginTransmission(IMU_I2C_ADDR);
        Wire.write(0x3B);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom(int(IMU_I2C_ADDR), 14) != 14) return false;
        uint8_t h, l;
        h = Wire.read(); l = Wire.read(); ax = int16_t((h << 8) | l);
        h = Wire.read(); l = Wire.read(); ay = int16_t((h << 8) | l);
        h = Wire.read(); l = Wire.read(); az = int16_t((h << 8) | l);
        Wire.read(); Wire.read();
        h = Wire.read(); l = Wire.read(); gx = int16_t((h << 8) | l);
        h = Wire.read(); l = Wire.read(); gy = int16_t((h << 8) | l);
        h = Wire.read(); l = Wire.read(); gz = int16_t((h << 8) | l);
        return true;
    }

    bool readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                 int16_t& gx, int16_t& gy, int16_t& gz) {
        ++totalReads_;
        if (readRawOnce(ax, ay, az, gx, gy, gz)) return true;
        if (retryEnabled_ && readRawOnce(ax, ay, az, gx, gy, gz)) {
            ++retriesWon_; return true;
        }
        ++errorReads_;
        return false;
    }
};