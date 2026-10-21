#pragma once
#include <Arduino.h>
#include "config.h"

// PID with anti-windup. Output convention:
//   error      = measurement (e.g. tilt, deg)
//   derivative = rate of measurement (e.g. gyro, deg/s)
//   returns    = kp*error + kd*derivative + ki*integral   (caller applies sign)
class PidController {
public:
    struct Gains { float kp, ki, kd; };

    void  setGains(const Gains& g)    { gains_ = g; }
    void  setIntegralLimit(float lim) { integralLimit_ = lim; }
    void  reset()                     { integral_ = 0.0f; }
    float integral() const            { return integral_; }

    float update(float error, float derivative, float dt) {
        if (gains_.ki > 1e-5f) {
            integral_ = constrain(integral_ + error * dt,
                                  -integralLimit_, integralLimit_);
        } else {
            integral_ = 0.0f;
        }
        return gains_.kp * error + gains_.kd * derivative + gains_.ki * integral_;
    }

private:
    Gains gains_{0.0f, 0.0f, 0.0f};
    float integralLimit_ = 80.0f;
    float integral_      = 0.0f;
};

// 1D linear Kalman filter: state = [angle, gyro_bias].
class KalmanFilter1D {
public:
    void setNoise(float qAngle, float qBias, float rMeas) {
        qAngle_ = qAngle; qBias_ = qBias; rMeas_ = rMeas;
    }
    void reset(float angle) {
        angle_ = angle; bias_ = 0.0f;
        P_[0][0] = 1.0f; P_[0][1] = 0.0f;
        P_[1][0] = 0.0f; P_[1][1] = 1.0f;
    }
    // Free-run the angle prediction (used during I2C coast-through)
    void coastBy(float rate, float dt) { angle_ += rate * dt; }

    float update(float rate, float accelAngle, float dt, float accelMagSq) {
        // Predict
        angle_ += dt * rate;
        P_[0][0] += dt * (dt * P_[1][1] - P_[0][1] - P_[1][0] + qAngle_);
        P_[0][1] -= dt * P_[1][1];
        P_[1][0] -= dt * P_[1][1];
        P_[1][1] += qBias_ * dt;

        // Correct, with dynamic gating when accelerometer magnitude deviates
        float r = rMeas_;
        if (fabsf(accelMagSq - 1.0f) > KF_ACCEL_GATE) r *= KF_ACCEL_PENALTY;

        const float S  = P_[0][0] + r;
        const float K0 = P_[0][0] / S;
        const float K1 = P_[1][0] / S;
        const float y  = accelAngle - angle_;
        angle_ += K0 * y;
        bias_  += K1 * y;

        const float p00 = P_[0][0], p01 = P_[0][1];
        P_[0][0] -= K0 * p00;  P_[0][1] -= K0 * p01;
        P_[1][0] -= K1 * p00;  P_[1][1] -= K1 * p01;
        return angle_;
    }

    float angle() const { return angle_; }
    float bias()  const { return bias_;  }

private:
    float angle_ = 0.0f, bias_ = 0.0f;
    float P_[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
    float qAngle_ = KF_Q_ANGLE, qBias_ = KF_Q_BIAS;
    float rMeas_  = DEF_KALMAN_R;
};