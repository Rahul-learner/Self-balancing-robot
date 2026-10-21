#pragma once
#include <Arduino.h>
#include "config.h"

class MotorDriver {
public:
    void begin() {
        pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
        pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
        pinMode(PIN_STBY, OUTPUT);
        ledcAttach(PIN_PWMA, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
        ledcAttach(PIN_PWMB, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
        stop();
    }

    // Coast (driver disabled → wheels free-spin)
    void stop() {
        writeOne(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0, MOTOR_INVERT_RIGHT);
        writeOne(PIN_BIN1, PIN_BIN2, PIN_PWMB, 0, MOTOR_INVERT_LEFT);
        digitalWrite(PIN_STBY, LOW);
    }

    void enable(bool en) { digitalWrite(PIN_STBY, en ? HIGH : LOW); }

    void setLeft(int s)  { writeOne(PIN_BIN1, PIN_BIN2, PIN_PWMB, s, MOTOR_INVERT_LEFT); }
    void setRight(int s) { writeOne(PIN_AIN1, PIN_AIN2, PIN_PWMA, s, MOTOR_INVERT_RIGHT); }

    // Deadband kick: commands below 0.5 are 0, above get +deadband added so the
    // motor actually turns. Sign of cmd is preserved.
    static int applyDeadband(float cmd, float deadband, float limit) {
        const float mag = fabsf(cmd);
        if (mag < 0.5f) return 0;
        const int pwm = int(constrain(mag + deadband, 0.0f, limit));
        return cmd < 0.0f ? -pwm : pwm;
    }

private:
    void writeOne(uint8_t in1, uint8_t in2, uint8_t pwmPin, int speed, bool invert) {
        if (invert) speed = -speed;
        speed = constrain(speed, -255, 255);
        if (speed > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
        else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
        else                { digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); }  // brake
        ledcWrite(pwmPin, abs(speed));
    }
};