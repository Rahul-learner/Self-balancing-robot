#pragma once
#include <Arduino.h>

// ── Pins ──────────────────────────────────────────────────────────
constexpr uint8_t PIN_SDA = 21, PIN_SCL = 22;
constexpr uint8_t PIN_PWMA = 25, PIN_AIN1 = 26, PIN_AIN2 = 27;   // right motor
constexpr uint8_t PIN_PWMB = 33, PIN_BIN1 = 32, PIN_BIN2 = 23;   // left motor
constexpr uint8_t PIN_STBY = 4;
constexpr uint8_t PIN_ENC_LA = 34, PIN_ENC_LB = 35;              // left (input-only)
constexpr uint8_t PIN_ENC_RA = 5,  PIN_ENC_RB = 15;              // right

// ── Motor driver ──────────────────────────────────────────────────
constexpr uint32_t MOTOR_PWM_FREQ_HZ  = 20000;
constexpr uint8_t  MOTOR_PWM_RES_BITS = 8;
constexpr bool     MOTOR_INVERT_LEFT  = true;
constexpr bool     MOTOR_INVERT_RIGHT = true;

// ── IMU ───────────────────────────────────────────────────────────
constexpr uint8_t  IMU_I2C_ADDR         = 0x68;
constexpr float    IMU_ACCEL_LSB_PER_G  = 8192.0f;   // ±4 g
constexpr float    IMU_GYRO_LSB_PER_DPS = 32.8f;     // ±1000 dps
constexpr uint32_t IMU_I2C_CLOCK_HZ     = 400000;
constexpr uint32_t IMU_I2C_TIMEOUT_MS   = 5;
constexpr int      IMU_CAL_SAMPLES      = 400;
constexpr float    IMU_CAL_MOTION_LIMIT = 8.0f;      // dps
constexpr int      IMU_FAIL_THRESHOLD   = 15;
constexpr int      IMU_COAST_CYCLES     = 5;

// ── Control loop ──────────────────────────────────────────────────
constexpr int   LOOP_PERIOD_MS      = 2;             // 500 Hz
constexpr float LOOP_DT_NOMINAL     = 0.002f;
constexpr float LOOP_DT_MAX         = 0.05f;
constexpr int   FALL_CYCLES         = 20;            // 40 ms
constexpr float UPRIGHT_HOLD_S      = 0.15f;
constexpr float VELOCITY_TILT_LIMIT = 6.0f;          // deg
constexpr float VELOCITY_SPEED_SCALE  = 0.01f;
constexpr float VELOCITY_POS_SCALE    = 0.001f;
constexpr float POS_INT_CLAMP       = 2000.0f;
constexpr float SPEED_FILTER_ALPHA  = 0.15f;

// ── Kalman ────────────────────────────────────────────────────────
constexpr float KF_Q_ANGLE       = 0.001f;
constexpr float KF_Q_BIAS        = 0.003f;
constexpr float KF_ACCEL_GATE    = 0.18f;
constexpr float KF_ACCEL_PENALTY = 100.0f;

// ── Watchdog ──────────────────────────────────────────────────────
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
constexpr uint32_t WATCHDOG_CHECK_MS   = 250;

// ── Tuning defaults (match verified working tune) ─────────────────
constexpr float DEF_KP           = 75.0f;
constexpr float DEF_KD           = 2.80f;
constexpr float DEF_KI           = 0.0f;
constexpr float DEF_ANGLE_OFFSET = 14.45f;
constexpr float DEF_START_WINDOW = 8.0f;
constexpr float DEF_DEADBAND     = 24.0f;
constexpr float DEF_PWM_LIMIT    = 255.0f;
constexpr float DEF_CUTOFF       = 18.0f;
constexpr float DEF_KP_V         = 15.0f;
constexpr float DEF_KI_V         = 0.0f;
constexpr float DEF_KALMAN_R     = 0.03f;
constexpr float DEF_D_FILTER     = 0.75f;
constexpr float DEF_MOTOR_TRIM   = 1.0f;
constexpr float DEF_MOTOR_DIR    = 1.0f;

// ── WiFi ──────────────────────────────────────────────────────────
constexpr bool   WIFI_USE_STATION_MODE = false;
constexpr char   WIFI_AP_SSID[]   = "BalanceBot";
constexpr char   WIFI_AP_PASS[]   = "balance123";
constexpr char   WIFI_STA_SSID[]  = "YOUR_WIFI_NAME";
constexpr char   WIFI_STA_PASS[]  = "YOUR_WIFI_PASSWORD";
constexpr uint32_t WIFI_STA_TIMEOUT_MS = 15000;

// ── NVS ───────────────────────────────────────────────────────────
constexpr char NVS_NAMESPACE[] = "bal_v3";