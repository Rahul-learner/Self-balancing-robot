// Self-balancing robot (Upright Pitch Balance Only): ESP32 + MPU6050 + TB6612FNG
// Hardware: N20 140 RPM gearmotors + 43 mm wheels
// Board: ESP32 DevKit (ESP-WROOM-32), Arduino-ESP32 core 3.x (pioarduino)
//
// Focused strictly on inner-loop upright stabilization & PID tuning:
// - 500 Hz (2 ms) FreeRTOS pitch balance loop on Core 1
// - 1D Linear Kalman Filter (estimates Pitch + Gyro Bias) with Dynamic Accel Gating
// - PT1 low-pass filter on Gyro D-term to suppress N20 gearbox vibration noise
// - Sustained-time fall detector (prevents false cutoffs during quick hand shakes)
// - Streamlined Web UI for live P-I-D tuning and real-time oscilloscope plotting

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ======================= Pins =======================
#define SDA_PIN 21
#define SCL_PIN 22

#define PWMA 25   // driver channel A = RIGHT motor
#define AIN1 26
#define AIN2 27
#define PWMB 33   // driver channel B = LEFT motor
#define BIN1 32
#define BIN2 23
#define STBY 4

// ======================= WiFi =======================
// 0 = robot creates its own WiFi network (no router needed), page at 192.168.4.1
// 1 = robot joins your router (the IP is printed on the serial monitor)
#define USE_STATION_MODE 0
const char *AP_SSID  = "BalanceBot";
const char *AP_PASS  = "balance123";      // at least 8 characters
const char *STA_SSID = "YOUR_WIFI_NAME";
const char *STA_PASS = "YOUR_WIFI_PASSWORD";

// ======================= Motor settings =======================
const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_BITS = 8;

// Same values as the motor test (so "forward" matches what you already verified)
const bool INVERT_LEFT  = true;
const bool INVERT_RIGHT = true;

// ======================= MPU6050 =======================
const uint8_t MPU_ADDR = 0x68;
const float ACCEL_SENS = 16384.0f;  // LSB/g       (+/-2 g)
const float GYRO_SENS  = 65.5f;     // LSB/(deg/s) (+/-500 dps)

// ======================= 3-Layer Chassis Geometry =======================
// Layer 1 (0.0 cm): Bottom PCB with N20 motors (43 mm wheels)
// Layer 2 (4.5 cm): Middle PCB with MPU6050 IMU (r_imu = 0.045 m above wheel axle)
// Layer 3 (9.0 cm): Top PCB with 3S 850mAh LiPo (~75 g top-heavy inverted pendulum)
const float IMU_HEIGHT_M = 0.045f;  // 4.5 cm distance from wheel axle to MPU6050
const float GRAVITY_MSS  = 9.80665f;

// ======================= Tunable parameters =======================
// (Defaults tuned for 3-Layer PCB + Top 3S 850mAh LiPo + N20 140RPM + 43mm wheels)
volatile float kp          = 22.0f;   // PWM per degree of pitch error (stiff for 140RPM)
volatile float kd          = 0.85f;   // PWM per (deg/s) of pitch rate (higher for top 3S LiPo inertia)
volatile float ki          = 0.0f;    // PWM per (deg * sec) of pitch error
volatile float angleOffset = 0.0f;    // mechanical balance pitch angle (deg)
volatile float deadband    = 24.0f;   // minimum PWM needed for N20 gear friction (on 3S / 12V)
volatile float pwmLimit    = 255.0f;  // maximum motor PWM command
volatile float cutoff      = 45.0f;   // disarm/cut motors if tilted past this angle (deg)
volatile float kalmanR     = 0.03f;   // Kalman measurement noise R (higher = trust gyro more)
volatile float dFilter     = 0.30f;   // PT1 low-pass filter on gyro rate (N20 vibration damping)
volatile float motorTrim   = 1.0f;    // right/left motor multiplier ratio (0.8 to 1.2)
volatile float motorDir    = 1.0f;    // +1 or -1: flip if robot drives AWAY from a fall

struct Param {
  const char *name;
  const char *label;
  volatile float *ptr;
  float def, minV, maxV, step;
};

Param params[] = {
  {"kp",          "Kp: Proportional (stiffness)",                 &kp,          22.0f,  0.0f, 100.0f, 0.2f},
  {"kd",          "Kd: Derivative (gyro damping)",                &kd,           0.85f, 0.0f,   5.0f, 0.01f},
  {"ki",          "Ki: Integral (steady-state hold)",             &ki,           0.0f,  0.0f,  60.0f, 0.1f},
  {"angleOffset", "Balance angle offset (deg)",                   &angleOffset,  0.0f, -15.0f, 15.0f, 0.05f},
  {"deadband",    "Motor deadband (PWM)",                         &deadband,    24.0f,  0.0f, 100.0f, 1.0f},
  {"pwmLimit",    "Max PWM limit",                                &pwmLimit,   255.0f, 50.0f, 255.0f, 5.0f},
  {"cutoff",      "Fall cutoff angle (deg)",                      &cutoff,      45.0f, 15.0f,  75.0f, 1.0f},
  {"kalmanR",     "Kalman accel noise R (higher=smoother)",       &kalmanR,      0.03f, 0.005f, 0.20f, 0.005f},
  {"dFilter",     "D-term gyro LPF (1=raw, 0.1=smooth)",          &dFilter,      0.30f, 0.05f,  1.0f, 0.05f},
  {"motorTrim",   "Right/Left motor trim",                        &motorTrim,    1.0f,  0.80f,  1.20f, 0.01f},
  {"motorDir",    "Motor direction (+1 / -1)",                    &motorDir,     1.0f, -1.0f,   1.0f, 2.0f},
};
const int NUM_PARAMS = sizeof(params) / sizeof(params[0]);

// ======================= Shared state =======================
enum State { ST_DISARMED = 0, ST_WAITING = 1, ST_RUNNING = 2, ST_FALLEN = 3, ST_IMUFAIL = 4 };

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile int      g_state    = ST_DISARMED;
volatile bool     g_armed    = false;
volatile bool     g_recalReq = false;
volatile uint32_t g_hz       = 0;

struct Telemetry {
  int      state;
  bool     armed;
  float    pitch;
  float    tilt;
  float    rate;
  float    u;
  int      pwmL;
  int      pwmR;
  uint32_t hz;
};

Telemetry g_telem = {ST_DISARMED, false, 0, 0, 0, 0, 0, 0, 0};

// ======================= Motors =======================
void setDriverEnabled(bool en) {
  digitalWrite(STBY, en ? HIGH : LOW);
}

void setMotorRaw(int in1, int in2, int pwmPin, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Short-brake at zero crossing for crisp response without backlash coasting
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  }
  ledcWrite(pwmPin, abs(speed));
}

int applyDeadbandAndLimit(float cmd, float dead, float lim) {
  float mag = fabsf(cmd);
  if (mag < 0.5f) return 0;
  int pwm = (int)constrain(mag + dead, 0.0f, lim);
  return (cmd < 0.0f) ? -pwm : pwm;
}

void setLeft(int speed)  { setMotorRaw(BIN1, BIN2, PWMB, speed, INVERT_LEFT); }
void setRight(int speed) { setMotorRaw(AIN1, AIN2, PWMA, speed, INVERT_RIGHT); }

void stopMotors() {
  setLeft(0);
  setRight(0);
  setDriverEnabled(false);
}

// ======================= MPU6050 =======================
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;
float pitch = 0, gyroRate = 0, gyroRateFilt = 0, prevGyroRateFilt = 0, alphaRadFilt = 0;
bool  mpuPresent = false;
int   imuFail = 0;

// 1D Linear Kalman Filter state for Pitch Angle & Gyro Bias
float kf_angle = 0.0f;
float kf_bias  = 0.0f;
float kf_P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
const float KF_Q_ANGLE = 0.001f;  // process noise variance for accelerometer angle
const float KF_Q_BIAS  = 0.003f;  // process noise variance for gyro drift

// Recovers a stuck I2C bus if motor EMI holds SDA low
void recoverI2CBus() {
  Wire.end();
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    if (digitalRead(SDA_PIN) == HIGH) break;
  }
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(5);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(5);
}

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 1) != 1) return false;
  uint8_t id = Wire.read();
  Serial.printf("MPU WHO_AM_I = 0x%02X\n", id);
  if (!mpuWrite(0x6B, 0x00)) return false; // Wake up
  delay(50);
  mpuWrite(0x19, 0x01);   // Sample rate divider: 500 Hz
  mpuWrite(0x1A, 0x03);   // DLPF ~44 Hz bandwidth
  mpuWrite(0x1B, 0x08);   // Gyro +/-500 dps
  mpuWrite(0x1C, 0x00);   // Accel +/-2 g
  return true;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro... keep the robot STILL.");
  const int N = 400;
  for (int attempt = 0; attempt < 3; attempt++) {
    long sx = 0, sy = 0, sz = 0;
    int16_t minGy = 32767, maxGy = -32768;
    int good = 0;
    for (int i = 0; i < N; i++) {
      int16_t ax, ay, az, gx, gy, gz;
      if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
        sx += gx; sy += gy; sz += gz;
        if (gy < minGy) minGy = gy;
        if (gy > maxGy) maxGy = gy;
        good++;
      }
      delay(3);
    }
    if (good > N / 2) {
      gyroOffX = (float)sx / good;
      gyroOffY = (float)sy / good;
      gyroOffZ = (float)sz / good;
      if ((maxGy - minGy) < (int16_t)(8.0f * GYRO_SENS)) {
        Serial.printf("Gyro offsets locked: %.1f %.1f %.1f (%d samples)\n",
                      gyroOffX, gyroOffY, gyroOffZ, good);
        return;
      }
      Serial.println("Motion detected during calibration, retrying...");
    }
  }
  Serial.printf("Gyro offsets (fallback): %.1f %.1f %.1f\n", gyroOffX, gyroOffY, gyroOffZ);
}

void seedAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
    float axg = ax / ACCEL_SENS, ayg = ay / ACCEL_SENS, azg = az / ACCEL_SENS;
    pitch = atan2f(axg, sqrtf(ayg * ayg + azg * azg)) * 180.0f / PI;
    kf_angle = pitch;
    kf_bias  = 0.0f;
    kf_P[0][0] = 1.0f; kf_P[0][1] = 0.0f;
    kf_P[1][0] = 0.0f; kf_P[1][1] = 1.0f;
    gyroRate = 0;
    gyroRateFilt = 0;
    prevGyroRateFilt = 0;
    alphaRadFilt = 0;
  }
}

bool imuStep(float dt) {
  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadRaw(ax, ay, az, gx, gy, gz)) {
    if (++imuFail > 15) mpuPresent = false;
    return false;
  }
  imuFail = 0;
  float axg = ax / ACCEL_SENS, ayg = ay / ACCEL_SENS, azg = az / ACCEL_SENS;
  float gy_dps = (gy - gyroOffY) / GYRO_SENS;

  // Bias-corrected angular rate & PT1 low-pass filter
  float rate = gy_dps - kf_bias;
  gyroRate = rate;
  float dAlpha = constrain((float)dFilter, 0.05f, 1.0f);
  gyroRateFilt += dAlpha * (rate - gyroRateFilt);

  // --- 2nd-Layer PCB IMU Lever-Arm Compensation (r = 4.5 cm above wheel axle) ---
  // Because the MPU6050 sits 4.5 cm above the pivot axis, pitching at angular acceleration
  // alpha (rad/s^2) and angular velocity omega (rad/s) induces non-gravitational forces:
  //   1. Tangential acceleration along X: a_tangential = r * alpha
  //   2. Centripetal acceleration along Z: a_centripetal = r * omega^2
  // Subtracting these prevents the accelerometer from misreading rapid tilt corrections as gravity.
  float omegaRad = gyroRateFilt * (PI / 180.0f);
  float rawAlphaRad = ((gyroRateFilt - prevGyroRateFilt) / dt) * (PI / 180.0f);
  prevGyroRateFilt = gyroRateFilt;
  alphaRadFilt += 0.25f * (rawAlphaRad - alphaRadFilt);

  float aTangentialG  = constrain((IMU_HEIGHT_M * alphaRadFilt) / GRAVITY_MSS, -0.5f, 0.5f);
  float aCentripetalG = constrain((IMU_HEIGHT_M * omegaRad * omegaRad) / GRAVITY_MSS, 0.0f, 0.5f);

  // Forward tilt -> pitch negative -> axg is negative, so tangential accel opposes sign of alphaRad
  float axg_corr = axg + aTangentialG;
  float azg_corr = azg + aCentripetalG;

  // Compensated accelerometer angle & total G-force magnitude
  float accPitch = atan2f(axg_corr, sqrtf(ayg * ayg + azg_corr * azg_corr)) * 180.0f / PI;
  float accMagSq = axg_corr * axg_corr + ayg * ayg + azg_corr * azg_corr;

  // --- Step 1: Kalman Predict (Time Update using Gyroscope) ---
  kf_angle += dt * rate;

  kf_P[0][0] += dt * (dt * kf_P[1][1] - kf_P[0][1] - kf_P[1][0] + KF_Q_ANGLE);
  kf_P[0][1] -= dt * kf_P[1][1];
  kf_P[1][0] -= dt * kf_P[1][1];
  kf_P[1][1] += KF_Q_BIAS * dt;

  // --- Step 2: Kalman Correct (Measurement Update with Dynamic Accel Gating) ---
  float rMeas = constrain((float)kalmanR, 0.002f, 0.5f);
  if (fabsf(accMagSq - 1.0f) > 0.18f) {
    rMeas *= 100.0f;
  }

  float S = kf_P[0][0] + rMeas;
  float K0 = kf_P[0][0] / S;
  float K1 = kf_P[1][0] / S;

  float y = accPitch - kf_angle;
  kf_angle += K0 * y;
  kf_bias  += K1 * y;

  float P00_temp = kf_P[0][0];
  float P01_temp = kf_P[0][1];
  kf_P[0][0] -= K0 * P00_temp;
  kf_P[0][1] -= K0 * P01_temp;
  kf_P[1][0] -= K1 * P00_temp;
  kf_P[1][1] -= K1 * P01_temp;

  pitch = kf_angle;

  return true;
}

// ======================= Balance task (500 Hz / 2 ms) =======================
void controlTask(void *) {
  const TickType_t period = pdMS_TO_TICKS(2); // 500 Hz inner loop
  TickType_t lastWake = xTaskGetTickCount();
  unsigned long prevUs = micros();
  unsigned long hzMs = millis();
  uint32_t cnt = 0;
  unsigned long lastReinitMs = 0;

  float integ = 0, uprightTime = 0;
  // Require tilt to exceed cutoff continuously for 120 ms (60 cycles) before disarming
  int fallCounter = 0;

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    unsigned long nowUs = micros();
    float dt = (nowUs - prevUs) / 1e6f;
    prevUs = nowUs;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.002f;

    cnt++;
    if (millis() - hzMs >= 1000) {
      g_hz = cnt;
      cnt = 0;
      hzMs = millis();
    }

    if (g_recalReq && !g_armed && mpuPresent) {
      stopMotors();
      calibrateGyro();
      seedAngle();
      g_recalReq = false;
      lastWake = xTaskGetTickCount();
      prevUs = micros();
      continue;
    }
    g_recalReq = false;

    if (!mpuPresent) {
      stopMotors();
      g_state = ST_IMUFAIL;
      if (millis() - lastReinitMs > 1000) {
        lastReinitMs = millis();
        recoverI2CBus();
        if (mpuInit()) {
          mpuPresent = true;
          imuFail = 0;
          seedAngle();
          lastWake = xTaskGetTickCount();
          prevUs = micros();
        }
      }
      continue;
    }
    if (!imuStep(dt)) {
      stopMotors();
      g_state = ST_IMUFAIL;
      continue;
    }

    const float Kp   = kp, Ki = ki, Kd = kd, off = angleOffset, dead = deadband;
    const float lim  = pwmLimit, cut = cutoff, trim = motorTrim;
    const float dirf = (motorDir >= 0.0f) ? 1.0f : -1.0f;

    const float tilt = pitch - off;
    float u = 0;
    int pwmL = 0, pwmR = 0;

    if (!g_armed) {
      stopMotors();
      g_state = ST_DISARMED;
      integ = 0;
      uprightTime = 0;
      fallCounter = 0;
    } else if (g_state == ST_RUNNING) {
      // Require tilt to exceed cutoff for 60 consecutive cycles (120 ms) so quick hand shakes don't trip it
      if (fabsf(tilt) > cut) {
        if (++fallCounter > 60) {
          stopMotors();
          g_state = ST_FALLEN;
          integ = 0;
          uprightTime = 0;
        }
      } else {
        fallCounter = 0;
        setDriverEnabled(true);

        if (Ki > 1e-5f) {
          float nextInteg = integ + tilt * dt;
          float iLim = 80.0f / Ki;
          integ = constrain(nextInteg, -iLim, iLim);
        } else {
          integ = 0.0f;
        }

        u = -(Kp * tilt + Kd * gyroRateFilt + Ki * integ);
        float baseCmd = dirf * u;

        float trimL = (trim > 1.0f) ? (1.0f / trim) : 1.0f;
        float trimR = (trim < 1.0f) ? trim : 1.0f;

        pwmL = applyDeadbandAndLimit(baseCmd * trimL, dead, lim);
        pwmR = applyDeadbandAndLimit(baseCmd * trimR, dead, lim);

        setLeft(pwmL);
        setRight(pwmR);
      }
    } else {
      // ARMED: wait for robot to be held upright (within 4 deg for 0.4 s)
      stopMotors();
      if (g_state == ST_DISARMED || g_state == ST_IMUFAIL) g_state = ST_WAITING;
      if (fabsf(tilt) < 4.0f) uprightTime += dt; else uprightTime = 0;
      if (uprightTime > 0.4f) {
        integ = 0;
        g_state = ST_RUNNING;
        fallCounter = 0;
      }
    }

    portENTER_CRITICAL(&stateMux);
    g_telem.state = g_state;
    g_telem.armed = g_armed;
    g_telem.pitch = pitch;
    g_telem.tilt  = tilt;
    g_telem.rate  = gyroRateFilt;
    g_telem.u     = u;
    g_telem.pwmL  = pwmL;
    g_telem.pwmR  = pwmR;
    g_telem.hz    = g_hz;
    portEXIT_CRITICAL(&stateMux);
  }
}

// ======================= Web server =======================
WebServer server(80);
Preferences prefs;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Balance Bot PID Tuner</title>
<style>
:root{--bg:#0f1115;--card:#181b22;--fg:#e8eaf0;--mut:#8b93a7;--ok:#2ecc71;--bad:#e74c3c;--acc:#4da3ff;--warn:#f5a623;--purp:#a56eff}
*{box-sizing:border-box}
body{margin:0;padding:12px;background:var(--bg);color:var(--fg);font:14px system-ui,-apple-system,sans-serif;max-width:760px;margin:auto}
h1{font-size:20px;margin:4px 0 10px;display:flex;justify-content:space-between;align-items:center}
.card{background:var(--card);border-radius:10px;padding:12px;margin-bottom:12px;border:1px solid #232733}
.bar{display:flex;flex-wrap:wrap;gap:8px 18px;align-items:center}
.badge{padding:4px 10px;border-radius:12px;font-weight:700;font-size:12px;background:#333}
.b0{background:#444}.b1{background:var(--warn);color:#000}.b2{background:var(--ok);color:#000}.b3,.b4{background:var(--bad)}
.stat{color:var(--mut)}.stat b{color:var(--fg);font-variant-numeric:tabular-nums}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
button{background:#2a2f3a;color:var(--fg);border:0;border-radius:8px;padding:9px 14px;font:inherit;cursor:pointer;transition:filter .15s}
button:hover{filter:brightness(1.15)}
button:active{filter:brightness(1.3)}
.go{background:var(--ok);color:#000;font-weight:700}.stp{background:var(--bad);font-weight:700}
canvas#plot{width:100%;height:190px;background:#0b0d11;border-radius:8px;display:block}
.leg{font-size:12px;color:var(--mut);margin-top:6px;display:flex;justify-content:space-between;flex-wrap:wrap;gap:6px}
.p{display:grid;grid-template-columns:1fr 90px;gap:4px 10px;align-items:center;margin:10px 0}
.p label{grid-column:1/3;color:var(--mut);font-size:13px;display:flex;justify-content:space-between}
.p label span{color:var(--acc);font-variant-numeric:tabular-nums}
.p input[type=range]{width:100%;accent-color:var(--acc)}
.p input[type=number]{width:100%;background:#0b0d11;color:var(--fg);border:1px solid #2a2f3a;border-radius:6px;padding:6px}
#msg{color:var(--acc);font-size:13px;min-height:18px;margin-top:6px}
.tips{font-size:12.5px;color:var(--mut);line-height:1.5;margin:0;padding-left:18px}
.tips b{color:var(--fg)}
</style></head><body>
<h1><span>Balance Bot &mdash; Upright PID Tuner</span><span id="st" class="badge b0">CONNECTING...</span></h1>

<div class="card">
  <div class="bar">
    <span class="stat">pitch <b id="pitch">0.00</b>&deg;</span>
    <span class="stat">tilt err <b id="tilt">0.00</b>&deg;</span>
    <span class="stat">gyro rate <b id="rate">0.0</b>&deg;/s</span>
    <span class="stat">pwm L/R <b id="pwm">0 / 0</b></span>
    <span class="stat">loop <b id="hz">0</b> Hz</span>
  </div>
</div>

<div class="card">
  <div class="row">
    <button class="go" id="arm">ARM</button>
    <button class="stp" id="stop">STOP (SPACE)</button>
    <button id="zero">Set Balance Angle = Current Pitch</button>
    <button id="recal">Recalibrate Gyro</button>
  </div>
  <div id="msg"></div>
  <div class="leg"><span>Hold the robot upright (&plusmn;4&deg;) for 0.4 s after clicking ARM. Press Spacebar anytime to disarm.</span></div>
</div>

<div class="card">
  <canvas id="plot" width="740" height="190"></canvas>
  <div class="leg">
    <div>
      <span style="color:#4da3ff">&#9632;</span> tilt error (&plusmn;40&deg;) &nbsp;
      <span style="color:#a56eff">&#9632;</span> gyro rate (&plusmn;200&deg;/s) &nbsp;
      <span style="color:#f5a623">&#9632;</span> motor PWM (&plusmn;255)
    </div>
    <button id="pausePlot" style="padding:3px 10px;font-size:12px">Pause Plot</button>
  </div>
</div>

<div class="card">
  <div id="params"></div>
  <div class="row" style="margin-top:10px">
    <button id="save" class="go">Save to Flash</button>
    <button id="defaults">Reset to Defaults</button>
  </div>
</div>

<script>
const $=id=>document.getElementById(id);
const STATES=["DISARMED","WAITING (hold upright)","RUNNING","FALLEN","IMU ERROR"];
function msg(t){$("msg").textContent=t;clearTimeout(msg.t);msg.t=setTimeout(()=>$("msg").textContent="",2500);}
function call(url,m){return fetch(url).then(r=>r.text()).then(t=>{if(m)msg(m);return t;}).catch(()=>msg("No connection to robot"));}
function send(n,v){call("/set?name="+n+"&value="+v);}

function build(list){
  const box=$("params");box.innerHTML="";
  list.forEach(p=>{
    const d=document.createElement("div");d.className="p";
    if(p.n==="motorDir"){
      d.innerHTML='<label style="cursor:pointer"><sp><input type="checkbox" id="p_'+p.n+'"> Reverse motor direction (tick if robot drives away from a fall)</sp></label>';
      box.appendChild(d);
      const c=d.querySelector("input");c.checked=p.v<0;
      c.onchange=()=>send(p.n,c.checked?-1:1);
      return;
    }
    d.innerHTML='<label><b>'+p.l+'</b><span id="v_'+p.n+'">'+p.v+'</span></label>'+
                '<input type="range" min="'+p.min+'" max="'+p.max+'" step="'+p.s+'" value="'+p.v+'">'+
                '<input type="number" min="'+p.min+'" max="'+p.max+'" step="'+p.s+'" value="'+p.v+'">';
    const lbl=d.querySelector("span"),r=d.children[1],n=d.children[2];
    let dbTimer=null;
    r.oninput=()=>{n.value=r.value;lbl.textContent=r.value;clearTimeout(dbTimer);dbTimer=setTimeout(()=>send(p.n,r.value),80);};
    r.onchange=()=>{clearTimeout(dbTimer);send(p.n,r.value);};
    n.onchange=()=>{r.value=n.value;lbl.textContent=n.value;send(p.n,n.value);};
    box.appendChild(d);
  });
}
function loadParams(){fetch("/params").then(r=>r.json()).then(build).catch(()=>msg("Could not load parameters"));}

$("arm").onclick=()=>call("/arm?on=1","Armed");
$("stop").onclick=()=>call("/arm?on=0","Stopped");
$("zero").onclick=()=>call("/zero","Balance angle updated").then(loadParams);
$("recal").onclick=()=>call("/recal","Recalibrating gyro: keep still (disarm first)");
$("save").onclick=()=>call("/save","Saved parameters to flash");
$("defaults").onclick=()=>call("/defaults","Defaults restored (click Save to keep)").then(loadParams);

window.addEventListener("keydown",e=>{if(e.code==="Space"&&document.activeElement.tagName!=="INPUT"){e.preventDefault();call("/arm?on=0","Emergency Stop");}});

const N=300,tlt=[],rt=[],out=[];
let paused=false;
$("pausePlot").onclick=()=>{paused=!paused;$("pausePlot").textContent=paused?"Resume Plot":"Pause Plot";};
const cv=$("plot"),cx=cv.getContext("2d");
function draw(){
  const W=cv.width,H=cv.height;cx.clearRect(0,0,W,H);
  cx.strokeStyle="#232733";cx.beginPath();cx.moveTo(0,H/2);cx.lineTo(W,H/2);cx.stroke();
  function line(a,sc,col){cx.strokeStyle=col;cx.lineWidth=1.6;cx.beginPath();
    a.forEach((v,i)=>{const x=i/(N-1)*W,y=H/2-Math.max(-1,Math.min(1,v/sc))*(H/2-6);i?cx.lineTo(x,y):cx.moveTo(x,y);});cx.stroke();}
  line(tlt,40,"#4da3ff");line(rt,200,"#a56eff");line(out,255,"#f5a623");
}

let busy=false;
function poll(){
  if(busy)return;busy=true;
  fetch("/telemetry").then(r=>r.json()).then(t=>{
    const s=$("st");s.textContent=STATES[t.s]||"?";s.className="badge b"+t.s;
    $("pitch").textContent=t.p.toFixed(2);$("tilt").textContent=t.t.toFixed(2);
    $("rate").textContent=t.r.toFixed(1);$("pwm").textContent=t.pl+" / "+t.pr;
    $("hz").textContent=t.hz;
    if(!paused){
      tlt.push(t.t);rt.push(t.r);out.push((t.pl+t.pr)*0.5);
      if(tlt.length>N){tlt.shift();rt.shift();out.shift();}
      draw();
    }
  }).catch(()=>{$("st").textContent="NO LINK";$("st").className="badge b3";}).finally(()=>{busy=false;});
}
loadParams();setInterval(poll,100);
</script></body></html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleParams() {
  char buf[2048];
  size_t pos = 0;
  pos += snprintf(buf + pos, sizeof(buf) - pos, "[");
  for (int i = 0; i < NUM_PARAMS; i++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos,
                    "%s{\"n\":\"%s\",\"l\":\"%s\",\"v\":%.4f,\"min\":%.4f,\"max\":%.4f,\"s\":%.4f}",
                    (i ? "," : ""), params[i].name, params[i].label,
                    (float)*params[i].ptr, params[i].minV, params[i].maxV, params[i].step);
  }
  snprintf(buf + pos, sizeof(buf) - pos, "]");
  server.send(200, "application/json", buf);
}

void handleSet() {
  String n = server.arg("name");
  float v = server.arg("value").toFloat();
  for (int i = 0; i < NUM_PARAMS; i++) {
    if (n == params[i].name) {
      v = constrain(v, params[i].minV, params[i].maxV);
      *params[i].ptr = v;
      server.send(200, "text/plain", "ok");
      return;
    }
  }
  server.send(404, "text/plain", "unknown parameter");
}

void handleTelemetry() {
  Telemetry snap;
  portENTER_CRITICAL(&stateMux);
  snap = g_telem;
  portEXIT_CRITICAL(&stateMux);

  char buf[240];
  snprintf(buf, sizeof(buf),
           "{\"s\":%d,\"a\":%d,\"p\":%.3f,\"t\":%.3f,\"r\":%.2f,\"u\":%.2f,"
           "\"pl\":%d,\"pr\":%d,\"hz\":%u}",
           snap.state, snap.armed ? 1 : 0, snap.pitch, snap.tilt, snap.rate, snap.u,
           snap.pwmL, snap.pwmR, (unsigned)snap.hz);
  server.send(200, "application/json", buf);
}

void handleArm() {
  g_armed = (server.arg("on") == "1");
  server.send(200, "text/plain", "ok");
}

void handleZero() {
  portENTER_CRITICAL(&stateMux);
  float curPitch = g_telem.pitch;
  portEXIT_CRITICAL(&stateMux);
  angleOffset = constrain(curPitch, -15.0f, 15.0f);
  server.send(200, "text/plain", "ok");
}

void handleRecal() {
  if (!g_armed) g_recalReq = true;
  server.send(200, "text/plain", "ok");
}

void saveParams() {
  prefs.begin("bal_v2", false);
  for (int i = 0; i < NUM_PARAMS; i++) {
    prefs.putFloat(params[i].name, *params[i].ptr);
  }
  prefs.end();
}

void loadParams() {
  prefs.begin("bal_v2", true);
  for (int i = 0; i < NUM_PARAMS; i++) {
    *params[i].ptr = prefs.getFloat(params[i].name, params[i].def);
  }
  prefs.end();
}

void handleSave() {
  saveParams();
  server.send(200, "text/plain", "ok");
}

void handleDefaults() {
  for (int i = 0; i < NUM_PARAMS; i++) {
    *params[i].ptr = params[i].def;
  }
  server.send(200, "text/plain", "ok");
}

void startWifi() {
#if USE_STATION_MODE
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Open http://");
    Serial.println(WiFi.localIP());
    WiFi.setSleep(false);
    return;
  }
  Serial.println("Could not join WiFi, starting own network instead.");
#endif
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);
  Serial.printf("WiFi network '%s' (password '%s')\n", AP_SSID, AP_PASS);
  Serial.print("Open http://");
  Serial.println(WiFi.softAPIP());
}

// ======================= Arduino entry points =======================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nSelf-balancing robot (Upright PID Only)");

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_BITS);
  ledcAttach(PWMB, PWM_FREQ, PWM_BITS);
  stopMotors();

  loadParams();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(5);
  if (mpuInit()) {
    mpuPresent = true;
    calibrateGyro();
    seedAngle();
    Serial.println("MPU6050 ready.");
  } else {
    Serial.println("MPU6050 NOT found. Check SDA(G21), SCL(G22), 3V3, GND. Will keep retrying.");
  }

  startWifi();
  server.on("/", HTTP_GET, handleRoot);
  server.on("/params", HTTP_GET, handleParams);
  server.on("/set", HTTP_GET, handleSet);
  server.on("/telemetry", HTTP_GET, handleTelemetry);
  server.on("/arm", HTTP_GET, handleArm);
  server.on("/zero", HTTP_GET, handleZero);
  server.on("/recal", HTTP_GET, handleRecal);
  server.on("/save", HTTP_GET, handleSave);
  server.on("/defaults", HTTP_GET, handleDefaults);
  server.begin();

  // Balance loop: high priority on Core 1
  xTaskCreatePinnedToCore(controlTask, "balance", 8192, nullptr, 5, nullptr, 1);
}

void loop() {
  server.handleClient();
  delay(2);

  static unsigned long lastMs = 0;
  if (millis() - lastMs >= 1000) {
    lastMs = millis();
    Telemetry snap;
    portENTER_CRITICAL(&stateMux);
    snap = g_telem;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("state=%d pitch=%.2f tilt=%.2f pwmL=%d pwmR=%d hz=%u\n",
                  snap.state, snap.pitch, snap.tilt, snap.pwmL, snap.pwmR, (unsigned)snap.hz);
  }
}