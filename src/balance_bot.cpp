#include "balance_bot.h"
#include <Preferences.h>

// ISR counters (kept global for safe ISR access on ESP32)
static volatile long g_encL = 0;
static volatile long g_encR = 0;

static void IRAM_ATTR isrEncL() {
    g_encL += digitalRead(PIN_ENC_LB) ? 1 : -1;
}
static void IRAM_ATTR isrEncR() {
    g_encR += digitalRead(PIN_ENC_RB) ? 1 : -1;
}

// ── Task trampolines ──────────────────────────────────────────
void BalanceBot::controlTrampoline(void* p)  { static_cast<BalanceBot*>(p)->controlLoop();  }
void BalanceBot::watchdogTrampoline(void* p) { static_cast<BalanceBot*>(p)->watchdogLoop(); }

// ── Lifecycle ─────────────────────────────────────────────────
bool BalanceBot::begin() {
    buildParamTable();
    motors_.begin();

    pinMode(PIN_ENC_LA, INPUT);        pinMode(PIN_ENC_LB, INPUT);
    pinMode(PIN_ENC_RA, INPUT_PULLUP); pinMode(PIN_ENC_RB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LA), isrEncL, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RA), isrEncR, RISING);

    loadParamsFromNvs();
    const bool ok = imu_.begin();
    if (ok) { imu_.calibrateGyro(); imu_.seedAngle(); }
    return ok;
}

void BalanceBot::startTasks() {
    xTaskCreatePinnedToCore(controlTrampoline, "balance", 8192, this, 5, nullptr, 1);
    xTaskCreatePinnedToCore(watchdogTrampoline, "wdog",   2048, this, 2, nullptr, 0);
}

// ── Control loop ──────────────────────────────────────────────
void BalanceBot::controlLoop() {
    TickType_t lastWake  = xTaskGetTickCount();
    unsigned long prevUs = micros();
    unsigned long hzWin  = millis();
    uint32_t hzCnt = 0;
    unsigned long lastReinitMs = 0;

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
        lastBeatMs_ = millis();

        const unsigned long nowUs = micros();
        float dt = (nowUs - prevUs) * 1e-6f;
        prevUs = nowUs;
        if (dt <= 0.0f || dt > LOOP_DT_MAX) dt = LOOP_DT_NOMINAL;

        if (++hzCnt, millis() - hzWin >= 1000) {
            hz_ = hzCnt; hzCnt = 0; hzWin = millis();
        }

        // Recalibration (only when disarmed)
        if (recalReq_ && !armed_ && imu_.isPresent()) {
            motors_.stop();
            imu_.calibrateGyro();
            imu_.seedAngle();
            recalReq_ = false;
            lastWake = xTaskGetTickCount();
            prevUs = micros();
            continue;
        }
        recalReq_ = false;

        // IMU health check + reinit
        if (!imu_.isPresent()) {
            motors_.stop();
            state_ = BotState::ImuFail;
            if (millis() - lastReinitMs > 1000) {
                lastReinitMs = millis();
                if (imu_.reinit()) {
                    lastWake = xTaskGetTickCount();
                    prevUs = micros();
                }
            }
            continue;
        }
        if (!imu_.update(dt)) {
            motors_.stop();
            state_ = BotState::ImuFail;
            continue;
        }

        runControllerCycle(dt);
    }
}

void BalanceBot::runControllerCycle(float dt) {
    const float kp   = kp_,   kd   = kd_,   ki   = ki_,   off = angleOffset_;
    const float dead = deadband_, lim = pwmLimit_, cut = cutoff_;
    const float trim = motorTrim_;
    const float sw   = startWindow_;            // added
    const float kpV  = kpV_;                    // added
    const float kiV  = kiV_;                    // added
    const float dir  = (motorDir_ >= 0.0f) ? 1.0f : -1.0f;

    // Encoder velocity
    const long encL = g_encL, encR = g_encR;
    const long encAvg = (encL + encR) / 2;
    const float rawSpeed = float(encAvg - prevEncAvg_) / dt;
    prevEncAvg_ = encAvg;
    wheelSpeedFilt_ += SPEED_FILTER_ALPHA * (rawSpeed - wheelSpeedFilt_);

    // Outer-loop velocity-tilt bias
    float velTilt = 0.0f;
    if (state_ == BotState::Running && (kpV > 1e-4f || kiV > 1e-4f)) {
        posInteg_ = constrain(posInteg_ + float(encAvg) * dt,
                              -POS_INT_CLAMP, POS_INT_CLAMP);
        velTilt = kpV * (wheelSpeedFilt_ * VELOCITY_SPEED_SCALE)
                + kiV * (posInteg_ * VELOCITY_POS_SCALE);
        velTilt = constrain(velTilt, -VELOCITY_TILT_LIMIT, VELOCITY_TILT_LIMIT);
    } else {
        posInteg_ = 0.0f;
    }

    const float mechTilt = imu_.pitchDeg() - off;
    const float tilt     = mechTilt + velTilt;

    float u = 0.0f;
    int pwmL = 0, pwmR = 0;

    if (!armed_) {
        motors_.stop();
        state_ = BotState::Disarmed;
        resetControllerState();
    } else if (state_ == BotState::Running) {
        if (fabsf(mechTilt) > cut) {
            if (++fallCounter_ > FALL_CYCLES) {
                motors_.stop();
                state_ = BotState::Fallen;
                resetControllerState();
            }
        } else {
            fallCounter_ = 0;
            motors_.enable(true);

            pid_.setGains({kp, ki, kd});
            pid_.setIntegralLimit(80.0f / max(ki, 1e-5f));   // ki is non-volatile local → OK
            const float rawPid = pid_.update(tilt, imu_.gyroRateDps(), dt);
            u = -rawPid * dir;

            const float trimL = (trim > 1.0f) ? (1.0f / trim) : 1.0f;
            const float trimR = (trim < 1.0f) ? trim : 1.0f;
            pwmL = MotorDriver::applyDeadband(u * trimL, dead, lim);
            pwmR = MotorDriver::applyDeadband(u * trimR, dead, lim);
            motors_.setLeft(pwmL);
            motors_.setRight(pwmR);
        }
    } else {
        motors_.stop();
        if (state_ == BotState::Disarmed || state_ == BotState::ImuFail) {
            state_ = BotState::Waiting;
        }
        const float win = max(sw, 2.0f);      // both non-volatile → template deduces _Tp=float
        uprightTimer_ = (fabsf(mechTilt) < win) ? uprightTimer_ + dt : 0.0f;
        if (uprightTimer_ > UPRIGHT_HOLD_S) {
            pid_.reset();
            state_ = BotState::Running;
            fallCounter_ = 0;
        }
    }

    publishTelemetry(encL, encR, tilt, u, pwmL, pwmR);
}

void BalanceBot::publishTelemetry(long encL, long encR, float tilt,
                                  float u, int pwmL, int pwmR) {
    portENTER_CRITICAL(&stateMux_);
    telem_.state = state_;
    telem_.armed = armed_;
    telem_.pitch = imu_.pitchDeg();
    telem_.tilt  = tilt;
    telem_.rate  = imu_.gyroRateDps();
    telem_.u     = u;
    telem_.pwmL  = pwmL;
    telem_.pwmR  = pwmR;
    telem_.hz    = hz_;
    telem_.encL  = encL;
    telem_.encR  = encR;
    telem_.spd   = wheelSpeedFilt_;
    portEXIT_CRITICAL(&stateMux_);
}

// ── Watchdog ──────────────────────────────────────────────────
void BalanceBot::watchdogLoop() {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_CHECK_MS));
        const uint32_t now = millis();
        if (armed_ && (now - lastBeatMs_) > WATCHDOG_TIMEOUT_MS) {
            motors_.stop();
            armed_ = false;
            state_ = BotState::Disarmed;
            Serial.println("WATCHDOG: control loop stalled — motors stopped");
        }
    }
}

// ── Public API ────────────────────────────────────────────────
void BalanceBot::arm(bool on)    { armed_ = on; }
void BalanceBot::forceStart()    { if (armed_) state_ = BotState::Running; }
void BalanceBot::requestRecalibration() { recalReq_ = true; }
void BalanceBot::requestSave()   { saveReq_ = true; }

void BalanceBot::serviceSave() {
    if (saveReq_ && state_ != BotState::Running) {
        commitParamsToNvs();
        saveReq_ = false;
        Serial.println("Params saved to NVS");
    }
}

void BalanceBot::zeroAngleFromCurrent() {
    portENTER_CRITICAL(&stateMux_);
    const float cur = telem_.pitch;
    portEXIT_CRITICAL(&stateMux_);
    angleOffset_ = constrain(cur, -30.0f, 30.0f);
    requestSave();
}

Telemetry BalanceBot::snapshot() const {
    portENTER_CRITICAL(&stateMux_);
    Telemetry t = telem_;
    portEXIT_CRITICAL(&stateMux_);
    return t;
}

// ── Param management ──────────────────────────────────────────
void BalanceBot::buildParamTable() {
    params_[0]  = {"kp",          "Kp: Proportional (stiffness)",          &kp_,           DEF_KP,           0.0f,   100.0f, 0.2f};
    params_[1]  = {"kd",          "Kd: Derivative (gyro damping)",         &kd_,           DEF_KD,           0.0f,   5.0f,   0.01f};
    params_[2]  = {"ki",          "Ki: Integral (steady-state hold)",      &ki_,           DEF_KI,           0.0f,   60.0f,  0.1f};
    params_[3]  = {"angleOffset", "Balance angle offset (deg)",            &angleOffset_,  DEF_ANGLE_OFFSET, -30.0f, 30.0f,  0.05f};
    params_[4]  = {"startWindow", "Start angle window (±deg)",             &startWindow_,  DEF_START_WINDOW, 2.0f,   20.0f,  0.5f};
    params_[5]  = {"deadband",    "Motor deadband (PWM)",                  &deadband_,     DEF_DEADBAND,     0.0f,   100.0f, 1.0f};
    params_[6]  = {"pwmLimit",    "Max PWM limit",                         &pwmLimit_,     DEF_PWM_LIMIT,    50.0f,  255.0f, 5.0f};
    params_[7]  = {"cutoff",      "Fall cutoff angle (deg)",               &cutoff_,       DEF_CUTOFF,       10.0f,  45.0f,  1.0f};
    params_[8]  = {"kp_v",        "Kp_v: Velocity damping (stop drift)",   &kpV_,          DEF_KP_V,         0.0f,   0.20f,  0.002f};
    params_[9]  = {"ki_v",        "Ki_v: Position hold (stay in place)",   &kiV_,          DEF_KI_V,         0.0f,   0.05f,  0.001f};
    params_[10] = {"kalmanR",     "Kalman accel noise R (higher=smoother)",&kalmanR_,      DEF_KALMAN_R,     0.005f, 0.20f,  0.005f};
    params_[11] = {"dFilter",     "D-term gyro LPF (1=raw, 0.1=smooth)",   &dFilter_,      DEF_D_FILTER,     0.05f,  1.0f,   0.05f};
    params_[12] = {"motorTrim",   "Right/Left motor trim",                 &motorTrim_,    DEF_MOTOR_TRIM,   0.80f,  1.20f,  0.01f};
    params_[13] = {"motorDir",    "Motor direction (+1 / -1)",             &motorDir_,     DEF_MOTOR_DIR,    -1.0f,  1.0f,   2.0f};
}

void BalanceBot::applyTuningToSubsystems() {
    imu_.setKalmanR(kalmanR_);
    imu_.setDFilterAlpha(dFilter_);
}

bool BalanceBot::setParam(const String& name, float value) {
    for (int i = 0; i < NUM_PARAMS; ++i) {
        if (name == params_[i].name) {
            *params_[i].ptr = constrain(value, params_[i].minV, params_[i].maxV);
            applyTuningToSubsystems();
            return true;
        }
    }
    return false;
}

void BalanceBot::loadDefaults() {
    for (int i = 0; i < NUM_PARAMS; ++i) *params_[i].ptr = params_[i].def;
    applyTuningToSubsystems();
}

String BalanceBot::paramsJson() const {
    String out = "[";
    for (int i = 0; i < NUM_PARAMS; ++i) {
        if (i) out += ',';
        char item[192];
        snprintf(item, sizeof(item),
                 "{\"n\":\"%s\",\"l\":\"%s\",\"v\":%.4f,\"min\":%.4f,\"max\":%.4f,\"s\":%.4f}",
                 params_[i].name, params_[i].label, (float)*params_[i].ptr,
                 params_[i].minV, params_[i].maxV, params_[i].step);
        out += item;
    }
    out += ']';
    return out;
}

void BalanceBot::loadParamsFromNvs() {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, true);
    for (int i = 0; i < NUM_PARAMS; ++i) {
        *params_[i].ptr = prefs.getFloat(params_[i].name, params_[i].def);
    }
    prefs.end();
    applyTuningToSubsystems();
}

void BalanceBot::commitParamsToNvs() {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, false);
    for (int i = 0; i < NUM_PARAMS; ++i) prefs.putFloat(params_[i].name, *params_[i].ptr);
    prefs.end();
}

void BalanceBot::resetControllerState() {
    pid_.reset();
    uprightTimer_   = 0.0f;
    fallCounter_    = 0;
    posInteg_       = 0.0f;
}