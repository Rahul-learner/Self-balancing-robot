#pragma once
#include <Arduino.h>
#include "config.h"
#include "control.h"
#include "imu.h"
#include "motors.h"

enum class BotState : int {
    Disarmed = 0,
    Waiting  = 1,
    Running  = 2,
    Fallen   = 3,
    ImuFail  = 4,
};

struct Telemetry {
    BotState state     = BotState::Disarmed;
    bool     armed     = false;
    float    pitch     = 0;
    float    tilt      = 0;
    float    rate      = 0;
    float    u         = 0;
    int      pwmL      = 0;
    int      pwmR      = 0;
    uint32_t hz        = 0;
    long     encL      = 0;
    long     encR      = 0;
    float    spd       = 0;
};

struct Param {
    const char* name;
    const char* label;
    volatile float* ptr;
    float def, minV, maxV, step;
};

class BalanceBot {
public:
    static constexpr int NUM_PARAMS = 14;

    // ── Lifecycle ──────────────────────────────────────────────
    bool begin();               // init hardware, load params from NVS, seed estimator
    void startTasks();          // spawn control (core 1, prio 5) + watchdog (core 0, prio 2)

    // ── Control ────────────────────────────────────────────────
    void  arm(bool on);
    void  forceStart();          // jump straight to Running (useful when held)
    bool  isArmed() const { return armed_; }
    BotState state() const { return state_; }

    // ── Tuning ─────────────────────────────────────────────────
    void   requestRecalibration();
    void   requestSave();        // queued; commits when not Running
    void   serviceSave();        // call from loop() — writes NVS when safe
    void   loadDefaults();
    bool   setParam(const String& name, float value);
    String paramsJson() const;

    // ── Angle calibration ──────────────────────────────────────
    void zeroAngleFromCurrent();

    // ── Telemetry ──────────────────────────────────────────────
    Telemetry snapshot() const;

    // ── Subsystem access (for diagnostics/serial CLI) ──────────
    Mpu6050& imu() { return imu_; }

private:
    Mpu6050       imu_;
    MotorDriver   motors_;
    PidController pid_;

    mutable portMUX_TYPE stateMux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile BotState state_     = BotState::Disarmed;
    volatile bool     armed_     = false;
    volatile bool     recalReq_  = false;
    volatile bool     saveReq_   = false;
    volatile uint32_t hz_        = 0;
    volatile uint32_t lastBeatMs_ = 0;
    Telemetry         telem_;

    // Tuning (web task writes, control task reads)
    volatile float kp_          = DEF_KP;
    volatile float kd_          = DEF_KD;
    volatile float ki_          = DEF_KI;
    volatile float angleOffset_ = DEF_ANGLE_OFFSET;
    volatile float startWindow_ = DEF_START_WINDOW;
    volatile float deadband_    = DEF_DEADBAND;
    volatile float pwmLimit_    = DEF_PWM_LIMIT;
    volatile float cutoff_      = DEF_CUTOFF;
    volatile float kpV_         = DEF_KP_V;
    volatile float kiV_         = DEF_KI_V;
    volatile float kalmanR_     = DEF_KALMAN_R;
    volatile float dFilter_     = DEF_D_FILTER;
    volatile float motorTrim_   = DEF_MOTOR_TRIM;
    volatile float motorDir_    = DEF_MOTOR_DIR;

    Param params_[NUM_PARAMS];

    // Controller state (control task only)
    float uprightTimer_    = 0.0f;
    int   fallCounter_     = 0;
    long  prevEncAvg_      = 0;
    float wheelSpeedFilt_  = 0.0f;
    float posInteg_        = 0.0f;

    // Task entry points
    void controlLoop();
    void watchdogLoop();
    static void controlTrampoline(void* p);
    static void watchdogTrampoline(void* p);

    // Internals
    void  buildParamTable();
    void  applyTuningToSubsystems();
    void  loadParamsFromNvs();
    void  commitParamsToNvs();
    void  resetControllerState();
    void  runControllerCycle(float dt);
    void  publishTelemetry(long encL, long encR, float tilt, float u, int pwmL, int pwmR);
};