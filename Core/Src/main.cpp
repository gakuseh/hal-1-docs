//
// Created by dyrel on 2/7/2026.
//

/**
 * @file main.cpp
 * @ingroup tasks
 * @brief FreeRTOS task bodies and the flight state machine.
 *
 * MX_FREERTOS_Init() in freertos.c creates these tasks:
 *
 * | Task         | Entry                    | Priority    | Stack |
 * |--------------|--------------------------|-------------|-------|
 * | defaultTask  | FC_Init()                | Normal      | 4 KB  |
 * | Radio        | Radio()                  | Realtime    | 4 KB  |
 * | updateSensor | updateSensorTask()       | Realtime    | 4 KB  |
 * | updateData   | updateDataTask()         | Realtime    | 4 KB  |
 * | CTRLs        | CTRLs()                  | Realtime    | 12 KB |
 * | SDLog        | SDLogTask()              | AboveNormal | 2 KB  |
 * | ICTare       | Initial_Condition_Tare() | BelowNormal | 512 B |
 *
 * Every task except FC_Init() waits for #initDone. Data moves between them
 * through g_telemNow, g_SensorData and g_gndData under g_ctrls_sensor_mutex,
 * and the sensor pipeline is chained with task notifications:
 * updateSensorTask() → updateDataTask() → CTRLs().
 *
 * @see docs/ARCHITECTURE.md
 */

#include "tim.h"
#include "cmsis_os.h"
#include "constants.h"
#include "TEST_FUNCTIONS.h"
#include "timing.h"

#include "FlightComputer_SENSORS.h"
#include "CTRLS_Controls.h"

TaskHandle_t updateDataTaskHandle;  ///< updateData task. updateSensorTask() notifies it after each frame.
TaskHandle_t CTRLIndicationHandle;  ///< CTRLs task. updateDataTask() notifies it after each frame.


// ======================= THREADS =========================

Sensors S;                      ///< Every sensor. FC_Init() initializes it; updateSensorTask() updates it.
Telemetry telem;                ///< The radio link. FC_Init() initializes it; the Radio task runs it.
Servo_Axon_Mini_MKII Servos;    ///< The canard servos the CTRLs task commands. Sensors has its own instance for feedback.

static volatile bool initDone = false;  ///< Set by FC_Init() once every subsystem is up. Every other task waits for it.

/// @brief Vertical (world z) velocity in m/s from the EKF state: the body velocity rotated into the world frame.
static float modelVerticalVelocityMS(const StateVec& x)
{
    float R[3][3];
    Physics::R_BW(x(6,0), x(7,0), x(8,0), x(9,0), R);
    return R[0][2]*x(3,0) + R[1][2]*x(4,0) + R[2][2]*x(5,0);
}

/**
 * @brief Boot task (CubeMX's defaultTask): brings up every subsystem, then suspends itself.
 *
 * Starts the DWT timer and turns the user LED on, then initializes, in order,
 * the sensors, the radio, the servos and the SD card. Each step retries every
 * 100 ms until it succeeds; the LED blinks while the SD card is retrying.
 * When all four are up it sets #initDone, toggles the LED, and suspends.
 *
 * @warning Every step retries forever, so one missing piece (no SD card, a GPS
 *          that never answers, servos not attached) keeps every other task,
 *          the radio included, from starting.
 */
extern "C" void FC_Init(void*) {
    MICROS_DWT_Timebase_Init();

    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);

    // Retry until every sensor (IMU A/B/C, baro A/B/C, GPS) reports success.
    // Radio init comes next, here in FC_Init; the Radio task only waits for initDone.
    while (S.Init() != STATUS_OK)
        osDelay(100);

    while (telem.Init() != STATUS_OK)
        osDelay(100);

    while (Servos.Init({0,0}, TENTH_DEGREE, false) != true)
        osDelay(100);

    while (SD_Init() != 0) {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        osDelay(100);
    }

    initDone = true;
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);

    vTaskSuspend(NULL); // suspend permanently
}


/**
 * @brief Sensor task: reads every sensor, then wakes updateDataTask(). Up to about 500 Hz.
 *
 * Each loop calls Sensors::Update(), notifies the updateData task, and sleeps 2 ms.
 */
extern "C" void updateSensorTask(void*) {
    while (!initDone) osDelay(10);

    for (;;) {
        S.Update();
        xTaskNotifyGive(updateDataTaskHandle);
        osDelay(2); // ~500 Hz ceiling; yields CPU to lower-priority tasks (e.g. Radio)
    }
}

#include "FlightComputer_DATAUPDATE.h"
#include "DataFilter.h"

extern telemetryData g_telemNow;    ///< Latest sensor, attitude and servo values, shared between tasks under g_ctrls_sensor_mutex.
extern telemetryData g_telemPrev;   ///< Copy of g_telemNow from the Radio task's last loop. Written, never read.
extern GndStationData g_gndData;    ///< Servo trim from the last GND command, set by Telemetry::receiveCommands().
extern osMutexId_t g_ctrls_sensor_mutex;

bool STATE_IGNITION  = false;  ///< Latched by updateDataTask() when ignition is detected.
bool STATE_BURNOUT   = false;   ///< Latched by updateDataTask() when burnout is detected.
bool STATE_BACKUP_PID = true; ///< true selects CTRLs_PID(). Cleared when ignition latches, which selects CTRLs_STATESPACE().
bool ALLOW_ACTUATION  = false;  ///< Gates the CTRLs task. Set at burnout unless locked out; cleared by any lockout.
uint32_t CTRLs_t      = 0;   ///< Kernel tick at ignition (0 = pre-launch). Written but never read.

static volatile bool g_request_tare = false; ///< Set by Initial_Condition_Tare(); updateDataTask() performs the tare and clears it.
static float g_alt_pad_m = 0.0f; ///< Pad altitude (barometric, MSL) recorded by the tare. The Radio task subtracts it so altitude goes out as AGL.

/**
 * @brief Data task: filtering, attitude, vertical velocity, and the flight state machine.
 *
 * Woken by updateSensorTask() after each frame (100 ms timeout). Holding
 * g_ctrls_sensor_mutex, it performs a pending pad tare, updates g_dt, filters
 * the frame with SensorFilters::apply(), runs the attitude estimator, and
 * computes vertical velocity. Then it runs the latched state machine:
 *
 * - **Ignition:** vertical acceleration above #LIFTOFF_ACCEL_G for
 *   #IGNITION_SUSTAIN_MS. Sets #STATE_IGNITION and clears #STATE_BACKUP_PID.
 * - **Burnout:** vertical acceleration below #BURNOUT_ACCEL_G for
 *   #BURNOUT_SUSTAIN_MS. Sets #STATE_BURNOUT and, unless locked out,
 *   #ALLOW_ACTUATION.
 * - **Lockout (permanent until the next tare):** |pitch| above
 *   #MAX_PITCH_ANGLE_DEG at any time, or after burnout |vertical velocity|
 *   below #MIN_ACTUATION_VEL_MS or more than #MAX_ACTUATION_DURATION_MS
 *   elapsed. Clears #ALLOW_ACTUATION and g_ctrls_enabled.
 *
 * Finally it notifies the CTRLs task.
 *
 * The pad tare resets the attitude estimator, records #g_alt_pad_m, and resets
 * the state machine, which includes clearing g_ctrls_enabled. See the bug note
 * on g_ctrls_enabled: nothing sets it again afterwards.
 */
extern "C" void updateDataTask(void*)
{
    while (!initDone) osDelay(10);

    static stateestimation::AttitudeEstimator Est;

    // Sustain timers for latched state transitions
    static uint32_t ignDetectMs   = 0U;
    static uint32_t burnDetectMs  = 0U;
    static uint32_t burnoutMs     = 0U; // kernel tick when burnout was confirmed
    static bool     actuation_locked = false; // permanent actuation disable latch

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)); // 100 ms watchdog if sensor task stalls

        if (osMutexAcquire(g_ctrls_sensor_mutex, 20) == osOK) {
            // Apply tare requested by Initial_Condition_Tare task
            if (g_request_tare) {
                g_request_tare = false;
                Est.reset(true, false); // reset attitude to identity, keep gyro bias from 3-min window
                g_alt_pad_m = g_telemNow.altitude;
                CTRLs_t = 0;

                // Full flight state machine reset — clears any lockouts accumulated
                // while the rocket was powered on the ground before tare.
                STATE_IGNITION   = false;
                STATE_BURNOUT    = false;
                STATE_BACKUP_PID = true;   // PID on by default until ignition
                ALLOW_ACTUATION  = false;
                g_ctrls_enabled  = false;
                actuation_locked = false;
                ignDetectMs      = 0U;
                burnDetectMs     = 0U;
                burnoutMs        = 0U;
            }

            DataUpdate::ComputeDt();
            g_filters.apply();
            DataUpdate::FuseAttitude(&Est);
            g_telemNow.verticalVelocity = DataUpdate::getVerticalVelocity();
            // g_telemNow.altitude stays MSL; AGL offset applied at radio snapshot only
            osMutexRelease(g_ctrls_sensor_mutex);
        }

        // Snapshot state machine inputs (benign race — worst case is one stale sample).
        // mAccZ = remapped BMI Y axis = physical rocket vertical (reads +1g upright at rest).
        const float az   = g_telemNow.mAccZ;
        const float pitch = g_telemNow.pitch;
        //const float yaw = g_telemNow.yaw;
        //const float roll = g_telemNow.roll;
        const float vvel  = g_telemNow.verticalVelocity;
        const uint32_t nowMs = osKernelGetTickCount();

        // Ignition: vertical-axis accel exceeds LIFTOFF_ACCEL_G for IGNITION_SUSTAIN_MS
        // Latched permanently once confirmed.
        if (!STATE_IGNITION) {
            if (az > LIFTOFF_ACCEL_G) {
                if (ignDetectMs == 0U) ignDetectMs = nowMs;
                if (nowMs - ignDetectMs >= IGNITION_SUSTAIN_MS) {
                    CTRLs_t          = nowMs;
                    STATE_IGNITION   = true;
                    STATE_BACKUP_PID = false;
                }
            } else {
                ignDetectMs = 0U;
            }
        }

        // Burnout: vertical axis reads downward (negative) with magnitude > BURNOUT_ACCEL_G
        // for BURNOUT_SUSTAIN_MS.
        if (!STATE_BURNOUT) {
            if (az < BURNOUT_ACCEL_G) {
                if (burnDetectMs == 0U) burnDetectMs = nowMs;
                if (nowMs - burnDetectMs >= BURNOUT_SUSTAIN_MS) {
                    STATE_BURNOUT   = true;
                    burnoutMs       = nowMs;
                    ALLOW_ACTUATION = !actuation_locked;
                }
            } else {
                burnDetectMs = 0U;
            }
        }

        // Permanent actuation disable: pitch tipped too far, velocity too low post-burnout,
        // or MAX_ACTUATION_DURATION_MS has elapsed since burnout.
        // vvel check is gated on STATE_BURNOUT — vvel is 0 on the pad and must not
        // trip the lockout before the rocket has left the rail.
        if (!actuation_locked) {
            const bool timeout  = STATE_BURNOUT &&
                                  (nowMs - burnoutMs) >= MAX_ACTUATION_DURATION_MS;
            const bool vvel_low = STATE_BURNOUT &&
                                  fabsf(vvel) < MIN_ACTUATION_VEL_MS;
            if (fabsf(pitch) > MAX_PITCH_ANGLE_DEG || vvel_low || timeout) {
                actuation_locked = true;
                ALLOW_ACTUATION  = false;
                g_ctrls_enabled  = false;
            }
        }

        // Wake CTRLs so it reads the freshest gyro data with minimum latency.
        xTaskNotifyGive(CTRLIndicationHandle);
    }
}


/**
 * @brief Radio task: sends telemetry and listens for ground-station commands.
 *
 * Each loop copies g_telemNow into telem.HALOutData (and g_telemPrev) under
 * g_ctrls_sensor_mutex, subtracting #g_alt_pad_m so altitude goes out as AGL,
 * then calls Telemetry::Update() and toggles the user LED. There is no sleep
 * in the loop; Telemetry::Update() paces it through the E22's busy waits.
 */
extern "C" void Radio(void*)
{
    while (!initDone)
        osDelay(10);

    for (;;)
    {
        // Snapshot g_telemNow into the radio's outbound buffer under the mutex,
        // then release before TX so other tasks aren't blocked during transmission.
        // AGL offset is applied here rather than mutating g_telemNow, so the
        // sensor task's continuous MSL writes don't race against the correction.
        if (osMutexAcquire(g_ctrls_sensor_mutex, 20U) == osOK) {
            telem.HALOutData          = g_telemNow;
            telem.HALOutData.altitude = g_telemNow.altitude - g_alt_pad_m;
            g_telemPrev               = g_telemNow;
            osMutexRelease(g_ctrls_sensor_mutex);
        }
        uint8_t status = telem.Update();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    }
}




extern GPS_Data g_GPS;  ///< Latest GPS fix, written by Sensors::Update() and logged by SDLogTask().

/**
 * @brief Logging task: writes three CSV-style lines to the SD card every 10 ms.
 *
 * After a header, each loop logs SENS1 (position, altitude, vertical velocity,
 * temperature), SENS2 (acceleration, rates, attitude, servo commands, RSSI,
 * pyro flags) and GPS (position, UTC time, satellites, HDOP). g_telemNow is
 * copied under the mutex; g_GPS is copied without it.
 *
 * @note Each SD_LogNewline() syncs to the card, so the real rate is limited by
 *       how fast the card syncs. The task gets no CPU during the radio's 2 s
 *       receive window.
 */
extern "C" void SDLogTask(void*) {
    while (!initDone)
        osDelay(10);

    SD_LogNewline("=== HAL-1 LOG START ===");
    SD_LogNewline("SENS1,tick_ms\tlat,lon\tbaro_alt_m,gps_alt_m\tvvel_ms\ttemp_c,");
    SD_LogNewline("SENS2,tick_ms\taccX,accY,accZ,gyrX,gyrY,gyrZ,roll,pitch,yaw,s1cmd_deg,s2cmd_deg,rssi,pyro_dm,pyro_db,pyro_mc");
    SD_LogNewline("GPS,tick_ms\tlat,lon\talt_m\thh:mm:ss\tsats\thdop");
    SD_LogNewline("DYN,tick_ms\tt_s\talt_m\tvz_ms,vh_ms\troll_rate_rads");
    SD_LogNewline("EVENT,tick_ms\tdescription");

    for (;;) {
        telemetryData s = {};
        if (osMutexAcquire(g_ctrls_sensor_mutex, 5) == osOK) {
            s = g_telemNow;
            osMutexRelease(g_ctrls_sensor_mutex);
        }
        GPS_Data gps = g_GPS;

        char ln1[112], ln2[152], ln3[96];

        snprintf(ln1, sizeof(ln1),
            "SENS1,%lu\t%.5f,%.5f\t%.1f,%.1f\t%.2f\t%.1f",
            HAL_GetTick(),
            (double)s.latitude, (double)s.longitude,
            (double)s.altitude, (double)s.GPSaltitude,
            (double)s.verticalVelocity, (double)s.temperature);

        snprintf(ln2, sizeof(ln2),
            "SENS2,%lu\t%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d,%d,%d",
            HAL_GetTick(),
            (double)s.mAccX,  (double)s.mAccY,  (double)s.mAccZ,
            (double)s.mGyrX,  (double)s.mGyrY,  (double)s.mGyrZ,
            (double)s.roll,   (double)s.pitch,   (double)s.yaw,
            (double)s.servoTarget1, (double)s.servoTarget2,
            (int)s.RSSI,
            (int)s.pyroMainDrogueFired,
            (int)s.pyroBackupDrogueFired,
            (int)s.pyroMainChuteFired);

        snprintf(ln3, sizeof(ln3),
            "GPS,%lu\t%.7f,%.7f\t%.2f\t%02u:%02u:%02u\t%u\t%.2f",
            HAL_GetTick(),
            gps.latitude, gps.longitude, gps.altitude,
            (unsigned)gps.hour, (unsigned)gps.minute, (unsigned)gps.second,
            (unsigned)gps.satellites, (double)gps.hdop);

        SD_LogNewline(ln1);
        SD_LogNewline(ln2);
        SD_LogNewline(ln3);

        osDelay(10); // 100 Hz
    }
}


// ------------- BACKUP PID CONTROLLER IF INITIAL ACCEL NOT DETECTED -------------

/**
 * @brief Backup roll damper: one proportional step, then wait for the next frame.
 *
 * The CTRLs task calls it when actuation is allowed (after burnout) but
 * ignition was never latched, so #STATE_BACKUP_PID is still set. It commands
 * both canards to −0.30° per deg/s of roll rate, limited to ±7°, writes the
 * command plus GND trim into g_telemNow.servoTarget1/2, waits up to 10 ms for
 * the next frame, and returns.
 *
 * @bug The dead-band constant is π/180 (1 deg/s expressed in rad/s) but it is
 *      compared with a rate in deg/s, so the dead-band is about 0.017 deg/s
 *      instead of 1 deg/s. The comment above it says 6 deg/s.
 * @note Does not check g_ctrls_enabled, so GND command 217 does not stop it.
 */
void CTRLs_PID()
{
    // Proportional-only roll-rate damper.
    // Closed-loop: J*wdot = -(b + Keff*KP)*w  => exponential decay for any KP > 0.
    // Theoretically stable regardless of gain magnitude; larger KP = faster settling
    // at the cost of larger canard deflections.  Tune KP on the bench: spin the
    // rocket by hand, confirm canards oppose the spin, then increase until satisfied.
    static constexpr float KP              = 0.30f;  // deg deflection per deg/s roll rate -- TUNE
    static constexpr float MAX_DEFLECT_DEG = 7.0f;  // canard travel limit -- verify against stops
    // Dead-band: suppress control below 6 deg/s to avoid fighting
    // natural spin-up and reduce servo wear at low roll rates.
    static constexpr float ROLL_RATE_THRESHOLD_RPS = M_PI/180.0f * 1.0; // last number is deg/sec

    while (!initDone) osDelay(10);

        // if (!g_ctrls_enabled) {
        //     Servos.Update(0.0f, 0.0f);
        //     continue;
        // }

        // mGyrZ = BMI.ang_vel.y = rocket spin-axis rate in deg/s (after axis remap)
        float roll_rate_dps = 0.0f;
        if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
            roll_rate_dps = g_telemNow.mGyrZ;
            osMutexRelease(g_ctrls_sensor_mutex);
        }

        if (fabsf(roll_rate_dps) <= ROLL_RATE_THRESHOLD_RPS) {
            Servos.Update(0.0f, 0.0f);
            if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
                g_telemNow.servoTarget1 = g_gndData.servoOffset1;
                g_telemNow.servoTarget2 = g_gndData.servoOffset2;
                osMutexRelease(g_ctrls_sensor_mutex);
            }
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
            return;
        }

        // Oppose the roll: positive rate -> negative deflection, and vice versa.
        // "positive angle -> positive roll moment" (servo docstring), so negating
        // the rate produces a restoring moment.
        float cmd = -KP * roll_rate_dps;

        if (cmd >  MAX_DEFLECT_DEG) cmd =  MAX_DEFLECT_DEG;
        if (cmd < -MAX_DEFLECT_DEG) cmd = -MAX_DEFLECT_DEG;

        // AngleOffsetDEGREES (ground-station trim) is applied inside Actuate().
        Servos.Update(cmd, cmd);

        if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
            g_telemNow.servoTarget1 = cmd + g_gndData.servoOffset1;
            g_telemNow.servoTarget2 = cmd + g_gndData.servoOffset2;
            osMutexRelease(g_ctrls_sensor_mutex);
        }

        // Block until updateDataTask signals that a fresh sensor frame is ready.
        // 10 ms timeout is a safety fallback only — under normal operation the
        // notification arrives every ~2 ms (updateSensorTask rate).
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
}


// ------------------------- STATE SPACE MODEL -------------------------
/**
 * @brief Main roll controller: EKF plus gain-scheduled control law, in its own loop.
 *
 * Reloads the Control Freak config on every entry; the Physics, EKF and
 * ControlLaw objects are built once. Each iteration:
 *
 * 1. Zeroes the servos and returns if g_ctrls_enabled or #ALLOW_ACTUATION is
 *    clear, or #STATE_BACKUP_PID is set.
 * 2. Takes the latest g_SensorData sample if it is fresh.
 * 3. Runs EKF::update(), or EKF::predictOnly() without a fresh sample, and the
 *    roll effectiveness monitor.
 * 4. Detects apogee (modelled vertical speed below 3 m/s after burnout), which
 *    disables control and strobes the LED for 2 s.
 * 5. Computes the command with ControlLaw::computeControl() and drives both
 *    canards with it.
 * 6. Waits up to 10 ms for the next frame.
 *
 * While active it also logs a DYN line to the SD card every 20 ms. Flight time
 * comes from g_SensorData.flight_time_s, which starts at liftoff detection in
 * Sensors::Update().
 *
 * @bug When it returns early, the CTRLs task calls it again immediately with no
 *      delay, so while #ALLOW_ACTUATION is set and g_ctrls_enabled is clear,
 *      the CTRLs task spins at Realtime priority.
 * @warning The DYN log calls SD_LogNewline(), which syncs to the card, from
 *          inside the control loop.
 */
void CTRLs_STATESPACE() {
    // ── Init: build algorithm objects ────────────────────────────────────────
    // Mutexes (g_ctrls_sensor_mutex, g_ctrls_output_mutex) are created by
    // ctrlsInit() in MX_FREERTOS_Init before the scheduler starts, so they are
    // safe to acquire as soon as this task runs.
    static RocketConfig cfg;
    loadControlFreakRocketConfig(cfg);
    cfg.computeDerived();
    static Physics    phys(cfg);
    static EKF        ekf(phys, cfg);
    static ControlLaw ctrl(phys, cfg);

    while (!initDone) osDelay(10);


    // ── Update loop ──────────────────────────────────────────────────────────
    static uint32_t prev_ms         = 0U;
    static uint32_t launch_start_ms = 0U;
    static float    model_alt_m     = 0.0f;
    static float    t_zero_s        = 0.0f;  // subtracted from snap.flight_time_s to reset EKF clock
    static bool     apogee_latched  = false;
    static uint32_t apogee_blink_ms = 0U;
    static bool     apogee_blink_done = false;
    static uint32_t dyn_log_ms      = 0U;

    float u_last_rad         = 0.0f;
    CtrlsSensorSnapshot snap = {};
    snap.temperature_K       = 288.15f;

    for (;;)
        {
            // Remote kill or algorithm switch — return so the task loop re-evaluates.
            if (!g_ctrls_enabled || !ALLOW_ACTUATION || STATE_BACKUP_PID) {
                Servos.Update(0.0f, 0.0f);
                return;
            }

            // Reset flight clock if GND commanded state-space from t=0.
            const uint32_t now_ms = millis();
            const float dt = (prev_ms == 0U) ? 0.01f
                             : static_cast<float>(now_ms - prev_ms) * 1e-3f;
            prev_ms = now_ms;
            if (dt <= 0.0f || dt > 0.5f) { osDelay(1); continue; }

            // ── Read sensor snapshot ─────────────────────────────────────────────
            bool fresh = false;
            if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
                if (g_SensorData.fresh) {
                    snap = g_SensorData;
                    g_SensorData.fresh = false;
                    fresh = true;
                }
                osMutexRelease(g_ctrls_sensor_mutex);
            }

            // Reset flight clock if GND commanded state-space from t=0.
            // Capture current snap.flight_time_s as the new zero so that
            // subsequent t values are relative to the moment of the command.
            if (g_ctrls_reset_time) {
                g_ctrls_reset_time = false;
                launch_start_ms    = 0U;
                t_zero_s           = snap.flight_time_s;
                ekf.reset(cfg);
                ctrl.reset();
            }

            const float t = snap.flight_time_s - t_zero_s;
            const float sensor_alt_m = snap.altitude_m;
            const float ekf_alt_for_step = fresh ? sensor_alt_m : model_alt_m;

            // ── EKF step ─────────────────────────────────────────────────────────
            if (fresh) {
                const WeatherSample wx = phys.weatherAtAltitude(ekf_alt_for_step);
                float va[3];
                phys.airRelativeVelocity(ekf.xhat, wx.wind_x, wx.wind_y, va);
                ctrl.updateRollEffectivenessSign(t, snap.gyro_rad_s[2], u_last_rad,
                                                  sqrtf(va[0]*va[0] + va[1]*va[1] + va[2]*va[2]));
                MeasVec y;
                for (int i = 0; i < 3; i++) {
                    y(i,   0) = snap.accel_g[i];
                    y(3+i, 0) = snap.gyro_rad_s[i];
                }
                ekf.update(t, dt, y, u_last_rad, ekf_alt_for_step);
            } else {
                ekf.predictOnly(t, dt, u_last_rad, ekf_alt_for_step);
            }

            // ── Sensor-aided model altitude + apogee detection ────────────────────
            const float model_vz_ms = modelVerticalVelocityMS(ekf.xhat);
            if (fresh)
                model_alt_m = sensor_alt_m;
            else
                model_alt_m += model_vz_ms * dt;
            if (model_alt_m < 0.0f) model_alt_m = 0.0f;

            // DYN log at 10 Hz — only while controls are active
            if (g_ctrls_enabled && !apogee_latched && now_ms - dyn_log_ms >= 20U) {
                dyn_log_ms = now_ms;
                const float vx = ekf.xhat(3,0), vy = ekf.xhat(4,0);
                const float horiz_v   = sqrtf(vx*vx + vy*vy);
                const float roll_rate = ekf.xhat(2,0);
                char dyn_ln[72];
                snprintf(dyn_ln, sizeof(dyn_ln), "DYN,%lu\t%.2f\t%.1f\t%.2f,%.2f\t%.3f",
                    now_ms, t, model_alt_m, model_vz_ms, horiz_v, roll_rate);
                SD_LogNewline(dyn_ln);
            }

            // Apogee: vz drops below 3 m/s after burnout
            if (!apogee_latched && t > cfg.t_burnout && model_vz_ms < 3.0f) {
                apogee_latched  = true;
                apogee_blink_ms = now_ms;
                g_ctrls_enabled = false;
                Servos.Update(0.0f, 0.0f);
                SD_LogNewline("EVENT,ModelApogee");
            }

            // Non-blocking LED strobe for 2 s on apogee
            if (apogee_latched && !apogee_blink_done) {
                const uint32_t elapsed = now_ms - apogee_blink_ms;
                if (elapsed < 2000U) {
                    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin,
                        (elapsed % 200U) < 100U ? GPIO_PIN_SET : GPIO_PIN_RESET);
                } else {
                    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
                    apogee_blink_done = true;
                }
            }

            // ── Control law ───────────────────────────────────────────────────────
            // EKF always runs above to keep the state estimate warm.
            // Output is zeroed when controls are disabled so servos hold neutral.
            const float control_alt = model_alt_m;
            const float u_rad = g_ctrls_enabled
                ? ctrl.computeControl(t, ekf.xhat, control_alt)
                : 0.0f;
            u_last_rad = u_rad;
            const float u_deg = u_rad * (180.0f / static_cast<float>(M_PI));

            // ── Write outputs ─────────────────────────────────────────────────────
            if (osMutexAcquire(g_ctrls_output_mutex, 2U) == osOK) {
                g_ctrls_canard_cmd_deg = u_deg;
                osMutexRelease(g_ctrls_output_mutex);
            }

            // Both canards receive the same deflection command to produce roll.
            // Ground-station offsets apply the per-servo zero-point trim.
            Servos.Update(u_deg, u_deg);
            if (osMutexAcquire(g_ctrls_sensor_mutex, 20) == osOK) {
                g_telemNow.servoTarget1 = u_deg + g_gndData.servoOffset1;
                g_telemNow.servoTarget2 = u_deg + g_gndData.servoOffset2;
                osMutexRelease(g_ctrls_sensor_mutex);
            }

            // Block until updateDataTask signals that a fresh sensor frame is ready.
            // 10 ms timeout is a safety fallback only.
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        }
}


// CONTROLS TASK. Selects between PID and state space controllers based on flight conditions.
// Algorithm selection is one-way: PID runs until ignition latches STATE_BACKUP_PID=false,
// then CTRLs_STATESPACE() runs its own for(;;) loop. It returns whenever control is
// disabled, and this loop calls it again immediately.
// When ALLOW_ACTUATION is false, yield via ulTaskNotifyTake rather than busy-loop.
extern "C" void CTRLs(void*)
{
    while (!initDone) osDelay(10);

    for (;;) {
        if (!ALLOW_ACTUATION) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
            continue;
        }
        if (STATE_BACKUP_PID) {
            CTRLs_PID();
        } else {
            CTRLs_STATESPACE();
        }
    }
}

/**
 * @brief Pad-tare task: waits for 3 minutes of stillness, then asks updateDataTask() to tare.
 *
 * Monitors the IMU for a 3-minute stability window (rocket sitting upright on
 * the pad), sampling g_telemNow at 10 Hz. A sample counts as still when every
 * change from the previous one is under #TARE_ACC_DELTA_G and
 * #TARE_GYRO_DELTA_DPS; any larger change restarts the #TARE_STABILITY_MS
 * clock. When the clock runs out it sets #g_request_tare, so updateDataTask
 * can zero the attitude estimator, pad altitude, and CTRLs_t in a mutex-safe
 * context, and suspends itself. The tare therefore happens once per boot.
 */
extern "C" void Initial_Condition_Tare(void*) {
    while (!initDone) osDelay(10);

    float    prevAx = 0.0f, prevAy = 0.0f, prevAz = 0.0f;
    float    prevGx = 0.0f, prevGy = 0.0f, prevGz = 0.0f;
    uint32_t stableStart = 0U;
    bool     firstSample = true;

    for (;;) {
        osDelay(100U); // 10 Hz sampling

        float ax, ay, az, gx, gy, gz;
        if (osMutexAcquire(g_ctrls_sensor_mutex, 5) != osOK) continue;
        ax = g_telemNow.mAccX; ay = g_telemNow.mAccY; az = g_telemNow.mAccZ;
        gx = g_telemNow.mGyrX; gy = g_telemNow.mGyrY; gz = g_telemNow.mGyrZ;
        osMutexRelease(g_ctrls_sensor_mutex);

        if (firstSample) {
            prevAx = ax; prevAy = ay; prevAz = az;
            prevGx = gx; prevGy = gy; prevGz = gz;
            stableStart = osKernelGetTickCount();
            firstSample = false;
            continue;
        }

        const bool stable =
            fabsf(ax - prevAx) < TARE_ACC_DELTA_G  &&
            fabsf(ay - prevAy) < TARE_ACC_DELTA_G  &&
            fabsf(az - prevAz) < TARE_ACC_DELTA_G  &&
            fabsf(gx - prevGx) < TARE_GYRO_DELTA_DPS &&
            fabsf(gy - prevGy) < TARE_GYRO_DELTA_DPS &&
            fabsf(gz - prevGz) < TARE_GYRO_DELTA_DPS;

        prevAx = ax; prevAy = ay; prevAz = az;
        prevGx = gx; prevGy = gy; prevGz = gz;

        if (!stable) {
            stableStart = osKernelGetTickCount();
        } else if (osKernelGetTickCount() - stableStart >= TARE_STABILITY_MS) {
            g_request_tare = true;
            vTaskSuspend(NULL); // tare requested; no further work to do
        }
    }
}
