// Created by admin on 5/10/2026.

/**
 * @file CTRLS_Controls.h
 * @ingroup controls
 * @brief Canard roll controller, the sensor snapshot it consumes, and the controls globals.
 */

#ifndef KINGFISHER_SW_CONTROLS_H
#define KINGFISHER_SW_CONTROLS_H

#include <cstdint>
#include "CTRLS_ControlAlgorithm.h"
#include "CTRLS_SensorFusion.h"
#include "cmsis_os.h"
#include "FlightComputer_SENSORS.h"

// ─── Sensor snapshot shared between the main FC task and ctrlsTask ───────────
/**
 * @brief One sensor sample handed from the sensor pipeline to the CTRLs task.
 *
 * Sensors::Update() writes it, and SensorFilters::apply() then overwrites the
 * altitude, acceleration and rates with filtered values, both under
 * #g_ctrls_sensor_mutex. CTRLs_STATESPACE() copies it and clears #fresh.
 * Body axes: x and y lateral, z along the rocket, positive forward.
 */
struct CtrlsSensorSnapshot {
    float    accel_g[3]     = {};       ///< Body-frame acceleration (g).
    float    gyro_rad_s[3]  = {};       ///< Body-frame angular rate (rad/s).

    /// @brief Filtered barometric altitude (m). This is pressure altitude above sea
    /// level, not height above the pad as older comments said.
    /// @bug CTRLs_STATESPACE() passes this to the EKF and controller, whose
    ///      weather table is indexed by height above the launch site, so every
    ///      weather lookup is off by the site's elevation.
    float    altitude_m     = 0.0f;
    float    temperature_K  = 288.15f;  ///< Barometer temperature (K). Diagnostics only.
    float    flight_time_s  = 0.0f;     ///< Seconds since liftoff was detected. 0 before liftoff.
    STAGE    stage          = PREFLIGHT;    ///< Never set.
    uint32_t timestamp_ms   = 0;        ///< HAL_GetTick() when written.
    bool     fresh          = false;    ///< true until CTRLs_STATESPACE() consumes the sample.
};

// ─── Globals (defined in CTRLS_Controls.cpp) ─────────────────────────────────
/// Latest sensor sample for the controller. Guarded by #g_ctrls_sensor_mutex.
extern CtrlsSensorSnapshot g_SensorData;
/// Priority-inheritance mutex guarding g_SensorData, g_telemNow and g_gndData. Created by ctrlsInit().
extern osMutexId_t         g_ctrls_sensor_mutex;
extern float               g_ctrls_canard_cmd_deg;   ///< Last state-space canard command (deg), written under #g_ctrls_output_mutex. Nothing reads it.
extern osMutexId_t         g_ctrls_output_mutex;     ///< Guards #g_ctrls_canard_cmd_deg. Created by ctrlsInit().
/**
 * Master enable for the state-space controller. Starts true. Cleared by the
 * pad tare, GND command 217, apogee detection, and the actuation lockouts.
 *
 * @bug Only Telemetry::processPyros() sets it back to true, and nothing calls
 *      that. The pad tare (after 3 minutes still on the pad) clears it, so in a
 *      normal launch the state-space controller never actuates:
 *      CTRLs_STATESPACE() returns at its first check.
 */
extern volatile bool       g_ctrls_enabled;
extern volatile bool       g_ctrls_test_mode;         ///< Unused.
extern volatile bool       g_ctrls_reset_time;        ///< Pulse true to restart the state-space flight clock and filters. Only Telemetry::processPyros() sets it.

/// @brief Creates g_ctrls_sensor_mutex and g_ctrls_output_mutex (priority inheritance).
///        MX_FREERTOS_Init() calls it before any task starts.
extern "C" void ctrlsInit(void);

// ─── Gain-scheduled state-feedback roll controller ───────────────────────────
/**
 * @brief Gain-scheduled proportional roll-rate damper for the canards.
 *
 * The command is u = −K · s · ω3, where ω3 is the estimated roll rate, s is the
 * roll-effectiveness sign (±1), and K = I3 · λ / (∂M/∂ζ) places the
 * closed-loop roll pole at −λ (RocketConfig::roll_damping_lambda). The output
 * is rate limited, clamped, and forced to zero before burnout when
 * RocketConfig::irec_compliant is set.
 */
class ControlLaw {
public:
    /// @brief Stores the model and config, which must outlive this object.
    ControlLaw(const Physics& phys, const RocketConfig& cfg);

    /**
     * @brief Computes the canard deflection. Call after the EKF step.
     *
     * Zero below RocketConfig::min_control_speed, at t ≤ 0, and before
     * t_burnout when irec_compliant. Rate limited to max_deflection_rate and
     * clamped to ±max_deflection_rad.
     *
     * @param t Flight time (s) since liftoff.
     * @param xhat EKF state estimate.
     * @param alt Altitude (m) for the weather lookup.
     * @return Deflection (rad), the same for both canards.
     */
    float computeControl(float t, const StateVec& xhat, float alt = 0.0f);

    /**
     * @brief Roll effectiveness monitor: flips the gain sign if the canards act backwards.
     *
     * Compares the measured roll acceleration (from successive gyro readings)
     * with what the canard model predicts for the previous command. After
     * rem_required_mismatches consecutive disagreements it flips the sign that
     * computeControl() uses. Skipped until rem_post_burn_delay_s after burnout,
     * for small commands, and for small accelerations. Does nothing unless
     * RocketConfig::rem_enabled. Call once per loop.
     *
     * @param t Flight time (s).
     * @param w3_meas Current measured roll rate (rad/s).
     * @param u_prev_rad Command applied in the previous step (rad).
     * @param vmag Current air-relative speed (m/s).
     */
    void  updateRollEffectivenessSign(float t, float w3_meas,
                                       float u_prev_rad, float vmag);

    /// @brief Returns the current roll-effectiveness sign, +1 or −1.
    float rollEffectivenessSign() const { return rem_sign_; }

    /// @brief Clears the rate limiter and the effectiveness monitor (sign back to +1).
    void reset();

private:
    const Physics&      phys_;  ///< Plant model.
    const RocketConfig& cfg_;   ///< Tuning and limits.
    float last_u_  = 0.0f;      ///< Previous command (rad), for rate limiting.
    float last_t_  = 0.0f;      ///< Previous call time (s), for rate limiting.

    // Roll effectiveness monitor state
    float rem_sign_          =  1.0f;   ///< Gain sign, +1 or −1.
    int   rem_flip_count_    =  0;      ///< Flips so far.
    int   rem_mismatch_count_=  0;      ///< Consecutive sign disagreements.
    float rem_prev_t_        = -1.0f;   ///< Previous sample time. Negative until the first call.
    float rem_prev_w3_       =  0.0f;   ///< Previous roll-rate sample (rad/s).

    /**
     * @brief Gain K = I3 · λ / (∂M/∂ζ) for the current airspeed.
     * @return K, or 0 below min_control_speed. Does not include the
     *         effectiveness sign; computeControl() applies it.
     */
    float gainSchedule_(float t, const StateVec& xhat, float alt) const;
};

// ─── FreeRTOS controls task ───────────────────────────────────────────────────
/**
 * @brief Entry point of the CTRLs task, defined in main.cpp.
 *
 * Created in MX_FREERTOS_Init() at Realtime priority with a 12 KB stack.
 * Idles until ALLOW_ACTUATION is set at burnout, then runs CTRLs_PID() while
 * STATE_BACKUP_PID is set and CTRLs_STATESPACE() once ignition has latched.
 * updateDataTask() wakes it with a task notification after each sensor frame.
 *
 * @param arg Unused.
 */
extern "C" void CTRLs(void* arg);

#endif // KINGFISHER_SW_CONTROLS_H
