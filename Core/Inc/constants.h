//
// Created by dyrel on 4/16/2026.
//

/**
 * @file constants.h
 * @ingroup platform
 * @brief Flight thresholds, timing constants, and the master radio channel.
 *
 * The flight state machine in updateDataTask() and the sensor code read these.
 * The apogee, descent and main-deploy thresholds are not used by any compiled
 * code; they belonged to an earlier stage-tracking design.
 */

#ifndef KINGFISHER_SW_CONSTANTS_H
#define KINGFISHER_SW_CONSTANTS_H

#include "stm32h7xx_hal.h"
#include "Ebyte_E22_900T22S_defs.h"

// -- MASTER RADIO FREQUENCY --
/// E22 channel used at boot and in every outgoing frame header (909.125 MHz).
/// HAL-1-GND must use the same channel.
#define GLOBAL_RADIO_CHANNEL CH909

// ── Abort accumulator ─────────────────────────────────────────────────────────
#define COMMS_ERR_THRESHOLD     5       ///< Unused. Consecutive TX/RX failures before warning.

// ── Liftoff detection ─────────────────────────────────────────────────────────
#define LIFTOFF_ACCEL_G         3.0f    ///< Liftoff and ignition threshold (g). Sensors::Update() compares the total acceleration magnitude; updateDataTask() compares vertical acceleration.
#define LIFTOFF_SUSTAIN_MS      200u    ///< Sensors::Update(): acceleration must stay above #LIFTOFF_ACCEL_G this long to start the flight clock.

// ── Stage transition thresholds ───────────────────────────────────────────────
#define BURNOUT_ACCEL_G         (-1.0f) ///< updateDataTask(): vertical acceleration (g) below this counts as burnout. Signed, not a magnitude.
#define APOGEE_APPROACH_VEL_MS  10.0f   ///< Unused.
#define APOGEE_VEL_MS           3.0f    ///< Unused. (CTRLs_STATESPACE() hard-codes its own 3 m/s apogee check.)
#define APOGEE_PASS_VEL_MS      (-3.0f) ///< Unused.
#define DESCENT_VEL_MS          (-7.0f) ///< Unused.
#define FINAL_DESCENT_VEL_MS    (-9.0f) ///< Unused.

// ── Altitude thresholds (metres MSL — calibrate to launch site) ───────────────
#define MIN_APOGEE_ALT_M        2500.0f ///< Unused.
#define MAX_MAIN_DEPLOY_ALT_M   400.0f  ///< Unused.
#define MIN_MAIN_DEPLOY_ALT_M   250.0f  ///< Unused.
#define TARGET_MAIN_ALT_M       300.0f  ///< Unused.

// ── Ignition detection ────────────────────────────────────────────────────────
#define IGNITION_SUSTAIN_MS     300u    ///< updateDataTask(): vertical acceleration must exceed #LIFTOFF_ACCEL_G this long to latch ignition.

// ── Burnout detection ─────────────────────────────────────────────────────────
#define BURNOUT_SUSTAIN_MS          200u    ///< updateDataTask(): vertical acceleration must stay below #BURNOUT_ACCEL_G this long to latch burnout.
#define MAX_ACTUATION_DURATION_MS   25000u  ///< updateDataTask(): actuation is locked out this long after burnout.

// ── Controls activation gate ──────────────────────────────────────────────────
#define MAX_PITCH_ANGLE_DEG     30.0f   ///< updateDataTask(): |pitch| above this locks out actuation for the rest of the flight.
#define MIN_ACTUATION_VEL_MS    2.0f    ///< updateDataTask(): after burnout, |vertical velocity| below this locks out actuation.

// ── Vertical velocity ─────────────────────────────────────────────────────────
#define VVEL_UPDATE_PERIOD_MS   50u     ///< DataUpdate::getVerticalVelocity(): minimum interval between altitude-difference samples.

// ── Initial condition tare ────────────────────────────────────────────────────
#define TARE_STABILITY_MS       (3u * 60u * 1000u)  ///< Initial_Condition_Tare(): the IMU must be still this long (3 min) before the pad tare runs.
#define TARE_ACC_DELTA_G        0.2f    ///< Initial_Condition_Tare(): largest sample-to-sample acceleration change (g) that still counts as still.
#define TARE_GYRO_DELTA_DPS     7.0f    ///< Initial_Condition_Tare(): largest sample-to-sample rate change (deg/s) that still counts as still.

// ── Pyrotechnic pending bitmask (written by Radio task, consumed by PyroTask) ─
#define PYRO_MAIN_BIT           (1u << 2)   ///< Unused. There is no pyro task.
/// Unused. Defined in Initial_conditions.cpp; nothing writes or reads it.
extern volatile uint32_t g_pyroPending;

// ── Sensor sanity bounds ──────────────────────────────────────────────────────
#define BARO_PRESS_MIN_PA       10000.0f    ///< Barometer::Update() discards readings below this (Pa).
#define BARO_PRESS_MAX_PA       110000.0f   ///< Barometer::Update() discards readings above this (Pa).

#endif //KINGFISHER_SW_CONSTANTS_H
