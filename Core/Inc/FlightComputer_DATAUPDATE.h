//
// Created by admin on 5/19/2026.
//

/**
 * @file FlightComputer_DATAUPDATE.h
 * @ingroup estimation
 * @brief Per-frame derived quantities: time step, attitude, and vertical velocity.
 *
 * updateDataTask() calls these once per sensor frame while holding
 * g_ctrls_sensor_mutex.
 */

#ifndef KINGFISHER_SW_FLIGHTCOMPUTER_DATAUPDATE_H
#define KINGFISHER_SW_FLIGHTCOMPUTER_DATAUPDATE_H

#include "attitude_estimator.h"
#include "Math/Math.h"
#include "Telemetry.h"
#include "timing.h"

extern float g_dt; ///< Seconds between the last two ComputeDt() calls. 0 after the first call.

/// Turns the raw readings in g_telemNow into derived quantities.
namespace DataUpdate
{
    /**
     * @brief Updates #g_dt from the DWT cycle counter.
     *
     * Subtracts raw cycle counts before converting to seconds, so the result
     * stays correct when the counter wraps (unlike differences of micros()).
     */
    void  ComputeDt();

    /**
     * @brief Runs one attitude-estimator step and writes the quaternion and Euler angles into g_telemNow.
     *
     * Feeds the gyro (converted to rad/s) and accelerometer from g_telemNow,
     * with no magnetometer. Does nothing while #g_dt is 0.
     *
     * @param est The estimator owned by updateDataTask().
     * @warning The roll and yaw assignments do not match the comment above them
     *          in the source. The comment maps eulerYaw() to physical roll, but
     *          the code stores eulerPitch() in roll and eulerYaw() in yaw.
     *          Pitch matches. Check on the bench which one is right.
     */
    void  FuseAttitude(stateestimation::AttitudeEstimator* est);

    /**
     * @brief Vertical velocity (m/s) from barometric altitude differences.
     *
     * Samples g_telemNow.altitude at most every #VVEL_UPDATE_PERIOD_MS and
     * returns the previous value in between. Altitude changes under 0.1 m read
     * as zero, so speeds below about 2 m/s report 0. Clamped to ±300 m/s.
     *
     * @return Vertical velocity, positive up. 0 on the first call.
     */
    float getVerticalVelocity();
}

#endif //KINGFISHER_SW_FLIGHTCOMPUTER_DATAUPDATE_H
