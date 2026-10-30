// Created by admin on 5/10/2026.

/**
 * @file CTRLS_SensorFusion.h
 * @ingroup estimation
 * @brief Extended Kalman filter that estimates roll rate, velocity and attitude for the controller.
 */

#ifndef KINGFISHER_SW_SENSORFUSION_H
#define KINGFISHER_SW_SENSORFUSION_H

#include "CTRLS_ControlAlgorithm.h"

// ─── EKF — Extended Kalman Filter ─────────────────────────────────────────────
/**
 * @brief Extended Kalman filter built on the Physics model.
 *
 * State x = [ω1, ω2, ω3, v1, v2, v3, qw, qx, qy, qz]: body angular rates
 * (rad/s), body velocity (m/s), and attitude quaternion. Measurement
 * y = [ax, ay, az, gx, gy, gz]: accelerometer (g) and gyroscope (rad/s).
 *
 * The prediction integrates Physics::eom() with one Euler step and propagates
 * the covariance through a numerical Jacobian. The correction limits which
 * measurements can update which states (see correct_()). Before t_rail the
 * state is held to the launch rail.
 *
 * Owned by CTRLs_STATESPACE() as a function-local static.
 */
class EKF {
public:
    StateVec xhat;  ///< Current state estimate. ControlLaw reads it.

    /**
     * @brief Stores the model and config and calls reset().
     * @param phys Plant model. Must outlive this object.
     * @param cfg Noise settings and timing. Must outlive this object.
     */
    EKF(const Physics& phys, const RocketConfig& cfg);

    /**
     * @brief One prediction and correction. Call when a fresh IMU sample is available.
     * @param t Flight time (s) since liftoff. Selects the rail and burn behaviour.
     * @param dt Time step (s). The prediction is skipped unless 0 < dt ≤ 0.5.
     * @param y_meas Measurement: accelerometer (g), gyroscope (rad/s).
     * @param u Canard command applied over this step (rad).
     * @param alt Altitude (m) for the weather lookup.
     * @return The updated #xhat.
     */
    StateVec update(float t, float dt, const MeasVec& y_meas, float u,
                    float alt = 0.0f);

    /**
     * @brief Prediction only. Call when no new IMU sample is available.
     * @param t Flight time (s).
     * @param dt Time step (s).
     * @param u Canard command (rad).
     * @param alt Altitude (m) for the weather lookup.
     */
    void predictOnly(float t, float dt, float u, float alt = 0.0f);

    /**
     * @brief Resets the state to cfg.x0 and the covariances to their initial values.
     * @param cfg Supplies x0, P0_scale, the process noise and the measurement noise.
     */
    void reset(const RocketConfig& cfg);

private:
    const Physics&      phys_;  ///< Plant model.
    const RocketConfig& cfg_;   ///< Noise settings and timing.

    StateMat P_;   ///< State covariance.
    StateMat Q_;   ///< Process noise (diagonal).
    Mat<6,6> R_;   ///< Measurement noise (diagonal), normal-flight values.

    /**
     * @brief Euler-integrates #xhat over @p dt and propagates P with F = I + A·dt.
     * @param t Flight time (s).
     * @param dt Time step (s). Out-of-range steps return without changing anything.
     * @param u Canard command (rad).
     * @param alt Altitude (m) for the weather lookup.
     * @param[out] A_out ∂f/∂x at the new state, reused by correct_().
     */
    void predict_(float t, float dt, float u, float alt, StateMat& A_out);

    /**
     * @brief Measurement update with gain limiting and a Joseph-form covariance update.
     *
     * Accelerometer innovations may correct only velocity, gyroscope
     * innovations only angular rates, and nothing corrects the quaternion.
     * While @p burning, the accelerometer noise is raised to accel_burn_std_g
     * and its gains are zeroed entirely.
     */
    void correct_(float t, float u, const MeasVec& y_meas,
                  float alt, const StateMat& A,
                  bool burning);

    /// @brief Rail constraint: zeroes pitch/yaw rate and lateral velocity, keeps v3 ≥ 0, and locks the quaternion to x0.
    void applyRailConstraint_();

    /// @brief Renormalizes the quaternion part of #xhat, or resets it to identity if its norm is near zero.
    void normalizeQuaternion_();
};

#endif // KINGFISHER_SW_SENSORFUSION_H
