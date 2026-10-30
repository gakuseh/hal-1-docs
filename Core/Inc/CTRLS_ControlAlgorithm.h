// Created by admin on 5/10/2026.

/**
 * @file CTRLS_ControlAlgorithm.h
 * @ingroup controls
 * @brief Rocket model for the controller: fixed-size matrices, rocket parameters, and the 6-DOF physics.
 *
 * Physics implements the equations of motion that the EKF predicts with and
 * that ControlLaw uses for its gain. The model and its constants are ported
 * from the FV-Controls simulation for the Control Freak rocket.
 */

#ifndef KINGFISHER_SW_CONTROLALGORITHM_H
#define KINGFISHER_SW_CONTROLALGORITHM_H

#include <cmath>
#include <cstring>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846f    ///< π, if `<cmath>` did not define it.
#endif

// ─── Fixed-size matrix library (zero heap, all stack) ────────────────────────
/**
 * @brief Fixed-size row-major float matrix. Zero-initialized, no heap.
 * @tparam R Rows.
 * @tparam C Columns.
 */
template<int R, int C>
struct Mat {
    float d[R][C];  ///< Elements, row-major.

    /// @brief Creates a zero matrix.
    Mat() { memset(d, 0, sizeof(d)); }
    /// @brief Element at row @p r, column @p c. No bounds checking.
    float&       operator()(int r, int c)       { return d[r][c]; }
    /// @brief Element at row @p r, column @p c (const). No bounds checking.
    const float& operator()(int r, int c) const { return d[r][c]; }
};

/// @brief Matrix product A·B.
template<int R, int K, int C>
inline Mat<R,C> operator*(const Mat<R,K>& A, const Mat<K,C>& B) {
    Mat<R,C> out;
    for(int r=0;r<R;r++) for(int c=0;c<C;c++) for(int k=0;k<K;k++)
        out(r,c) += A(r,k)*B(k,c);
    return out;
}

/// @brief Element-wise sum.
template<int R, int C>
inline Mat<R,C> operator+(const Mat<R,C>& A, const Mat<R,C>& B) {
    Mat<R,C> out;
    for(int r=0;r<R;r++) for(int c=0;c<C;c++) out(r,c) = A(r,c)+B(r,c);
    return out;
}

/// @brief Element-wise difference.
template<int R, int C>
inline Mat<R,C> operator-(const Mat<R,C>& A, const Mat<R,C>& B) {
    Mat<R,C> out;
    for(int r=0;r<R;r++) for(int c=0;c<C;c++) out(r,c) = A(r,c)-B(r,c);
    return out;
}

/// @brief Transpose.
template<int R, int C>
inline Mat<C,R> Tr(const Mat<R,C>& A) {
    Mat<C,R> out;
    for(int r=0;r<R;r++) for(int c=0;c<C;c++) out(c,r) = A(r,c);
    return out;
}

/// @brief N×N identity matrix.
template<int N>
inline Mat<N,N> eye() {
    Mat<N,N> I;
    for(int i=0;i<N;i++) I(i,i) = 1.0f;
    return I;
}

/// @brief Scalar times matrix.
template<int R, int C>
inline Mat<R,C> operator*(float s, const Mat<R,C>& A) {
    Mat<R,C> out;
    for(int r=0;r<R;r++) for(int c=0;c<C;c++) out(r,c) = s*A(r,c);
    return out;
}

/**
 * @brief Inverts a 6×6 matrix by LU decomposition with partial pivoting.
 *
 * Used for the innovation covariance in the EKF correction.
 *
 * @param A Matrix to invert.
 * @return A⁻¹, or the identity if a pivot is below 1e-12 (singular).
 */
Mat<6,6> inv6(const Mat<6,6>& A);

// Type aliases matching the spec
/// EKF state: [ω1, ω2, ω3, v1, v2, v3, qw, qx, qy, qz]. Body rates (rad/s), body velocity (m/s), attitude quaternion.
using StateVec = Mat<10,1>;
/// 10×10 state matrix (covariance, Jacobian).
using StateMat = Mat<10,10>;
/// IMU measurement: [ax, ay, az, gx, gy, gz]. Acceleration (g), angular rate (rad/s).
using MeasVec  = Mat<6,1>;
/// Measurement Jacobian ∂y/∂x.
using MeasMat  = Mat<6,10>;
/// Input Jacobian ∂f/∂u for the scalar canard command.
using InputMat = Mat<10,1>;

/// Atmosphere and wind at one altitude, from Physics::weatherAtAltitude().
struct WeatherSample {
    float rho           = 1.225f;   ///< Air density (kg/m³).
    float temperature_K = 288.15f;  ///< Air temperature (K).
    float g             = 9.80665f; ///< Gravity (m/s²).
    float wind_x        = 0.0f;     ///< Wind, world frame (m/s).
    float wind_y        = 0.0f;     ///< Wind, world frame (m/s).
};

// ─── Rocket configuration ─────────────────────────────────────────────────────
/**
 * @brief Rocket mass, geometry, aerodynamics, controller limits and EKF noise.
 *
 * The defaults are fallbacks only. CTRLs_STATESPACE() calls
 * loadControlFreakRocketConfig() and then computeDerived() before use.
 */
struct RocketConfig {
    // Mass (kg)
    float m_0   = 14.0f;   ///< Launch mass (kg).
    float m_f   = 11.5f;   ///< Burnout mass (kg).
    float m_prop= 2.5f;    ///< Propellant mass (kg).

    // Inertia (kg·m²)
    float I_0   = 0.85f;   ///< Pitch/yaw inertia at ignition (kg·m²).
    float I_f   = 0.70f;   ///< Pitch/yaw inertia at burnout (kg·m²).
    float I_3_0 = 0.008f;  ///< Roll inertia at ignition (kg·m²).
    float I_3_f = 0.006f;  ///< Roll inertia at burnout (kg·m²).

    // CG travel (m from nose)
    float x_CG_0 = 0.95f;   ///< CG at ignition (m from nose).
    float x_CG_f = 0.90f;   ///< CG at burnout (m from nose).

    // Geometry
    float d    = 0.098f;   ///< Airframe diameter (m).
    float L_ne = 1.85f;    ///< Nose-to-nozzle length (m).

    // Timing (s)
    float t_rail    = 0.6f;     ///< Time to leave the rail (s after liftoff). The EKF applies the rail constraint before this.
    float t_burnout = 3.5f;     ///< Motor burnout (s after liftoff).
    float t_apogee  = 18.0f;    ///< Expected apogee (s). Unused.

    // Barrowman aero
    float base_cn              = 3.2f;   ///< Body normal-force slope (per rad).
    float base_cp              = 1.15f;  ///< Body centre of pressure (m from nose).
    float canard_cn            = 0.8f;   ///< Canard normal-force slope (per rad).
    float canard_cp            = 0.12f;  ///< Canard centre of pressure (m from nose).
    float canard_plane_angle_deg = 45.0f;   ///< Angle of the canard plane about the body axis (deg). Sets canard_dir_1/2.

    // Fin roll geometry
    int   N_fins       = 4;      ///< Number of fins.
    float Cr           = 0.12f;  ///< Fin root chord (m).
    float Ct           = 0.06f;  ///< Fin tip chord (m).
    float s            = 0.07f;  ///< Fin semi-span (m).
    float delta_deg    = 2.0f;   ///< Fin cant angle (deg). Drives the fin roll forcing.
    float Cnalpha_fin  = 2.0f;   ///< Fin normal-force slope (per rad).

    // Canard CFD roll moment
    float canard_moment_coeff_per_deg = -2.23e-6f;  ///< Canard roll moment per (m/s)² of airspeed per degree of deflection (N·m), from CFD.

    // Controller
    float roll_damping_lambda  = 30.0f;    ///< Desired closed-loop roll pole (1/s).
    float max_deflection_rad   = 0.2618f;  ///< Canard deflection limit (rad).
    float max_deflection_rate  = 1.0472f;  ///< Canard rate limit (rad/s).
    float min_control_speed    = 30.0f;    ///< Below this airspeed (m/s) the gain is zero.
    bool  irec_compliant       = true;     ///< true: no canard command before burnout.

    // EKF process noise
    float Q_angular[3]    = {1e-4f, 1e-4f, 1e-4f};          ///< Process noise, angular rates.
    float Q_velocity[3]   = {1e-2f, 1e-2f, 1e-2f};          ///< Process noise, velocity.
    float Q_quat[4]       = {1e-6f, 1e-6f, 1e-6f, 1e-6f};   ///< Process noise, quaternion.

    // EKF measurement noise
    float accel_noise_std_g  = 0.05f;   ///< Accelerometer noise (g), normal flight.
    float gyro_noise_std     = 0.01f;   ///< Gyroscope noise (rad/s).
    float accel_model_std_g  = 0.1f;    ///< Model-mismatch noise (g). Unused.
    float accel_burn_std_g   = 0.5f;    ///< Accelerometer noise during the burn (g).
    float P0_scale           = 0.1f;    ///< Initial covariance is P0_scale × I.

    // Environment
    float rho_0     = 1.225f;   ///< Sea-level density (kg/m³). Fallback only; the weather table is used.
    float g_0       = 9.80665f; ///< Gravity (m/s²). Fallback only.
    float v_wind_x  = 0.0f;     ///< Wind, world frame (m/s). Used only by the 3-argument airRelativeVelocity().
    float v_wind_y  = 0.0f;     ///< Wind, world frame (m/s). Used only by the 3-argument airRelativeVelocity().

    // Initial state. Control Freak runtime overrides this from rail-angle setup.
    float x0[10] = {0,0,0, 0,0,0, 1,0,0,0};   ///< Initial EKF state (see #StateVec). The quaternion also fixes attitude on the rail.

    // Thrust curve table (time vs Newtons)
    static constexpr int MAX_TABLE = 132;   ///< Capacity of each lookup table.
    float thrust_t[MAX_TABLE] = {};         ///< Thrust curve times (s).
    float thrust_N[MAX_TABLE] = {};         ///< Thrust at each time (N).
    int   thrust_n = 0;                     ///< Points in the thrust curve.

    // Power-on drag table (Mach vs Cd)
    float drag_on_mach[MAX_TABLE] = {};     ///< Power-on drag table, Mach.
    float drag_on_cd[MAX_TABLE]   = {};     ///< Power-on drag coefficient.
    int   drag_on_n = 0;                    ///< Points in the power-on table.

    // Power-off drag table
    float drag_off_mach[MAX_TABLE] = {};    ///< Power-off drag table, Mach.
    float drag_off_cd[MAX_TABLE]   = {};    ///< Power-off drag coefficient.
    int   drag_off_n = 0;                   ///< Points in the power-off table.

    // ── Roll effectiveness monitor (REM) ──────────────────────────────────────
    // Detects reversed canard roll effectiveness at runtime and flips the
    // control gain sign.  Disabled by default.
    bool  rem_enabled             = false;   ///< Turns on ControlLaw::updateRollEffectivenessSign(). Off by default; Control Freak enables it.
    float rem_post_burn_delay_s   = 0.25f;   ///< Seconds after burnout before monitoring starts.
    float rem_min_cmd_rad         = 0.0349f; ///< Ignore commands smaller than this (rad, about 2°).
    float rem_min_expected_accel  = 0.5f;    ///< Ignore samples where the predicted |ω̇3| is below this (rad/s²).
    float rem_min_measured_accel  = 0.5f;    ///< Ignore samples where the measured |ω̇3| is below this (rad/s²).
    int   rem_required_mismatches = 5;       ///< Consecutive sign disagreements needed to flip.
    bool  rem_allow_flip_back     = true;    ///< Allow more than one flip.

    // Derived — call computeDerived() once after populating all fields above
    float delta_rad      = 0.0f;  ///< Derived: fin cant angle (rad).
    float canard_dir_1   = 1.0f;  ///< Derived: cos(canard_plane_angle).
    float canard_dir_2   = 0.0f;  ///< Derived: sin(canard_plane_angle).
    float A_ref          = 0.0f;  ///< Derived: reference area π(d/2)² (m²).

    /// @brief Fills the derived fields. Call once after setting everything else.
    void computeDerived() {
        delta_rad  = delta_deg * (M_PI / 180.0f);
        const float cpa = canard_plane_angle_deg * (M_PI / 180.0f);
        canard_dir_1 = cosf(cpa);
        canard_dir_2 = sinf(cpa);
        A_ref = M_PI * (d*0.5f) * (d*0.5f);
    }
};

/**
 * @brief Loads the Control Freak flight parameters into @p cfg.
 *
 * Values come from the FV-Controls RocketPy setup: masses, inertias, CG,
 * aerodynamics, controller limits (9° deflection, about 429°/s rate), EKF
 * noise, the launch attitude for a 45° rail, the roll effectiveness monitor
 * (enabled), and the thrust and drag tables. Call computeDerived() afterwards.
 *
 * @param[out] cfg Config to fill.
 */
void loadControlFreakRocketConfig(RocketConfig& cfg);

/**
 * @brief Piecewise-linear table lookup with binary search. Clamps outside the table.
 * @param xs Ascending x values.
 * @param ys y values.
 * @param n Number of points.
 * @param xi Query point.
 * @return Interpolated y, the end value outside the range, or 0 if @p n is 0.
 */
float lerp(const float* xs, const float* ys, int n, float xi);

// ─── Physics ──────────────────────────────────────────────────────────────────
/**
 * @brief 6-DOF rocket model: time-varying mass properties, aerodynamics, and equations of motion.
 *
 * Mass, inertia and CG vary linearly from ignition to burnout and are constant
 * after. Atmosphere and wind come from a built-in weather profile. Forces and
 * moments follow FV-Controls momentsforces.py.
 */
class Physics {
public:
    const RocketConfig& cfg; ///< Parameters. Must outlive this object.

    /**
     * @brief Stores @p cfg and precomputes the fin roll-forcing and roll-damping constants.
     * @param cfg Parameters, with computeDerived() already applied.
     */
    explicit Physics(const RocketConfig& cfg);

    // Time-varying parameter accessors
    /// @brief Mass (kg) at time @p t: linear from m_0 down by m_prop during the burn, then m_f.
    float getMass(float t) const;
    /// @brief Pitch/yaw inertia (kg·m²) at time @p t: linear from I_0 to I_f during the burn.
    float getInertiaPitch(float t) const;
    /// @brief Roll inertia (kg·m²) at time @p t: linear from I_3_0 to I_3_f during the burn.
    float getInertiaRoll(float t) const;
    /// @brief Rate of change of pitch/yaw inertia (kg·m²/s). Constant during the burn, 0 after.
    float getInertiaDotPitch(float t) const;
    /// @brief Rate of change of roll inertia (kg·m²/s). Constant during the burn, 0 after.
    float getInertiaDotRoll(float t) const;
    /// @brief CG position (m from nose) at time @p t.
    float getCG(float t) const;
    /// @brief Thrust (N) from the thrust table. 0 before ignition, after burnout, or outside the table.
    float getThrust(float t) const;
    /**
     * @brief Drag coefficient from the power-on table before burnout and the power-off table after.
     * @param mach Mach number.
     * @param t Flight time (s).
     * @return Cd, or 0.45 if the tables are empty.
     */
    float getDragCoeff(float mach, float t) const;
    /**
     * @brief Atmosphere and wind at @p alt_m, interpolated from the built-in weather profile.
     *
     * The profile comes from FV-Controls weather_profile.csv and covers 0 to
     * 4000 m above the launch site in 50 m steps; values clamp outside that.
     *
     * @param alt_m Height above the launch site (m).
     */
    WeatherSample weatherAtAltitude(float alt_m) const;
    /// @brief Speed of sound (m/s) in air at @p T_kelvin: √(1.4 · 287.05 · T).
    static float speedOfSound(float T_kelvin);

    // Rotation helpers
    /**
     * @brief World-to-body rotation matrix (DCM) from a unit quaternion.
     * @param qw,qx,qy,qz Quaternion components.
     * @param[out] R The 3×3 matrix.
     */
    static void R_BW(float qw, float qx, float qy, float qz, float R[3][3]);
    /// @brief Air-relative velocity in the body frame, using the config's fixed wind. Only jacobianB() uses this.
    void airRelativeVelocity(const StateVec& x, float v_air[3]) const;
    /**
     * @brief Air-relative velocity in the body frame: body velocity minus the wind rotated into the body frame.
     * @param x State (velocity and quaternion are used).
     * @param wind_x,wind_y World-frame wind (m/s).
     * @param[out] v_air Air-relative velocity (m/s).
     */
    void airRelativeVelocity(const StateVec& x, float wind_x, float wind_y,
                             float v_air[3]) const;

    // Canard roll moment (CFD numeric model)
    /// @brief Canard roll moment (N·m) for airspeed @p v_mag and deflection @p zeta_rad: coeff · v² · ζ(deg).
    float canardMoment(float v_mag, float zeta_rad) const;
    /// @brief ∂M/∂ζ (N·m per rad) at airspeed @p v_mag. ControlLaw divides by this to set its gain.
    float canardMomentJacobian(float v_mag) const;

    /**
     * @brief Full nonlinear equations of motion, ẋ = f(t, x, u).
     *
     * Forces: gravity, thrust along +z, drag, body normal force, and canard
     * normal force in the canard plane. Moments: corrective (base and canard),
     * pitch/yaw damping, fin-cant roll forcing, roll damping, and the canard
     * roll moment. Angle of attack is clamped to ±15°. Before t_rail there is
     * no lift or moment. Rotation follows Euler's equations with I1 = I2.
     *
     * @param t Flight time (s).
     * @param x State.
     * @param u_zeta Canard deflection (rad).
     * @param alt Height above the launch site (m) for the weather lookup.
     * @return ẋ.
     */
    StateVec eom(float t, const StateVec& x, float u_zeta, float alt = 0.0f) const;

    /// @brief Numerical Jacobian ∂f/∂x by central differences (step 1e-5, 20 eom() calls).
    StateMat jacobianA(float t, const StateVec& x, float u, float alt = 0.0f) const;

    /// @brief Analytic ∂f/∂u. Only the roll-rate row is non-zero. Unused.
    InputMat jacobianB(float t, const StateVec& x) const;

    /**
     * @brief Measurement Jacobian ∂y/∂x (6×10).
     *
     * Accelerometer rows use the velocity rows of @p A plus the derivatives of
     * ω×v and of gravity in the body frame, scaled to g, with the angular-rate
     * columns zeroed. Gyroscope rows are the identity on the rate columns.
     *
     * @param t Flight time (s).
     * @param x State.
     * @param u Canard deflection (rad).
     * @param A ∂f/∂x at the same state, from jacobianA().
     * @param alt Height above the launch site (m) for the weather lookup.
     */
    MeasMat jacobianC(float t, const StateVec& x, float u,
                      const StateMat& A, float alt = 0.0f) const;

    /**
     * @brief Predicted IMU reading for state @p x.
     *
     * Accelerometer: specific force (v̇ + ω×v − g_body) in g. Gyroscope: ω.
     *
     * @return [accel (g), gyro (rad/s)].
     */
    MeasVec predictMeasurement(float t, const StateVec& x, float u,
                               float alt = 0.0f) const;

private:
    // Precomputed fin roll geometry constants (computed in constructor)
    float K_f;   ///< Fin cant roll forcing coefficient (N·m per unit dynamic pressure).
    float K_d;   ///< Roll damping coefficient, used as 0.5·ρ·v·K_d·ω3.
    float y_ma;  ///< Mean aerodynamic chord lateral offset (m).
    float C_ldw; ///< Roll damping derivative.
    float r_t;   ///< Body radius at the fin root (m).
    float tau;   ///< Barrowman span ratio (s + r_t) / r_t.
};

#endif // KINGFISHER_SW_CONTROLALGORITHM_H
