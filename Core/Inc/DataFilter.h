/**
 * @file DataFilter.h
 * @ingroup estimation
 * @brief Exponential moving average (EMA) filters for the sensor data.
 *
 * alpha (0–1) is the smoothing factor: near 0 smooths heavily and responds
 * slowly; near 1 follows the signal closely. No sample rate or cutoff
 * frequency is needed.
 */

#ifndef KINGFISHER_SW_DATAFILTER_H
#define KINGFISHER_SW_DATAFILTER_H

// ── EmaFilter ─────────────────────────────────────────────────────────────────
/// Single-channel EMA. Seeds itself with the first sample to avoid a start-up transient.
class EmaFilter {
public:
    /// @brief Creates a filter with smoothing factor @p alpha (0–1).
    explicit EmaFilter(float alpha) : alpha_(alpha) {}

    /// @brief Adds a sample and returns the filtered value, y = αx + (1 − α)y.
    float update(float x) {
        if (!seeded_) { y_ = x; seeded_ = true; }
        y_ = alpha_ * x + (1.0f - alpha_) * y_;
        return y_;
    }

    /// @brief Sets the filter output directly.
    void seed(float value) { y_ = value; seeded_ = true; }

private:
    float alpha_;           ///< Smoothing factor.
    float y_      = 0.0f;   ///< Current output.
    bool  seeded_ = false;  ///< false until the first update() or seed().
};

// ── EmaFilter3 ────────────────────────────────────────────────────────────────
/// Three independent EMA channels with the same alpha, for a 3-axis signal.
class EmaFilter3 {
public:
    /// @brief Creates three filters that share smoothing factor @p alpha (0–1).
    explicit EmaFilter3(float alpha) : fx(alpha), fy(alpha), fz(alpha) {}

    /// @brief Filters @p x, @p y and @p z in place.
    void update(float& x, float& y, float& z) {
        x = fx.update(x);
        y = fy.update(y);
        z = fz.update(z);
    }

    /// @brief Sets all three outputs directly.
    void seed(float x, float y, float z) { fx.seed(x); fy.seed(y); fz.seed(z); }

private:
    ///@{
    EmaFilter fx, fy, fz;   ///< One filter per axis.
    ///@}
};

// ── SensorFilters ─────────────────────────────────────────────────────────────
/**
 * @brief The filters applied to each sensor frame.
 *
 * updateDataTask() calls apply() under g_ctrls_sensor_mutex, after
 * Sensors::Update() has published a frame.
 */
class SensorFilters {
public:
    /// @brief Sets altitude α = 0.1, and acceleration and rate α = 0.5.
    SensorFilters();

    /**
     * @brief Filters g_telemNow's altitude, acceleration and rates in place and copies them into g_SensorData.
     *
     * Rates are converted from deg/s to rad/s for g_SensorData. The caller
     * must hold g_ctrls_sensor_mutex.
     */
    void apply();

private:
    static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;   ///< Degrees to radians.

    EmaFilter  pressureFilter;  ///< Filters altitude (not pressure), α = 0.1. An older comment said 0.2.
    EmaFilter3 accelFilter;     ///< Acceleration, α = 0.5.
    EmaFilter3 gyroFilter;      ///< Angular rate, α = 0.5.
};

/// The single SensorFilters instance, defined in DataFilter.cpp.
extern SensorFilters g_filters;

#endif // KINGFISHER_SW_DATAFILTER_H
