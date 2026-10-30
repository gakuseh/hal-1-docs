//
// Created by dyrel on 2/21/2026.
//

/**
 * @file Servo_Axon_Mini_MKII.h
 * @ingroup actuation
 * @brief Driver for the two Axon Mini MK2 canard servos: PWM out on TIM3, angle feedback on ADC3.
 *
 * @attention
 * This library is intended for use with the Axon Mini MK2 Servo ONLY.
 * operations are specialized for the specific use on Ctrl-Frk, but
 * users may find portions of the code written here useful for future
 * rocketry applications.
 */

#ifndef KINGFISHER_SW_SERVO_AXON_MINI_MKII_H
#define KINGFISHER_SW_SERVO_AXON_MINI_MKII_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "projdefs.h"

/// Tolerance for the position check in Servo_Axon_Mini_MKII::Init().
enum PRECISION {
    ONE_DEGREE,         ///< 1°.
    TENTH_DEGREE,       ///< 0.1°, about one ADC count. Used by the flight code.
    HUNDREDTH_DEGREE,   ///< 0.01°.
    THOUSANTHS_DEGREE   ///< 0.001°.
};

/// A pair of values, one per canard servo.
template <typename T>
struct SAsym {
    T S1;   ///< Servo 1: PWM on TIM3 CH1 (PC6), feedback on ADC3 INP0 (PC2).
    T S2;   ///< Servo 2: PWM on TIM3 CH2 (PC7), feedback on ADC3 INP1 (PC3).
};

/// Commands, feedback and status for both servos.
struct DATA_Axon_Mini_MKII{
    SAsym<float> targetAngle;   ///< Last commanded angles (deg, before offsets).
    SAsym<float> currentAngle;  ///< Last measured angles (deg).
    SAsym<float> trackedError;  ///< |measured − target| from the last Update() or Init() (deg).
    SAsym<float> trackedAngle;  ///< Copy of targetAngle, used by calculateError().
    SAsym<bool> Nominal = {false, false};   ///< Per-servo result of Init()'s position check.
    SAsym<float> smoothedCorrection = {0.0f, 0.0f}; ///< Always zero; closed-loop correction is disabled.
};

/// Settings shared by both servos.
struct CFG_Axon_Mini_MKII{
    uint8_t Precision;              ///< A #PRECISION value.
    bool ENABLE_DEBUG;              ///< Stored but not used.
    SAsym<float> AngleOffsetDEGREES;    ///< Trim added to every command (deg).
};

/**
 * @brief Both canard servos: open-loop PWM commands and analog angle feedback.
 *
 * Commands map angle to pulse width at 1500 µs + 5.56 µs per degree (TIM3
 * counts at 2 MHz). Feedback reads each servo's analog position output on
 * ADC3 and converts volts to degrees at 360° per 3.3 V, relative to the
 * voltage captured at centre during Init().
 *
 * There are two instances: the global `Servos` in main.cpp, which the CTRLs
 * task commands, and the one inside Sensors, which Sensors::Update() reads.
 *
 * @warning S1 drives TIM3 CH1 (PC6, which CubeMX names SERVO2_PWM) but reads
 *          ADC3 INP0 (PC2, named SERVO1_ENC); S2 is crossed the same way. In
 *          CubeMX's naming, each servo's command and feedback belong to
 *          different connectors. Telemetry::receiveCommands() swaps the GND
 *          offsets, probably to compensate.
 */
class Servo_Axon_Mini_MKII{
private:
    CFG_Axon_Mini_MKII config = {}; ///< Settings shared by both servos.
    DATA_Axon_Mini_MKII data = {};  ///< Latest commands and feedback.

    /**
     * @brief Reads both feedback voltages from ADC3 and smooths them.
     *
     * Starts one scan conversion, polls both channels (5 ms timeout each),
     * converts to volts, and applies an EMA with α = 0.5.
     *
     * @return Smoothed voltages (V), no offset applied.
     * @note The EMA state is a function-local static, so every instance
     *       shares it. The comment in the source says α = 0.1.
     */
    SAsym<float> readRawVoltage();
    /// @brief Converts readRawVoltage() to degrees relative to #_zeroVoltage (360° per 3.3 V).
    SAsym<float> readCurrentAngle();
    /// @brief Relative error between the measured angle and trackedAngle. Unused.
    SAsym<float> calculateError();
    /// @brief Writes both PWM compare registers for @p input plus the configured offsets (deg).
    void Actuate(SAsym<float> input);

    SAsym<float> _zeroVoltage = {1.65f, 1.65f}; ///< Feedback voltage at mechanical centre, captured in Init(). Defaults to 1.65 V.

public:
    /// @brief Does nothing. Call Init().
    Servo_Axon_Mini_MKII();

    /**
     * @brief Starts PWM, centres both servos, records the zero reference, and checks they hold position.
     *
     * Calibrates ADC3, starts TIM3 CH1 and CH2, commands 0°, waits 200 ms, and
     * records the feedback voltages as zero. Then, for up to 3 s at 200 Hz, it
     * commands the offset angles and checks the feedback is within
     * @p precision.
     *
     * @param AngOffset Trim added to every command (deg).
     * @param precision Tolerance for the check.
     * @param debug Stored in the config; not used.
     * @return true if both servos were within tolerance.
     * @warning At #TENTH_DEGREE the tolerance is about one ADC count (0.9 mV),
     *          so the check can fail on noise alone. With no servos attached
     *          it always fails, and FC_Init() retries it forever.
     */
    bool Init(  SAsym<float> AngOffset,
                PRECISION precision,
                bool debug);

    /**
     * @brief Commands both servos and refreshes the feedback data.
     *
     * Open loop: closed-loop correction is disabled because the feedback
     * angle is not in the same frame as the command.
     *
     * @param S1_Target_Deg Servo 1 angle (deg, before offset).
     * @param S2_Target_Deg Servo 2 angle (deg, before offset).
     */
    void Update(float S1_Target_Deg, float S2_Target_Deg);

    /// @brief Reads both feedback angles (deg from the Init() zero).
    SAsym<float> getCurrentAngle();

    /**
     * @brief Changes the trim and immediately re-sends the last command.
     * @param offset New offsets (deg).
     * @note Unused.
     */
    void SetOffset(SAsym<float> offset);

    /// @brief Returns the latest commands, feedback and status.
    DATA_Axon_Mini_MKII getData() const;
};

#endif //KINGFISHER_SW_SERVO_AXON_MINI_MKII_H
