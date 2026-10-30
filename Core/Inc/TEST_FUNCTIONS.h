//
// Created by dyrel on 2/23/2026.
//

/**
 * @file TEST_FUNCTIONS.h
 * @ingroup platform
 * @brief Bench tests for individual subsystems. The flight code never calls them.
 *
 * Most tests loop forever, so call one from a task in place of the normal
 * flight tasks.
 */

#ifndef HAL_1_SW_TEST_FUNCTIONS_H
#define HAL_1_SW_TEST_FUNCTIONS_H

#include <stdint.h>
#include "Servo_Axon_Mini_MKII.h"
#include "IMUs.h"
#include "Barometer.h"
#include "LIS2MDL.h"
#include "Telemetry.h"
#include "usart.h"
#include <cstring>
#include "task.h"
#include "SDCard.h"

/// Bench tests, one per subsystem.
class TEST{
public:
    /// @brief Does nothing.
    TEST();
    /// @brief Inits both servos, then cycles them through 90/−33°, −45/45° and 0/0° forever.
    /// @return 0, only if servo init failed.
    int SERVO_TEST();
    /// @brief Inits the three BMI323s and reads them every tick forever, toggling the LED.
    int IMU_TEST();
    /// @brief Inits the three barometers and reads them every 100 ms forever, toggling the LED.
    /// Uses HAL_Delay() because vTaskDelay() hard-faulted here (a TODO in the source).
    int BARO_TEST();
    /// @brief Inits the magnetometer and reads it every 100 ms forever, toggling the LED.
    int LIS2_TEST();
    /// @brief Standalone radio link: sends barometer altitude through Telemetry::Update() every 10 ms forever.
    int RADIO_TEST();
    /// @brief Fires the main drogue pyro channel for 5 s after a 1 s delay, then blinks forever.
    int PYRO_TEST();
    /// @brief Writes 10 test rows to the SD card and closes it, then blinks slowly on success or quickly on failure, forever.
    int SD_TEST();

    /// @brief Logs voted IMU readings to the SD card as fast as it can, forever.
    void NoiseTest();
};

#endif //HAL_1_SW_TEST_FUNCTIONS_H
