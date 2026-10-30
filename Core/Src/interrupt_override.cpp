//
// Created by dyrel on 2/28/2026.
//

/**
 * @file interrupt_override.cpp
 * @ingroup tasks
 * @brief The application's GPIO interrupt callback.
 */

#include "main.h"

extern volatile bool e22_data_ready;

/**
 * @brief HAL callback for every EXTI line (overrides the HAL's weak default).
 *
 * The only active branch sets #e22_data_ready on a radio AUX rising edge,
 * which nothing acts on. The IMU and barometer interrupt branches are empty,
 * and the ISM6HG256X branch is commented out.
 *
 * @param GPIO_Pin The pin that fired.
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // if (GPIO_Pin == ISM_INT_Pin)  // ISM6HGx not used this flight
    // {
    //     IMU_ISM6HGx::ism6hg256x_read_data_drdy_handler();
    // }

    if(GPIO_Pin == RADIO_AUX_Pin)
    {
        // AUX high
        e22_data_ready = true;
    }

    // Note: LIS2MDL uses a different approach to interrupts in 4-wire SPI mode.
    // We must read the STATUS_REG register, instead.

    if (GPIO_Pin == SIMU_INT1_Pin) {

    }

    if (GPIO_Pin == SIMU_INT2_Pin) {

    }

    if (GPIO_Pin == SIMU_INT3_Pin) {

    }

    if (GPIO_Pin == BMP390_INT1_Pin) {

    }

    if (GPIO_Pin == BMP390_INT2_Pin) {

    }

    if (GPIO_Pin == BMP390_INT3_Pin) {

    }
}