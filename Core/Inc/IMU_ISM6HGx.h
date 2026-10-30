//
// Created by dyrel on 2/26/2026.
//

/**
 * @file IMU_ISM6HGx.h
 * @ingroup sensors
 * @brief Driver wrapper for the ST ISM6HG256X high-g IMU on SPI3.
 *
 * Not used in the current flight build: IMUs never calls Init() or Update().
 */

#ifndef KINGFISHER_SW_IMU_ISM6HGX_H
#define KINGFISHER_SW_IMU_ISM6HGX_H

#include "ism6hg256x_reg.h"
#include "main.h"
#include "Math/Vector3D.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"

/// One averaged ISM6HG256X reading.
struct ISM_Data {
    Vector3D<float> acceleration;       ///< Low-g accelerometer (mg, ±2 g range).
    Vector3D<float> accelerationHighG;  ///< High-g accelerometer (mg, ±256 g range).
    Vector3D<float> angular_velocity;   ///< Gyroscope (mdps, ±2000 deg/s range).
    float temperature_degC;             ///< Die temperature (°C).
};

/**
 * @brief ISM6HG256X accelerometer, high-g accelerometer and gyroscope.
 *
 * Update() accumulates samples while the data-ready interrupt flag is set and
 * publishes averages every 100 low-g samples (about every 1.7 s at 60 Hz).
 */
class IMU_ISM6HGx {
    ISM_Data data;  ///< Last averaged reading.

    int16_t data_raw_motion[3];     ///< Scratch buffer for one raw 3-axis sample.
    int16_t data_raw_temperature;   ///< Scratch buffer for one raw temperature sample.
    float_t temperature_degC;       ///< Unused.
    uint8_t whoamI;                 ///< WHO_AM_I value read during Init().
    uint8_t tx_buffer[1000];        ///< Unused.
    ism6hg256x_filt_settling_mask_t filt_settling_mask; ///< Filter-settling options applied in Init().

    ism6hg256x_pin_int_route_hg_t pin_int;  ///< Routes high-g data-ready to INT1.

    ///@{
    double_t lowg_xl_sum[3], hg_xl_sum[3], gyro_sum[3], temp_sum;  ///< Running sums for averaging.
    ///@}
    ///@{
    float_t acceleration_mg[3], angular_rate_mdps[3]; ///< Unused.
    ///@}
    ///@{
    uint16_t lowg_xl_cnt, hg_xl_cnt, gyro_cnt, temp_cnt;   ///< Sample counts for averaging.
    ///@}

  stmdev_ctx_t dev_ctx;             ///< ST driver context (SPI3 callbacks).
  uint8_t lg_xl_data_valid;         ///< Unused.
  uint8_t hg_xl_data_valid;         ///< Unused.
  uint8_t gyro_data_valid;          ///< Unused.
  uint8_t temp_data_valid;          ///< Unused.
  static volatile uint8_t thread_wake;  ///< Set by ism6hg256x_read_data_drdy_handler(); consumed by Update().

public:
    /// @brief Does nothing. Call Init().
    IMU_ISM6HGx();

    /**
     * @brief Data-ready interrupt hook: sets the flag that Update() consumes.
     * @note The call in HAL_GPIO_EXTI_Callback() is commented out.
     */
    static void ism6hg256x_read_data_drdy_handler();

    /**
     * @brief Resets the chip, checks WHO_AM_I, and applies the config.
     *
     * Low-g accelerometer 60 Hz ±2 g, high-g accelerometer 960 Hz ±256 g,
     * gyroscope 120 Hz ±2000 deg/s, with low-pass filtering and high-g
     * data-ready routed to INT1.
     *
     * @return 0 on success, -1 if WHO_AM_I does not match.
     */
    int8_t Init();

    /**
     * @brief Accumulates any new samples and publishes averages when enough have arrived.
     * @return 0 when a new average was published, 1 while still accumulating.
     */
    int8_t Update();

    /// @brief Returns the last averaged reading.
    ISM_Data GetData();
};

#endif //KINGFISHER_SW_IMU_ISM6HGX_H
