//
// Created by admin on 3/16/2026.
//

/**
 * @file LIS2MDL.h
 * @ingroup sensors
 * @brief Wrapper for the ST LIS2MDL magnetometer on SPI1.
 *
 * Sensors initializes it but never reads it, and the attitude estimator runs
 * without magnetometer input.
 */

#ifndef KINGFISHER_SW_LIS2MDL_H
#define KINGFISHER_SW_LIS2MDL_H

#include "lis2mdl_reg.h"
#include "spi.h"
#include "main.h"

/// One magnetometer reading.
typedef struct {
    float B_x              = 0.0F;  ///< Magnetic field, X (gauss).
    float B_y              = 0.0F;  ///< Magnetic field, Y (gauss).
    float B_z              = 0.0F;  ///< Magnetic field, Z (gauss).
    float LIS2_Temperature = 0.0F;  ///< Die temperature (°C).
} LIS2_Data;

/**
 * @brief LIS2MDL magnetometer: 100 Hz continuous mode with temperature compensation.
 *
 * The SPI callbacks are static and use the chip-select pin CS1 directly, so
 * only one instance can exist.
 */
class Magnetometer {
public:
    /**
     * @brief Binds the ST driver to @p spi. Does not touch hardware.
     * @param spi SPI bus (&hspi1 on HAL-1).
     */
    explicit Magnetometer(SPI_HandleTypeDef* spi);

    /**
     * @brief Checks WHO_AM_I, resets the chip, and starts 100 Hz continuous mode.
     * @return 0 on success, 1 if WHO_AM_I does not match.
     * @warning Waits for the reset bit to clear with no timeout.
     */
    uint8_t   Init();

    /**
     * @brief Reads a new sample if one is ready. Does not block.
     * @return 0 on success or when no new sample is ready (the last reading is
     *         kept); 1 not initialized; 2–4 for a failed status, field, or
     *         temperature read.
     */
    uint8_t   Update();

    /// @brief Returns the reading stored by the most recent Update().
    LIS2_Data getRawData();

private:
    SPI_HandleTypeDef* _spi;                ///< SPI bus.
    stmdev_ctx_t       dev_ctx{};           ///< ST driver context.
    bool               _sensor_active = false;  ///< true once Init() succeeds.
    LIS2_Data          _raw;                ///< Last reading.

    /// @brief ST write callback. Builds the frame in a stack buffer and sends it.
    static int32_t platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len);
    /// @brief ST read callback. Sends reg | 0x80, then reads @p len bytes.
    static int32_t platform_read (void* handle, uint8_t reg, uint8_t* bufp,       uint16_t len);
    /// @brief ST delay callback. Uses HAL_Delay(), which busy-waits.
    static void    platform_delay(uint32_t millisec);

    /// @brief Drives chip select CS1 low.
    static void cs_low();
    /// @brief Drives chip select CS1 high.
    static void cs_high();
};

#endif //KINGFISHER_SW_LIS2MDL_H
