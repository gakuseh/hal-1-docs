//
// Created by dyrel on 4/4/2026.
//

/**
 * @file ISM6_Platform.h
 * @ingroup sensors
 * @brief SPI and delay callbacks that connect the ST ISM6HG256X driver to the STM32 HAL.
 *
 * Used only by IMU_ISM6HGx, which the flight build does not run.
 */

#ifndef KINGFISHER_SW_ISM6_PLATFORM_H
#define KINGFISHER_SW_ISM6_PLATFORM_H

#include <stdint.h>
#include <cstring>
#include <spi.h>

/// Shared SPI TX buffer (512 bytes) for the ISM6HG256X.
inline uint8_t  ISM_GTX[512] = {0},
                ISM_GRX[2048] = {0};    ///< Shared SPI RX buffer (2048 bytes) for the ISM6HG256X.

/// @brief Drives the ISM6HG256X chip select low (selected).
void ISM_CS_Low();
/// @brief Drives the ISM6HG256X chip select high (deselected).
void ISM_CS_High();
/// @brief Does nothing. CubeMX already configures SPI3 and the chip-select pin.
void Platform_Init();

/**
 * @brief ST write callback: writes @p len bytes starting at register @p reg.
 *
 * Sets the auto-increment bit for multi-byte writes.
 *
 * @param handle SPI handle (&hspi3).
 * @param reg Register address.
 * @param bufp Bytes to write.
 * @param len Number of bytes. Must be less than 512.
 * @return Always 0.
 */
int32_t ISM_SPI_Write(  void *handle,
                        uint8_t reg,
                        const uint8_t *bufp,
                        uint16_t len);

/**
 * @brief ST read callback: reads @p len bytes starting at register @p reg.
 *
 * Sets the read bit, and the auto-increment bit for multi-byte reads.
 *
 * @param handle SPI handle (&hspi3).
 * @param reg Register address.
 * @param[out] bufp Destination.
 * @param len Number of bytes. Must be less than 2048.
 * @return Always 0.
 */
int32_t ISM_SPI_Read(   void *handle,
                        uint8_t reg,
                        uint8_t *bufp,
                        uint16_t len);

/// @brief ST delay callback. Calls osDelay(), so it yields to the scheduler.
/// @param ms Delay in milliseconds.
void ISM_platform_MSdelay(uint32_t ms);

#endif //KINGFISHER_SW_ISM6_PLATFORM_H
