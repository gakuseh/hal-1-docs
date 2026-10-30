/**
 * @file BMP3xx_platform.h
 * @ingroup sensors
 * @brief SPI and delay callbacks that connect the Bosch BMP390 driver to the STM32 HAL.
 *
 * Each of the three barometers has its own TX/RX buffer pair. Transfers are
 * blocking HAL calls with a 5 s timeout.
 */

#pragma once
#ifndef BMP3XX_PLATFORM_H
#define BMP3XX_PLATFORM_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/// SPI context passed to the Bosch driver callbacks as `intf_ptr`.
typedef struct
{
    SPI_HandleTypeDef* spi_handle;  ///< SPI bus (&hspi2).
    GPIO_TypeDef* cs_port;          ///< Chip-select port.
    uint16_t cs_pin;                ///< Chip-select pin. Also used to pick the buffer pair.
} bmp3_spi_intf;

/*
 * Function prototypes required by Bosch BMP390 driver
 */

/**
 * @brief Bosch read callback: reads @p len bytes starting at register @p reg_addr.
 *
 * Sends reg | 0x80, clocks len + 1 bytes, and copies everything after the
 * address byte into @p data. The Bosch driver already adds the BMP390's
 * dummy byte to @p len.
 *
 * @param reg_addr Register address.
 * @param[out] data Destination.
 * @param len Number of bytes to read.
 * @param ptr The sensor's bmp3_spi_intf.
 * @return Always 0, even if the SPI transfer failed.
 */
int8_t BMP_SPI_Read(uint8_t reg_addr,
                     uint8_t *data,
                     uint32_t len,
                     void *ptr);

/**
 * @brief Bosch write callback: writes @p len bytes starting at register @p reg_addr.
 * @param reg_addr Register address. Bit 7 is cleared to mark a write.
 * @param data Bytes to write.
 * @param len Number of bytes. Must be less than 512.
 * @param ptr The sensor's bmp3_spi_intf.
 * @return Always 0, even if the SPI transfer failed.
 */
int8_t BMP_SPI_Write(uint8_t reg_addr,
                      const uint8_t *data,
                      uint32_t len,
                      void *ptr);

/**
 * @brief Bosch delay callback: busy-waits on the DWT cycle counter.
 * @param period Delay in microseconds.
 * @param intf_ptr Unused.
 * @note Needs the DWT counter running (MICROS_DWT_Timebase_Init()). Does not
 *       yield to the scheduler.
 */
void BMP_delay_us(uint32_t period, void *intf_ptr);

#endif
