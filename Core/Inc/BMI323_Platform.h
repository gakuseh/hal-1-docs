//
// Created by dyrel on 3/31/2026.
//

/**
 * @file BMI323_Platform.h
 * @ingroup sensors
 * @brief SPI and delay callbacks that connect the Bosch BMI323 driver to the STM32 HAL.
 *
 * The Bosch driver calls these through bmi3_dev, with `ptr` pointing at the
 * sensor's BMIobjPtr. Each of the three sensors has its own TX/RX buffer pair.
 * Transfers are blocking HAL calls with a 5 s timeout.
 */

#ifndef KINGFISHER_SW_BMI323_PLATFORM_H
#define KINGFISHER_SW_BMI323_PLATFORM_H

#include "bmi323.h"

/// @warning Declared but never defined. The chip-select helpers in
///          BMI323_Platform.cpp take a BMIobjPtr* and are not in any header.
void BMI_CS_Select();
/// @warning Declared but never defined (note the spelling; the real helper is
///          BMI_CS_Deselect(BMIobjPtr*) in BMI323_Platform.cpp).
void BMI_CS_DeSelect();

/**
 * @brief Bosch read callback: reads @p len bytes starting at register @p reg.
 *
 * Sends reg | 0x80, clocks len + 1 bytes, and copies everything after the
 * address byte into @p data. The Bosch driver already adds the BMI323's dummy
 * byte to @p len.
 *
 * @param reg Register address.
 * @param[out] data Destination.
 * @param len Number of bytes to read.
 * @param ptr The sensor's BMIobjPtr.
 * @return Always 0, even if the SPI transfer failed.
 */
int8_t BMI3_SPI_Read(uint8_t reg, uint8_t* data, uint32_t len, void *ptr);

/**
 * @brief Bosch write callback: writes @p len bytes starting at register @p reg.
 * @param reg Register address. Bit 7 is cleared to mark a write.
 * @param data Bytes to write.
 * @param len Number of bytes. Must be less than 512 (the TX buffer size).
 * @param ptr The sensor's BMIobjPtr.
 * @return Always 0, even if the SPI transfer failed.
 */
int8_t BMI3_SPI_Write(uint8_t reg, const uint8_t* data, uint32_t len, void *ptr);

/**
 * @brief Bosch delay callback: busy-waits on the DWT cycle counter.
 * @param microseconds Delay length.
 * @param intf_ptr Unused.
 * @note Needs the DWT counter running (MICROS_DWT_Timebase_Init()). Does not
 *       yield to the scheduler.
 */
void BMI323_delay_us(uint32_t microseconds, void *intf_ptr);

#endif //KINGFISHER_SW_BMI323_PLATFORM_H
