/**
 * @file timing.h
 * @ingroup platform
 * @brief Millisecond and microsecond time from the HAL tick and the DWT cycle counter.
 */

#ifndef KINGFISHER_SW_MILLIS_H
#define KINGFISHER_SW_MILLIS_H

#include "stm32h7xx_hal.h"
#include "core_cm7.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    /// @brief Enables the DWT cycle counter and resets it to 0.
    void MICROS_DWT_DWT_Init(void);
    /// @brief Caches the core clock in MHz for micros() and enables the DWT counter.
    ///        FC_Init() calls this first, before any sensor driver needs the counter.
    void MICROS_DWT_Timebase_Init(void);
    /// @brief Milliseconds since boot (HAL_GetTick()).
    uint32_t millis(void);
    /**
     * @brief Microseconds since the DWT counter was reset.
     * @warning The cycle counter wraps every 2³² cycles (about 8.9 s at
     *          480 MHz). Because this divides before wrapping, the difference
     *          of two readings is wrong across a wrap. Subtract raw
     *          DWT->CYCCNT values instead, as DataUpdate::ComputeDt() does.
     */
    uint32_t micros(void);

#ifdef __cplusplus
}
#endif

#endif
