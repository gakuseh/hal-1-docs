//
// Created by admin on 3/16/2026.
//

/**
 * @file BMP390.h
 * @ingroup sensors
 * @brief Wrapper for one Bosch BMP390 barometer.
 */

#ifndef KINGFISHER_SW_BMP390_H
#define KINGFISHER_SW_BMP390_H

#include "BMP3xx_platform.h"
#include "bmp3.h"

/* Sea level reference pressure for altitude calculation */
/// Reference pressure (Pa) for the altitude formula: the standard atmosphere, not the local sea-level pressure.
#define SEA_LEVEL_PRESSURE 101325.0f

/// One barometer reading.
typedef struct
{
    float heightMeters = 0.0F;  ///< Pressure altitude (m) relative to #SEA_LEVEL_PRESSURE. Not corrected for the day's weather.
    float Pressure = 0.0F;      ///< Pressure (Pa).
    float Temperature = 0.0F;   ///< Temperature (°C).
} BMP_Data;

/**
 * @brief One BMP390 barometer, configured for flight.
 *
 * Init() sets normal mode at 100 Hz with 2× oversampling on pressure and
 * temperature.
 */
class Barometer
{
public:
    /// Which of the three BMP390s, which selects the chip-select pin.
    enum BMP_INDEX
    {
        SENSOR1_I,  ///< Chip select BMP390_CS1.
        SENSOR2_I,  ///< Chip select BMP390_CS2.
        SENSOR3_I   ///< Chip select BMP390_CS3.
    };

    /**
     * @brief Stores the bus and sensor index. Does not touch hardware.
     * @param spi SPI bus (&hspi2 on HAL-1).
     * @param DeviceNum Which sensor.
     */
    Barometer(SPI_HandleTypeDef* spi, BMP_INDEX DeviceNum);

    /**
     * @brief Initializes the chip and starts continuous measurement.
     * @return 0 on success; 1 bad index; 2 chip init failed; 3 settings write
     *         failed; 4 mode change failed.
     */
    uint8_t Init();

    /**
     * @brief Reads a new sample if one is ready and converts it to altitude.
     *
     * Keeps the previous reading if no new sample is ready, or if the pressure
     * is outside #BARO_PRESS_MIN_PA to #BARO_PRESS_MAX_PA. Altitude uses the
     * standard barometric formula, 44330 × (1 − (p / 101325)^0.1903).
     *
     * @return 0 on success or when no new sample is ready; 1 not initialized
     *         or status read failed; 2 data read failed; 3 status re-read failed.
     */
    uint8_t Update();

    /// @brief Returns the last reading stored by Update().
    BMP_Data getRawData();

private:
    SPI_HandleTypeDef* _spi;    ///< SPI bus.
    bmp3_spi_intf spi_intf;     ///< SPI context passed to the Bosch callbacks.
    bmp3_dev device;            ///< Bosch driver state.
    bool _sensor_active;        ///< true once Init() succeeds.
    BMP_INDEX InitDev;          ///< Which sensor this is.

    // config stuff in init and in update too
    bmp3_settings settings = {0};   ///< Sensor settings applied in Init().
    uint16_t settings_sel;          ///< Which settings Init() writes.
    bmp3_status status = {{0}};     ///< Last status read (data-ready flag).

    BMP_Data _raw;  ///< Last reading.
};

#endif //KINGFISHER_SW_BMP390_H
