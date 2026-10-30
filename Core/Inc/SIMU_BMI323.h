//
// Created by dyrel on 3/3/2026.
//

/**
 * @file SIMU_BMI323.h
 * @ingroup sensors
 * @brief Wrapper for one Bosch BMI323 accelerometer and gyroscope on SPI4.
 */

#ifndef KINGFISHER_SW_SIMU_BMI323_H
#define KINGFISHER_SW_SIMU_BMI323_H

#include "bmi323.h"
#include "main.h"
#include "spi.h"
#include "Math/Math.h"
#include "BMI323_Platform.h"

/// Which of the three BMI323s an object talks to. Selects the chip-select pin.
typedef enum {
    SENSOR1_I,  ///< Sensor A, chip select SIMU_CS1.
    SENSOR2_I,  ///< Sensor B, chip select SIMU_CS2.
    SENSOR3_I   ///< Sensor C, chip select SIMU_CS3.
} BMI_INDEX;

/// One IMU reading in the sensor's own axes.
struct BMI_Data
{
    Vector3D<float> accel_linear;   ///< Acceleration (g). The sensor Y axis reads about +1 g with the rocket upright.
    Vector3D<float> ang_vel;        ///< Angular rate (deg/s).
};

/// SPI context passed to the Bosch driver callbacks as `intf_ptr`.
typedef struct {
    SPI_HandleTypeDef* SPIbus;  ///< Always &hspi4.
    BMI_INDEX index;            ///< Which sensor this is.
    GPIO_TypeDef* cs_port;      ///< Chip-select port.
    uint16_t cs_pin;            ///< Chip-select pin.
} BMIobjPtr;

/**
 * @brief One BMI323 accelerometer and gyroscope, configured for flight.
 *
 * Init() runs both at 200 Hz: the accelerometer at ±16 g with 64-sample
 * averaging (50 Hz bandwidth), the gyroscope at ±2000 deg/s with no averaging
 * (100 Hz bandwidth).
 */
class SIMU_BMI323
{
public:
    /// @brief Zeroes the cached reading. Does not touch hardware.
    SIMU_BMI323();

    /**
     * @brief Selects the chip-select pin, initializes the chip, and applies the flight config.
     * @param DeviceNum Which sensor.
     * @return 0 on success; 1 bad index; 2 chip init failed; 3 config read
     *         failed; 4 interrupt mapping failed; 5 config write failed.
     */
    int Init(BMI_INDEX DeviceNum);

    /**
     * @brief Reads a new sample if one is ready.
     *
     * Returns 0 without reading unless the accelerometer, gyroscope and
     * temperature data-ready flags are all set. New data arrives every 5 ms and
     * the sensor task polls every 2 ms. The temperature is read but discarded.
     *
     * @return 0 on success or when no sample is ready; -999 if Init() has not
     *         succeeded; otherwise a negative Bosch BMI3 error code.
     */
    int Update();

    /// @brief Returns the last reading stored by Update().
    BMI_Data getRawData();

private:
    bmi3_dev device;        ///< Bosch driver state.
    bool _sensor_active;    ///< true once Init() succeeds.
    BMI_INDEX InitDev;      ///< Unused.
    BMI_Data _raw;          ///< Last reading.
    BMIobjPtr _obj;         ///< SPI context passed to the Bosch callbacks.

    bmi3_sens_config config[2];     ///< [0] accelerometer settings, [1] gyroscope settings.
    bmi3_map_int map_int = {0};     ///< Routes the data-ready interrupts to INT1.

    /**
     * @brief Converts a raw accelerometer value to g.
     * @param val Raw sensor value (LSB).
     * @param g_range Full-scale range in g (16 on HAL-1).
     * @param bit_width Sensor resolution in bits.
     * @return Acceleration in g.
     */
    static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width);

    /**
     * @brief Converts a raw gyroscope value to deg/s.
     * @param val Raw sensor value (LSB).
     * @param dps Full-scale range in deg/s (2000 on HAL-1).
     * @param bit_width Sensor resolution in bits.
     * @return Angular rate in deg/s.
     */
    static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
};

#endif //KINGFISHER_SW_SIMU_BMI323_H
