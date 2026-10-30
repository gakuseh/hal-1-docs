//
// Created by dyrel on 4/11/2026.
//

/**
 * @file Barometer.h
 * @ingroup sensors
 * @brief Triple-redundant barometer front end: three BMP390s voted into one reading.
 */

#ifndef KINGFISHER_SW_BAROMETER_H
#define KINGFISHER_SW_BAROMETER_H
#include <cstdint>
#include "BMP390.h"
#include "spi.h"
#include "Math/FloatTripleModRedundancy.h"

/// Readings from each barometer plus the voted result.
typedef struct {
    ///@{
    BMP_Data SensorA, SensorB, SensorC = {};   ///< Last reading from each sensor.
    ///@}
    BMP_Data Filtered = {};                     ///< Voted result. In single-sensor mode, a copy of SensorA.
} BARO_DATA;

/**
 * @brief Reads three BMP390 barometers on SPI2 and votes each quantity.
 *
 * In TMR mode, which Sensors uses, pressure, altitude and temperature are
 * voted separately with TMRFloat::Vote(), which averages the two closest
 * values.
 */
class Baro_Unified {
    ///@{
    Barometer SensorA, SensorB, SensorC;    ///< The three barometers, on chip selects BMP390_CS1, CS2 and CS3.
    ///@}
    BARO_DATA data;                         ///< Last readings and voted result.
    bool TMR_on = false;                    ///< true to read all three and vote.

    /// Per-sensor status codes from Barometer::Init() or Barometer::Update(). 0 means OK.
    typedef struct {
        ///@{
        int8_t A, B, C;     ///< Status of SensorA, SensorB and SensorC.
        ///@}
    } BMP_Status;

    BMP_Status status;  ///< Status from the last Init() or Update().

public:
    /// @brief Binds the three sensors to SPI2 and their chip selects. Does not touch hardware.
    Baro_Unified();

    /**
     * @brief Initializes sensor A, plus B and C if @p TMR_Toggle.
     * @param TMR_Toggle true to use all three sensors and vote.
     * @return Per-sensor init codes (see Barometer::Init()).
     */
    BMP_Status Init(bool TMR_Toggle = false);

    /**
     * @brief Reads each active sensor and recomputes the voted result.
     * @return Per-sensor update codes (see Barometer::Update()).
     */
    BMP_Status Update();

    /// @brief Returns the last readings and voted result.
    BARO_DATA getData();
};

#endif //KINGFISHER_SW_BAROMETER_H
