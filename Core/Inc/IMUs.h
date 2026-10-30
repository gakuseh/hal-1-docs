//
// Created by dyrel on 3/6/2026.
//

/**
 * @file IMUs.h
 * @ingroup sensors
 * @brief Triple-redundant IMU front end: three BMI323s voted into one reading.
 */

#ifndef KINGFISHER_SW_IMUS_H
#define KINGFISHER_SW_IMUS_H

#include "SIMU_BMI323.h"
#include "IMU_ISM6HGx.h"
#include "Math/Vector3TripleModRedundancy.h"


/// Per-sensor status from IMUs::Init() and IMUs::Update(). 0 means OK.
struct IMUsStatus {
    int8_t A = 0;   ///< BMI323 A. After Init(), a SIMU_BMI323::Init() code; after Update(), 0 or 1.
    int8_t B = 0;   ///< BMI323 B. Only set in TMR mode.
    int8_t C = 0;   ///< BMI323 C. Only set in TMR mode.
    int8_t ISM = 0; ///< ISM6HG256X. Always 0, because that sensor is not used.
};

/**
 * @brief Reads three BMI323 IMUs and votes them into one acceleration and rate.
 *
 * In TMR mode, which Sensors uses, Update() reads all three sensors and
 * TMR::Vote() averages the two that agree best. Otherwise sensor A is used
 * alone. The ISM6HG256X high-g IMU is a member, but its init and update calls
 * are commented out.
 */
class   IMUs {
    SIMU_BMI323 BMI323_A;   ///< Sensor A (chip select SIMU_CS1). Always used.
    SIMU_BMI323 BMI323_B;   ///< Sensor B (SIMU_CS2). TMR mode only.
    SIMU_BMI323 BMI323_C;   ///< Sensor C (SIMU_CS3). TMR mode only.
    IMU_ISM6HGx ISM6HGx;    ///< High-g IMU. Never initialized or read.
    TMR tmrComp;            ///< Voting helper.

    struct {
        BMI_Data BMIA_d;    ///< From BMI323_A.
        BMI_Data BMIB_d;    ///< From BMI323_B.
        BMI_Data BMIC_d;    ///< From BMI323_C.
    } Raw_BMI;              ///< Last good reading from each BMI323.

    BMI_Data Voted_BMI;         ///< Output of the last Update().

    IMUsStatus sensorStatus;    ///< Status from the last Init() or Update().

    bool TMR_STATE = false;     ///< true to vote all three sensors; false to use A only.

    ISM_Data Data_ISM;          ///< Always zero, because the ISM6HG256X is not read.

    /// @brief Votes Raw_BMI into Voted_BMI, acceleration and rate separately. @return 0.
    uint8_t TMR_Compute();
    /// @brief Copies sensor A into Voted_BMI. @return 0.
    uint8_t Normal_Compute();

public:
    /// @brief Constructs the sensor objects. Does not touch hardware.
    IMUs();

    /**
     * @brief Initializes BMI323 A, plus B and C if @p tmr_on.
     * @param tmr_on true to use all three sensors and vote.
     * @return Per-sensor init codes; 0 is OK (see SIMU_BMI323::Init()).
     */
    IMUsStatus Init(bool tmr_on);

    /**
     * @brief Reads each active sensor and recomputes the voted output.
     *
     * A sensor whose read fails keeps its previous value in the vote.
     *
     * @return Per-sensor status: 0 if the read succeeded or no new sample was
     *         ready, 1 if it failed.
     */
    IMUsStatus Update();

    /**
     * @brief Returns the last good reading from one sensor.
     * @param index 0, 1 or 2 for sensor A, B or C.
     * @return The reading, or all zeros for any other index.
     */
    BMI_Data getRawBMI(uint8_t index);

    /// @brief Returns the voted reading: acceleration in g, rate in deg/s, sensor axes.
    BMI_Data getVotedBMI();

    /// @brief Returns the ISM6HG256X reading, which is always zero because that sensor is not read.
    ISM_Data getRawISM();
};

#endif //KINGFISHER_SW_IMUS_H
