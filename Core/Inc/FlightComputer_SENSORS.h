//
// Created by bzhan on 5/15/2026.
//

/**
 * @file FlightComputer_SENSORS.h
 * @ingroup sensors
 * @brief The Sensors aggregate that reads every sensor and publishes g_telemNow and g_SensorData.
 *
 * FlightComputer_SENSORS.cpp also defines the shared globals g_telemNow,
 * g_telemPrev, g_gndData, g_BMI, g_BMP, g_GPS and g_servoAngles.
 */

#ifndef KINGFISHER_SW_FLIGHTCOMPUTER_H
#define KINGFISHER_SW_FLIGHTCOMPUTER_H

#include <stdint.h>
#include "Servo_Axon_Mini_MKII.h"
#include "IMUs.h"
#include "Barometer.h"
#include "LIS2MDL.h"
#include "Telemetry.h"
#include "MAXM10S.h"

// FLIGHT STAGE PROGRESSION
/// Flight stages. Only used as the type of CtrlsSensorSnapshot::stage, which is never set.
typedef enum {
    PREFLIGHT,
    ASCENT,
    CONTROLS_TEST,
    APOGEE_APPROACH,
    APOGEE,
    APOGEE_PASS,
    DESCENT,
    MAIN_APPROACH,
    MAIN,
    FINAL_DESCENT
} STAGE;

// STANDARD RETURN CODES
/// Return codes for the flight-computer procedures. Sensors::Init() returns STATUS_OK or STATUS_SENSOR_FAIL.
typedef enum {
    STATUS_ABORT_ACTIVE     = -4,
    STATUS_SENSOR_FAIL      = -3,
    STATUS_COMMS_FAILURE    = -2,
    STATUS_INIT_FAILURE     = -1,
    STATUS_OK               =  0,
    STATUS_TIMEOUT          =  1,
    STATUS_LOCAL_READ_ERR   =  2,
    STATUS_LOCAL_DATA_ERR   =  3,
    STATUS_REMOTE_READ_ERR  =  4,
    STATUS_REMOTE_DATA_ERR  =  5,
    STATUS_ABORT_TRIGGERED  = 127
} ProcedureStatus;

/// GPS fields the SD logger writes. Copied from MAXM10S::gpsData into g_GPS by Sensors::Update().
typedef struct {
    double  latitude;   ///< Degrees, positive north.
    double  longitude;  ///< Degrees, positive east.
    double  altitude;   ///< Metres MSL.
    uint8_t hour;       ///< UTC hour.
    uint8_t minute;     ///< UTC minute.
    uint8_t second;     ///< UTC second.
    uint8_t satellites; ///< Satellites in use.
    float   hdop;       ///< Horizontal dilution of precision.
} GPS_Data;

/// Unused. The global g_servoAngles is defined but never written or read.
typedef struct {
    float currS1;       ///< Current servo 1 angle.
    float currS2;       ///< Current servo 2 angle.
    float targetS1;     ///< Target servo 1 angle.
    float targetS2;     ///< Target servo 2 angle.
} ServoAngles;

/**
 * @brief Owns every sensor driver and publishes their readings each cycle.
 *
 * One global instance, `S` in main.cpp. FC_Init() calls Init() until it
 * returns STATUS_OK, then updateSensorTask() calls Update() at up to 500 Hz.
 *
 * @warning This class has its own Servo_Axon_Mini_MKII, separate from the
 *          global `Servos` that the CTRLs task drives. Both write TIM3 and read
 *          ADC3, and Update() reads the feedback angles from this one.
 */
class Sensors {
    IMUs IMU;               ///< Three BMI323s, voted.
    Baro_Unified Baro;      ///< Three BMP390s, voted.
    MAXM10S GPS;            ///< GPS on I2C4.
    Servo_Axon_Mini_MKII Servos;    ///< Used only to read canard feedback angles. See the class warning.
    Magnetometer mag;       ///< LIS2MDL on SPI1. Initialized but never read.

public:
    /// @brief Binds the magnetometer to SPI1. Does not touch hardware.
    Sensors();

    /**
     * @brief Initializes the IMUs, barometers, GPS, servos and magnetometer.
     *
     * The IMUs and barometers run in triple-redundant mode. A servo or
     * magnetometer failure does not fail the result.
     *
     * @return STATUS_OK, or STATUS_SENSOR_FAIL if any IMU, barometer or the GPS
     *         failed. The servo check takes about 3 s, so each failed attempt
     *         is slow.
     * @bug The body of `if (mag.Init() != 0)` is commented out, so that `if`
     *      now guards the next statement: g_telemNow and g_telemPrev are
     *      zeroed only when the magnetometer fails to initialize.
     */
    int8_t Init();

    /**
     * @brief Reads every sensor and publishes the results.
     *
     * Updates the IMUs, barometers and GPS, then detects liftoff (total
     * acceleration above #LIFTOFF_ACCEL_G for #LIFTOFF_SUSTAIN_MS) to start the
     * flight clock. Under g_ctrls_sensor_mutex (20 ms timeout) it writes
     * g_telemNow (GPS, altitude, temperature, acceleration, rates, servo
     * feedback), g_GPS, and g_SensorData with `fresh` set. The IMU's Y and Z
     * axes are swapped so Z runs along the rocket. If the mutex is busy the
     * results are dropped for this cycle.
     *
     * @return STATUS_OK, always.
     */
    int8_t Update();
};

#endif //KINGFISHER_SW_FLIGHTCOMPUTER_H
