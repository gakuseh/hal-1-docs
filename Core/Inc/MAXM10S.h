//
// Created by chris on 4/1/2026.
// Revised by Claude Code 2026-04-17.
//

/**
 * @file MAXM10S.h
 * @ingroup sensors
 * @brief Driver for the u-blox MAX-M10S GPS module on I2C (DDC).
 */

#ifndef KINGFISHER_SW_MAXM10S_H
#define KINGFISHER_SW_MAXM10S_H

#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "TinyGPSPlus.h"

/**
 * @brief MAX-M10S GPS: reads NMEA over I2C and parses it with TinyGPSPlus.
 *
 * Init() sets the airborne (<4 g) dynamic model and enables GGA, RMC and GSA
 * output on I2C. Each update() drains the module's buffer and parses whole
 * sentences.
 */
class MAXM10S {

public:
    /// Return codes from update().
    enum UpdateStatus : uint8_t {
        GPS_NEW_DATA        = 0,  ///< At least one position, altitude, speed or time field changed.
        GPS_READ_FAIL       = 1,  ///< The I2C read failed.
        GPS_NO_BYTES        = 2,  ///< The module reported nothing waiting.
        GPS_NO_NEW_SENTENCE = 3,  ///< Bytes were read but no field changed.
    };

    /// Latest GPS solution. Fields keep their last valid values until updated.
    struct gpsData {
        // Position
        double  latitude   = 0.0;   ///< Degrees, positive north.
        double  longitude  = 0.0;   ///< Degrees, positive east.
        double  altitude   = 0.0;   ///< Metres MSL. Needs a 3D fix.
        float   speedKmh   = 0.0f;  ///< Ground speed (km/h).

        // Fix quality
        uint8_t satellites = 0;     ///< Satellites in use.
        float   hdop       = 99.9f; ///< Horizontal dilution of precision. Lower is better.
        bool    hasFix     = false; ///< true when both position and altitude are valid.

        // Validity flags (set independently; time often valid before position)
        bool isLocated   = false;   ///< Position valid.
        bool isAltituded = false;   ///< Altitude valid.
        bool isTimed     = false;   ///< Time valid. Often true before position is.

        // Time (UTC)
        uint8_t hour   = 0;         ///< UTC hour.
        uint8_t minute = 0;         ///< UTC minute.
        uint8_t second = 0;         ///< UTC second.

        uint32_t lastFixMs = 0;     ///< HAL_GetTick() time of the last position update (ms).
    };

    /// @brief Creates an unbound driver. Call Init().
    MAXM10S();

    /**
     * @brief Releases the module from reset, checks it answers, and sends the config.
     *
     * Releases GPS_RST, waits 500 ms for boot, reads the bytes-available
     * register to confirm the module is on the bus, then sends a UBX-CFG-VALSET
     * that sets the airborne (<4 g) model and enables GGA, RMC and GSA on I2C.
     * The config is saved to the module's RAM, battery-backed RAM and flash.
     *
     * @param hi2c I2C bus (&hi2c4 on HAL-1).
     * @return 0 on success, 1 if the module did not answer. The config write's
     *         result is not checked.
     */
    uint8_t Init(I2C_HandleTypeDef *hi2c);

    /**
     * @brief Reads everything the module has buffered and parses it.
     *
     * Reads up to 1024 bytes per call (I2C timeouts of 50 and 100 ms). Every
     * complete sentence updates the satellite count, HDOP and validity flags;
     * position, altitude, speed and time update when they change.
     *
     * @return An #UpdateStatus value.
     */
    uint8_t update();

    /// @brief Returns the latest solution.
    gpsData getData() const;

private:
    I2C_HandleTypeDef *_hi2c;   ///< I2C bus.
    TinyGPSPlus        gpsParser;   ///< NMEA parser.
    gpsData            gpsDataStruct;   ///< Latest solution.

    static constexpr uint16_t I2C_ADDRESS     = 0x84; ///< 8-bit HAL address (7-bit 0x42 << 1).
    static constexpr uint16_t REG_BYTES_AVAIL = 0xFD; ///< Register holding the number of bytes waiting (2 bytes, big-endian).
    static constexpr uint16_t REG_DATA_STREAM = 0xFF; ///< Register that streams the buffered NMEA bytes.
    static constexpr uint16_t RX_BUF_SIZE     = 1024; ///< Most bytes read per update().

    uint8_t  rxBuf[RX_BUF_SIZE];    ///< Read buffer.

    /// @brief Reads the bytes-waiting count. @return The count, or 0 on I2C error or a 0xFFFF stall.
    uint16_t getAvailableBytes();
    /// @brief Reads @p numBytes (capped at #RX_BUF_SIZE) into rxBuf. @return Bytes read, or 0 on error.
    uint16_t readStream(uint16_t numBytes);
    /// @brief Sends a raw UBX message. @return true if the I2C write succeeded.
    bool     sendUBX(const uint8_t *msg, uint16_t len);
};

#endif //KINGFISHER_SW_MAXM10S_H
