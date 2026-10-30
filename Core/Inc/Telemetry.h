/**
 * @file Telemetry.h
 * @ingroup radio
 * @brief Radio protocol between the flight computer (FC) and the ground station (GND).
 *
 * Defines the two packet payloads (#telemetryData, FC → GND, and
 * #GndStationData, GND → FC), the framing bytes, radio addresses and command
 * bytes, and the #Telemetry class that frames packets and decides when to
 * transmit and when to listen.
 *
 * Every constant and struct here must match HAL-1-GND (RADIO_DEFNS.h and
 * Radio.h) byte for byte.
 *
 * @see Ebyte_E22_900T22S.h for the module driver.
 * @see docs/ARCHITECTURE.md for the frame layout and the radio duty cycle.
 */

#ifndef KINGFISHER_SW_TELEMETRY_H
#define KINGFISHER_SW_TELEMETRY_H

#include "Ebyte_E22_900T22S.h"
#include "main.h"

// ── Protocol constants ── must match GND RADIO_DEFNS.h exactly ────────────────
#define TELEMETRY_SYNC1         0xAA    ///< First frame-start byte. Also the last byte of every frame.
#define TELEMETRY_SYNC2         0x55    ///< Second frame-start byte. Also the second-to-last byte of every frame.
#define TELEMETRY_MAX_PAYLOAD   240     ///< Size of the TX and RX frame buffers (E22 sub-packet limit). A payload must leave 12 bytes for framing.

// Radio addresses
#define HAL1_RADIO_NETID        0xE6    ///< E22 network ID shared by the FC and GND modules.
#define HAL1_RADIO_ADDRLOW      0x06    ///< FC module address, low byte. FC address = 0x0706.
#define HAL1_RADIO_ADDRHIGH     0x07    ///< FC module address, high byte.
#define GND_RADIO_ADDRLOW       0x4A    ///< GND module address, low byte. GND address = 0x4A4A.
#define GND_RADIO_ADDRHIGH      0x4A    ///< GND module address, high byte.

// ── GND → FC command bytes ── (values mirror RADIO_DEFNS.h on GND side) ───────
#define SHUTDOWN_KEEPALIVE      217u    ///< BYTE_ABORT on GND. Sets Telemetry::shutdownFlag and clears g_ctrls_enabled.
#define SERVO_OFFSET_CMD_BYTE   0xD4u   ///< BYTE_SERVO_TARE on GND. Not handled specially; Telemetry::processCmd() treats it as a channel number.

// ── E22 driver error codes ─────────────────────────────────────────────────────
#define E22_NO_DATA             2   ///< Telemetry::Update() sent a telemetry frame instead of receiving one.
#define E22_RECEIVE_ERR         3   ///< Telemetry::receiveCommands(): the UART read returned nothing usable.
#define E22_BAD_LENGTH          4   ///< Telemetry::receiveCommands(): fewer than 7 bytes. Unreachable, because recieve_e22_900t22s() already returns -2 for that.

// ── TX burst size ─────────────────────────────────────────────────────────────
/// Unused. Left over from the FC_MASTER / GND_MASTER scheme that
/// Telemetry::Update() no longer implements.
#define TX_BURST_SIZE           5u

// ── EXTI flag (set by interrupt, polled by radio task) ────────────────────────
/// Set on every AUX rising edge by HAL_GPIO_EXTI_Callback(). Only
/// e22_available() reads it, and nothing calls that.
extern volatile bool e22_data_ready;

// ── Pyrotechnic keys ───────────────────────────────────────────────────────────
/**
 * @brief 32-bit keys GND can send in GndStationData::pyroActivation.
 *
 * Despite the names, Telemetry::processPyros() maps them to controller modes,
 * not pyro channels. That function is never called, so the keys currently do
 * nothing.
 */
typedef enum {
    PYROMAIN        = 121734683,    ///< Run the PID roll damper indefinitely.
    PYRODROGUEBKP   = 402746912,    ///< Restart the state-space controller from t = 0.
    PYRODROGUEMAIN  = 243656272     ///< Disable both controllers.
} pyroActivateKeys;

// ── GND → FC payload ──────────────────────────────────────────────────────────
/**
 * @brief GND → FC packet payload, 16 bytes including padding.
 *
 * Sent by the ground station inside the FC's receive window and decoded by
 * Telemetry::receiveCommands(). The layout must match HAL-1-GND exactly.
 */
typedef struct
{
    float    servoOffset1      = 0.0f;  ///< Canard trim (deg). Copied into g_gndData.servoOffset2: the two offsets are swapped on receipt.
    float    servoOffset2      = 0.0f;  ///< Canard trim (deg). Copied into g_gndData.servoOffset1.
    uint32_t pyroActivation    = 0;     ///< A #pyroActivateKeys value, or 0. Currently ignored.
    uint8_t  CommandByte; ///< 217 (#SHUTDOWN_KEEPALIVE) stops the controllers. Any other value is treated as an E22 channel number.
} GndStationData;

// ── FC → GND payload ──────────────────────────────────────────────────────────
/**
 * @brief FC → GND packet payload, 112 bytes including padding.
 *
 * The Radio task builds it from a snapshot of g_telemNow and sends it with
 * Telemetry::sendData(). Fields are ordered to minimise struct padding.
 *
 * @attention Layout must be byte-for-byte identical to GND's telemetryData.
 *            Any change here requires the same change in HAL-1-GND Radio.h.
 */
typedef struct
{
    ///@{
    float altitude; ///< Filtered barometric altitude (m). MSL in g_telemNow; the Radio task subtracts the pad altitude, so the transmitted value is AGL once the pad tare has run.
    float verticalVelocity; ///< Vertical velocity (m/s) from barometric altitude differences. See DataUpdate::getVerticalVelocity().
    ///@}
    ///@{
    float longitude, latitude, GPSaltitude; ///< GPS position: degrees, degrees, metres MSL.
    ///@}
    ///@{
    float mAccX, mAccY, mAccZ; ///< Voted, filtered BMI323 acceleration (g). Body frame with Z along the rocket axis (sensor Y and Z swapped).
    ///@}
    ///@{
    float mGyrX, mGyrY, mGyrZ; ///< Voted, filtered BMI323 angular rate (deg/s). Same axes as mAcc*.
    ///@}
    ///@{
    float Qx, Qy, Qz, Qw; ///< Attitude quaternion from the attitude estimator.
    ///@}
    ///@{
    float pitch, yaw, roll; ///< Euler angles (deg) from the quaternion. See DataUpdate::FuseAttitude() for the axis mapping.
    ///@}
    ///@{
    float servoTarget1, servoTarget2; ///< Canard commands (deg) including GND trim, written by the CTRLs task.
    ///@}
    ///@{
    float servoPos1, servoPos2; ///< Canard angles (deg) read from the servo feedback pins.
    ///@}
    ///@{
    float temperature; ///< Voted barometer temperature (°C).
    uint8_t callsign[12] = {75, 69, 57, 69, 82, 73, 95, 65, 76, 69, 80, 72}; ///< ASCII "KE9ERI_ALEPH". Not NUL-terminated.
    uint8_t CommandResponseByte; ///< Meant to echo the last GND CommandByte, but always sent as 0. See Telemetry::processCmd().
    int8_t RSSI; ///< Received signal strength. Never measured, so always 0.
    bool pyroMainDrogueFired   = false; ///< Pyro status. Never set, so always false.
    bool pyroBackupDrogueFired = false; ///< Pyro status. Never set, so always false.
    bool pyroMainChuteFired    = false; ///< Pyro status. Never set, so always false.
    ///@}
} telemetryData;

/// @brief Field-by-field equality, ignoring #telemetryData::callsign. Currently unused.
inline bool operator==(const telemetryData& a, const telemetryData& b) {
    return a.altitude          == b.altitude          &&
           a.verticalVelocity  == b.verticalVelocity  &&
           a.longitude         == b.longitude         &&
           a.latitude          == b.latitude          &&
           a.GPSaltitude       == b.GPSaltitude       &&
           a.mAccX == b.mAccX && a.mAccY == b.mAccY && a.mAccZ == b.mAccZ &&
           a.mGyrX == b.mGyrX && a.mGyrY == b.mGyrY && a.mGyrZ == b.mGyrZ &&
           a.Qx == b.Qx && a.Qy == b.Qy && a.Qz == b.Qz && a.Qw == b.Qw &&
           a.pitch == b.pitch && a.yaw == b.yaw && a.roll == b.roll &&
           a.servoTarget1      == b.servoTarget1      &&
           a.servoTarget2      == b.servoTarget2      &&
           a.servoPos1         == b.servoPos1         &&
           a.servoPos2         == b.servoPos2         &&
           a.temperature       == b.temperature       &&
           a.CommandResponseByte == b.CommandResponseByte &&
           a.RSSI              == b.RSSI              &&
           a.pyroMainDrogueFired   == b.pyroMainDrogueFired   &&
           a.pyroBackupDrogueFired == b.pyroBackupDrogueFired &&
           a.pyroMainChuteFired    == b.pyroMainChuteFired;
}
/// @brief Negation of operator==(const telemetryData&, const telemetryData&).
inline bool operator!=(const telemetryData& a, const telemetryData& b) { return !(a == b); }

// ── GND-master receive timeout ────────────────────────────────────────────────
/// Unused. Belonged to the GND_MASTER mode that Telemetry::Update() no longer
/// implements (see #TX_BURST_SIZE).
#define GND_MASTER_TIMEOUT_MS   5000u

// ── Telemetry class ─────────────────────────────────────────────────────────────
/**
 * @brief Frames packets, schedules TX and RX on the E22, and applies GND commands.
 *
 * There is one global instance, `telem` in main.cpp. FC_Init() calls Init()
 * once at boot. After that only the Radio task uses it: each loop copies
 * g_telemNow into #HALOutData and calls Update().
 *
 * Update() follows a fixed duty cycle. Every 5 s it opens a receive window of
 * up to 2 s; the rest of the time each call sends one telemetry frame.
 *
 * @note Older comments describe an FC_MASTER / GND_MASTER scheme with
 *       #TX_BURST_SIZE and #GND_MASTER_TIMEOUT_MS. It is not implemented, and
 *       the members that supported it are unused.
 */
class Telemetry {

    config_e22_900t22s des_cfg = {}; ///< Config written to the E22 in Init(). REG2 is not updated when the channel changes.

    uint8_t  tx_buffer[TELEMETRY_MAX_PAYLOAD]{}; ///< Outgoing frame, including the 3-byte fixed-point header.
    uint8_t  rx_buffer[TELEMETRY_MAX_PAYLOAD]{}; ///< Incoming bytes from the last receive.
    uint16_t lastSeq = 0; ///< Sequence number for the next TX frame. Overwritten by each decoded GND frame's number.

    // ── FC-master TX burst counter ─────────────────────────────────────────
    uint8_t  tx_burst_count = 0; ///< Unused.

    // ── GND-master receive state ───────────────────────────────────────────
    bool     gndMasterMode     = false; ///< Unused.
    uint32_t gndMasterDeadline = 0;   ///< Unused. Was the FreeRTOS tick at which GND-master mode times out.

    /**
     * @brief Sends one telemetry frame. FC → GND data path.
     * @param data Payload to send.
     * @return 0 on success, otherwise the negative E22_Status from
     *         transmit_e22_900t22s() cast to uint8_t.
     */
    uint8_t sendData(const telemetryData &data);

    /**
     * @brief Reads one GND frame from the module and applies it. GND → FC data path.
     *
     * Calls recieve_e22_900t22s() with the scheduler suspended, decodes the
     * frame, copies the servo offsets into g_gndData under
     * g_ctrls_sensor_mutex (offsets 1 and 2 swapped), then calls processCmd().
     *
     * @param[out] gnd The decoded payload.
     * @return 0 on success; #E22_RECEIVE_ERR if nothing usable arrived; or
     *         1–6 if decodeData() rejected the frame (its error code negated).
     * @warning The UART read runs with the scheduler suspended for up to
     *          ~57 ms, so the sensor and control tasks stop for that time.
     */
    uint8_t receiveCommands(GndStationData &gnd);

    /**
     * @brief Applies a GND command byte.
     *
     * Echoes @p cmd into HALOutData.CommandResponseByte. 217
     * (#SHUTDOWN_KEEPALIVE) sets #shutdownFlag and clears g_ctrls_enabled. Any
     * other value is treated as an E22 channel and passed to
     * changeOpFreq_e22_900t22s() if it differs from des_cfg.REG2.
     *
     * @param cmd GndStationData::CommandByte from the decoded frame.
     *
     * @bug The channel-change path has three problems:
     *   - changeOpFreq_e22_900t22s() sends the config at 38400 baud, but the
     *     E22 accepts config commands only at 9600. The write fails and leaves
     *     the module in CONFIG mode, so the link stays down until reboot.
     *   - des_cfg.REG2 is never updated, so after a change every packet
     *     repeats the config write.
     *   - encodeAndSend() always puts #GLOBAL_RADIO_CHANNEL in the header, so
     *     transmissions stay on the old channel.
     *   Nothing checks the byte is a valid channel (52–78), so
     *   #SERVO_OFFSET_CMD_BYTE or 0 takes the same path.
     * @bug The echo never reaches GND. The Radio task overwrites #HALOutData
     *      from g_telemNow before the next send.
     */
    void    processCmd(uint8_t cmd);

    /**
     * @brief Switches controller mode from a #pyroActivateKeys value.
     *
     * #PYROMAIN enables the PID roll damper; #PYRODROGUEBKP restarts the
     * state-space controller from t = 0; #PYRODROGUEMAIN disables both. Other
     * values are ignored.
     *
     * @param pyroActivation GndStationData::pyroActivation.
     * @note Never called.
     */
    void    processPyros(uint32_t pyroActivation);

    /**
     * @brief Finds and validates a frame in rx_buffer and copies out its payload.
     *
     * Scans the first @p buf_len bytes for SYNC1 SYNC2, then checks the length
     * byte against sizeof(T), the frame bounds, the checksum, and the trailing
     * SYNC2 SYNC1. On success it also sets lastSeq from the frame.
     *
     * @tparam T Payload type. Its size must equal the frame's length byte.
     * @param[out] payload Receives the payload bytes.
     * @param buf_len Number of valid bytes in rx_buffer.
     * @return 0 on success, or -1 no sync found, -2 header truncated,
     *         -3 length mismatch, -4 frame truncated, -5 checksum mismatch,
     *         -6 trailing sync mismatch.
     */
    template<typename T> int8_t  decodeData(T &payload, uint16_t buf_len);

    /**
     * @brief Frames @p payload into tx_buffer and transmits it.
     *
     * Layout: GND address (2 bytes) and #GLOBAL_RADIO_CHANNEL (the E22
     * fixed-point header, stripped by the module), SYNC1, SYNC2, length,
     * sequence number (little-endian), payload, 16-bit checksum
     * (little-endian), SYNC2, SYNC1. Increments lastSeq on success.
     *
     * @tparam T Payload type. Must fit in #TELEMETRY_MAX_PAYLOAD minus 12 bytes.
     * @param payload Data to send.
     * @return 0 on success, otherwise the negative E22_Status cast to uint8_t.
     */
    template<typename T> uint8_t encodeAndSend(const T &payload);

    /**
     * @brief 16-bit additive checksum: the byte sum, wrapping. Not a CRC.
     * @param data First byte to include.
     * @param length Number of bytes.
     * @return The sum modulo 65536.
     */
    uint16_t Checksum(uint8_t *data, uint16_t length);

    /**
     * @brief Writes a new module config to flash and stores it in des_cfg.
     * @param cfg_new Config to write.
     * @note Never called. It would hit the same 9600-baud problem as processCmd().
     */
    void Reconfigure(const config_e22_900t22s *cfg_new);

public:
    /// @brief Resets the sequence and receive state. Does not touch hardware; call Init().
    Telemetry();

    /**
     * @brief Configures the E22 module and leaves it in TRANS mode.
     *
     * Fills des_cfg with the HAL-1 settings: address 0x0706, NETID 0xE6,
     * UART 38400 8N1, 4.8 kbps air rate, 240-byte sub-packets, 22 dBm,
     * channel #GLOBAL_RADIO_CHANNEL, RSSI byte enabled, fixed-point mode. Then
     * calls init_e22_900t22s(). Blocks for a few seconds.
     *
     * @return 0 on success, 1 on failure. FC_Init() retries every 100 ms until
     *         this succeeds.
     */
    uint8_t Init();

    /**
     * @brief Runs one Radio-task cycle: receive if a window is due, otherwise send.
     *
     * If 5 s have passed since the last window opened, polls AUX for up to
     * 2 s. Whenever AUX is low it calls receiveCommands(), and it returns as
     * soon as one frame decodes. Otherwise, or if the window ends without a
     * valid frame, it sends #HALOutData.
     *
     * @return 0 if a GND frame was decoded (nothing was sent this call), or
     *         #E22_NO_DATA if a telemetry frame was sent. Send errors are
     *         ignored.
     * @warning The receive window busy-waits without yielding at the Radio
     *          task's Realtime priority, so lower-priority tasks get no CPU
     *          for up to 2 s.
     */
    uint8_t Update();

    telemetryData  HALOutData = {}; ///< Next payload to send. The Radio task refreshes it from g_telemNow before each Update().
    GndStationData GNDOutData = {}; ///< The most recently decoded GND payload.

    bool shutdownFlag = false; ///< Set when GND sends 217. Nothing reads it.

    /// @brief true after a cycle that decoded a GND frame; false after a TX cycle or a
    /// failed receive. Nothing reads it.
    bool lastRxGood = false;
};

#endif // KINGFISHER_SW_TELEMETRY_H
