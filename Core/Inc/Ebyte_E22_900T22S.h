/**
 * @file Ebyte_E22_900T22S.h
 * @ingroup radio
 * @brief Driver for the EBYTE E22-900T22S LoRa UART radio module.
 *
 * The module is wired to UART8 plus three GPIOs: M0 and M1 select the mode
 * (see #EBYTE_MODE) and AUX goes low while the module is busy. All UART
 * transfers are blocking HAL calls, and the waits use vTaskDelay(), so every
 * function here must run inside a FreeRTOS task.
 *
 * The driver keeps one static copy of the config, so it supports one module.
 *
 * @note The internal UART helpers always use huart8, whatever
 *       config_e22_900t22s::huart says.
 * @note CONFIG mode accepts commands only at 9600 baud, but after
 *       init_e22_900t22s() the MCU UART runs at 38400. Every function that
 *       rewrites the config after init (changeOpFreq_e22_900t22s(),
 *       setAddress_e22_900t22s(), the parameter setters, reset_e22_900t22s())
 *       therefore talks to the module at the wrong baud rate.
 */

#ifndef EBYTE_E22_900T22S_H
#define EBYTE_E22_900T22S_H

#include <stdint.h>
#include <stddef.h>
#include <Ebyte_E22_900T22S_defs.h>

/**
 * @brief Powers up the module, writes @p cfg to its flash, verifies it, and enters TRANS mode.
 *
 * Switches the MCU UART to 9600 baud for CONFIG mode, releases RADIO_RST,
 * waits for AUX, writes the config, reads it back and compares (rewriting once
 * if it differs), then switches the MCU UART to 38400 and the module to TRANS
 * mode. Takes a few seconds.
 *
 * @param cfg Register values plus the UART and GPIO handles to use.
 * @return E22_OK; E22_ERR_INVALID_PARAM if @p cfg is null; E22_ERR_UART or
 *         E22_ERR_TIMEOUT if a step fails; or 1 if the read-back still differs
 *         after the rewrite.
 * @warning The final MCU baud rate is hard-coded to 38400 and must match the
 *          UART bits in REG0.
 */
int8_t init_e22_900t22s(config_e22_900t22s *cfg);

/**
 * @brief Hands bytes to the module for transmission.
 *
 * Waits up to 200 ms for AUX to go high, writes the bytes over UART, then
 * waits up to 200 ms again. In fixed-point mode the first three bytes must be
 * the target address (high, low) and channel.
 *
 * @param data Bytes to send.
 * @param length Number of bytes.
 * @return E22_OK, E22_ERR_UART, or E22_ERR_NOT_INITIALIZED.
 * @warning If AUX is still low after the first wait, nothing is written and
 *          the function still returns E22_OK.
 */
int8_t transmit_e22_900t22s(uint8_t *data, size_t length);

/**
 * @brief Prepends a fixed-point header and transmits.
 *
 * @param address Target module address.
 * @param channel Target channel (REG2 value).
 * @param data Payload bytes.
 * @param length Payload length.
 * @return Same as transmit_e22_900t22s().
 * @note Unused. Telemetry::encodeAndSend() builds the header itself.
 */
int8_t transmit_fixed_e22_900t22s(
uint16_t address,
uint8_t channel,
uint8_t *data,
size_t length);

/**
 * @brief Blocking read of one incoming frame from the module.
 *
 * Reads the frame (5 header bytes + @p expected_payload_len + 4 trailer
 * bytes), plus 1 for the RSSI byte the module appends and 4 guard bytes that
 * absorb leftovers from an earlier timed-out read. The timeout is the time
 * those bytes take at 38400 baud plus 50 ms (about 57 ms for a 16-byte
 * payload). Sync and checksum checks are left to the caller
 * (Telemetry::decodeData()).
 *
 * @param[out] buffer Must hold at least @p expected_payload_len + 14 bytes.
 * @param expected_payload_len sizeof the payload struct being received.
 * @return Number of bytes placed in @p buffer (a partial read counts if at
 *         least 7 bytes arrived), or -2 if fewer than 7 arrived.
 */
int16_t recieve_e22_900t22s(uint8_t *buffer, uint16_t expected_payload_len);

/// Set by HAL_GPIO_EXTI_Callback() on every AUX rising edge; cleared by
/// e22_available().
inline volatile bool e22_data_ready = false;

/**
 * @brief Returns and clears #e22_data_ready.
 * @return true if AUX has risen since the last call.
 * @note Unused.
 */
bool e22_available(void);

/**
 * @brief Checks whether received data is arriving or waiting.
 * @return true if AUX is low (the module is outputting a packet) or the UART
 *         already holds at least one byte.
 * @note Unused. Telemetry::Update() polls e22_aux_high() instead.
 */
bool e22_rx_pending(void);

/**
 * @brief Asks the module for the RSSI of the last received packet.
 *
 * Sends the C0 C1 C2 C3 register-read command and parses the reply. Works
 * only in TRANS or WOR mode.
 *
 * @return Signal strength in dBm, computed as raw − 256 per the E22 manual,
 *         or 0 if the reply was not understood, or -1 in the wrong mode.
 * @note Unused, which is why telemetryData::RSSI is always 0. Readings below
 *       -128 dBm wrap when cast to int8_t.
 */
int8_t get_rssi_e22_900t22s(void);

/**
 * @brief Sets M0/M1 for @p mode, then waits up to 200 ms for AUX to go high.
 * @param mode TRANS (normal), WOR, CONFIG, or OFF (deep sleep).
 */
void changeMode(EBYTE_MODE mode);

/**
 * @brief Changes the module's channel by rewriting REG2 (not saved to flash).
 * @param channel New channel.
 * @bug Runs at 38400 baud after init, but CONFIG mode accepts only 9600. The
 *      write fails and writeConfig_e22_900t22s() returns before switching back
 *      to TRANS, leaving the module in CONFIG mode. The driver's REG2 copy is
 *      already changed by then, so getOpFreq_e22_900t22s() reports a channel
 *      the module is not on.
 */
void changeOpFreq_e22_900t22s(R2_E22Channel915 channel);

/// @brief Returns the channel in the driver's config copy, which may differ from the module's (see changeOpFreq_e22_900t22s()).
R2_E22Channel915 getOpFreq_e22_900t22s(void);

/**
 * @brief Changes the module address (not saved to flash).
 * @param address New 16-bit address.
 * @note Unused. Same 9600-baud problem as changeOpFreq_e22_900t22s().
 */
void setAddress_e22_900t22s(uint16_t address);

/// @brief Returns the address in the driver's config copy.
uint16_t getAddress_e22_900t22s(void);

/**
 * @brief Changes the air data rate (REG0 bits 2–0, not saved to flash).
 * @param rate New air rate. Both modules must use the same rate.
 * @note Unused. Same 9600-baud problem as changeOpFreq_e22_900t22s().
 */
void setAirDataRate_e22_900t22s(R0_210_E22_AIR_DATA_RATE rate);

/**
 * @brief Changes the module's UART baud rate (REG0 bits 7–5, not saved to flash).
 * @param baud New baud rate. The MCU UART is not changed to match.
 * @note Unused. Same 9600-baud problem as changeOpFreq_e22_900t22s().
 */
void setUARTBaud_e22_900t22s(R0_765_E22_UART_BAUD baud);

/**
 * @brief Changes the transmit power (REG1 bits 1–0, not saved to flash).
 * @param power New power level.
 * @note Unused. Same 9600-baud problem as changeOpFreq_e22_900t22s().
 */
void setTxPower_e22_900t22s(R1_10_E22_TX_POWER power);

/**
 * @brief Reads ADDH through REG3 from the module.
 *
 * Switches to CONFIG mode if needed, sends C1 00 07, checks the echoed
 * header, and returns to TRANS mode.
 *
 * @param[out] cfg Receives the seven register bytes. Other fields are untouched.
 * @return E22_OK, E22_ERR_UART, or E22_ERR_DATA_VERIFICATION if the echo is wrong.
 * @warning The MCU UART must be at 9600 baud, which is true only during
 *          init_e22_900t22s().
 */
int8_t readConfig_e22_900t22s(config_e22_900t22s *cfg);

/**
 * @brief Writes ADDH through REG3 to the module.
 *
 * Switches to CONFIG mode if needed, flushes the UART, sends the 10-byte
 * command, checks the 10-byte echo, returns to TRANS mode, and stores @p cfg
 * as the driver's copy.
 *
 * @param cfg Register values to write.
 * @param save_to_flash true sends C0 (persist across power cycles); false
 *        sends C2 (until next power-down).
 * @return E22_OK or E22_ERR_UART.
 * @warning On failure it returns without switching back to TRANS, leaving the
 *          module in CONFIG mode. The MCU UART must be at 9600 baud, which is
 *          true only during init_e22_900t22s().
 */
int8_t writeConfig_e22_900t22s( const config_e22_900t22s *cfg,
                                bool save_to_flash);

/**
 * @brief Waits for AUX to go high (module idle), polling every 1 ms.
 * @param timeout_ms Maximum wait.
 * @return E22_OK, or E22_ERR_TIMEOUT if AUX is still low.
 */
int8_t waitAux_e22_900t22s(uint32_t timeout_ms);

/// @brief Returns true while AUX is low: the module is self-checking, transmitting, or outputting received data.
bool e22_isBusy(void);

/**
 * @brief Sends the software reset command (CF) in CONFIG mode, then returns to TRANS.
 * @note Unused. Same 9600-baud problem as changeOpFreq_e22_900t22s().
 */
void reset_e22_900t22s(void);

/// @brief Returns true once init_e22_900t22s() has succeeded.
bool e22_initialized(void);

/// @brief Returns true if AUX is high (module idle).
bool e22_aux_high(void);

#endif
