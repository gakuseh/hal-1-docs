//
// Created by dyrel on 3/7/2026.
//

/**
 * @file Ebyte_E22_900T22S_defs.h
 * @ingroup radio
 * @brief Command bytes, register bit fields, and the config struct for the E22-900T22S.
 *
 * Register field names follow `<register>_<bits>_<function>`, so
 * `R0_765_E22_UART_BAUD` is REG0 bits 7–5. OR the fields together to build a
 * register value; Telemetry::Init() shows the HAL-1 settings.
 */

#include "stm32h7xx_hal.h"
#include <stdint.h>

#include "FreeRTOS.h"

#ifndef KINGFISHER_SW_EBYTE_E22_900T22S_DEFS_H
#define KINGFISHER_SW_EBYTE_E22_900T22S_DEFS_H

// CFG STARTING BITS
#define COMMAND_BYTE_WRITE_CFG_SAVE_FLASH 0xC0      ///< Write registers and keep them across power cycles.
#define COMMAND_BYTE_READ_CFG 0xC1                  ///< Read registers. The module also starts its replies to writes with C1.
#define COMMAND_BYTE_WRITE_CFG_NOSAVE_FLASH 0xC2    ///< Write registers until the next power-down.
#define COMMAND_BYTE_RESET_MODULE 0xCF              ///< Sent by reset_e22_900t22s().


// General usage enums

/// Operating mode, selected with the M0 and M1 pins (see changeMode()).
typedef enum {
    TRANS,  ///< M1=0 M0=0: normal transmit and receive.
    WOR,    ///< M1=0 M0=1: wake-on-radio.
    CONFIG, ///< M1=1 M0=0: configuration. Accepts commands only at 9600 8N1.
    OFF     ///< M1=1 M0=1: deep sleep.
} EBYTE_MODE;

/// Return codes for the driver functions.
typedef enum
{
    E22_OK                = 0,  ///< Success.
    E22_ERR_TIMEOUT       = -1, ///< AUX stayed low for too long.
    E22_ERR_UART          = -2, ///< A UART transfer failed or the module's echo was wrong.
    E22_ERR_BUSY          = -3, ///< Unused.
    E22_ERR_INVALID_PARAM = -4, ///< Null config pointer.
    E22_ERR_NOT_INITIALIZED = -5,   ///< init_e22_900t22s() has not succeeded yet.
    E22_ERR_DATA_VERIFICATION   = -6    ///< readConfig_e22_900t22s() got an unexpected echo.
} E22_Status;

/*
 * The general format for everything under here, describing the bits of registers,
 * follows this:
 *
 * <Register number>_<Bit in register>_<Function>
 *
 * This allows for easy reconstruction
 */


// REG0

/// REG0 bits 7–5: the module's UART baud rate. HAL-1 uses 38400.
typedef enum
{
    E22_UART_BAUD_1200    = 0b000 << 5,
    E22_UART_BAUD_2400    = 0b001 << 5,
    E22_UART_BAUD_4800    = 0b010 << 5,
    E22_UART_BAUD_9600    = 0b011 << 5,
    E22_UART_BAUD_19200   = 0b100 << 5,
    E22_UART_BAUD_38400   = 0b101 << 5,
    E22_UART_BAUD_57600   = 0b110 << 5,
    E22_UART_BAUD_115200  = 0b111 << 5
} R0_765_E22_UART_BAUD;

/// REG0 bits 4–3: UART parity.
typedef enum
{
    MODE_8N1 = 0b00 << 3, ///< 8 data bits, no parity, 1 stop bit. HAL-1 uses this.
    MODE_8O1 = 0b01 << 3, ///< Odd parity.
    MODE_8E1 = 0b10 << 3, ///< Even parity.
} R0_43_SERIAL_PORT_PARITY_BIT;

/// REG0 bits 2–0: air data rate. Both modules must match. HAL-1 uses 4.8 kbps.
typedef enum
{
    E22_AIR_RATE_2_4K   = 0b010,  ///< 2.4 kbps
    E22_AIR_RATE_4_8K   = 0b011,  ///< 4.8 kbps
    E22_AIR_RATE_9_6K   = 0b100,  ///< 9.6 kbps
    E22_AIR_RATE_19_2K  = 0b101,   ///< 19.2 kbps
    E22_AIR_RATE_38_4K  = 0b110,  ///< 38.4 kbps
    E22_AIR_RATE_62_5K  = 0b111  ///< 62.5 kbps
} R0_210_E22_AIR_DATA_RATE;


// REG1

/// REG1 bits 7–6: maximum sub-packet length. HAL-1 uses 240 bytes.
typedef enum
{
    BYTES_240 = 0b00 << 6,
    BYTES_128 = 0b01 << 6,
    BYTES_64  = 0b10 << 6,
    BYTES_32  = 0b11 << 6
} R1_76_SUB_PACKET_SETTING;

#define R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_ENABLE    0b1 << 5    ///< REG1 bit 5: enable the C0 C1 C2 C3 RSSI query used by get_rssi_e22_900t22s().
#define R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_DISABLE   0b0 << 5    ///< REG1 bit 5: disable the RSSI query.

// Bits 4 and 3 are reserved by the radio and are unusable

#define R1_2_SOFTWARE_MODE_SWITCHING_ON 0b1 << 2    ///< REG1 bit 2: allow mode changes by UART command.
#define R1_2_SOFTWARE_MODE_SWITCHING_OFF 0b0 << 2   ///< REG1 bit 2: modes set only by M0/M1. Always use this; the driver drives M0 and M1.

/// REG1 bits 1–0: transmit power. HAL-1 uses 22 dBm.
typedef enum
{
    E22_TX_POWER_22DBM = 0b00,
    E22_TX_POWER_17DBM = 0b01,
    E22_TX_POWER_13DBM = 0b10,
    E22_TX_POWER_10DBM = 0b11
} R1_10_E22_TX_POWER;


// REG2
/**
 * REG2: channel. Frequency = 850.125 MHz + channel × 1 MHz, so CH902 is
 * 902.125 MHz. Only the 902–928 MHz ISM channels (52–78) are listed. HAL-1
 * uses #GLOBAL_RADIO_CHANNEL.
 */
typedef enum {
    CH902 = 52,
    CH903,
    CH904,
    CH905,
    CH906,
    CH907,
    CH908,
    CH909,
    CH910,
    CH911,
    CH912,
    CH913,
    CH914,
    CH915,
    CH916,
    CH917,
    CH918,
    CH919,
    CH920,
    CH921,
    CH922,
    CH923,
    CH924,
    CH925,
    CH926,
    CH927,
    CH928
} R2_E22Channel915;


// REG3

#define R3_7_RSSI_BYTE_ENABLE               0b1 << 7    ///< REG3 bit 7: append an RSSI byte to each received packet. HAL-1 uses this.
#define R3_7_RSSI_BYTE_DISABLE              0b0 << 7    ///< REG3 bit 7: no RSSI byte.
#define R3_6_TRANSFER_METHOD_TRANSPARENT    0b0 << 6    ///< REG3 bit 6: send to modules with the same address and channel.
#define R3_6_TRANSFER_METHOD_FIXED_POINT    0b1 << 6    ///< REG3 bit 6: the first 3 bytes of each write pick the target address and channel. HAL-1 uses this.
#define R3_5_REPEATER_OFF                   0b0 << 5    ///< REG3 bit 5: repeater (relay) off.
#define R3_5_REPEATER_ON                    0b1 << 5    ///< REG3 bit 5: repeater (relay) on.
#define R3_4_LBT_ENABLED                    0b1 << 4    ///< REG3 bit 4: listen before talk.
#define R3_4_LBT_DISABLED                   0b0 << 4    ///< REG3 bit 4: transmit immediately.
#define R3_3_WOR_MODE_RECIEVER              0b0 << 3    ///< REG3 bit 3: wake-on-radio receiver.
#define R3_3_WOR_MODE_TRANSMIT              0b1 << 3    ///< REG3 bit 3: wake-on-radio transmitter.

/// REG3 bits 2–0: wake-on-radio cycle period. Only matters in WOR mode.
typedef enum
{
    _500_ms     = 0b000,
    _1000_ms    = 0b001,
    _1500_ms    = 0b010,
    _2000_ms    = 0b011,
    _2500_ms    = 0b100,
    _3000_ms    = 0b101,
    _3500_ms    = 0b110,
    _4000_ms    = 0b111,
} R3_210_WOR_CYCLE_TIME; // See datasheet pg 18 for cycle time calculation


// Packet structure

/**
 * @brief Register values and pin assignments for one E22 module.
 *
 * writeConfig_e22_900t22s() sends the first seven fields to the module in
 * this order. The rest tell the driver which UART and GPIOs to use.
 */
typedef struct
{
    uint8_t ADDH;   ///< Module address, high byte.
    uint8_t ADDL;   ///< Module address, low byte.
    uint8_t NETID;  ///< Network ID. Modules must share it to talk.
    uint8_t REG0;   ///< UART baud, parity, and air rate.
    uint8_t REG1;   ///< Sub-packet size, RSSI query, mode switching, and TX power.
    uint8_t REG2;   ///< Channel (#R2_E22Channel915).
    uint8_t REG3;   ///< RSSI byte, transfer method, repeater, LBT, and WOR settings.

    UART_HandleTypeDef *huart;  ///< UART whose baud the driver changes. Data transfers always use huart8.

    GPIO_TypeDef *E22_M0_PORT;  ///< M0 mode-select pin port.
    uint16_t E22_M0_PIN;        ///< M0 mode-select pin.

    GPIO_TypeDef *E22_M1_PORT;  ///< M1 mode-select pin port.
    uint16_t E22_M1_PIN;        ///< M1 mode-select pin.

    GPIO_TypeDef *E22_AUX_PORT; ///< AUX busy-indicator pin port.
    uint16_t E22_AUX_PIN;       ///< AUX busy-indicator pin.
} config_e22_900t22s;

#endif //KINGFISHER_SW_EBYTE_E22_900T22S_DEFS_H
