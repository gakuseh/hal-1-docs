//
// Created by dyrel on 4/16/2026.
//

/**
 * @file SDCard.h
 * @ingroup platform
 * @brief Append-only text logging to HAL1_LOG.TXT on the SD card (FatFs).
 *
 * Every write takes one shared mutex and calls f_sync() straight away, so a
 * power loss loses at most the line being written, but each call can block
 * for as long as the card takes to sync.
 */

#ifndef KINGFISHER_SW_SDCARD_H
#define KINGFISHER_SW_SDCARD_H

#include <stdint.h>

/**
 * @brief Mounts the card and opens HAL1_LOG.TXT for appending.
 *
 * Call once before any SD_Log* call. FC_Init() retries it every 100 ms until
 * it succeeds.
 *
 * @return 0 on success or if already open; -20 if the FatFs driver was not
 *         linked (MX_FATFS_Init() not called first); -(FRESULT) if mounting
 *         failed (for example -3 = FR_NOT_READY; check sd_init_fail_step);
 *         -(100 + FRESULT) if opening the file failed after mounting.
 */
int8_t SD_Init();

/**
 * @brief Appends @p msg and a newline, then flushes to the card.
 * @param msg NUL-terminated text.
 * @return 0; -1 if not open; -2 write failed; -3 flush failed; -4 if the
 *         mutex was busy for 50 ms.
 */
int8_t SD_LogNewline(const char* msg);

/**
 * @brief Appends @p msg without a newline, then flushes to the card.
 * @param msg NUL-terminated text.
 * @return Same codes as SD_LogNewline().
 */
int8_t SD_LogInline(const char* msg);

/**
 * @brief Appends one GPS fix as a CSV row: lat,lon,alt,hh,mm,ss.
 * @return Same codes as SD_LogNewline().
 */
int8_t SD_LogGPS(double lat, double lon, double alt,
                 uint8_t hour, uint8_t min, uint8_t sec);

/**
 * @brief Was meant to open HAL1_DYN.TXT for dynamics-model logging.
 * @warning Declared but never defined. Calling it fails to link.
 */
int8_t SD_DynInit();

/**
 * @brief Appends one dynamics line. Writes to HAL1_LOG.TXT through
 *        SD_LogNewline(), not to a separate file. Unused.
 * @return Same codes as SD_LogNewline().
 */
int8_t SD_LogDyn(const char* msg);

/**
 * @brief Closes the log file and unmounts the card. Call before power-down.
 * @return 0.
 */
int8_t SD_Close();

#endif //KINGFISHER_SW_SDCARD_H
