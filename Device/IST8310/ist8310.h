/**
 * @file ist8310.h
 * @brief ISENTEK IST8310 3-axis magnetometer driver
 *
 * This driver uses a weak-linking HAL pattern: platform-level functions are
 * declared __weak and must be overridden by the user for their specific MCU.
 */

#pragma once

#include "config.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/** @brief Magnetic field scale factor, uT per LSB */
#define IST8310_SCALE 0.3f

/** @brief IST8310 I2C address (7-bit) */
#define IST8310_ADDR 0x0E

/* ======== Register map ======== */

/** @brief Who-Am-I register */
#define IST8310_WHO_AM_I 0x00
/** @brief Output data start — X-axis low byte */
#define IST8310_DATA_XL 0x03
/** @brief Control register 2 — operating mode select */
#define IST8310_CNTL2 0x0A
/** @brief Control register 1 — interrupt config */
#define IST8310_CNTL1 0x0B
/** @brief Averaging control register */
#define IST8310_AVGCNTL 0x41
/** @brief Pulse-duration control register */
#define IST8310_PDCNTL 0x42

/* ======== Register bit fields ======== */

/** @brief Who-Am-I expected value */
#define IST8310_WHOAMI 0x10
/** @brief CNTL1: enable interrupt pin */
#define IST8310_CNTL1_INT_EN 0x0D
/** @brief AVGCNTL: 4× averaging */
#define IST8310_AVGCNTL_4X 0x12
/** @brief PDCNTL: pulse duration configuration */
#define IST8310_PDCNTL_CFG 0xC0
/** @brief CNTL2: continuous measurement mode @ 200 Hz */
#define IST8310_CNTL2_CONT_200HZ 0x03

/**
 * @brief IST8310 data handle
 *
 * Contains calibrated magnetic field readings (uT) with an acquisition
 * timestamp.
 */
typedef struct {
    uint32_t timestamp_ms; /**< Timestamp (ms) when the sample was acquired */
    float mag_x;           /**< Magnetic field along X-axis (uT) */
    float mag_y;           /**< Magnetic field along Y-axis (uT) */
    float mag_z;           /**< Magnetic field along Z-axis (uT) */
} IST8310_handle;

/* ======== User-overridable HAL stubs ( __weak ) ======== */

/** @brief User-specific pin and peripheral configuration */
__weak void IST8310_USR_CONFIG(void);
/** @brief Blocking delay in milliseconds */
__weak void IST8310_DELAY_MS(uint32_t ms);
/** @brief Free-running millisecond counter */
__weak uint32_t IST8310_GetStamp_ms(void);
/** @brief Hardware reset of the sensor */
__weak void IST8310_RESET(void);
/** @brief Read a single register byte over I2C/SPI */
__weak uint8_t IST8310_ReadReg(uint8_t reg);
/** @brief Write a single register byte over I2C/SPI */
__weak void IST8310_WriteReg(uint8_t reg, uint8_t data);
/** @brief Burst-read registers over I2C/SPI */
__weak void IST8310_ReadRegs(uint8_t start_reg, uint8_t *buffer, uint8_t length);

/* ======== Public API ======== */

/**
 * @brief  Initialize the IST8310 sensor
 * @return 0 on success, non-zero on failure (e.g. Who-Am-I mismatch)
 */
uint8_t IST8310_Init();

/**
 * @brief Read magnetometer data and populate a handle
 * @param handle Pointer to the handle to fill
 */
void IST8310_ReadData(IST8310_handle *handle);

/**
 * @brief Pretty-print a handle over stdio
 * @param handle Pointer to the handle to print
 */
void IST8310_Print(IST8310_handle *handle);
