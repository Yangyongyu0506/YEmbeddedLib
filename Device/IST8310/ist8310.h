/**
 * @file ist8310.h
 * @brief ISENTEK IST8310 3-axis magnetometer driver
 *
 * This driver uses a handle-based HAL pattern: platform-level I/O routines
 * are stored as function pointers in an IST8310_handle struct provided by
 * the user.  Multiple IST8310 instances can coexist simply by using separate
 * handles.
 */

#pragma once

#include "config.h"
#include <stdint.h>
#include <math.h>
#include "mag.h"

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

/* ======== Handle types ======== */

/** @brief DMA transaction state used by the driver internally */
typedef enum {
    IST8310_BUSY = 0, /**< A DMA transfer is currently in progress */
    IST8310_IDLE = 1  /**< No DMA transfer outstanding */
} IST8310_DMA_state;

/**
 * @brief Per-instance hardware abstraction handle
 *
 * Users allocate one handle per IST8310 sensor, fill in the function
 * pointers matching their MCU platform, and pass the handle to every
 * API call.  The DMA buffers are 7 bytes:
 *  - tx_buffer[0] = register address (sent during DMA address phase)
 *  - rx_buffer[0] = dummy byte   (received during address phase)
 *  - rx_buffer[1..6] = 6 bytes of sensor data
 */
typedef struct {
    volatile IST8310_DMA_state dma_state;         /**< Guard to prevent overlapping DMA requests */
    uint8_t dma_tx_buffer[7];                     /**< Scratch buffer for DMA TX (1 addr + 6 data) */
    uint8_t dma_rx_buffer[7];                     /**< Scratch buffer for DMA RX (1 dummy + 6 data) */
    void (*ist8310_usr_cfg)(void);                /**< GPIO / pin-mux initialisation */
    void (*ist8310_delay_ms)(uint32_t ms);        /**< Blocking millisecond delay */
    uint32_t (*ist8310_get_stamp_ms)(void);       /**< Free-running millisecond counter */
    void (*ist8310_reset)(void);                  /**< Hardware reset line toggle */
    uint8_t (*ist8310_read_reg)(uint8_t reg);     /**< Single register read (I2C / SPI) */
    void (*ist8310_write_reg)(uint8_t reg, uint8_t data);  /**< Single register write */
    void (*ist8310_read_regs)(uint8_t start_reg, uint8_t *buffer, uint8_t length);        /**< Burst register read (blocking) */
    void (*ist8310_read_regs_dma)(uint8_t start_reg, uint8_t *tx_buffer, uint8_t *rx_buffer, uint8_t length); /**< Burst register read (DMA, non-blocking) */
} IST8310_handle;

/* ======== Public API ======== */

/**
 * @brief  Initialise the IST8310: validates Who-Am-I and configures
 *         measurement mode (200 Hz continuous, 4× averaging)
 * @param  handle  Initialised handle with all function pointers set
 * @return 0 on success, 1 on Who-Am-I mismatch
 */
uint8_t IST8310_Init(IST8310_handle *handle);

/**
 * @brief Read the latest magnetometer sample (blocking I2C / SPI)
 * @param handle  Initialised handle
 * @param mag     Output: populated MagneticField (µT, timestamp_ms)
 */
void IST8310_ReadData(IST8310_handle *handle, MagneticField *mag);

/**
 * @brief Start a non-blocking DMA read of the latest sample
 *
 * The actual data extraction happens in the completion callback
 * IST8310_On_ReadData_DMA_Cplt, which the user must call from their
 * DMA-transfer-complete ISR.
 *
 * @param handle  Initialised handle
 * @param mag     Output: timestamp is written immediately;
 *                x/y/z are written later by the completion callback
 */
void IST8310_ReadData_DMA(IST8310_handle *handle, MagneticField *mag);

/**
 * @brief DMA-transfer-complete handler — call from ISR
 *
 * Parses the DMA rx buffer, scales raw values to µT, and marks the
 * DMA state back to IDLE so the next IST8310_ReadData_DMA can proceed.
 *
 * @param handle  Initialised handle (must be in IST8310_BUSY state)
 * @param mag     Output: x/y/z are populated here
 */
void IST8310_On_ReadData_DMA_Cplt(IST8310_handle *handle, MagneticField *mag);
