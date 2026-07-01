/**
 * @file mpu6500.h
 * @brief InvenSense MPU-6500 6-axis IMU (gyro + accelerometer) driver
 *
 * This driver uses a weak-linking HAL pattern: platform-level functions are
 * declared __weak and must be overridden by the user for their specific MCU.
 *
 * The MPU-6500 also exposes AUX I2C master functions
 * (MPU6500_AUXIIC_ReadReg / MPU6500_AUXIIC_WriteReg) used to interface an
 * external sensor (e.g. IST8310) via the MPU-6500's auxiliary I2C bus.
 */

#pragma once

#include "config.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "imu.h"
#include "temp.h"

/** @brief Accelerometer scale factor, g per LSB */
#define ACCEL_SCALE (2 * ACCEL_RANGE / 65536.0f)
/** @brief Gyroscope scale factor, dps per LSB */
#define GYRO_SCALE (2 * GYRO_RANGE / 65536.0f)
/** @brief Standard gravity, used to convert g → m/s^2 */
#define ACCEL_G 9.81f

/* ======== Register map ======== */

#define MPU6500_XG_OFFS_USRH      0x13 /**< Gyro X offset, high byte */
#define MPU6500_XG_OFFS_USRL      0x14 /**< Gyro X offset, low byte  */
#define MPU6500_YG_OFFS_USRH      0x15 /**< Gyro Y offset, high byte */
#define MPU6500_YG_OFFS_USRL      0x16 /**< Gyro Y offset, low byte  */
#define MPU6500_ZG_OFFS_USRH      0x17 /**< Gyro Z offset, high byte */
#define MPU6500_ZG_OFFS_USRL      0x18 /**< Gyro Z offset, low byte  */
#define MPU6500_SMPLRT_DIV        0x19 /**< Sample-rate divider */
#define MPU6500_CONFIG            0x1A /**< DLPF / ext-sync config */
#define MPU6500_GYRO_CONFIG       0x1B /**< Gyro full-scale select */
#define MPU6500_ACCEL_CONFIG      0x1C /**< Accel full-scale select */
#define MPU6500_ACCEL_CONFIG2     0x1D /**< Accel DLPF config */
#define MPU6500_I2C_SLV4_ADDR     0x31 /**< AUX I2C slave 4 address */
#define MPU6500_I2C_SLV4_REG      0x32 /**< AUX I2C slave 4 register */
#define MPU6500_I2C_SLV4_DO       0x33 /**< AUX I2C slave 4 data output */
#define MPU6500_I2C_SLV4_CTRL     0x34 /**< AUX I2C slave 4 control */
#define MPU6500_I2C_SLV4_DI       0x35 /**< AUX I2C slave 4 data input */
#define MPU6500_I2C_STATUS        0x36 /**< AUX I2C status */
#define MPU6500_INT_ENABLE        0x38 /**< Interrupt enable */
#define MPU6500_ACCEL_XOUT_H      0x3B /**< Accel X output, high byte */
#define MPU6500_SIGNAL_PATH_RESET 0x68 /**< Signal-path reset */
#define MPU6500_USER_CTRL         0x6A /**< User control */
#define MPU6500_PWR_MGMT_1        0x6B /**< Power management 1 */
#define MPU6500_WHO_AM_I          0x75 /**< Who-Am-I register */

/* ======== Register bit fields ======== */

/** @brief PWR_MGMT_1: device reset */
#define MPU6500_DEVICE_RESET  0x80
/** @brief SIGNAL_PATH_RESET: reset gyro, accel, and temp */
#define MPU6500_GYRO_RST      0x04
/** @brief SIGNAL_PATH_RESET: reset accel */
#define MPU6500_ACCEL_RST     0x02
/** @brief SIGNAL_PATH_RESET: reset temp */
#define MPU6500_TEMP_RST      0x01
/** @brief USER_CTRL: enable AUX I2C master */
#define MPU6500_I2C_MST_EN    0x20
/** @brief USER_CTRL: disable primary I2C (use SPI only) */
#define MPU6500_I2C_IF_DIS    0x10
/** @brief CONFIG / ACCEL_CONFIG2: DLPF bandwidth ~5 Hz */
#define MPU6500_DLPF_CFG_5HZ  0x06
/** @brief GYRO_CONFIG: ±250 dps */

#define MPU6500_FS_SEL_250    0x00
/** @brief GYRO_CONFIG: ±500 dps */
#define MPU6500_FS_SEL_500    0x08
/** @brief GYRO_CONFIG: ±1000 dps */
#define MPU6500_FS_SEL_1000   0x10
/** @brief GYRO_CONFIG: ±2000 dps */
#define MPU6500_FS_SEL_2000   0x18
/** @brief ACCEL_CONFIG: ±2 g */
#define MPU6500_AFS_SEL_2G    0x00
/** @brief ACCEL_CONFIG: ±4 g */
#define MPU6500_AFS_SEL_4G    0x08
/** @brief ACCEL_CONFIG: ±8 g */
#define MPU6500_AFS_SEL_8G    0x10
/** @brief ACCEL_CONFIG: ±16 g */
#define MPU6500_AFS_SEL_16G   0x18
/** @brief INT_ENABLE: data-ready interrupt */
#define MPU6500_DATA_RDY_EN   0x01
/** @brief I2C_SLV0_ADDR: read / write bit */
#define MPU6500_SLV_RNW       0x80
/** @brief I2C_SLV0_CTRL: enable slave transaction */
#define MPU6500_SLV_EN        0x80
/** @brief I2C_STATUS: slave transaction complete */
#define MPU6500_SLV_DONE      0x40

typedef enum {
    MPU6500_BUSY = 0,
    MPU6500_IDLE = 1
} MPU6500_DMA_state;

typedef struct {
    MPU6500_DMA_state dma_state;
    uint8_t dma_tx_buffer[15];
    uint8_t dma_rx_buffer[15];
    void (*mpu6500_delay_ms)(uint32_t ms);
    uint32_t (*mpu6500_get_stamp_ms)(void);
    void (*mpu6500_enact)(void);
    void (*mpu6500_deact)(void);
    uint8_t (*mpu6500_read_reg)(uint8_t reg);
    void (*mpu6500_write_reg)(uint8_t reg, uint8_t data);
    void (*mpu6500_read_regs)(uint8_t start_reg, uint8_t *buffer, uint8_t length);
    void (*mpu6500_read_regs_dma)(uint8_t start_reg,uint8_t *tx_buffer, uint8_t *rx_buffer, uint8_t length);
} MPU6500_handle;

/* ======== Public API ======== */

/**
 * @brief  Initialize the MPU6500, configure filters and ranges
 * @return 0 on success, non-zero on failure (e.g. Who-Am-I mismatch)
 */
uint8_t MPU6500_Init(MPU6500_handle *handle);

/**
 * @brief Read IMU data (accel, gyro, temperature) and populate a handle
 * @param handle Pointer to the handle to fill
 * @param imu Pointer to the Imu structure to populate
 * @param temp Pointer to the Temperature_Celsius structure to populate
 */
void MPU6500_ReadData(MPU6500_handle *handle, Imu *imu, Temperature_Celsius *temp);

void MPU6500_ReadData_DMA(MPU6500_handle *handle, Imu *imu, Temperature_Celsius *temp);

void MPU6500_On_ReadData_DMA_Cplt(MPU6500_handle *handle, Imu *imu, Temperature_Celsius *temp);

/**
 * @brief Set gyroscope offset calibration values
 * @param offset_x X-axis offset (deg/s)
 * @param offset_y Y-axis offset (deg/s)
 * @param offset_z Z-axis offset (deg/s)
 */
void MPU6500_Set_Gyro_Offset(MPU6500_handle *handle, float offset_x, float offset_y, float offset_z);

uint8_t MPU6500_AUXIIC_ReadReg(MPU6500_handle *handle, uint8_t addr, uint8_t reg);

void MPU6500_AUXIIC_WriteReg(MPU6500_handle *handle, uint8_t addr, uint8_t reg, uint8_t data);