/**
 * @file mpu6500.c
 * @brief InvenSense MPU-6500 6-axis IMU driver — handle-based implementation
 */

#include "mpu6500.h"

/**
 * @brief  Initialise the MPU6500: validates Who-Am-I, resets the signal
 *         path, configures DLPF (~5 Hz), full-scale ranges, sample-rate
 *         divider, and enables the data-ready interrupt.
 * @param  handle  Initialised handle with all function pointers set
 * @return 0 on success, 1 on Who-Am-I mismatch
 */
uint8_t MPU6500_Init(MPU6500_handle *handle) {

    if (handle->mpu6500_read_reg(MPU6500_WHO_AM_I) != MPU6500_WHOAMI_VAL) {
        return 1;
    }

    handle->mpu6500_write_reg(MPU6500_PWR_MGMT_1,       MPU6500_DEVICE_RESET);
    handle->mpu6500_delay_ms(100);
    handle->mpu6500_write_reg(MPU6500_SIGNAL_PATH_RESET,
                     MPU6500_GYRO_RST | MPU6500_ACCEL_RST | MPU6500_TEMP_RST);
    handle->mpu6500_delay_ms(100);

    handle->mpu6500_write_reg(MPU6500_USER_CTRL,
                     MPU6500_I2C_MST_EN | MPU6500_I2C_IF_DIS);

    handle->mpu6500_write_reg(MPU6500_CONFIG, MPU6500_DLPF_CFG_5HZ);

    uint8_t gyro_cfg;
    switch (GYRO_RANGE) {
        case 250:
            gyro_cfg = MPU6500_FS_SEL_250;
            break;
        case 500:
            gyro_cfg = MPU6500_FS_SEL_500;
            break;
        case 1000:
            gyro_cfg = MPU6500_FS_SEL_1000;
            break;
        case 2000:
            gyro_cfg = MPU6500_FS_SEL_2000;
            break;
        default:
            gyro_cfg = MPU6500_FS_SEL_250;
    }
    handle->mpu6500_write_reg(MPU6500_GYRO_CONFIG, gyro_cfg);

    uint8_t accel_cfg;
    switch (ACCEL_RANGE) {
        case 2:
            accel_cfg = MPU6500_AFS_SEL_2G;
            break;
        case 4:
            accel_cfg = MPU6500_AFS_SEL_4G;
            break;
        case 8:
            accel_cfg = MPU6500_AFS_SEL_8G;
            break;
        case 16:
            accel_cfg = MPU6500_AFS_SEL_16G;
            break;
        default:
            accel_cfg = MPU6500_AFS_SEL_2G;
    }
    handle->mpu6500_write_reg(MPU6500_ACCEL_CONFIG, accel_cfg);
    handle->mpu6500_write_reg(MPU6500_ACCEL_CONFIG2, MPU6500_DLPF_CFG_5HZ);
    handle->mpu6500_write_reg(MPU6500_SMPLRT_DIV, 19);
    handle->mpu6500_write_reg(MPU6500_INT_ENABLE, MPU6500_DATA_RDY_EN);

    return 0;
}

/**
 * @brief Read the latest IMU + temperature sample (blocking I2C / SPI)
 * @param handle  Initialised handle
 * @param imu     Output: populated Imu (accel m/s^2, gyro deg/s, timestamp_ms)
 * @param temp    Output: populated Temperature_Celsius (timestamp_ms, degC)
 */
void MPU6500_ReadData(MPU6500_handle *handle, Imu *imu, Temperature_Celsius *temp) {
    imu->timestamp_ms = handle->mpu6500_get_stamp_ms();
    temp->timestamp_ms = imu->timestamp_ms;

    uint8_t buf[14];
    handle->mpu6500_read_regs(MPU6500_ACCEL_XOUT_H, buf, 14);

    imu->acceleration.x    = (int16_t)((buf[0] << 8) | buf[1]) * ACCEL_SCALE * ACCEL_G;
    imu->acceleration.y    = (int16_t)((buf[2] << 8) | buf[3]) * ACCEL_SCALE * ACCEL_G;
    imu->acceleration.z    = (int16_t)((buf[4] << 8) | buf[5]) * ACCEL_SCALE * ACCEL_G;
    temp->data = ((int16_t)((buf[6] << 8) | buf[7]) / 333.87f) + 21.0f;
    imu->angular_velocity.x     = (int16_t)((buf[8]  << 8) | buf[9])  * GYRO_SCALE;
    imu->angular_velocity.y     = (int16_t)((buf[10] << 8) | buf[11]) * GYRO_SCALE;
    imu->angular_velocity.z     = (int16_t)((buf[12] << 8) | buf[13]) * GYRO_SCALE;
}

/**
 * @brief Start a non-blocking DMA read of the latest sample
 *
 * Timestamps are written immediately; actual data extraction happens in
 * MPU6500_On_ReadData_DMA_Cplt (called from DMA ISR).
 *
 * @param handle  Initialised handle
 * @param imu     Output: timestamp_ms is written now; accel/gyro later
 * @param temp    Output: timestamp_ms is written now; data later
 */
void MPU6500_ReadData_DMA(MPU6500_handle *handle, Imu *imu, Temperature_Celsius *temp) {
    if (handle->dma_state == MPU6500_BUSY) {
        return;
    }
    imu->timestamp_ms = handle->mpu6500_get_stamp_ms();
    temp->timestamp_ms = imu->timestamp_ms;

    handle->mpu6500_read_regs_dma(MPU6500_ACCEL_XOUT_H, handle->dma_tx_buffer, handle->dma_rx_buffer, 14);
    handle->dma_state = MPU6500_BUSY;
}

/**
 * @brief DMA-transfer-complete handler — call from ISR
 *
 * Parses the DMA rx buffer (rx[1..14] are the 14 sensor-data bytes;
 * rx[0] is a dummy byte from the address phase), scales raw values
 * to physical units, and marks DMA state back to IDLE.
 *
 * @param handle  Initialised handle (must be in MPU6500_BUSY state)
 * @param imu     Output: acceleration & angular_velocity populated here
 * @param temp    Output: temperature data populated here
 */
void MPU6500_On_ReadData_DMA_Cplt(MPU6500_handle *handle, Imu *imu, Temperature_Celsius *temp) {
    if (handle->dma_state != MPU6500_BUSY) {
        return;
    }
    handle->mpu6500_deact();

    imu->acceleration.x    = (int16_t)((handle->dma_rx_buffer[1] << 8) | handle->dma_rx_buffer[2]) * ACCEL_SCALE * ACCEL_G;
    imu->acceleration.y    = (int16_t)((handle->dma_rx_buffer[3] << 8) | handle->dma_rx_buffer[4]) * ACCEL_SCALE * ACCEL_G;
    imu->acceleration.z    = (int16_t)((handle->dma_rx_buffer[5] << 8) | handle->dma_rx_buffer[6]) * ACCEL_SCALE * ACCEL_G;
    temp->data = ((int16_t)((handle->dma_rx_buffer[7] << 8) | handle->dma_rx_buffer[8]) / 333.87f) + 21.0f;
    imu->angular_velocity.x     = (int16_t)((handle->dma_rx_buffer[9]  << 8) | handle->dma_rx_buffer[10])  * GYRO_SCALE;
    imu->angular_velocity.y     = (int16_t)((handle->dma_rx_buffer[11] << 8) | handle->dma_rx_buffer[12]) * GYRO_SCALE;
    imu->angular_velocity.z     = (int16_t)((handle->dma_rx_buffer[13] << 8) | handle->dma_rx_buffer[14]) * GYRO_SCALE;

    handle->dma_state = MPU6500_IDLE; // Reset the state to IDLE after processing
}

/**
 * @brief Set gyroscope offset calibration values (applied in hardware)
 * @param handle   Initialised handle
 * @param offset_x X-axis offset (deg/s)
 * @param offset_y Y-axis offset (deg/s)
 * @param offset_z Z-axis offset (deg/s)
 */
void MPU6500_Set_Gyro_Offset(MPU6500_handle *handle, float offset_x, float offset_y, float offset_z) {
    int16_t offset_x_raw = (int16_t)(offset_x * 32.8f);
    int16_t offset_y_raw = (int16_t)(offset_y * 32.8f);
    int16_t offset_z_raw = (int16_t)(offset_z * 32.8f);

    uint16_t x_reg = (uint16_t)offset_x_raw;
    uint16_t y_reg = (uint16_t)offset_y_raw;
    uint16_t z_reg = (uint16_t)offset_z_raw;

    handle->mpu6500_write_reg(MPU6500_XG_OFFS_USRH, x_reg >> 8);
    handle->mpu6500_write_reg(MPU6500_XG_OFFS_USRL, x_reg & 0xFF);
    handle->mpu6500_write_reg(MPU6500_YG_OFFS_USRH, y_reg >> 8);
    handle->mpu6500_write_reg(MPU6500_YG_OFFS_USRL, y_reg & 0xFF);
    handle->mpu6500_write_reg(MPU6500_ZG_OFFS_USRH, z_reg >> 8);
    handle->mpu6500_write_reg(MPU6500_ZG_OFFS_USRL, z_reg & 0xFF);
}

/**
 * @brief Read from an external sensor via the MPU6500 AUX I2C bus
 * @param handle      Initialised handle
 * @param addr        7-bit I2C address of the external sensor
 * @param reg         Register to read from the external sensor
 * @param timeout_ms  Maximum wait time in milliseconds
 * @return Register value, or best-effort read on timeout
 */
uint8_t MPU6500_AUXIIC_ReadReg(MPU6500_handle *handle, uint8_t addr, uint8_t reg, uint32_t timeout_ms) {
    handle->mpu6500_write_reg(MPU6500_I2C_SLV4_ADDR, addr | MPU6500_SLV_RNW);
    handle->mpu6500_write_reg(MPU6500_I2C_SLV4_REG, reg);
    handle->mpu6500_write_reg(MPU6500_I2C_SLV4_CTRL, MPU6500_SLV_EN);
    uint32_t now = handle->mpu6500_get_stamp_ms();
    while (!(handle->mpu6500_read_reg(MPU6500_I2C_STATUS) & MPU6500_SLV_DONE) && (handle->mpu6500_get_stamp_ms() - now < timeout_ms)) {
    }
    return handle->mpu6500_read_reg(MPU6500_I2C_SLV4_DI);
}

/**
 * @brief Write to an external sensor via the MPU6500 AUX I2C bus
 * @param handle      Initialised handle
 * @param addr        7-bit I2C address of the external sensor
 * @param reg         Register to write
 * @param data        Value to write
 * @param timeout_ms  Maximum wait time in milliseconds
 */
void MPU6500_AUXIIC_WriteReg(MPU6500_handle *handle, uint8_t addr, uint8_t reg, uint8_t data, uint32_t timeout_ms) {
    handle->mpu6500_write_reg(MPU6500_I2C_SLV4_ADDR, addr);
    handle->mpu6500_write_reg(MPU6500_I2C_SLV4_REG, reg);
    handle->mpu6500_write_reg(MPU6500_I2C_SLV4_DO, data);
    handle->mpu6500_write_reg(MPU6500_I2C_SLV4_CTRL, MPU6500_SLV_EN);
    uint32_t now = handle->mpu6500_get_stamp_ms();
    while (!(handle->mpu6500_read_reg(MPU6500_I2C_STATUS) & MPU6500_SLV_DONE) && (handle->mpu6500_get_stamp_ms() - now < timeout_ms)) {
    }
}