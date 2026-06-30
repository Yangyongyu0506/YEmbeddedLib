/**
 * @file mpu6500.c
 * @brief InvenSense MPU-6500 6-axis IMU driver implementation
 */

#include "mpu6500.h"

/**
 * @brief  Initialize the MPU6500, configure filters and ranges
 * @return 0 on success, 1 if Who-Am-I check fails
 */
uint8_t MPU6500_Init() {
    MPU6500_USR_CONFIG();

    if (MPU6500_ReadReg(MPU6500_WHO_AM_I) != 0x70) {
        return 1;
    }

    MPU6500_WriteReg(MPU6500_PWR_MGMT_1,       MPU6500_DEVICE_RESET);
    MPU6500_DELAY_MS(100);
    MPU6500_WriteReg(MPU6500_SIGNAL_PATH_RESET,
                     MPU6500_GYRO_RST | MPU6500_ACCEL_RST | MPU6500_TEMP_RST);
    MPU6500_DELAY_MS(100);

    MPU6500_WriteReg(MPU6500_USER_CTRL,
                     MPU6500_I2C_MST_EN | MPU6500_I2C_IF_DIS);

    MPU6500_WriteReg(MPU6500_CONFIG, MPU6500_DLPF_CFG_5HZ);

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
    MPU6500_WriteReg(MPU6500_GYRO_CONFIG, gyro_cfg);

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
    MPU6500_WriteReg(MPU6500_ACCEL_CONFIG, accel_cfg);

    MPU6500_WriteReg(MPU6500_ACCEL_CONFIG2, MPU6500_DLPF_CFG_5HZ);
    MPU6500_WriteReg(MPU6500_SMPLRT_DIV, 19);
    MPU6500_WriteReg(MPU6500_INT_ENABLE, MPU6500_DATA_RDY_EN);

    return 0;
}

/**
 * @brief Read IMU data (accel, gyro, temperature) and populate a handle
 * @param handle Pointer to the handle to fill
 */
void MPU6500_ReadData(Imu *imu, Temperature_Celsius *temp) {
    imu->timestamp_ms = MPU6500_GetStamp_ms();

    uint8_t buf[14];
    MPU6500_ReadRegs(MPU6500_ACCEL_XOUT_H, buf, 14);

    imu->acceleration.x    = (int16_t)((buf[0] << 8) | buf[1]) * ACCEL_SCALE * ACCEL_G;
    imu->acceleration.y    = (int16_t)((buf[2] << 8) | buf[3]) * ACCEL_SCALE * ACCEL_G;
    imu->acceleration.z    = (int16_t)((buf[4] << 8) | buf[5]) * ACCEL_SCALE * ACCEL_G;
    temp->data = ((int16_t)((buf[6] << 8) | buf[7]) / 333.87f) + 21.0f;
    imu->angular_velocity.x     = (int16_t)((buf[8]  << 8) | buf[9])  * GYRO_SCALE;
    imu->angular_velocity.y     = (int16_t)((buf[10] << 8) | buf[11]) * GYRO_SCALE;
    imu->angular_velocity.z     = (int16_t)((buf[12] << 8) | buf[13]) * GYRO_SCALE;
}

/**
 * @brief Set gyroscope offset calibration values
 * @param offset_x X-axis offset (deg/s)
 * @param offset_y Y-axis offset (deg/s)
 * @param offset_z Z-axis offset (deg/s)
 */
void MPU6500_Set_Gyro_Offset(float offset_x, float offset_y, float offset_z) {
    int16_t offset_x_raw = (int16_t)(offset_x * 32.8f);
    int16_t offset_y_raw = (int16_t)(offset_y * 32.8f);
    int16_t offset_z_raw = (int16_t)(offset_z * 32.8f);

    uint16_t x_reg = (uint16_t)offset_x_raw;
    uint16_t y_reg = (uint16_t)offset_y_raw;
    uint16_t z_reg = (uint16_t)offset_z_raw;

    MPU6500_WriteReg(MPU6500_XG_OFFS_USRH, x_reg >> 8);
    MPU6500_WriteReg(MPU6500_XG_OFFS_USRL, x_reg & 0xFF);
    MPU6500_WriteReg(MPU6500_YG_OFFS_USRH, y_reg >> 8);
    MPU6500_WriteReg(MPU6500_YG_OFFS_USRL, y_reg & 0xFF);
    MPU6500_WriteReg(MPU6500_ZG_OFFS_USRH, z_reg >> 8);
    MPU6500_WriteReg(MPU6500_ZG_OFFS_USRL, z_reg & 0xFF);
}

/**
 * @brief Read from an external sensor via the MPU6500 AUX I2C bus
 * @param addr 7-bit I2C address of the external sensor
 * @param reg  Register to read from the external sensor
 * @return Register value, or 0 on failure
 */
uint8_t MPU6500_AUXIIC_ReadReg(uint8_t addr, uint8_t reg) {
    MPU6500_WriteReg(MPU6500_I2C_SLV0_ADDR, addr | MPU6500_SLV_RNW);
    MPU6500_WriteReg(MPU6500_I2C_SLV0_REG, reg);
    MPU6500_WriteReg(MPU6500_I2C_SLV0_CTRL, MPU6500_SLV_EN);
    while (!(MPU6500_ReadReg(MPU6500_I2C_STATUS) & MPU6500_SLV_DONE)) {
    }
    return MPU6500_ReadReg(MPU6500_I2C_SLV0_DI);
}

/**
 * @brief Write to an external sensor via the MPU6500 AUX I2C bus
 * @param addr 7-bit I2C address of the external sensor
 * @param reg  Register to write
 * @param data Value to write
 */
void MPU6500_AUXIIC_WriteReg(uint8_t addr, uint8_t reg, uint8_t data) {
    MPU6500_WriteReg(MPU6500_I2C_SLV0_ADDR, addr);
    MPU6500_WriteReg(MPU6500_I2C_SLV0_REG, reg);
    MPU6500_WriteReg(MPU6500_I2C_SLV0_DO, data);
    MPU6500_WriteReg(MPU6500_I2C_SLV0_CTRL, MPU6500_SLV_EN);
    while (!(MPU6500_ReadReg(MPU6500_I2C_STATUS) & MPU6500_SLV_DONE)) {
    }
}
