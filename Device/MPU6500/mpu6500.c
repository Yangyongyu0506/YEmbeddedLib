#include "mpu6500.h"

uint8_t MPU6500_Init() {
    MPU6500_USR_CONFIG(); // User-defined configuration
    // whoami
    if (MPU6500_ReadReg(0x75) != 0x70) {
        return 1; // MPU6500 not detected
    }
    // reset
    MPU6500_WriteReg(107, 0b10000000);
    MPU6500_DELAY_MS(100);
    MPU6500_WriteReg(104, 0b00000111);
    MPU6500_DELAY_MS(100);

    // prevent switching to IIC mode and enable IIC master mode for auxiliary sensor access
    MPU6500_WriteReg(106, 0b00110000);
    // configure low-pass filters
    MPU6500_WriteReg(26, 0b00000110); // Set DLPF to 5Hz for both gyro and accel
    // configure measurement ranges
    uint8_t gyro_cfg;
    switch (GYRO_RANGE) {
        case 250:
            gyro_cfg = 0b00000000;
            break;
        case 500:
            gyro_cfg = 0b00001000;
            break;
        case 1000:
            gyro_cfg = 0b00010000;
            break;
        case 2000:
            gyro_cfg = 0b00011000;
            break;
        default:
            gyro_cfg = 0b00000000; // Default to +/-250 dps
    }
    MPU6500_WriteReg(27, gyro_cfg); // Set Gyro range to selected value
    uint8_t accel_cfg;
    switch (ACCEL_RANGE) {
        case 2:
            accel_cfg = 0b00000000;
            break;
        case 4:
            accel_cfg = 0b00001000;
            break;
        case 8:
            accel_cfg = 0b00010000;
            break;
        case 16:
            accel_cfg = 0b00011000;
            break;
        default:
            accel_cfg = 0b00000000; // Default to +/-2 g
    }
    MPU6500_WriteReg(28, accel_cfg); // Set Accel range to selected value
    MPU6500_WriteReg(29, 0b00000110); // Set Accel DLPF to 5Hz
    MPU6500_WriteReg(25, 19); // Set sample rate to 1kHz (1kHz / (1 + 19) = 50Hz)
    MPU6500_WriteReg(56, 0b00000001); // Enable data-ready interrupt
    return 0; // Initialization successful
}

void MPU6500_ReadData(MPU6500_handle *handle) {
    handle->timestamp_ms = MPU6500_GetStamp_ms();
    uint8_t buf[14];
    MPU6500_ReadRegs(59, buf, 14); // Read 14 bytes starting from ACCEL_XOUT_H
    handle->accel_x = (int16_t)((buf[0] << 8) | buf[1]) * ACCEL_SCALE * ACCEL_G; // Convert to m/s^2
    handle->accel_y = (int16_t)((buf[2] << 8) | buf[3]) * ACCEL_SCALE * ACCEL_G; // Convert to m/s^2
    handle->accel_z = (int16_t)((buf[4] << 8) | buf[5]) * ACCEL_SCALE * ACCEL_G; // Convert to m/s^2
    handle->temperature = ((int16_t)((buf[6] << 8) | buf[7]) / 333.87f) + 21.0f; // Convert to degree Celsius
    handle->gyro_x = (int16_t)((buf[8] << 8) | buf[9]) * GYRO_SCALE; // Convert to deg/s
    handle->gyro_y = (int16_t)((buf[10] << 8) | buf[11]) * GYRO_SCALE; // Convert to deg/s
    handle->gyro_z = (int16_t)((buf[12] << 8) | buf[13]) * GYRO_SCALE; // Convert to deg/s
}

void MPU6500_Print(MPU6500_handle *handle) {
    printf("======\r\n");
    printf("stamp=%d\r\n", handle->timestamp_ms);
    printf("a=[%d, %d, %d]\r\n", (int)(100 * handle->accel_x), (int)(100 * handle->accel_y), (int)(100 * handle->accel_z));
    printf("|a|=%d\r\n", (int)(sqrt(handle->accel_x * handle->accel_x + handle->accel_y * handle->accel_y + handle->accel_z * handle->accel_z)));
    printf("g=[%d, %d, %d]\r\n", (int)(100 * handle->gyro_x), (int)(100 * handle->gyro_y), (int)(100 * handle->gyro_z));
    printf("|g|=%d\r\n", (int)(sqrt(handle->gyro_x * handle->gyro_x + handle->gyro_y * handle->gyro_y + handle->gyro_z * handle->gyro_z)));
    printf("temp=%d\r\n", (int)(100 * handle->temperature));
}

void MPU6500_Set_Gyro_Offset(float offset_x, float offset_y, float offset_z) {
    int16_t offset_x_raw = (int16_t)(offset_x * 32.8f);
    int16_t offset_y_raw = (int16_t)(offset_y * 32.8f);
    int16_t offset_z_raw = (int16_t)(offset_z * 32.8f);
    uint16_t x_reg = (uint16_t)offset_x_raw;
    uint16_t y_reg = (uint16_t)offset_y_raw;
    uint16_t z_reg = (uint16_t)offset_z_raw;
    MPU6500_WriteReg(19, x_reg >> 8);
    MPU6500_WriteReg(20, x_reg & 0xFF);
    MPU6500_WriteReg(21, y_reg >> 8);
    MPU6500_WriteReg(22, y_reg & 0xFF);
    MPU6500_WriteReg(23, z_reg >> 8);
    MPU6500_WriteReg(24, z_reg & 0xFF);
}

uint8_t MPU6500_AUXIIC_ReadReg(uint8_t addr, uint8_t reg) {
    MPU6500_WriteReg(49, addr | 0b10000000); // Set read bit
    MPU6500_WriteReg(50, reg);
    MPU6500_WriteReg(52, 0b10000000); // Enable read
    while (!(MPU6500_ReadReg(54) & 0b01000000)) {
        // Wait for read to complete
    }
    return MPU6500_ReadReg(53);
}

void MPU6500_AUXIIC_WriteReg(uint8_t addr, uint8_t reg, uint8_t data) {
    MPU6500_WriteReg(49, addr); // Set write bit
    MPU6500_WriteReg(50, reg);
    MPU6500_WriteReg(51, data);
    MPU6500_WriteReg(52, 0b10000000); // Enable write
    while (!(MPU6500_ReadReg(54) & 0b01000000)) {
        // Wait for write to complete
    }
}