# pragma once

#include "config.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define __weak __attribute__((weak))

#define ACCEL_SCALE (2 * ACCEL_RANGE / 65536.0f) // g per LSB
#define GYRO_SCALE (2 * GYRO_RANGE / 65536.0f) // dps per LSB
#define ACCEL_G 9.81f // Gravity acceleration constant used to convert accel output to m/s^2

typedef struct {
    uint32_t timestamp_ms; // HAL tick in milliseconds when this sample is acquired
    float accel_x; // Acceleration along X axis in m/s^2
    float accel_y; // Acceleration along Y axis in m/s^2
    float accel_z; // Acceleration along Z axis in m/s^2
    float gyro_x; // Angular velocity along X axis in deg/s
    float gyro_y; // Angular velocity along Y axis in deg/s
    float gyro_z; // Angular velocity along Z axis in deg/s
    float temperature; // Temperature in degree Celsius
} MPU6500_handle;

// These functions need to be realized by the user according to the specific hardware platform and communication protocol (SPI/I2C).
__weak void MPU6500_USR_CONFIG(void);
__weak void MPU6500_DELAY_MS(uint32_t ms);
__weak uint32_t MPU6500_GetStamp_ms(void);
__weak void MPU6500_ENACT(void);
__weak void MPU6500_DEACT(void);
__weak uint8_t MPU6500_ReadReg(uint8_t reg);
__weak void MPU6500_WriteReg(uint8_t reg, uint8_t data);
__weak void MPU6500_ReadRegs(uint8_t start_reg, uint8_t *buffer, uint8_t length);

uint8_t MPU6500_Init();
void MPU6500_ReadData(MPU6500_handle *handle);
void MPU6500_Print(MPU6500_handle *handle);
void MPU6500_Set_Gyro_Offset(float offset_x, float offset_y, float offset_z);

__weak uint8_t MPU6500_AUXIIC_ReadReg(uint8_t addr, uint8_t reg);
__weak void MPU6500_AUXIIC_WriteReg(uint8_t addr, uint8_t reg, uint8_t data);