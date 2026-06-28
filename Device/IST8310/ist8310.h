#pragma once

#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define __weak __attribute__((weak))

#define IST8310_SCALE 0.3f // uT per LSB
#define IST8310_ADDR 0x0E

typedef struct {
    uint32_t timestamp_ms; // timestamp in milliseconds when this sample is acquired
    float mag_x; // Magnetic field along X axis in uT
    float mag_y; // Magnetic field along Y axis in uT
    float mag_z; // Magnetic field along Z axis in uT
} IST8310_handle;

__weak void IST8310_USR_CONFIG(void);
__weak void IST8310_DELAY_MS(uint32_t ms);
__weak uint32_t IST8310_GetStamp_ms(void);
__weak void IST8310_RESET(void);
__weak uint8_t IST8310_ReadReg(uint8_t reg);
__weak void IST8310_WriteReg(uint8_t reg, uint8_t data);
__weak void IST8310_ReadRegs(uint8_t start_reg, uint8_t *buffer, uint8_t length);

uint8_t IST8310_Init();
void IST8310_ReadData(IST8310_handle *handle);
void IST8310_Print(IST8310_handle *handle);