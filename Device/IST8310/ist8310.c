/**
 * @file ist8310.c
 * @brief ISENTEK IST8310 3-axis magnetometer driver implementation
 */

#include "ist8310.h"

/**
 * @brief  Initialize the IST8310 sensor
 * @return 0 on success, 1 if Who-Am-I check fails
 */
uint8_t IST8310_Init() {
    IST8310_USR_CONFIG();
    IST8310_RESET();

    if (IST8310_ReadReg(IST8310_WHO_AM_I) != IST8310_WHOAMI) return 1;

    IST8310_WriteReg(IST8310_CNTL1, IST8310_CNTL1_INT_EN);
    IST8310_WriteReg(IST8310_AVGCNTL, IST8310_AVGCNTL_4X);
    IST8310_WriteReg(IST8310_PDCNTL, IST8310_PDCNTL_CFG);
    IST8310_WriteReg(IST8310_CNTL2, IST8310_CNTL2_CONT_200HZ);

    return 0;
}

/**
 * @brief Read magnetometer data and populate a handle
 * @param handle Pointer to the handle to fill
 */
void IST8310_ReadData(MagneticField *mag) {
    uint8_t buffer[6];
    IST8310_ReadRegs(IST8310_DATA_XL, buffer, 6);
    mag->timestamp_ms = IST8310_GetStamp_ms();
    mag->data.x = ((int16_t)(buffer[1] << 8 | buffer[0])) * IST8310_SCALE;
    mag->data.y = ((int16_t)(buffer[3] << 8 | buffer[2])) * IST8310_SCALE;
    mag->data.z = ((int16_t)(buffer[5] << 8 | buffer[4])) * IST8310_SCALE;
}