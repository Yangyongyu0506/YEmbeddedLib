#include "ist8310.h"

uint8_t IST8310_Init() {
    IST8310_USR_CONFIG(); // User-defined configuration
    IST8310_RESET(); // Reset the IST8310 sensor

    // whoami
    if (IST8310_ReadReg(0x00) != 0x10) return 1;
    IST8310_WriteReg(0x0B, 0b00001101); // Enable INT
    IST8310_WriteReg(0x41, 0b00010010); // avg settings
    IST8310_WriteReg(0x42, 0b11000000); // pulse duration ctrl
    IST8310_WriteReg(0x0A, 0b00000011); // set to continuous measurement mode
    return 0;
}

void IST8310_ReadData(IST8310_handle *handle) {
    uint8_t buffer[6];
    IST8310_ReadRegs(0x03, buffer, 6);
    handle->timestamp_ms = IST8310_GetStamp_ms();
    handle->mag_x = ((int16_t)(buffer[1] << 8 | buffer[0])) * IST8310_SCALE;
    handle->mag_y = ((int16_t)(buffer[3] << 8 | buffer[2])) * IST8310_SCALE;
    handle->mag_z = ((int16_t)(buffer[5] << 8 | buffer[4])) * IST8310_SCALE;
}

void IST8310_Print(IST8310_handle *handle) {
    printf("======\r\n");
    printf("stamp=%d\r\n", handle->timestamp_ms);
    printf("m=[%d, %d, %d]\r\n", (int)(100 * handle->mag_x), (int)(100 * handle->mag_y), (int)(100 * handle->mag_z));
    printf("|m|=%d\r\n", (int)(sqrt(handle->mag_x * handle->mag_x + handle->mag_y * handle->mag_y + handle->mag_z * handle->mag_z)));
}