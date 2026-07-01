/**
 * @file ist8310.c
 * @brief ISENTEK IST8310 3-axis magnetometer driver implementation
 */

#include "ist8310.h"

/**
 * @brief  Initialize the IST8310 sensor
 * @return 0 on success, 1 if Who-Am-I check fails
 */
uint8_t IST8310_Init(IST8310_handle *handle) {
    handle->ist8310_usr_cfg();
    handle->ist8310_reset();

    if (handle->ist8310_read_reg(IST8310_WHO_AM_I) != IST8310_WHOAMI) return 1;

    handle->ist8310_write_reg(IST8310_CNTL1, IST8310_CNTL1_INT_EN);
    handle->ist8310_write_reg(IST8310_AVGCNTL, IST8310_AVGCNTL_4X);
    handle->ist8310_write_reg(IST8310_PDCNTL, IST8310_PDCNTL_CFG);
    handle->ist8310_write_reg(IST8310_CNTL2, IST8310_CNTL2_CONT_200HZ);

    return 0;
}

/**
 * @brief Read magnetometer data and populate a handle
 * @param handle Pointer to the handle to fill
 * @param mag Pointer to the magnetic field structure to populate
 */
void IST8310_ReadData(IST8310_handle *handle, MagneticField *mag) {
    uint8_t buffer[6];
    handle->ist8310_read_regs(IST8310_DATA_XL, buffer, 6);
    mag->timestamp_ms = handle->ist8310_get_stamp_ms();
    mag->data.x = ((int16_t)(buffer[1] << 8 | buffer[0])) * IST8310_SCALE;
    mag->data.y = ((int16_t)(buffer[3] << 8 | buffer[2])) * IST8310_SCALE;
    mag->data.z = ((int16_t)(buffer[5] << 8 | buffer[4])) * IST8310_SCALE;
}

void IST8310_ReadData_DMA(IST8310_handle *handle, MagneticField *mag) {
    if (handle->dma_state == IST8310_BUSY) {
        // DMA transfer is already in progress, return early
        return;
    }
    mag->timestamp_ms = handle->ist8310_get_stamp_ms();

    handle->ist8310_readregs_dma(IST8310_DATA_XL, handle->dma_tx_buffer, handle->dma_rx_buffer, 6);
    handle->dma_state = IST8310_BUSY;
}

void IST8310_On_ReadData_DMA_Cplt(IST8310_handle *handle, MagneticField *mag) {
    if (handle->dma_state != IST8310_BUSY) {
        // DMA transfer is not in progress, return early
        return;
    }

    mag->data.x = ((int16_t)(handle->dma_rx_buffer[2] << 8 | handle->dma_rx_buffer[1])) * IST8310_SCALE;
    mag->data.y = ((int16_t)(handle->dma_rx_buffer[4] << 8 | handle->dma_rx_buffer[3])) * IST8310_SCALE;
    mag->data.z = ((int16_t)(handle->dma_rx_buffer[6] << 8 | handle->dma_rx_buffer[5])) * IST8310_SCALE;
    handle->dma_state = IST8310_IDLE;
}