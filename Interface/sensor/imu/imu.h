#pragma once

#include "config.h"
#include "vector3.h"
#include <stdint.h>

/**
 * @brief Aggregated 6-axis IMU data with timestamp
 *
 * - acceleration:   linear acceleration in m/s^2
 * - angular_velocity: angular velocity in deg/s
 * - timestamp_ms:   millisecond timestamp of the sensor reading
 */
typedef struct {
    uint32_t timestamp_ms;
    Vector3 acceleration;
    Vector3 angular_velocity;
} Imu;

/**
 * @brief Serialise Imu data into a framed payload buffer
 *
 * Lay out: 2-byte header (config.h PAYLOAD_HEADER_*) + Imu fields
 * in big-endian float encoding.  Caller must supply a buffer large
 * enough for the serialised frame.
 *
 * @param imu     Input IMU data
 * @param payload Output payload buffer (must be pre-allocated)
 */
void Imu2payload(const Imu *imu, uint8_t *payload);