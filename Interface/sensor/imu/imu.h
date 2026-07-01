# pragma once

#include "config.h"
#include "vector3.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  uint32_t timestamp_ms;      // Timestamp of the IMU data
  Vector3 acceleration; // Accelerometer data (x, y, z)
  Vector3 angular_velocity;  // Gyroscope data (x, y, z)
} Imu;

void Imu2payload(const Imu* imu, uint8_t* payload);