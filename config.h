#pragma once

/** @brief Accelerometer and gyroscope full-scale ranges (compile-time configurable) */
#define ACCEL_RANGE 2 // Available options: 2, 4, 8, 16
#define GYRO_RANGE 250 // Available options: 250, 500, 1000, 2000

// payload design
#define PAYLOAD_HEADER_0 0xAA
#define PAYLOAD_HEADER_1 0x55