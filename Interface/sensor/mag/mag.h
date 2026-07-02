#pragma once

#include <stdint.h>
#include "vector3.h"

/** @brief 3-axis magnetometer reading, unit: µT, with millisecond timestamp */
typedef struct {
    uint32_t timestamp_ms;
    Vector3 data;
} MagneticField;