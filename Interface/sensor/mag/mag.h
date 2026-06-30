# pragma once

#include <stdint.h>
#include "vector3.h"

typedef struct {
    uint32_t timestamp_ms;
    Vector3 data; // Magnetic field data (x, y, z)
} MagneticField;