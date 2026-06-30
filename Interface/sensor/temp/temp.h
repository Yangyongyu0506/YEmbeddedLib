# pragma once

#include <stdint.h>

typedef struct {
    uint32_t timestamp_ms;
    float data;
} Temperature_Celsius;

typedef struct {
    uint32_t timestamp_ms;
    float data;
} Temperature_Fahrenheit;

typedef struct {
    uint32_t timestamp_ms;
    float data;
} Temperature_Kelvin;