#pragma once

#include <Arduino.h>


typedef struct
{
    struct { float x, y, z; } acc;
    struct { float x, y, z; } gyro;
    float temp;
} Imu_t;


extern Imu_t kimuData;


extern bool imuActive;

void imuInit(void);
void readIMU(void);
void calibrateIMU(void);
void imuSpiBegin(void);
void imuGpioBegin(void);
