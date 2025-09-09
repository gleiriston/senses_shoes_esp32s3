#pragma once

#include <Arduino.h>

// ===== Dados do IMU (compatível com seu main)
typedef struct
{
    struct { float x, y, z; } acc;
    struct { float x, y, z; } gyro;
    float temp;
} Imu_t;

// Instância global lida pelo main
extern Imu_t kimuData;

// Controla envio/uso do IMU no main (já existe lá)
extern bool imuActive;

// ==== API ====
/** Inicializa o IMU (SPI), configura FS/ODR e faz calibração simples. */
void imuInit(void);

/** Lê uma amostra do IMU, aplica offsets e filtros de Kalman, preenche kimuData. */
void readIMU(void);

// (opcionais, caso queira usar separado em outro ponto)
void calibrateIMU(void);
void imuSpiBegin(void);
void imuGpioBegin(void);
