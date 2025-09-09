#include "imu.hpp"
#include "device.hpp"
#include "system_constants.hpp"   // IMU_SPI_SCK/MISO/MOSI/CS, IMU_INT1/INT2
#include "kalman_filter.hpp"
#include <SPI.h>
#include "ICM42688.h"

// ===== Variáveis globais =====
Imu_t kimuData = {0};
extern bool imuActive;           // definida no main (true/false)

// ===== Driver do IMU (ICM42688 em SPI) =====
static SPIClass &SPI_IMU = SPI;
static SPISettings IMU_SPI_SETTINGS(8000000, MSBFIRST, SPI_MODE0);
static ICM42688 gImu(SPI_IMU, IMU_SPI_CS);

// ===== Offsets (preenchidos na calibração) =====
static float accelOffsetX = 0.0f, accelOffsetY = 0.0f, accelOffsetZ = 0.0f;
static float gyroOffsetX  = 0.0f, gyroOffsetY  = 0.0f, gyroOffsetZ  = 0.0f;

// ===== Filtros de Kalman (um por eixo) =====
// (q, r, p0, x0) — ajuste q/r conforme sua necessidade de suavização vs. resposta
static KalmanFilter kalmanAccX(0.1f, 0.1f, 0.1f, 0.0f);
static KalmanFilter kalmanAccY(0.1f, 0.1f, 0.1f, 0.0f);
static KalmanFilter kalmanAccZ(0.1f, 0.1f, 0.1f, 0.0f);
static KalmanFilter kalmanGyrX(0.1f, 0.1f, 0.1f, 0.0f);
static KalmanFilter kalmanGyrY(0.1f, 0.1f, 0.1f, 0.0f);
static KalmanFilter kalmanGyrZ(0.1f, 0.1f, 0.1f, 0.0f);

// ===== GPIO / SPI =====
void imuGpioBegin() {
#ifdef IMU_INT1_PIN
    pinMode(IMU_INT1_PIN, INPUT);
#endif
#ifdef IMU_INT2_PIN
    pinMode(IMU_INT2_PIN, INPUT);
#endif
}

void imuSpiBegin() {
    pinMode(IMU_SPI_CS, OUTPUT);
    digitalWrite(IMU_SPI_CS, HIGH);           // CS inativo
    SPI_IMU.begin(IMU_SPI_SCK, IMU_SPI_MISO, IMU_SPI_MOSI, IMU_SPI_CS);
}

// ===== Calibração (média simples) =====
void calibrateIMU()
{
    const int knumSamples = 100;
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;

    for (int i = 0; i < knumSamples; i++)
    {
        gImu.getAGT(); // lê acc/gyro/temp
        ax += gImu.accX();
        ay += gImu.accY();
        az += gImu.accZ();
        gx += gImu.gyrX();
        gy += gImu.gyrY();
        gz += gImu.gyrZ();
        delay(2); // pequeno intervalo ajuda a estabilizar
    }

    accelOffsetX = ax / knumSamples;
    accelOffsetY = ay / knumSamples;
    accelOffsetZ = az / knumSamples;
    gyroOffsetX  = gx / knumSamples;
    gyroOffsetY  = gy / knumSamples;
    gyroOffsetZ  = gz / knumSamples;
}

// ===== Init principal =====
void imuInit(void)
{
    // zera estrutura pública
    kimuData = {};

    imuGpioBegin();
    imuSpiBegin();

    // Inicializa o driver
    int status = gImu.begin();
    if (status < 0)
    {
        Serial.println("[IMU] ERRO: ICM42688.begin() falhou");
        // trava aqui se preferir, mas vamos só retornar pra não bloquear todo o app
        return;
    }

    // Config padrão (ajuste se quiser casar com 60 Hz do stream)
    gImu.setAccelFS(ICM42688::gpm8);
    gImu.setGyroFS(ICM42688::dps500);
    gImu.setAccelODR(ICM42688::odr12_5);
    gImu.setGyroODR(ICM42688::odr12_5);

    // Calibração inicial (pé parado)
    calibrateIMU();

    Serial.println("[IMU] ICM42688 inicializado e calibrado.");
}

// ===== Leitura + filtros =====
void readIMU(void)
{
    if (!imuActive) {
        return; // IMU desativado pelo app/comando
    }

    // Lê todos os dados do sensor
    gImu.getAGT();

    // Brutos
    float rawAccX = gImu.accX();
    float rawAccY = gImu.accY();
    float rawAccZ = gImu.accZ();
    float rawGyrX = gImu.gyrX();
    float rawGyrY = gImu.gyrY();
    float rawGyrZ = gImu.gyrZ();

    // Remove offsets (DC)
    rawAccX -= accelOffsetX;  rawAccY -= accelOffsetY;  rawAccZ -= accelOffsetZ;
    rawGyrX -= gyroOffsetX;   rawGyrY -= gyroOffsetY;   rawGyrZ -= gyroOffsetZ;

    // Filtragem Kalman
    kimuData.acc.x  = kalmanAccX.update(rawAccX);
    kimuData.acc.y  = kalmanAccY.update(rawAccY);
    kimuData.acc.z  = kalmanAccZ.update(rawAccZ);

    kimuData.gyro.x = kalmanGyrX.update(rawGyrX);
    kimuData.gyro.y = kalmanGyrY.update(rawGyrY);
    kimuData.gyro.z = kalmanGyrZ.update(rawGyrZ);

    // Temperatura direta (sem filtro)
    kimuData.temp   = gImu.temp();
}
