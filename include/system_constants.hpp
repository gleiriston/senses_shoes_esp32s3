#pragma once
#include <stdint.h>

/* ============== PINAGEM / HARDWARE ============== */
static const int INA_SDA_PIN = 13;
static const int INA_SCL_PIN = 14;
static const uint32_t I2C_SPEED = 400000; // 400 kHz (limite INA226)

static const int ASW_S0 = 9;  // D0
static const int ASW_S1 = 8;  // D1
static const int ASW_S2 = 12; // D2
static const int ASW_S3 = 11; // D3
static const int ASW_EN = 10; // EN (LOW = ativo)
static const bool ASW_EN_ACTIVE_LOW = true;

static const int LED_PIN = 17;
static const uint32_t LED_BLINK_MS_DISCONNECTED = 500;
static const uint32_t LED_BLINK_MS_CONNECTED = 120;

/* ============== PERFIS / AMOSTRAGEM ============== */
static const uint32_t kDEVICE_DELAY_LOOP_FAST_MS = 17;    // ~60 Hz alvo
static const uint32_t kDEVICE_DELAY_LOOP_NORMAL_MS = 100; // 10 Hz
static const uint32_t kDEVICE_DELAY_LOOP_LOW_MS = 500;    // 2 Hz
static const uint32_t kDEVICE_SAMPLE_TEST = 80;           // PM_SAMPLE

/* ============== BLE UUIDs / NOME ============== */
static const char *DEVICE_NAME = "SenseShoes-INA-S3";
static const char *NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

/* ============== INA226 ============== */
static const uint8_t INA226_ADDR = 0x40;
static const uint8_t REG_CONFIG = 0x00;
static const uint8_t REG_BUS_V = 0x02;
static const uint16_t INA226_CONFIG_NORMAL = 0x4127; // AVG=16, tempos médios
static const uint16_t INA226_CONFIG_FAST = 0x0007;   // AVG=1, tempos mínimos

// --- IMU (SPI) ---
static const int IMU_SPI_SCK = 36;
static const int IMU_SPI_MOSI = 35;
static const int IMU_SPI_MISO = 37;
static const int IMU_SPI_CS = 38;

static const int IMU_INT1_PIN = 4;
static const int IMU_INT2_PIN = 5;

/* ============== OUTROS ============== */
static const char SEP_CHAR = '~';
static const float DEFAULT_STREAM_HZ = 60.0f; // padrão
static const float DEAD_BAND_MV_FAST = 5.0f;  // ±5 mV -> 0 (FAST)

// I2C da bateria (MAX17048) no Wire1
static constexpr int MB_SDA_PIN = 47;
static constexpr int MB_SCL_PIN = 48;

// Endereço e registradores do MAX17048
static constexpr uint8_t MAX17048_I2C_ADDR   = 0x36;
static constexpr uint8_t MAX17048_REG_VCELL  = 0x02;
static constexpr uint8_t MAX17048_REG_SOC    = 0x04;
static constexpr uint8_t MAX17048_REG_MODE   = 0x06;
static constexpr uint8_t MAX17048_REG_VERSION= 0x08;

// Conversões (datasheet)
static constexpr float MAX17048_VCELL_V_PER_LSB = (1.25e-3f / 16.0f); // ~78.125 µV/LSB
static constexpr float MAX17048_SOC_PCT_PER_LSB = 3.90625e-3f;       // ~0.00390625 %/LSB

