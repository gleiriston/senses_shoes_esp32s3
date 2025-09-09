#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "system_constants.hpp"

// ====== Modo de envio ======
enum SendMode { smContinuous = 0, smSample = 1 };

// ====== Estado do dispositivo (comandos BLE alteram isso) ======
struct DeviceState {
  bool      sync        = false;                      // CS_ON/CS_OFF
  bool      active      = false;                      // enviar ou não
  uint32_t  delayLoop   = kDEVICE_DELAY_LOOP_NORMAL_MS;
  SendMode  sendMode    = smContinuous;               // PM_CONT / PM_SAMPLE
  uint32_t  sampleCounter = 0;                        // decrementa no smSample
};

// ====== Globais expostas (antes estavam no main) ======
extern DeviceState device;    // estado único
extern bool imuActive;        // controle do envio do IMU

extern float     g_streamHz;      // Hz atuais do stream
extern uint32_t  g_intervalUs;    // período em µs
extern uint32_t  g_lastSampleUs;  // timestamp micros() do último pacote

extern uint16_t  g_convWaitUs;    // espera de conversão do INA (µs)
extern uint16_t  g_muxSettleUs;   // settle do MUX (µs)
extern bool      g_fastActive;    // perfil FAST/TURBO vs NORMAL
extern bool      g_turboMode;     // se está no modo TURBO (alvo ~100Hz)

// ====== Config pendente do INA (aplicada com segurança no loop principal) ======
enum PendingInaCfg { CFG_NONE=0, CFG_FAST=1, CFG_NORMAL=2 };
extern volatile uint8_t g_pendingInaCfg;

// ====== Helpers I2C/INA (agora centralizados aqui) ======
bool i2cWrite16(uint8_t addr, uint8_t reg, uint16_t val);
bool i2cRead16 (uint8_t addr, uint8_t reg, uint16_t &out);

// Aplica um valor ao REG_CONFIG e confirma leitura
bool inaApplyConfig(uint16_t cfg);

// Perfis prontos (ajustam REG_CONFIG e tempos de espera)
void setProfileFAST60();   // conv≈220us / settle≈120us
void setProfileTURBO100(); // conv≈190us / settle≈40us (agressivo)
void setProfileNORMAL();   // conv≈1200us / settle≈200us

// Atualiza taxa e período do stream
void updateIntervalFromHz(float hz);
void updateIntervalFromMs(uint32_t ms);
void bleHandleCommand(const String& s);
