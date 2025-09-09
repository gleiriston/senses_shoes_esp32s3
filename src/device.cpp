#include "device.hpp"
#include "system_constants.hpp"
#include "ble_handler.hpp"   
#include <Arduino.h>
#include "max17048.hpp"


 
DeviceState device;               
bool imuActive = false;            

float     g_streamHz     = DEFAULT_STREAM_HZ;
uint32_t  g_intervalUs   = 0;
uint32_t  g_lastSampleUs = 0;

uint16_t  g_convWaitUs   = 220;   
uint16_t  g_muxSettleUs  = 120;    
bool      g_fastActive   = true;   
bool      g_turboMode    = false;

volatile uint8_t g_pendingInaCfg = CFG_NONE;

// ===== I2C PRIMITIVAS =====
bool i2cWrite16(uint8_t addr, uint8_t reg, uint16_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)((val>>8)&0xFF));
  Wire.write((uint8_t)(val&0xFF));
  return (Wire.endTransmission() == 0);
}

bool i2cRead16(uint8_t addr, uint8_t reg, uint16_t &out){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(Wire.endTransmission(false) != 0) return false;
  if(Wire.requestFrom((int)addr, 2) != 2) return false;
  uint8_t msb = Wire.read(), lsb = Wire.read();
  out = ((uint16_t)msb<<8) | lsb;
  return true;
}

bool inaApplyConfig(uint16_t cfg){
  if(!i2cWrite16(INA226_ADDR, REG_CONFIG, cfg)) return false;
  uint16_t rd=0;
  if(!i2cRead16(INA226_ADDR, REG_CONFIG, rd))   return false;
  return true;
}

// ===== Perfis prontos =====
void setProfileFAST60(){
  if(inaApplyConfig(INA226_CONFIG_FAST)){
    g_fastActive  = true;
    g_turboMode   = false;
    g_convWaitUs  = 220;
    g_muxSettleUs = 120;
    Serial.println("[INA] FAST60: conv=220us, settle=120us");
  } else {
    Serial.println("[INA] ERRO FAST60");
  }
}

void setProfileTURBO100(){
  if(inaApplyConfig(INA226_CONFIG_FAST)){
    g_fastActive  = true;
    g_turboMode   = true;
    g_convWaitUs  = 190;
    g_muxSettleUs = 40;
    Serial.println("[INA] TURBO100: conv=190us, settle=40us");
  } else {
    Serial.println("[INA] ERRO TURBO100");
  }
}

void setProfileNORMAL(){
  if(inaApplyConfig(INA226_CONFIG_NORMAL)){
    g_fastActive  = false;
    g_turboMode   = false;
    g_convWaitUs  = 1200;
    g_muxSettleUs = 200;
    Serial.println("[INA] NORMAL: conv=1200us, settle=200us");
  } else {
    Serial.println("[INA] ERRO NORMAL");
  }
}

// ===== Taxa/Período =====
void updateIntervalFromHz(float hz){
  if(hz < 1.0f)   hz = 1.0f;
  if(hz > 150.0f) hz = 150.0f;
  g_streamHz   = hz;
  g_intervalUs = (uint32_t)(1000000.0f / hz);
  g_lastSampleUs = micros();
}

void updateIntervalFromMs(uint32_t ms){
  if(ms < 1) ms = 1;
  updateIntervalFromHz(1000.0f / (float)ms);
}

void bleHandleCommand(const String &s) {
  // Vida
  if (s.equalsIgnoreCase("CS_ON"))  { device.sync = true;  g_streamEnabled = true;  nusSendLit("CS:ON\r\n");  return; }
  if (s.equalsIgnoreCase("CS_OFF")) { device.sync = false; device.active = false;  g_streamEnabled = false;   nusSendLit("CS:OFF\r\n"); return; }

  // IMU
  if (s.equalsIgnoreCase("IMU_ON"))  { imuActive = true;  nusSendLit("IMU:ON\r\n");  return; }
  if (s.equalsIgnoreCase("IMU_OFF")) { imuActive = false; nusSendLit("IMU:OFF\r\n"); return; }

  // Perfis (mudam INA e taxa)
  if (s.equalsIgnoreCase("PD_LOW")) {
    device.delayLoop = kDEVICE_DELAY_LOOP_LOW_MS;
    g_pendingInaCfg  = CFG_NORMAL;
    updateIntervalFromMs(device.delayLoop);
    device.active = true; g_streamEnabled = true;
    String hz = String(g_streamHz, 2); nusSend(hz.c_str(), hz.length());
    return;
  }
  if (s.equalsIgnoreCase("PD_NORMAL")) {
    device.delayLoop = kDEVICE_DELAY_LOOP_NORMAL_MS;
    g_pendingInaCfg  = CFG_NORMAL;
    updateIntervalFromMs(device.delayLoop);
    device.active = true; g_streamEnabled = true;
    String hz = String(g_streamHz, 2); nusSend(hz.c_str(), hz.length());
    return;
  }
  if (s.equalsIgnoreCase("PD_FAST")) {
    g_pendingInaCfg = CFG_FAST;
    updateIntervalFromHz(60.0f);
    device.active = true; g_streamEnabled = true;
    nusSendLit("60.00");
    return;
  }
  if (s.equalsIgnoreCase("PD_TURBO")) {
    g_pendingInaCfg = CFG_FAST;
    updateIntervalFromHz(100.0f);
    device.active = true; g_streamEnabled = true;
    nusSendLit("100.00");
    return;
  }

  // Modo de envio
  if (s.equalsIgnoreCase("PM_CONT")) {
    device.sendMode = smContinuous;
    device.sampleCounter = 0;
    device.active = true; g_streamEnabled = true;
    nusSendLit("PM:CONT\r\n");
    return;
  }
  if (s.equalsIgnoreCase("PM_SAMPLE")) {
    device.sendMode = smSample;
    device.sampleCounter = kDEVICE_SAMPLE_TEST;
    device.active = true; g_streamEnabled = true;
    nusSendLit("PM:SAMPLE\r\n");
    return;
  }

  // Número => Hz
  float hzVal = s.toFloat();
  if (hzVal > 0.0f) {
    g_pendingInaCfg = (hzVal >= 30.0f) ? CFG_FAST : CFG_NORMAL;
    updateIntervalFromHz(hzVal);
    device.active = true; g_streamEnabled = true;
    String hz = String(g_streamHz, 2); nusSend(hz.c_str(), hz.length());
    return;
  }

  // Ajuda
  if (s.equalsIgnoreCase("HELP")) {
    nusSendLit(
      "CMDs:\r\n"
      " CS_ON / CS_OFF\r\n"
      " IMU_ON / IMU_OFF\r\n"
      " KF_ON / KF_OFF\r\n"
      " PD_LOW / PD_NORMAL / PD_FAST / PD_TURBO\r\n"
      " PM_CONT / PM_SAMPLE\r\n"
      " <numero>: altera Hz (ex: 60)\r\n");
    return;
  }
}

