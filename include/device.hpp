#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "system_constants.hpp"


enum SendMode { smContinuous = 0, smSample = 1 };


struct DeviceState {
  bool      sync        = false;                      
  bool      active      = false;                     
  uint32_t  delayLoop   = kDEVICE_DELAY_LOOP_NORMAL_MS;
  SendMode  sendMode    = smContinuous;               
  uint32_t  sampleCounter = 0;                       
};


extern DeviceState device;   
extern bool imuActive;        

extern float     g_streamHz;     
extern uint32_t  g_intervalUs;    
extern uint32_t  g_lastSampleUs; 

extern uint16_t  g_convWaitUs;   
extern uint16_t  g_muxSettleUs;   
extern bool      g_fastActive;    
extern bool      g_turboMode;     


enum PendingInaCfg { CFG_NONE=0, CFG_FAST=1, CFG_NORMAL=2 };
extern volatile uint8_t g_pendingInaCfg;


bool i2cWrite16(uint8_t addr, uint8_t reg, uint16_t val);
bool i2cRead16 (uint8_t addr, uint8_t reg, uint16_t &out);
bool inaApplyConfig(uint16_t cfg);

void setProfileFAST60();  
void setProfileTURBO100(); 
void setProfileNORMAL();   
void updateIntervalFromHz(float hz);
void updateIntervalFromMs(uint32_t ms);
void bleHandleCommand(const String& s);
