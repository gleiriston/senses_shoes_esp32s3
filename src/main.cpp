#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "max17048.hpp"
#include "system_constants.hpp"
#include "device.hpp"        
#include "ble_handler.hpp"   
#include "kalman_filter.hpp"  
#include "imu.hpp"           
#include "status.hpp"         

 
static volatile uint32_t s_txWin = 0;
static uint64_t         s_txTot = 0;
static uint32_t         s_metricWinStartMs = 0;
static uint32_t         s_lastMetricEmitMs = 0;

 
static bool  s_kfEnabled = true;
static const float K_EMA_ALPHA   = 0.20f;  
static const int   K_QUANT_STEP  = 5;     
static const int   K_HYST_MV     = 8;    

// ================== MUX (mapa validado) ==================
static inline void muxEnable(bool en){
  if (ASW_EN_ACTIVE_LOW) digitalWrite(ASW_EN, en?LOW:HIGH);
  else                   digitalWrite(ASW_EN, en?HIGH:LOW);
}
static inline void muxSelectRaw(uint8_t ch){
  digitalWrite(ASW_S0, (ch>>0)&1);
  digitalWrite(ASW_S1, (ch>>1)&1);
  digitalWrite(ASW_S2, (ch>>2)&1);
  digitalWrite(ASW_S3, (ch>>3)&1);
}
static uint8_t MUX_MAP[16] = { 0,4,8,12, 1,5,9,13, 15,3,7,11, 14,10,6,2 };
static inline void muxSelect(uint8_t logicalCh){
  muxEnable(false);
  muxSelectRaw(MUX_MAP[logicalCh]);
  delayMicroseconds(10);
  muxEnable(true);
}

// ================== CONVERSÃO INA ==================
static inline int32_t inaBus_mV_fromRaw_i(uint16_t raw){
  return (((int32_t)raw * 5) + 2) / 4; // LSB=1.25mV -> mV inteiro
}

// ================== FILTROS DOS 16 CANAIS ==================
struct Med3 {
  float v[3] = {NAN,NAN,NAN};
  uint8_t i=0, filled=0;
  inline float push(float x){
    v[i]=x; i=(i+1)%3; if(filled<3) filled++;
    if(filled<3) return x;
    float a=v[0],b=v[1],c=v[2];
    return max(min(a,b), min(max(a,b), c));
  }
};
static Med3          s_med[16];
static KalmanFilter  s_kf[16] = {
  // Q, R, P0, x0
  KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0),
  KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0),
  KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0),
  KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0), KalmanFilter(4,900,1,0)
};
static float         s_ema[16]       = {0};
static bool          s_emaInit[16]   = {0};
static int32_t       s_lastMv[16]    = {0};
static bool          s_lastMvInit[16]= {0};

// ================== TAREFAS ==================
static void TaskInsoleTx(void*) {
  // seleção inicial segura
  muxSelect(0);

  TickType_t last = xTaskGetTickCount();

  for(;;){
    // Aplica config pendente do INA aqui
    uint8_t p = g_pendingInaCfg;
    if (p != CFG_NONE){
      if (p == CFG_FAST){
        if (g_streamHz >= 95.0f) setProfileTURBO100();
        else                     setProfileFAST60();
      } else {
        setProfileNORMAL();
      }
      g_pendingInaCfg = CFG_NONE;
    }

    const bool ready = (device.active && g_connected && g_txSubs > 0 && g_streamEnabled);
    if (ready){
      const uint16_t MUX_SETTLE_US = g_muxSettleUs;
      const uint16_t CONV_WAIT_US  = g_convWaitUs;

      char out[420];   // INA + IMU
      int idx = 0;

      // ===== INA x16 =====
      for (uint8_t ch = 0; ch < 16; ++ch){
        muxSelect(ch);
        delayMicroseconds(MUX_SETTLE_US);

        delayMicroseconds(CONV_WAIT_US);
        uint16_t raw = 0;
        bool ok = i2cRead16(INA226_ADDR, REG_BUS_V, raw);
        int32_t mv_i = ok ? inaBus_mV_fromRaw_i(raw) : -1;

        if (ok && mv_i >= 0){
          float mv = (float)mv_i;

          
          if (g_fastActive){
            mv = s_med[ch].push(mv);
            if (fabsf(mv) < DEAD_BAND_MV_FAST) mv = 0.0f;
          }

          
          if (s_kfEnabled) mv = s_kf[ch].update(mv);

           
          if (!s_emaInit[ch]) { s_ema[ch] = mv; s_emaInit[ch] = true; }
          else                { s_ema[ch] = K_EMA_ALPHA*mv + (1.0f-K_EMA_ALPHA)*s_ema[ch]; }
          mv = s_ema[ch];

          
          int32_t qi = (int32_t)lroundf(mv);
          qi = (qi + K_QUANT_STEP/2) / K_QUANT_STEP * K_QUANT_STEP;

          if (!s_lastMvInit[ch]) { s_lastMv[ch] = qi; s_lastMvInit[ch] = true; }
          else {
            if (abs(qi - s_lastMv[ch]) < K_HYST_MV) qi = s_lastMv[ch];
            else                                     s_lastMv[ch] = qi;
          }

          mv_i = qi;
        }

        idx += snprintf(&out[idx], sizeof(out)-idx, "%ld", (long)mv_i);
        if (ch < 15) out[idx++] = SEP_CHAR;
        if (idx >= (int)sizeof(out) - 80) break;
      }

     
      if (imuActive) {
        readIMU();   
        if (idx < (int)sizeof(out)-2) out[idx++] = SEP_CHAR;
        idx += snprintf(&out[idx], sizeof(out)-idx,
                        "IMU:%.2f~%.2f~%.2f~%.2f~%.2f~%.2f~%.2f",
                        kimuData.acc.x,  kimuData.acc.y,  kimuData.acc.z,
                        kimuData.gyro.x, kimuData.gyro.y, kimuData.gyro.z,
                        kimuData.temp);
      } else {
        if (idx < (int)sizeof(out)-2) out[idx++] = SEP_CHAR;
        idx += snprintf(&out[idx], sizeof(out)-idx, "IMU:OFF");
      }

      
      out[idx++] = '\r';
      out[idx++] = '\n';
      nusSend(out, idx);
      Serial.write(out, idx);

     
      s_txWin++; s_txTot++;
      const uint32_t nowMs = millis();
      if ((uint32_t)(nowMs - s_lastMetricEmitMs) >= 1000){
        uint32_t winMs = nowMs - s_metricWinStartMs; if (winMs==0) winMs=1;
        float r = (s_txWin * 1000.0f) / winMs;
        Serial.printf("[TX INA] rate: %.2f pkt/s  (win=%lums, total=%llu)\n",
                      r, (unsigned long)winMs, (unsigned long long)s_txTot);
        s_txWin = 0; s_metricWinStartMs = nowMs; s_lastMetricEmitMs = nowMs;
      }
    }

    
    float hz = g_streamHz; if (hz < 1.0f) hz = 1.0f; if (hz > 150.0f) hz = 150.0f;
    uint32_t periodMs = (uint32_t)lroundf(1000.0f / hz);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(periodMs));
  }
}

 
static void TaskBatteryTx(void*) {
  const uint32_t periodMs = 1000;  
  TickType_t last = xTaskGetTickCount();

  for(;;){
    const bool ready = (g_connected && g_txSubs > 0 && g_streamEnabled && max17048IsPresent());
    if (ready) {
      float v=0, soc=0;
      bool okV = max17048ReadVCell(v);
      bool okS = max17048ReadSOC(soc);

      if (okV && okS) {
        char out[64];
        int n = snprintf(out, sizeof(out), "BAT:%.3f~%.1f\r\n", v, soc);
        nusSend(out, n);
        Serial.printf("[BAT] V=%.3f V  SOC=%.1f%%\n", v, soc);
      } else {
        nusSend("BAT:ERR\r\n", 9);
        Serial.println("[BAT] ERRO de leitura");
      }
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(periodMs));
  }
}

// ================== SETUP ==================
void setup(){
  pinMode(ASW_S0, OUTPUT); pinMode(ASW_S1, OUTPUT);
  pinMode(ASW_S2, OUTPUT); pinMode(ASW_S3, OUTPUT);
  pinMode(ASW_EN, OUTPUT);
  muxEnable(true);

  Serial.begin(115200);
  Serial.println("\nESP32-S3 | Tasks: Palmilha(INA) + IMU + BAT (NimBLE NUS)");

   
  Wire.begin(INA_SDA_PIN, INA_SCL_PIN);
  Wire.setClock(I2C_SPEED);
 
  setProfileFAST60();
  updateIntervalFromHz(DEFAULT_STREAM_HZ);
  g_lastSampleUs = micros();

 
  bleInit();

  
  imuInit();
  imuActive = false;

   
  if (max17048Begin()) {
    Serial.println("[BAT] MAX17048 inicializado.");
  } else {
    Serial.println("[BAT] MAX17048 AUSENTE (vou re-tentar periodicamente).");
  }

  
  s_metricWinStartMs = millis();
  s_lastMetricEmitMs = s_metricWinStartMs;

   
  xTaskCreatePinnedToCore(
    TaskInsoleTx, "tx_ina", 6144, nullptr, 2,
    nullptr, ARDUINO_RUNNING_CORE
  );
  xTaskCreatePinnedToCore(
    TaskBatteryTx, "tx_bat", 3072, nullptr, 1,
    nullptr, ARDUINO_RUNNING_CORE
  );

  
  statusInit();
}

// ================== LOOP ==================
void loop(){
 
}
