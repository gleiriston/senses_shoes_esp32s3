#include <Arduino.h>
#include <Wire.h>
#include "max17048.hpp"
#include "system_constants.hpp"


static TwoWire* s_bus = &Wire1;
static bool s_present = false;
static bool s_stream  = true;

static int endTx(TwoWire* bus, bool sendStop = true) {
  return bus->endTransmission(sendStop);
}


static void i2cBusRecover(int sclPin, int sdaPin) {
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);
  pinMode(sdaPin, INPUT_PULLUP);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(5);

  for (int i = 0; i < 9; i++) {
    digitalWrite(sclPin, LOW);  delayMicroseconds(5);
    digitalWrite(sclPin, HIGH); delayMicroseconds(5);
  }

  
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sdaPin, LOW);  delayMicroseconds(5);
  digitalWrite(sclPin, HIGH); delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH); delayMicroseconds(5);

  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);
}


static bool readU16(uint8_t reg, uint16_t &out) {
  // A) repeated-start
  s_bus->beginTransmission(MAX17048_I2C_ADDR);
  s_bus->write(reg);
  int e = endTx(s_bus, false); 
  if (e == 0 && s_bus->requestFrom((int)MAX17048_I2C_ADDR, 2) == 2) {
    uint8_t msb = s_bus->read();
    uint8_t lsb = s_bus->read();
    out = ((uint16_t)msb << 8) | lsb;
    return true;
  }

  (void)endTx(s_bus, true);
  delayMicroseconds(80);

  s_bus->beginTransmission(MAX17048_I2C_ADDR);
  s_bus->write(reg);
  e = endTx(s_bus, true); 
  if (e != 0) {
    Serial.printf("[MAX17048] endTransmission(WRITE, STOP) err=%d\n", e);
    return false;
  }

  delayMicroseconds(80);
  if (s_bus->requestFrom((int)MAX17048_I2C_ADDR, 2) != 2) {
    Serial.println("[MAX17048] requestFrom(2) falhou no modo STOP.");
    return false;
  }

  uint8_t msb = s_bus->read();
  uint8_t lsb = s_bus->read();
  out = ((uint16_t)msb << 8) | lsb;
  return true;
}

static bool writeU16(uint8_t reg, uint16_t val) {
  s_bus->beginTransmission(MAX17048_I2C_ADDR);
  s_bus->write(reg);
  s_bus->write((uint8_t)((val >> 8) & 0xFF));
  s_bus->write((uint8_t)(val & 0xFF));       
  int e = endTx(s_bus, true);
  if (e != 0) {
    Serial.printf("[MAX17048] endTransmission(WRITE) err=%d\n", e);
    return false;
  }
  return true;
}

static bool pingAddr() {
  s_bus->beginTransmission(MAX17048_I2C_ADDR);
  int e = endTx(s_bus, true);
  return (e == 0);
}

// -----------------------------------------------------------------------------
// API
// -----------------------------------------------------------------------------
bool max17048Begin() {

  i2cBusRecover(MB_SCL_PIN, MB_SDA_PIN);

  
  pinMode(MB_SDA_PIN, INPUT_PULLUP);
  pinMode(MB_SCL_PIN, INPUT_PULLUP);

  
  bool ok = Wire1.begin(MB_SDA_PIN, MB_SCL_PIN, 100000);
  if (!ok) {
    Serial.println("[MAX17048] Wire1.begin() falhou.");
    s_present = false;
    return false;
  }
  Wire1.setClock(100000);
  Wire1.setTimeOut(50);
  delay(2);


  s_present = pingAddr();
  if (!s_present) {
    Serial.printf("[MAX17048] Nao respondeu em 0x%02X (verifique pinos/pull-ups).\n", MAX17048_I2C_ADDR);
    return false;  
  }

 
  uint16_t ver = 0;
  if (readU16(MAX17048_REG_VERSION, ver)) {
    Serial.printf("[MAX17048] OK em 0x%02X. VERSION=0x%04X\n",
                  MAX17048_I2C_ADDR, ver);
  } else {
    Serial.println("[MAX17048] ACK, mas falhou ler VERSION.");
   
  }

  return true;
}

bool max17048IsPresent() { return s_present; }

bool max17048ReadVCell(float &volts) {
  if (!s_present) return false;
  uint16_t raw;
  if (!readU16(MAX17048_REG_VCELL, raw)) return false;
  volts = (float)raw * MAX17048_VCELL_V_PER_LSB;
  return true;
}

bool max17048ReadSOC(float &pct) {
  if (!s_present) return false;
  uint16_t raw;
  if (!readU16(MAX17048_REG_SOC, raw)) return false;
  pct = (float)raw * MAX17048_SOC_PCT_PER_LSB;
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return true;
}

bool max17048QuickStart() {
  if (!s_present) return false;

  if (!writeU16(MAX17048_REG_MODE, 0x4000)) {
    Serial.println("[MAX17048] Falha ao enviar QuickStart.");
    return false;
  }
  delay(10);
  return true;
}

void max17048Scan() {
  Serial.println("[MAX17048] Scan no Wire1 (I2C1):");
  for (uint8_t a = 1; a < 127; ++a) {
    s_bus->beginTransmission(a);
    int e = endTx(s_bus, true);
    if (e == 0) {
      Serial.printf(" - 0x%02X%s\n", a, (a == MAX17048_I2C_ADDR ? " (MAX17048?)" : ""));
    }
    delay(2);
  }
}

void max17048SetStreaming(bool on) { s_stream = on; }
bool max17048GetStreaming()        { return s_stream; }
