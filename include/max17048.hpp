#pragma once
#include <Arduino.h>
#include <Wire.h>

// Inicializa o MAX17048 no Wire1 (MB_SDA_PIN/MB_SCL_PIN). Retorna true se OK.
bool max17048Begin();

// Presença (ACK no 0x36)
bool max17048IsPresent();

// Leituras
bool max17048ReadVCell(float &volts);
bool max17048ReadSOC(float &pct);

// QuickStart (opcional; recomeça o algoritmo de fuel gauge)
bool max17048QuickStart();

// Scan do barramento Wire1 (debug)
void max17048Scan();

// Controle de streaming (para o task)
void max17048SetStreaming(bool on);
bool max17048GetStreaming();
