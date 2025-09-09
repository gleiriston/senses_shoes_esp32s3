#pragma once
#include <Arduino.h>
#include <Wire.h>


bool max17048Begin();
bool max17048IsPresent();
bool max17048ReadVCell(float &volts);
bool max17048ReadSOC(float &pct);
bool max17048QuickStart();
void max17048Scan();
void max17048SetStreaming(bool on);
bool max17048GetStreaming();
