#pragma once

#include "Config.h"

#include <Arduino.h>

// Resistor Divider values
#define R1 150000
#define R2 47000
// For flying, set threshold to 11.1 volts (medium)


extern float battVoltage;
extern float getBattVoltage();
extern bool isBatteryLow();

