#ifndef BATTERY_H
#define BATTERY_H

// Resistor Divider values
#define R1 150000
#define R2 47000
// For flying, set threshold to 11.1 volts (medium)
#define LOW_VOLT_THRESHOLD 10.5f

extern float battVoltage;
extern float getBattVoltage();
extern bool isBatteryLow();

#endif