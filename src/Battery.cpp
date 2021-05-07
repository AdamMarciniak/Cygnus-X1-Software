#include "Battery.h"

Chrono batteryCheckTimer;

float getBattVoltage()
{
  analogReadResolution(12);

  int reading = analogRead(BATT_DETECT_PIN);
  float outVolts = reading * 3.3f / 4095.0f;
  return outVolts * (R1 + R2) / R2;
}

void handleBatteryCheck()
{

  if (batteryCheckTimer.hasPassed(50))
  {
    data.batteryVoltage = getBattVoltage();
    batteryCheckTimer.restart();
  }
}

bool isBatteryLow()
{
  if (getBattVoltage() <= BATTERY_VOLTAGE_MIN)
  {
    return true;
  }
  else
  {
    return false;
  }
}
