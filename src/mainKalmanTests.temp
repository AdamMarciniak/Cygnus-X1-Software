#include <Arduino.h>
#include "Data.h"
#include "Nav.h"
#include "Chrono.h"
#include "Data.h"
#include "Buzzer.h"
#include "Altimeter.h"
#include "Kalman.h"
#include "EUIMyLib.h"

Chrono altimeterTimer;
Chrono accelTimer;
Chrono kalmanTimer;

void setup()
{
  Serial.begin(115200);
  initEUI();
  initBuzzer();
  buzzStartup();
  initFlash();
  initIMU();
  delay(10);
  initAltimeter();
  delay(10);

  buzzStartup();
}

void loop()
{

  handleEUI();
  handleAltimeter();
  if (kalmanTimer.hasPassed(5))
  {
    data.kal_X = getKalmanPosition();
    data.kal_V = getKalmanVelocity();
    data.kal_A = getKalmanAcceleration();
    kalmanTimer.restart();
  }

  if (accelTimer.hasPassed(5))
  {
    getYPR();
    updateAccel(data.worldAx);
    accelTimer.restart();
  }

  if (isNewAltimeterData())
  {
    updateBaro(getAltitude());
  }
}