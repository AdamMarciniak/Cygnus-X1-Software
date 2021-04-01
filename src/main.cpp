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
Chrono biasTimer;

int i = 0;
int numSteps = 500;
float sum = 0;
float accelBias = 0;
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
  getAccel();
  getYPR();
  buzzStartup();

  while (i < numSteps)
  {
    if (biasTimer.hasPassed(5))
    {
      getAccel();
      getYPR();
      sum += data.worldAx;
      i += 1;
      biasTimer.restart();
    }
  }

  accelBias = sum / numSteps;
  zeroKalman();
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
    getAccel();
    getYPR();
    updateAccel(data.worldAx - accelBias);
    accelTimer.restart();
  }

  if (isNewAltimeterData())
  {
    updateBaro(getAltitude());
  }
}