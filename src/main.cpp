#include <Arduino.h>
#include "Chrono.h"
#include "Altimeter.h"

#define numPoints 1000

Chrono altimeterTimer;

float altitudes[numPoints];

unsigned long i = 0;
int measState = 0;
float mean = 0;
float sum = 0;
float valSum = 0;
float val = 0;
float variance = 0;
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  initAltimeter();
  delay(1000);

  while (i < numPoints)
  {

    handleAltimeter();
    if (isNewAltimeterData())
    {
      altitudes[i] = getAltitude();
      sum += altitudes[i];
      i += 1;
      delay(5);
    }
  }

  mean = sum / numPoints;

  for (int k = 0; k < numPoints; k += 1)
  {
    val = sq(altitudes[k] - mean);
    valSum += val;
  }

  variance = valSum / numPoints;

  Serial.print(" Variance: ");
  Serial.println(variance, 8);
}

void loop()
{
}