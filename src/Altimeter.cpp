#include "libraries/MS5607.h"
#include "Altimeter.h"
#include "Data.h"
float rawAltitude = 0;
float altitudeBias = 0;
float altitude = 0;
const int biasCount = 100;

MS5607 Altimeter(&rawAltitude);

bool initAltimeter()
{
  char beginState = Altimeter.begin();
  if (!beginState)
  {
    return 0;
  }
  Altimeter.setOSR(4096);

  while (rawAltitude == 0)
  {
    Altimeter.handleAltimeter();
    delay(10);
  }

  
  Serial.println();
  Serial.print("Init Raw Altitude: ");
  Serial.println(rawAltitude);
  delay(200);
  getAltitudeBias();
  delay(200);
  return true;
}

float getAltitude()
{
  Altimeter.handleAltimeter();
  data.altitude = rawAltitude - altitudeBias;
  return rawAltitude - altitudeBias;
}

// Updates altitude bias value. This is blocking. Must wait a bit..
void getAltitudeBias()
{
  Serial.println("Biasing Altimeter.");
  int count = 0;
  while (count < biasCount)
  {
    int newData = Altimeter.handleAltimeter();
    delay(10);
    if (newData == 1)
    {
      delay(10);
      altitudeBias += rawAltitude / (float)biasCount;
      Serial.print(".");
      count += 1;
    }
  }
  data.biasAltitude = altitudeBias;
  return;
}
