#include <Arduino.h>
#include "GPS.h"
#include "TinyGPS++.h"
#include "Data.h"
#include "Chrono.h"

Chrono gpsTimer;

TinyGPSPlus gps;

bool gpsReady = false;

void initGPS()
{
  Serial1.begin(9600);
}

void handleGPS()
{
  if (gpsTimer.hasPassed(500))
  {
    while (Serial1.available() > 0)
    {
      gps.encode(Serial1.read());
      if (gps.location.isUpdated())
      {
        gpsReady = true;
        data.lat = gps.location.lat();
        data.lng = gps.location.lng();
        data.gpsAltitude = gps.altitude.meters();
        data.hdop = gps.hdop.hdop();
        data.sats = gps.satellites.value();
      }
    }
  }
}