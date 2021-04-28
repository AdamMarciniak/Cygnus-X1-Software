#include <Arduino.h>
//#include <SerialFlash.h>
#include "SPI.h"
#include <TinyGPS++.h>
#include "Chrono.h"
#include "LED.h"

#define PERPAGE 8

float lng, sats, altitude, hdopVal, speed;
float lat = 111.111;
float course = 123.456;
float hdop = 999.999;

struct gpsData
{
  float lat, lng, sats, hdop, altitude, hdopVal, speed, course;
} page[PERPAGE];

TinyGPSPlus gps;

void setup()
{

  Serial.begin(115200);
  Serial1.begin(9600);
}

void loop()
{

  while (Serial1.available() > 0)
  {
    gps.encode(Serial1.read());

    if (gps.location.isUpdated())
    {
      lat = gps.location.lat();
      lng = gps.location.lng();
      sats = gps.satellites.value();
      hdop = gps.hdop.hdop();
      hdopVal = gps.hdop.value();
      altitude = gps.altitude.meters();
      speed = gps.speed.kmph();
      course = gps.course.deg();

      Serial.print(" LAT: ");
      Serial.println(lat);
    }
  }

}
