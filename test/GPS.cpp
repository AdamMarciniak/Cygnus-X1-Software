#include "GPS.h"
#include "Data.h"

TinyGPSPlus gps;

float gpsAltitudeBias = 0.0f;
int gpsBiasCount = 0;

float initialLatitude = 0.0f;
float initialLongitude = 0.0f;

float latitudeAve = 0.0f;
float longitudeAve = 0.0f;

float earthRadius = 6371000; //meters
float cosInitLat = 0.0f;

float xDistance = 0.0f;
float yDistance = 0.0f;

float getEastWestDistance(float longitudeDeg)
{
  return earthRadius * (longitudeDeg - initialLongitude) * DEG_TO_RAD;
}

float getNorthSouthDistance(float latitudeDeg)
{
  return earthRadius * (latitudeDeg - initialLatitude) * DEG_TO_RAD;
}

void getGPSAltitudeBias()
{
  while (Serial1.available() > 0)
  {
    gps.encode(Serial1.read());

    if (gps.location.isUpdated())
    {
      data.lat = gps.location.lat();
      data.lng = gps.location.lng();
      data.sats = gps.satellites.value();
      data.hdop = gps.hdop.hdop();
      data.hdopVal = gps.hdop.value();
      data.gpsAltitude = gps.altitude.meters();
      data.gpsSpeed = gps.speed.kmph();
      data.gpsCourse = gps.course.deg();
      Serial.print("Got GPS. HDOP = ");
      Serial.print(data.hdop);
      Serial.print(" ");
      Serial.print(data.hdopVal);
      Serial.print("  Sats: ");
      Serial.println(data.sats);
      if (data.hdop < 1.8 && data.sats > 5)
      {
        if (gpsBiasCount < 10)
        {
          data.gps_altitude_bias += data.gpsAltitude;
          latitudeAve += data.lat;
          longitudeAve += data.lng;
          gpsBiasCount += 1;
        }

        if (gpsBiasCount == 10)
        {
          data.gps_altitude_bias /= 10.0;
          initialLatitude = latitudeAve / 10.0;
          initialLongitude = longitudeAve / 10.0;

          Serial.println("Finishing BIAS MEasurement");
          goToState(IDLE);
        }
      }
    }
  }
}

void handleGPS()
{

  if (data.state == GPS_BIAS_GATHER)
  {
    return;
  }

  while (Serial1.available() > 0)
  {

    gps.encode(Serial1.read());

    if (gps.location.isUpdated())
    {
      data.lat = gps.location.lat();
      data.lng = gps.location.lng();
      data.sats = gps.satellites.value();
      data.hdop = gps.hdop.hdop();
      data.hdopVal = gps.hdop.value();
      data.gpsAltitude = gps.altitude.meters();
      data.gpsSpeed = gps.speed.kmph();
      data.gpsCourse = gps.course.deg();

      // xDistance = getEastWestDistance(data.lng);
      // yDistance = getNorthSouthDistance(data.lat);
      // Serial.print(initialLatitude);
      // Serial.print(" ");
      // Serial.print(initialLongitude);
      // Serial.print(" ");
      // Serial.print("LAT: ");
      // Serial.print(data.lat, 12);
      // Serial.print(" LNG: ");
      // Serial.print(data.lng, 12);
      // Serial.print("  X GPS:  ");
      // Serial.print(xDistance);
      // Serial.print(" Y GPS: ");
      // Serial.println(yDistance);
    }
  }
}