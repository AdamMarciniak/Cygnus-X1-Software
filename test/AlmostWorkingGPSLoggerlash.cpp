#include <Arduino.h>
#include <SerialFlash.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include "Chrono.h"

#define PERPAGE 8

const int FlashChipSelect = SS_FLASH;

Chrono saveDataTimer;

float lng, sats, altitude, hdopVal, speed;
float lat = 111.111;
float course = 123.456;
float hdop = 999.999;

struct gpsData
{
  float lat, lng, sats, hdop, altitude, hdopVal, speed, course;
} page[PERPAGE];

TinyGPSPlus gps;

// Fill up page array. If below 16 pages, keep filling. Else write the 16 pages to flash.
//Keeps track of address so it writes to next one.
int pageIdx = 0;
uint16_t pages_written = 0;
const int maxTimeMins = 1;
const int maxTimeSec = 8;
const int loggingRateMS = 1000;
const uint16_t totalDatapoints = (1000 / loggingRateMS) * maxTimeSec;

uint16_t currentDatapoint = 0;
uint16_t pointsPerPage = PERPAGE;
int pointInPage = 0;

void write_data()
{
  Serial.println("Writing Page To Flash");
  Serial.println(page[0].lat);
  Serial.println(page[0].course);

  SerialFlash.write(pages_written * 256, &page, sizeof(page));
  pages_written++;
}

void read_flash()
{
  Serial.println("Reading Flash");
  for (int i = 0; i < totalDatapoints / pointsPerPage; i += 1)
  {
    Serial.println("Reading Page");
    SerialFlash.read(i * 256, &page, sizeof(page));

    for (int k = 0; k < pointsPerPage; k += 1)
    {
      Serial.println(page[k].hdop, 6);
      Serial.println(page[k].lat, 6);
    }
  }
}

void saveData()
{
  page[pointInPage].altitude = (float)altitude;
  page[pointInPage].lat = (float)lat;
  page[pointInPage].lng = (float)lng;
  page[pointInPage].speed = (float)speed;
  page[pointInPage].hdop = (float)hdop;
  page[pointInPage].hdopVal = (float)hdopVal;
  page[pointInPage].sats = (float)sats;
  page[pointInPage].course = (float)course;
}

void saveToSDCard(){

};

SerialFlashFile file;
uint32_t capacity;
void setup()
{
  uint8_t id[3];

  Serial.begin(115200);
  Serial1.begin(9600);

  while (!Serial)
    ;
  SerialFlash.begin(SPI2, SS_FLASH);
  delay(3);
  SerialFlash.readID(id);
  Serial.printf("ID %0x %0x %0x\n", id[0], id[1], id[2]);
  capacity = SerialFlash.capacity(id);
  Serial.printf("capacity %d \n", capacity);

  SerialFlash.eraseAll();
  Serial.println("Erasing");
  while (!SerialFlash.ready())
  {
    // wait, 30 seconds to 2 minutes for most chips
    Serial.println("Erasing...");
  }
  delay(1000);
  Serial.println("Logging Data..");
}

void loop()
{

  // while (Serial1.available() > 0)
  // {
  //   gps.encode(Serial1.read());

  //   if (gps.location.isUpdated())
  //   {
  //     lat = gps.location.lat();
  //     lng = gps.location.lng();
  //     sats = gps.satellites.value();
  //     hdop = gps.hdop.hdop();
  //     hdopVal = gps.hdop.value();
  //     altitude = gps.altitude.meters();
  //     speed = gps.speed.kmph();
  //     course = gps.course.deg();
  //   }
  // }

  if (pointInPage == pointsPerPage)
  {
    pointInPage = 0;
    write_data();
  }

  if (currentDatapoint < totalDatapoints)
  {

    if (saveDataTimer.hasPassed(loggingRateMS))
    {

      Serial.print("Datapoint: ");
      Serial.print(currentDatapoint);
      Serial.print(" / ");
      Serial.print(totalDatapoints);
      Serial.print("  PointInPage");
      Serial.print(pointInPage);
      Serial.print(" / ");
      Serial.println(pointsPerPage);

      if (pointInPage < pointsPerPage)
      {
        saveData();
        pointInPage += 1;
      }

      currentDatapoint += 1;

      saveDataTimer.restart();
    }
  }
  else
  {

    read_flash();
    while (1)
      ;
    saveToSDCard();
  }
}
