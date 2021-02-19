#include <Arduino.h>
//#include <SerialFlash.h>
#include "SPI.h"
#include <TinyGPS++.h>
#include "Chrono.h"
#include "SdFat.h"
#include "LED.h"

#define PERPAGE 8

//const int FlashChipSelect = SS_FLASH;
const uint8_t chipSelect = SS_SD;

const uint16_t totalSDPoints = 7200;
uint16_t currentSDPoint = 0;

#define FILE_BASE_NAME "Data"
//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;
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

// Write data header.
void writeHeader()
{
  file.print(F("lat"));
  file.print(F(",lng"));
  file.print(F(",altitude"));
  file.print(F(",hdop"));
  file.print(F(",hdopVal"));
  file.print(F(",course"));
  file.print(F(",speed"));
  file.print(F(",sats"));
  file.println();
}
//------------------------------------------------------------------------------
// Log a data record.
void logData()
{
  // Write data to file.  Start with log time in micros.
  file.print(lat);
  file.write(',');
  file.print(lng);
  file.write(',');
  file.print(altitude);
  file.write(',');
  file.print(hdop);
  file.write(',');
  file.print(hdopVal);
  file.write(',');
  file.print(course);
  file.write(',');
  file.print(speed);
  file.write(',');
  file.print(sats);
  file.println();
}

// void write_data()
// {
//   Serial.println("Writing Page To Flash");
//   Serial.println(page[0].lat);
//   Serial.println(page[0].course);

//   SerialFlash.write(pages_written * 256, &page, sizeof(page));
//   pages_written++;
// }

// void read_flash()
// {
//   Serial.println("Reading Flash");
//   for (int i = 0; i < totalDatapoints / pointsPerPage; i += 1)
//   {
//     Serial.println("Reading Page");
//     SerialFlash.read(i * 256, &page, sizeof(page));

//     for (int k = 0; k < pointsPerPage; k += 1)
//     {
//       Serial.println(page[k].hdop, 6);
//       Serial.println(page[k].lat, 6);
//     }
//   }
// }
#define error(msg) sd.errorHalt(F(msg))

// void saveData()
// {
//   page[pointInPage].altitude = (float)altitude;
//   page[pointInPage].lat = (float)lat;
//   page[pointInPage].lng = (float)lng;
//   page[pointInPage].speed = (float)speed;
//   page[pointInPage].hdop = (float)hdop;
//   page[pointInPage].hdopVal = (float)hdopVal;
//   page[pointInPage].sats = (float)sats;
//   page[pointInPage].course = (float)course;
// }

// void saveToSDCard(){

// };

//SerialFlashFile file;
uint32_t capacity;
void setup()
{

  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";

  Serial.begin(115200);
  Serial1.begin(9600);

  initLED();

  delay(5000);
  // SerialFlash.begin(SPI2, SS_FLASH);
  // delay(3);
  // SerialFlash.readID(id);
  // Serial.printf("ID %0x %0x %0x\n", id[0], id[1], id[2]);
  // capacity = SerialFlash.capacity(id);
  // Serial.printf("capacity %d \n", capacity);

  // SerialFlash.eraseAll();
  // Serial.println("Erasing");
  // while (!SerialFlash.ready())
  // {
  //   // wait, 30 seconds to 2 minutes for most chips
  //   Serial.println("Erasing...");
  // }

  if (!sd.begin(SS_SD, SD_SCK_MHZ(50)))
  {
    while (1)
    {
      handleLEDBlink();
    }
    sd.initErrorHalt();
  }

  // Find an unused file name.
  if (BASE_NAME_SIZE > 6)
  {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName))
  {
    if (fileName[BASE_NAME_SIZE + 1] != '9')
    {
      fileName[BASE_NAME_SIZE + 1]++;
    }
    else if (fileName[BASE_NAME_SIZE] != '9')
    {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    }
    else
    {
      while (1)
      {
        handleLEDBlink();
      }
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL))
  {
    while (1)
    {
      handleLEDBlink();
    }
    error("file.open");
  }
  // Read any Serial data.

  // Write data header.

  delay(500);
  turnOnLED();
  delay(1000);
  turnOffLED();
  delay(1000);
  writeHeader();

  delay(1000);
  Serial.println("Logging Data..");
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
    }
  }

  if (saveDataTimer.hasPassed(1000))
  {
    logData();
    currentSDPoint += 1;
    saveDataTimer.restart();
  }

  if (!file.sync() || file.getWriteError())
  {
    while (1)
    {
      handleLEDBlink();
    }
    error("write error");
  }

  if (currentSDPoint >= totalSDPoints)
  {
    file.close();
    SysCall::halt();
  }

  // if (pointInPage == pointsPerPage)
  // {
  //   pointInPage = 0;
  //   write_data();
  // }

  // if (currentDatapoint < totalDatapoints)
  // {

  //   if (saveDataTimer.hasPassed(loggingRateMS))
  //   {

  //     Serial.print("Datapoint: ");
  //     Serial.print(currentDatapoint);
  //     Serial.print(" / ");
  //     Serial.print(totalDatapoints);
  //     Serial.print("  PointInPage");
  //     Serial.print(pointInPage);
  //     Serial.print(" / ");
  //     Serial.println(pointsPerPage);

  //     if (pointInPage < pointsPerPage)
  //     {
  //       saveData();
  //       pointInPage += 1;
  //     }

  //     currentDatapoint += 1;

  //     saveDataTimer.restart();
  //   }
  // }
  // else
  // {

  //   read_flash();
  //   while (1)
  //     ;
  //   saveToSDCard();
  // }
}
