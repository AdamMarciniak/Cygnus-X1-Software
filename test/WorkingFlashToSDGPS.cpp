
#include <Arduino.h>
#include "SPI.h"
#include <TinyGPS++.h>
#include "Chrono.h"
#include "SdFat.h"
#include "LED.h"
#include "Data.h"
#include <Tone.h>

//const int FlashChipSelect = SS_FLASH;
const uint8_t chipSelect = SS_SD;

#define FILE_BASE_NAME "Data"

#define error(msg) sd.errorHalt(F(msg))

//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;
Chrono saveDataTimer;

TinyGPSPlus gps;

// Write data header.
void writeHeader()
{
  file.print(F("lat"));
  file.print(F(",lng"));
  file.print(F(",altitude"));
  file.print(F(",hdop"));
  file.print(F(",sats"));
  file.print(F(",loop"));
  file.println();
}
//------------------------------------------------------------------------------
// Log a data record.
void logData()
{
  // Write data to file.  Start with log time in micros.
  file.print(gpsData.lat, 6);
  file.write(',');
  file.print(gpsData.lng, 6);
  file.write(',');
  file.print(gpsData.altitude, 6);
  file.write(',');
  file.print(gpsData.hdop, 6);
  file.write(',');
  file.print(gpsData.sats, 6);
  file.write(',');
  file.print(gpsData.loop, 6);
  file.println();
}

void initSD()
{

  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";
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
}

//SerialFlashFile file;
uint32_t capacity;
void setup()
{

  Serial.begin(115200);
  Serial1.begin(9600);

  while (!Serial)
    ;
  delay(1000);

  initLED();

  initFlash();

  delay(100);

  initSD();

  delay(500);
  turnOnLED();
  delay(1000);
  turnOffLED();
  delay(1000);
  writeHeader();

  delay(1000);
  Serial.println("Logging Data..");
}

bool gpsWorking = false;

unsigned long writeTime = 0;

void loop()
{

  while (Serial1.available() > 0)
  {
    gps.encode(Serial1.read());

    if (gps.location.isUpdated())
    {
      gpsWorking = true;
      gpsData.lat = gps.location.lat();
      gpsData.lng = gps.location.lng();
      gpsData.sats = gps.satellites.value();
      gpsData.hdop = gps.hdop.hdop();
      gpsData.altitude = gps.altitude.meters();
    }
  }

  if (gpsWorking == true)
  {
    if (gpsData.loop < totalSamples)
    {

      if (saveDataTimer.hasPassed(millisPerSample))
      {
        Serial.println("Logging");
        gpsData.loop += 1;
        //logData();
        writeTime = micros();
        writeToFlash();
        Serial.println(micros() - writeTime);
        saveDataTimer.restart();
      }
    }
    else
    {
      while (readFromFlash())
      {
        logData();
      }
      file.close();
      setLEDRed(200);
      SysCall::halt();
      while (1)
        ;
    }
  }

  if (!file.sync() || file.getWriteError())
  {
    while (1)
    {
      handleLEDBlink();
    }
    error("write error");
  }
}
