
#include <SPI.h>
#include "SdFat.h"
#include "Chrono.h"
#include "TinyGPS++.h"
#include "LED.h"
const uint8_t chipSelect = SS_SD;

//File Base name. 6 chars or less.
#define FILE_BASE_NAME "GPSDAT"
//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;

Chrono myTimer;
TinyGPSPlus tinyGPS;

float lat, lng, sats, hdop, alt;

const int interval = 1000;
const int numReadings = 15 * 60;
int readingCount = 0;

void writeHeader()
{
  file.print(F("lat"));
  file.print(F(",lng"));
  file.print(F(",sats"));
  file.print(F(",hdop"));
  file.print(F(",alt"));
  file.println();
}
//------------------------------------------------------------------------------
// Log a data record.
void logData()
{
  turnOnLED();
  delay(250);
  turnOffLED();
  file.print(lat, 10);
  file.print(",");
  file.print(lng, 10);
  file.print(",");
  file.print(sats, 10);
  file.print(",");
  file.print(hdop, 10);
  file.print(",");
  file.print(alt, 10);
  file.println();
}

void getGPSData()
{
  while (Serial1.available() > 0)
  {
    tinyGPS.encode(Serial1.read());

    if (tinyGPS.location.isUpdated())
    {

      lat = tinyGPS.location.lat();
      lng = tinyGPS.location.lng();
      sats = tinyGPS.satellites.value();
      hdop = tinyGPS.hdop.value();
      alt = tinyGPS.altitude.meters();

      Serial.print(lat, 10);
      Serial.print(" ");
      Serial.print(lng, 10);
      Serial.print(" ");
      Serial.print(sats);
      Serial.print(" ");
      Serial.print(hdop);
      Serial.print(" ");
      Serial.println(alt);
    };
  }
}

//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))
//------------------------------------------------------------------------------
void setup()
{
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";

  Serial.begin(115200);
  delay(1000);
  Serial1.begin(9600);
  delay(1000);

  initLED();
  turnOnLED();
  delay(1000);
  turnOffLED();

  // Wait for USB Serial
  // while (!Serial)
  // {
  //   SysCall::yield();
  // }
  delay(1000);

  // Serial.println(F("Type any character to start"));
  // while (!Serial.available())
  // {
  //   SysCall::yield();
  // }

  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50)))
  {
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
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL))
  {
    error("file.open");
  }
  // Read any Serial data.
  // do
  // {
  //   delay(10);
  // } while (Serial.available() && Serial.read() >= 0);

  Serial.print(F("Logging to: "));
  Serial.println(fileName);
  // Serial.println(F("Type any character to stop"));

  // Write data header.
  writeHeader();

  // Start on a multiple of the sample interval.
}
//------------------------------------------------------------------------------
void loop()
{

  if (myTimer.hasPassed(interval) && readingCount < numReadings)
  {
    getGPSData();
    logData();
    myTimer.restart();
    readingCount += 1;
  }

  if (readingCount >= numReadings)
  {
    file.close();
    Serial.println(F("Done"));
    while (1)
    {
      setLEDRed(10);
      delay(50);
      turnOffLED();
      delay(2000);
    }
  }

  // Force data to SD and update the directory entry to avoid data loss.
  if (!file.sync() || file.getWriteError())
  {
    error("write error");
  }

  // if (Serial.available())
  // {
  //   // Close file and stop.
  //   file.close();
  //   Serial.println(F("Done"));
  //   SysCall::halt();
  // }
}