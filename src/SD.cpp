
#include <Arduino.h>
#include "SPI.h"
#include "SdFat.h"
#include "Data.h"
#include "Buzzer.h"

// Base name must be 6 or less chars
#define FILE_BASE_NAME "Data"
const uint8_t chipSelect = SS_SD;
const int numDecimals = 8;

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13] = FILE_BASE_NAME "00.csv";
//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;

// Write data header.
void writeHeader()
{
  file.print(F("lat"));
  file.print(F(",lng"));
  file.print(F(",altitude"));
  file.print(F(",gpsAltitude"));
  file.print(F(",hdop"));
  file.print(F(",sats"));
  file.print(F(",ax"));
  file.print(F(",ay"));
  file.print(F(",az"));
  file.print(F(",yaw"));
  file.print(F(",pitch"));
  file.print(F(",roll"));
  file.print(F(",zServo"));
  file.print(F(",KP"));
  file.print(F(",KI"));
  file.print(F(",KD"));
  file.print(F(",yServo"));
  file.println();
}
//------------------------------------------------------------------------------
// Log a data record.
void logData()
{
  // Write data to file.  Start with log time in micros.
  file.print(data.lat, numDecimals);
  file.write(',');
  file.print(data.lng, numDecimals);
  file.write(',');
  file.print(data.altitude, numDecimals);
  file.write(',');
  file.print(data.gpsAltitude, numDecimals);
  file.write(',');
  file.print(data.hdop, numDecimals);
  file.write(',');
  file.print(data.sats, numDecimals);
  file.write(',');
  file.print(data.ax, numDecimals);
  file.write(',');
  file.print(data.ay, numDecimals);
  file.write(',');
  file.print(data.az, numDecimals);
  file.write(',');
  file.print(data.yaw, numDecimals);
  file.write(',');
  file.print(data.pitch, numDecimals);
  file.write(',');
  file.print(data.roll, numDecimals);
  file.write(',');
  file.print(data.servo_z, numDecimals);
  file.write(',');
  file.print(data.kp, numDecimals);
  file.write(',');
  file.print(data.ki, numDecimals);
  file.write(',');
  file.print(data.kd, numDecimals);
  file.write(',');
  file.print(data.servo_y, numDecimals);
  file.println();
}

int initSD()
{

  if (!sd.begin(SS_SD, SD_SCK_MHZ(50)))
  {
    return 0;
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
      return 0;
    }
  }
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL))
  {
    return 0;
  }
  Serial.println("Wrote CSV header");
  writeHeader();
  return 1;
}

int transferToSD()
{

  while (!initSD())
  {
    buzzerError();
    delay(1000);
  }
  while (readFromFlash())
  {
    logData();
  }
  file.close();
  return 1;
};
