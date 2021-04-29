
#include <Arduino.h>
#include "SPI.h"
#include "SdFat.h"
#include "Data.h"
#include "Buzzer.h"

// Base name must be 6 or less chars
#define FILE_BASE_NAME "throw"
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
  file.print(F("ms"));
  file.print(F(",ax"));
  file.print(F(",ay"));
  file.print(F(",az"));
  file.print(F(",worldAx"));
  file.print(F(",worldAy"));
  file.print(F(",worldAz"));
  file.print(F(",kalX"));
  file.print(F(",kalV"));
  file.print(F(",kalA"));
  file.print(F(",gx"));
  file.print(F(",gy"));
  file.print(F(",gz"));
  file.print(F(",yaw"));
  file.print(F(",pitch"));
  file.print(F(",roll"));
  file.print(F(",altitude"));
  file.print(F(",altitudeBias"));
  file.print(F(",state"));
  file.print(F(",zServo"));
  file.print(F(",yServo"));
  file.print(F(",KP_Y"));
  file.print(F(",KI_Y"));
  file.print(F(",KD_Y"));
  file.print(F(",KP_Z"));
  file.print(F(",KI_Z"));
  file.print(F(",KD_Z"));
  file.print(F(",p_err_y"));
  file.print(F(",i_err_y"));
  file.print(F(",d_err_y"));
  file.print(F(",p_err_z"));
  file.print(F(",i_err_z"));
  file.print(F(",d_err_z"));
  file.print(F(",yServoCenter"));
  file.print(F(",zServoCenter"));
  file.print(F(",batteryVoltage"));
  file.print(F(",Loop Time"));
  file.println();
}

//------------------------------------------------------------------------------
// Log a data record.
void logData()
{
  // Write data to file.  Start with log time in micros.
  file.print(data.ms, numDecimals);
  file.write(',');
  file.print(data.ax, numDecimals);
  file.write(',');
  file.print(data.ay, numDecimals);
  file.write(',');
  file.print(data.az, numDecimals);
  file.write(',');
  file.print(data.worldAx, numDecimals);
  file.write(',');
  file.print(data.worldAy, numDecimals);
  file.write(',');
  file.print(data.worldAz, numDecimals);
  file.write(',');
  file.print(data.kal_X_pos, numDecimals);
  file.write(',');
  file.print(data.kal_X_vel, numDecimals);
  file.write(',');
  file.print(data.kal_X_accel, numDecimals);
  file.write(',');
  file.print(data.gx, numDecimals);
  file.write(',');
  file.print(data.gy, numDecimals);
  file.write(',');
  file.print(data.gz, numDecimals);
  file.write(',');
  file.print(data.yaw, numDecimals);
  file.write(',');
  file.print(data.pitch, numDecimals);
  file.write(',');
  file.print(data.roll, numDecimals);
  file.write(',');
  file.print(data.altitude, numDecimals);
  file.write(',');
  file.print(data.biasAltitude, numDecimals);
  file.write(',');
  file.print(data.fState, numDecimals);
  file.write(',');
  file.print(data.servo_z, numDecimals);
  file.write(',');
  file.print(data.servo_y, numDecimals);
  file.write(',');
  file.print(data.kp_y, numDecimals);
  file.write(',');
  file.print(data.ki_y, numDecimals);
  file.write(',');
  file.print(data.kd_y, numDecimals);
  file.write(',');
  file.print(data.kp_z, numDecimals);
  file.write(',');
  file.print(data.ki_z, numDecimals);
  file.write(',');
  file.print(data.kd_z, numDecimals);
  file.write(',');
  file.print(data.p_err_y, numDecimals);
  file.write(',');
  file.print(data.i_err_y, numDecimals);
  file.write(',');
  file.print(data.d_err_y, numDecimals);
  file.write(',');
  file.print(data.p_err_z, numDecimals);
  file.write(',');
  file.print(data.i_err_z, numDecimals);
  file.write(',');
  file.print(data.d_err_z, numDecimals);
  file.write(',');
  file.print(data.Y_Servo_Center, numDecimals);
  file.write(',');
  file.print(data.Z_Servo_Center, numDecimals);
  file.write(',');
  file.print(data.batteryVoltage, numDecimals);
  file.write(',');
  file.print(data.loopTime, numDecimals);
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
    Serial.println("Logging data");
    logData();
  }
  file.close();
  return 1;
};
