
#include <Arduino.h>
#include "SPI.h"
#include "SdFat.h"
#include "Data.h"
#include "Buzzer.h"
#include "LED.h"

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
  file.print(F("Time_Ms"));
  file.print(F(",Raw-Acceleration-X"));
  file.print(F(",Raw-Acceleration-Y"));
  file.print(F(",Raw-Acceleration-Z"));
  file.print(F(",World-Ax"));
  file.print(F(",World-Ay"));
  file.print(F(",World-Az"));
  file.print(F(",Kalman-Altitude"));
  file.print(F(",Kalman-Velocity"));
  file.print(F(",Kalman-Acceleration"));
  file.print(F(",Raw-Gyro-X"));
  file.print(F(",Raw-Gyro-Y"));
  file.print(F(",Raw-Gyro-Z"));
  file.print(F(",Yaw"));
  file.print(F(",Pitch"));
  file.print(F(",Roll"));
  file.print(F(",Barometer-Altitude"));
  file.print(F(",Barometer-Altitude-Bias"));
  file.print(F(",State"));
  file.print(F(",Servo-Z"));
  file.print(F(",Servo-Y"));
  file.print(F(",KP_Y"));
  file.print(F(",KI_Y"));
  file.print(F(",KD_Y"));
  file.print(F(",KP_Z"));
  file.print(F(",KI_Z"));
  file.print(F(",KD_Z"));
  file.print(F(",P_err_y"));
  file.print(F(",I_err_y"));
  file.print(F(",D_err_y"));
  file.print(F(",P_err_z"));
  file.print(F(",I_err_z"));
  file.print(F(",D_err_z"));
  file.print(F(",Y-Servo-Center"));
  file.print(F(",Z-Servo-Center"));
  file.print(F(",Battery-Voltage"));
  file.print(F(",Main-Loop_Time"));
  file.print(F(",Kalman-Altitude-Covariance"));
  file.print(F(",Kalman-Velocity-Covariance"));
  file.print(F(",Kalman-Acceleration-Covariance"));
  file.print(F(",Kalman-Gain-1"));
  file.print(F(",Kalman-Gain-2"));
  file.print(F(",Kalman-Innovation"));
  file.print(F(",Yaw-Bias"));
  file.print(F(",Pitch-Bias"));
  file.print(F(",GPS-Altitude"));
  file.print(F(",GPS-Altitude-Bias"));
  file.print(F(",Num-Sats"));
  file.print(F(",World-Ax-Bias"));
  file.print(F(",World-Ay-Bias"));
  file.print(F(",World-Az-Bias"));
  file.print(F(",Raw-Accel-Magnitude"));
  file.print(F(",PID_DelT_y"));
  file.print(F(",PID_DelT_z"));
  file.print(F(",Latitude"));
  file.print(F(",Longitude"));
  file.print(F(",BNO-AX"));
  file.print(F(",BNO-AY"));
  file.print(F(",BNO-AZ"));
  file.print(F(",BNO-GX"));
  file.print(F(",BNO-GY"));
  file.print(F(",BNO-GZ"));
  file.print(F(",BNO_MX"));
  file.print(F(",BNO_MY"));
  file.print(F(",BNO_MZ"));
  file.print(F(",BNO_YAW"));
  file.print(F(",BNO_PITCH"));
  file.print(F(",BNO_ROLL"));
  file.print(F(",BNO_WAX"));
  file.print(F(",BNO_WAY"));
  file.print(F(",BNO_WAZ"));
  file.print(F(",y_setpoint"));
  file.print(F(",z_setpoint"));
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
  file.write(',');
  file.print(data.kal_X_posP, numDecimals);
  file.write(',');
  file.print(data.kal_X_velP, numDecimals);
  file.write(',');
  file.print(data.kal_X_accelP, numDecimals);
  file.write(',');
  file.print(data.kal_K_1, numDecimals);
  file.write(',');
  file.print(data.kal_K_2, numDecimals);
  file.write(',');
  file.print(data.kal_inno, numDecimals);
  file.write(',');
  file.print(data.yawBias, numDecimals);
  file.write(',');
  file.print(data.pitchBias, numDecimals);
  file.write(',');
  file.print(data.gpsAltitude, numDecimals);
  file.write(',');
  file.print(data.gps_altitude_bias, numDecimals);
  file.write(',');
  file.print(data.sats, numDecimals);
  file.write(',');
  file.print(data.worldAxBias, numDecimals);
  file.write(',');
  file.print(data.worldAyBias, numDecimals);
  file.write(',');
  file.print(data.worldAzBias, numDecimals);
  file.write(',');
  file.print(data.accelMag, numDecimals);
  file.write(',');
  file.print(data.pid_delT_y, numDecimals);
  file.write(',');
  file.print(data.pid_delT_z, numDecimals);
  file.write(',');
  file.print(data.lat, numDecimals);
  file.write(',');
  file.print(data.lng, numDecimals);
  file.write(',');
  file.print(data.bno_ax, numDecimals);
  file.write(',');
  file.print(data.bno_ay, numDecimals);
  file.write(',');
  file.print(data.bno_az, numDecimals);
  file.write(',');
  file.print(data.bno_gx, numDecimals);
  file.write(',');
  file.print(data.bno_gy, numDecimals);
  file.write(',');
  file.print(data.bno_gz, numDecimals);
  file.write(',');
  file.print(data.bno_magx, numDecimals);
  file.write(',');
  file.print(data.bno_magy, numDecimals);
  file.write(',');
  file.print(data.bno_magz, numDecimals);
  file.write(',');
  file.print(data.bno_yaw, numDecimals);
  file.write(',');
  file.print(data.bno_pitch, numDecimals);
  file.write(',');
  file.print(data.bno_roll, numDecimals);
  file.write(',');
  file.print(data.bno_worldAx, numDecimals);
  file.write(',');
  file.print(data.bno_worldAy, numDecimals);
  file.write(',');
  file.print(data.bno_worldAz, numDecimals);
  file.write(',');
  file.print(data.ySetpoint, numDecimals);
  file.write(',');
  file.print(data.zSetpoint, numDecimals);
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

int transferToSDDump()
{

  while (!initSD())
  {
    buzzerError();
    handleLEDBlink(0, 0, 255);
    delay(1000);
  }
  while (readFromFlashDump())
  {
    logData();
  }
  file.close();
  return 1;
};
