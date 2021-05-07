#pragma once

#include <Arduino.h>
#include "States.h"
#include "Config.h"
#include "./SdCard/SPI.h"
#include "./SdCard/SPIMemory.h"
#include "Chrono.h"

extern uint16_t rateHz;
extern uint16_t numSeconds;
extern uint16_t totalSamples;
extern uint16_t millisPerSample;

void goToState(State state);

struct Data
{
  float ms;
  float ax;
  float ay;
  float az;
  float worldAx;
  float worldAy;
  float worldAz;
  float gx;
  float gy;
  float gz;
  float yaw;
  float pitch;
  float roll;
  float altitude;
  float biasAltitude;
  int btleCmd;
  float Y_Servo_Center;
  float Z_Servo_Center;
  float loopTime;
  State state;
  float fState;
  float kal_X_pos;
  float kal_X_vel;
  float kal_X_accel;
  float kal_X_posP;
  float kal_X_velP;
  float kal_X_accelP;
  float kal_K_1;
  float kal_K_2;
  float kal_inno;
  float servo_z;
  float servo_y;
  float kp_y;
  float ki_y;
  float kd_y;
  float kp_z;
  float ki_z;
  float kd_z;
  float p_err_y;
  float i_err_y;
  float d_err_y;
  float p_err_z;
  float i_err_z;
  float d_err_z;
  float yawBias;
  float pitchBias;
  float batteryVoltage;
  float lat;
  float lng;
  float gpsAltitude;
  float gps_altitude_bias;
  float sats;
  float hdop;
  float worldAxBias;
  float worldAyBias;
  float worldAzBias;
  float accelMag;
  float pyro1Continuity;
};

struct NonLoggedData
{
  bool servoCentersAvailable;
  bool zeroGyrosStatus;
};

extern NonLoggedData nonLoggedData;
extern Data data;

extern unsigned long write_addr;
extern unsigned long read_addr;

extern unsigned long addrStep;
extern unsigned long maxAddr;

extern void initFlash();
extern bool writeToFlash();
extern bool readFromFlash();
extern bool handleWriteFlash();

extern void readTVCCenters();
extern void writeTVCCenters();
extern void eraseFlightData();