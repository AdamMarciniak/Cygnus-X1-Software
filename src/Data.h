#pragma once

#include <Arduino.h>
#include "States.h"

extern uint16_t rateHz;
extern uint16_t numSeconds;
extern uint16_t totalSamples;
extern uint16_t millisPerSample;

struct Data
{
  unsigned int ms;
  float ax;
  float ay;
  float az;
  float worldAx;
  float worldAy;
  float worldAz;
  float worldVx;
  float worldVy;
  float worldVz;
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
  float kal_X;
  float kal_V;
  float kal_A;
  float kal_XP;
  float kal_VP;
  float kal_AP;
  float baroNoise;
  float servo_z;
  float servo_y;
  float kp_y;
  float ki_y;
  float kd_y;
  float kp_z;
  float ki_z;
  float kd_z;
  float pid_delta_y;
  float pid_delta_z;
  float p_err_y;
  float i_err_y;
  float d_err_y;
  float p_err_z;
  float i_err_z;
  float d_err_z;
  float yawBias;
  float pitchBias;
  float batteryVoltage;
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