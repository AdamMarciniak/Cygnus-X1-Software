#pragma once

#include <Arduino.h>

extern uint16_t rateHz;
extern uint16_t numSeconds;
extern uint16_t totalSamples;
extern uint16_t millisPerSample;

struct Data
{
  float lat;
  float lng;
  float hdop;
  float sats;
  float gpsAltitude;
  unsigned int ms;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float yaw;
  float pitch;
  float roll;
  float altitude;
  float biasAltitude;
  int state;
  int servo_z;
  int servo_y;
  float kp;
  float ki;
  float kd;
};
extern Data data;

extern unsigned long write_addr;
extern unsigned long read_addr;

extern unsigned long addrStep;
extern unsigned long maxAddr;

extern void initFlash();
extern bool writeToFlash();
extern bool readFromFlash();
extern bool handleWriteFlash();
