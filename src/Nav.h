#pragma once

#include "Data.h"
#include "Quaternion.h"
#include "Altimeter.h"
#include "Arduino.h"
#include "libraries/BMI088.h"
#include "Config.h"
#include "./libraries/SparkFunLSM9DS1.h"

extern float ypr[3];
extern float accel_raw[3];
extern float vel_local[3];
void quatToEuler(float *qBody, float *ypr);
extern bool initNav();
extern void getAccel();
extern void getYPR();
extern void getAccel();
extern void zeroGyroscope();
extern void getCurrentYawAndPitchFromAccel();
extern void getInitYawAndPitchBiases();
extern void measureNav();
extern void getWorldABiases();
float getMovingAverageWorldXAccel(float worldXAccel);

void getTVCIMUAccel();
void getTVCAttitude();