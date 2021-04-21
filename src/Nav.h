#pragma once

#include "Data.h"
#include "Quaternion.h"
#include "Altimeter.h"
#include "Arduino.h"
#include "libraries/BMI088.h"

extern float ypr[3];
extern float accel_raw[3];
extern float vel_local[3];
extern bool initNav();
extern void getAccel();
extern void getYPR();
extern void getAccel();
extern void zeroGyroscope();
extern void getCurrentYawAndPitchFromAccel();
extern void getInitYawAndPitchBiases();
extern void measureNav();
extern void getWorldAxBiases();
