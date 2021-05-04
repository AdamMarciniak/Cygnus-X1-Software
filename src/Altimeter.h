#pragma once

#include "libraries/MS5607.h"
#include "Data.h"

void getAltitudeBias();
float getAltitude();
bool initAltimeter();
extern float altitude;
extern void handleAltimeter();
extern bool isNewAltimeterData();

float getMovingAverage(float altitude);
