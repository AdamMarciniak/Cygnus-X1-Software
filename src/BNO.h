#pragma once

#include "Quaternion.h"
#include "Wire.h"
#include "Data.h"
#include "./libraries/Adafruit_Sensor.h"
#include "./libraries/Adafruit_BNO055.h"
#include "./libraries/utility/imumaths.h"

void initBNO();
void getBNOData();