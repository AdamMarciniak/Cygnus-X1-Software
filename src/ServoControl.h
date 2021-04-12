#pragma once

#include "Data.h"
#include <Servo.h>

#define SERVO_RANGE 20
#define TVC_TO_SERVO_SCALE 4
#define Y_CENTER 91
#define Y_MAX Y_CENTER + SERVO_RANGE
#define Y_MIN Y_CENTER - SERVO_RANGE

#define Z_CENTER 102
#define Z_MAX Z_CENTER + SERVO_RANGE
#define Z_MIN Z_CENTER - SERVO_RANGE



extern void handleServoCentering();

extern void initServos();

extern void moveYServo(int val);

extern void moveZServo(int val);
