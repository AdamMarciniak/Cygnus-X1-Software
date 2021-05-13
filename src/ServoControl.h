#pragma once

#include "Data.h"
#include <Servo.h>
#include "Config.h"
#include "Chrono.h"

#define Y_MAX Y_CENTER + SERVO_RANGE
#define Y_MIN Y_CENTER - SERVO_RANGE
#define Z_MAX Z_CENTER + SERVO_RANGE
#define Z_MIN Z_CENTER - SERVO_RANGE

extern void handleServoCentering();

extern void initServos();

extern void moveYServo(int val);

extern void moveZServo(int val);

void handleTestServos();

void startServoTest();

void detachServos();
