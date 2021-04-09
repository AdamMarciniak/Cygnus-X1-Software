#include "Parachute.h"
#include <Arduino.h>
#include <Servo.h>
#define PARACHUTE_SERVO_DEPLOY 53
#define PARACHUTE_SERVO_INIT 97

Servo parachuteServo;

void initParachute()
{
  parachuteServo.attach(SERVO3_PIN);
  parachuteServo.write(PARACHUTE_SERVO_INIT);
}

void deployParachute()
{
  parachuteServo.write(PARACHUTE_SERVO_DEPLOY);
}