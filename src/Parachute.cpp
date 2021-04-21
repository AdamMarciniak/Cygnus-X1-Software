#include "Parachute.h"
#include <Arduino.h>
#include <Servo.h>
#include "Config.h"

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