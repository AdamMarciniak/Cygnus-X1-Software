#include "ServoControl.h"

Servo yServo;
Servo zServo;

Chrono servoTimer;
bool servoTestOn = false;



void handleServoCentering()
{
  if (nonLoggedData.servoCentersAvailable == true)
  {
    nonLoggedData.servoCentersAvailable = false;
    Serial.print("Writing To servos");
    Serial.print("   Y: ");
    Serial.print(data.Y_Servo_Center);
    Serial.print("   Z: ");
    Serial.println(data.Z_Servo_Center);
    yServo.write(data.Y_Servo_Center);
    zServo.write(data.Z_Servo_Center);
    writeTVCCenters();

    // Write new center values to servos
  }
}

void initServos()
{
  Serial.println("Attaching servos to pins");
  //Bottom Servo on PIN1
  //Top Servo on Pin 2
  yServo.attach(SERVO2_PIN);
  zServo.attach(SERVO1_PIN);

  readTVCCenters();
  Serial.println("Centering Servos. Wait 2 seconds..");
  yServo.write(data.Y_Servo_Center);
  zServo.write(data.Z_Servo_Center);
  data.servo_y = 0;
  data.servo_z = 0;
}



unsigned int servoStep = 0;

void startServoTest()
{
  servoTestOn = true;
}

void handleTestServos()
{

  if (servoTestOn == true)
  {

    if (servoTimer.hasPassed(16))
    {

      moveYServo(data.Y_Servo_Center + SERVO_RANGE * sin(0.1 * servoStep));
      moveZServo(data.Z_Servo_Center + SERVO_RANGE * sin(0.1 * servoStep - PI / 2.0));

      servoStep += 1;

      if (servoStep == 63)
      {
        servoTestOn = false;
        servoStep = 0;
        moveYServo(data.Y_Servo_Center);
        moveZServo(data.Z_Servo_Center);
      }

      servoTimer.restart();
    }
  }
}

void moveYServo(int val)
{
  yServo.write(val);
}

void moveZServo(int val)
{
  zServo.write(val);
}
