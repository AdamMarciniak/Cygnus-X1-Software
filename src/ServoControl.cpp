#include "ServoControl.h"


Servo yServo;
Servo zServo;

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
  yServo.attach(SERVO1_PIN);
  zServo.attach(SERVO2_PIN);

  readTVCCenters();
  Serial.println("Centering Servos. Wait 2 seconds..");
  yServo.write(data.Y_Servo_Center);
  zServo.write(data.Z_Servo_Center);
}

void moveYServo(int val){
  yServo.write(val);
}

void moveZServo(int val){
  zServo.write(val);
}
