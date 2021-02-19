#include <Arduino.h>
#include <Servo.h>
#include "Nav.h"
#include "MS5607.h"
#define Y_SERVO_PIN SERVO1_PIN
#define Z_SERVO_PIN SERVO2_PIN

Servo y_servo;
Servo z_servo;

#define z_body_angle ypr[0]
#define y_body_angle ypr[1]

#define Y_MAX 110
#define Y_CENTER 83
#define Y_MIN 56
#define Z_MAX 131
#define Z_CENTER 103
#define Z_MIN 75

float altitude;

MS5607 altimeter(&altitude);

int y_val = Y_CENTER;
int z_val = Z_CENTER;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  Serial.begin(115200);
  // while (!Serial)
  //   ;

  y_servo.attach(Y_SERVO_PIN);
  z_servo.attach(Z_SERVO_PIN);

  y_servo.write(Y_CENTER);
  z_servo.write(Z_CENTER);

  Serial.println("Waiting 5 sec for Servo Vibrations To Dampen..");

  delay(5000);
  Serial.println("Initializing Onboard IMU. Getting Gyro Biases..");

  initIMU();

  altimeter.begin();

  Serial.println("IMU Initialized Successfully.");
}

void loop()
{

  // int serDelay = 200;

  // y_servo.write(Y_MAX);
  // delay(serDelay);
  // y_servo.write(Y_CENTER);
  // delay(serDelay);
  // y_servo.write(Y_MIN);
  // delay(serDelay);
  // y_servo.write(Y_CENTER);
  // delay(serDelay);
  // z_servo.write(Z_MAX);
  // delay(serDelay);
  // z_servo.write(Z_CENTER);
  // delay(serDelay);
  // z_servo.write(Z_MIN);
  // delay(serDelay);
  // z_servo.write(Z_CENTER);

  // delay(serDelay);

  altimeter.handleAltimeter();
  //Serial.println(altitude);

  getYPR();

  y_val = mapfloat(ypr[1], -15.0, 15.0, Y_MIN, Y_MAX);
  z_val = mapfloat(ypr[0], -15.0, 15.0, Z_MIN, Z_MAX);

  Serial.print(ypr[1]);
  Serial.print(" ");
  Serial.print(y_val);
  Serial.print(" ");
  Serial.print(ypr[0]);
  Serial.print(" ");
  Serial.print(z_val);
  Serial.println(" ");

  if (z_val < Z_MAX && z_val > Z_MIN)
  {
    z_servo.write(z_val);
  }
  if (y_val < Y_MAX && y_val > Y_MIN)
  {
    y_servo.write(y_val);
  }

  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '7')
    {
      z_val -= 5;
    }

    if (c == '8')
    {
      z_val -= 1;
    }

    if (c == '9')
    {
      z_val += 1;
    }

    if (c == '0')
    {
      z_val += 5;
    }

    if (c == '1')
    {
      y_val -= 5;
    }

    if (c == '2')
    {
      y_val -= 1;
    }

    if (c == '3')
    {
      y_val += 1;
    }

    if (c == '4')
    {
      y_val += 5;
    }

    Serial.print("Y Val: ");
    Serial.print(y_val);
    Serial.print(" ");
    Serial.print("Z Val: ");
    Serial.print(z_val);
    Serial.println();

    // y_servo.write(y_val);
    // z_servo.write(z_val);
  }

  delay(20);
}