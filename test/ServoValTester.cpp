#include <Arduino.h>
#include <Servo.h>

Servo zServo;
Servo yServo;

int z_val;
int y_val;

#define X_CENTER ;
#define X_MAX ;
#define X_MIN ;

void setup()
{
  Serial.println(115200);

  zServo.attach(SERVO1_PIN);
  yServo.attach(SERVO2_PIN);
}

void loop()
{
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

    yServo.write(y_val);
    zServo.write(z_val);
  }
}