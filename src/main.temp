#include <Arduino.h>
#include <Servo.h>
#include "Data.h"
#include "Nav.h"
#include "Battery.h"
#include "Chrono.h"
#include "SD.h"
#include "Data.h"
#include "Buzzer.h"
#include "PID.h"

Servo yServo;
Servo zServo;

bool goMode = false;
bool writingMode = false;
bool finishedWriting = false;

Chrono writeTimer;
Chrono servoTimer;
int servoFlag = 0;
int writeSecond = 0;

int z_val;
int y_val;

PID z_PID;
PID y_PID;

// Z YAW
// Y PITCH

#define Y_CENTER 98
#define Y_MAX 107
#define Y_MIN 89

#define Z_CENTER 100
#define Z_MAX 109
#define Z_MIN 91
int y_max = Z_MAX;
int y_min = Z_MIN;

void setup()
{
  Serial.println(115200);

  while (!Serial)
    ;

  delay(100);
  initFlash();

  delay(100);
  initBuzzer();

  Serial.print("Battery Voltage: ");
  Serial.println(getBattVoltage());

  yServo.attach(SERVO1_PIN);
  zServo.attach(SERVO2_PIN);

  yServo.write(Y_CENTER);
  zServo.write(Z_CENTER);

  delay(2000);

  z_PID.setTunings(7, 4, 2.6);
  z_PID.setOutputLimits(Z_MIN, Z_MAX);

  y_PID.setTunings(7, 4, 2.6);
  y_PID.setOutputLimits(Y_MIN, Y_MAX);

  initIMU();
  delay(1000);

  Serial.println("DONE SETUP");
}

void loop()
{

  getYPR();

  yServo.write(map(data.yaw,-80, 80,Y_MIN,Y_MAX));
  zServo.write(map(data.pitch,-80, 80,Z_MIN,Z_MAX));

  

  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == 'l')
    {
      zeroGyroscope();
    }

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

    if (c == 'g')
    {
      goMode = true;
    }

    if (c == 'x')
    {
      z_PID.incrementKP(0.5);
    }

    if (c == 'z')
    {
      z_PID.incrementKP(-0.5);
    }

    if (c == 'v')
    {
      z_PID.incrementKI(0.5);
    }

    if (c == 'c')
    {
      z_PID.incrementKI(-0.5);
    }

    if (c == 'n')
    {
      z_PID.incrementKD(0.1);
    }

    if (c == 'b')
    {
      z_PID.incrementKD(-0.1);
    }

    if (c == 'r')
    {
      z_PID.setTunings(1, 0, 0);
    }

    // Serial.print(z_PID.kp);
    // Serial.print(" ");
    // Serial.print(z_PID.ki);
    // Serial.print(" ");
    // Serial.println(z_PID.kd);

    Serial.print("Y Val: ");
    Serial.print(y_val);
    Serial.print(" ");
    Serial.print("Z Val: ");
    Serial.print(z_val);
    Serial.println();

    yServo.write(y_val);
    zServo.write(z_val);
  }

  if (goMode)
  {
    writingMode = true;
    z_PID.setInput(data.yaw);
    y_PID.setInput(data.pitch);

    z_PID.compute();
    y_PID.compute();

    z_val = z_PID.Output;
    y_val = y_PID.Output;
    Serial.println(z_val);
    zServo.write(z_val);
    yServo.write(y_val);

    data.servo_z = z_val;
    data.servo_y = y_val;
    data.kp = z_PID.kp;
    data.ki = z_PID.ki;
    data.kd = z_PID.kd;

    if (writingMode == true)
    {
      if (!handleWriteFlash())
      {
        writingMode = false;
        finishedWriting = true;
      }
      else
      {
        if (writeTimer.hasPassed(1000))
        {
          writeSecond += 1;
          Serial.println(writeSecond);
          writeTimer.restart();
        }
      }
    }
  }

  if (finishedWriting)
  {
    Serial.println("Writing to SD");
    transferToSD();
    buzzComplete();
    Serial.println("SD writing complete");
    while (1)
      ;
  }

  // Serial.print(data.yaw);
  // Serial.print(" ");
  // Serial.print(Output);
  // Serial.println();
}