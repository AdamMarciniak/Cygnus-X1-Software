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

#define SERVO_RANGE 20
#define TVC_TO_SERVO_SCALE 4
#define Y_CENTER 102
#define Y_MAX Y_CENTER + SERVO_RANGE
#define Y_MIN Y_CENTER - SERVO_RANGE


#define Z_CENTER 95
#define Z_MAX Z_CENTER + SERVO_RANGE
#define Z_MIN Z_CENTER - SERVO_RANGE

int y_max = Z_MAX;
int y_min = Z_MIN;



#define Y_KP 1.5
#define Y_KI 0.0
#define Y_KD 0.9

#define Z_KP 1.5
#define Z_KI 0.0
#define Z_KD 0.9

void setup()
{
  Serial.println(115200);

  while (!Serial)
    ;

  delay(100);
  initFlash();
  Serial.println("Flash Initialized");

  delay(100);
  initBuzzer();
  Serial.println("Buzzer Initialized");

  Serial.print("Battery Voltage: ");
  Serial.println(getBattVoltage());

  Serial.println("Attaching servos to pins");
  yServo.attach(SERVO1_PIN);
  zServo.attach(SERVO2_PIN);

  Serial.println("Centering Servos. Wait 2 seconds..");
  yServo.write(Y_CENTER);
  zServo.write(Z_CENTER);

  delay(2000);

  Serial.println("Setting PID Tunings");
  z_PID.setTunings(Z_KP, Z_KI, Z_KD);
  z_PID.setOutputLimits(Z_MIN, Z_MAX);
  z_PID.setSetpoint(45.0);

  y_PID.setTunings(Y_KP, Y_KI, Y_KD);
  y_PID.setOutputLimits(Y_MIN, Y_MAX);
  y_PID.setSetpoint(0);


  initIMU();
  delay(1000);
  Serial.println("IMU INITIALIZED");

  Serial.println("DONE SETUP");
}

void loop()
{

  getYPR();

  //yServo.write(map(data.yaw,-80, 80,Y_MIN,Y_MAX));
  //zServo.write(map(data.pitch,-80, 80,Z_MIN,Z_MAX));

  
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == 'z')
    {
      zeroGyroscope();
    }

    if (c == 'g')
    {
      goMode = true;
    }

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

    zServo.write(Z_CENTER - (z_val - Z_CENTER ));
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