#include <Arduino.h>
#include <Servo.h>
#include "Data.h"
#include "Nav.h"
#include "Battery.h"
#include "Chrono.h"
#include "Data.h"
#include "Buzzer.h"
#include "PID.h"
#include "./SdCard/SD.h"
#include "Altimeter.h"
#include "BTLE.h"
#include "Parachute.h"
#include "ServoControl.h"
#include "Telemetry.h"
#include "Pyro.h"

#define FIRE_TO_PID_DELAY 500 //ms
#define BATTERY_VOLTAGE_MIN 11.5 //volts
#define NAV_RATE 10 //ms

bool goMode = false;
bool writingMode = false;
bool finishedWriting = false;

Chrono writeTimer;
Chrono servoTimer;
Chrono pidTimer;
Chrono printTimer;
Chrono batteryCheckTimer;


int servoFlag = 0;
int writeSecond = 0;


unsigned long passed = 0;
unsigned long t_ = 0;

void initExtraData()
{
  data.kp_y = Y_KP;
  data.ki_y = Y_KI;
  data.kd_y = Y_KD;
  data.kp_z = Z_KP;
  data.ki_z = Z_KI;
  data.kd_z = Z_KD;
}

void checkBatteryVoltage()
{
  if (getBattVoltage() < BATTERY_VOLTAGE_MIN)
  {
    while (1)
    {
      buzzLongs();
      delay(300);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  data.state = INITIALIZING;
  //checkBatteryVoltage();
  //initPyro();

  buzzStartup();

  initFlash();
  initBluetooth();
  initBuzzer();
  initExtraData();
  initPIDs();
  initServos();
  initParachute();

  delay(2000);

  initAltimeter();
  initIMU();

  buzzStartup();
  delay(500);
  buzzStartup();
  delay(500);
  buzzStartup();

  delay(2000);

  buzzStartup();
  delay(200);
  buzzStartup();

  passed = millis();
  t_ = 0;
  writingMode = true;
}

void handleDoNav()
{

  if (pidTimer.hasPassed(NAV_RATE))
  {
    getAltitude();
    setZPIDInput(data.pitch);
    setYPIDInput(data.yaw);
    computeBothPIDs();
    
    moveZServo(int(round(data.Z_Servo_Center + data.servo_z)));
    moveYServo(int(round(data.Y_Servo_Center + data.servo_y)));
  };
}

void handleWritingToFlash()
{
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

void loop()
{

  getYPR();
  getAccel();
  checkBTLE();

  if (data.state == INITIALIZING){
      handleServoCentering();
  }

  if (data.state == LAUNCH_COMMANDED)
  {
    t_ += (millis() - passed);
    passed = millis();

   

    //handleFirePyro();

    handleDoNav();
    handleWritingToFlash();
  }

  if (finishedWriting)
  {
    Serial.println("Writing to SD");
    transferToSD();
    buzzComplete();
    delay(200);
    buzzComplete();
    delay(200);
    buzzComplete();
    Serial.println("SD writing complete");
    while (1)
      ;
  }
}