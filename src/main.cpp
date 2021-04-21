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
#include "BTLE.h"
#include "Parachute.h"
#include "ServoControl.h"
#include "Telemetry.h"
#include "Pyro.h"
#include "Config.h"
#include "Kalman.h"
#include "./eui/EUIMyLib.h"



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
  delay(2000);
  Serial.begin(115200);
  data.state = LAUNCH_COMMANDED;
  
  //checkBatteryVoltage();

  //initPyro();

  buzzStartup();

  initFlash();
  initBluetooth();
  initBuzzer();
  initPIDs();
  initServos();
  initParachute();

  delay(2000);

  
  initNav();

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
    initEUI();
    initKalman();

}

void handleDoNav()
{

  handleAltimeter();


  if (pidTimer.hasPassed(NAV_RATE))
  {

    getAccel();
    getYPR();
    updateAccel(data.worldAx);

    setZPIDInput(data.pitch);
    setYPIDInput(data.yaw);
    computeBothPIDs();


    moveZServo(int(round(data.Z_Servo_Center + data.servo_z)));
    moveYServo(int(round(data.Y_Servo_Center + data.servo_y)));

    pidTimer.restart();
  };

  if (isNewAltimeterData())
  {
    updateBaro(getAltitude());
  }
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
  }
}

bool launchFlag = true;
Chrono launchTimer;

void loop()
{
  handleEUI();
  checkBTLE();

  handleDoNav();

  if (data.state == INITIALIZING){
      handleServoCentering();
      handleSendTelemetry();
      
  }

  if (data.state == LAUNCH_COMMANDED)
  {
    t_ += (millis() - passed);
    passed = millis();

    //handleFirePyro();
    
    
    //handleWritingToFlash();
  }

  if(data.worldAx > 5.0 && data.state == LAUNCH_COMMANDED){
    data.state = POWERED_ASCENT;
  }

  if(data.state == POWERED_ASCENT && data.kal_V <= -0.3){
    data.state = FREE_DESCENT;
  }

  if(data.state == FREE_DESCENT && data.kal_X < 30){
    
    data.state = IDLE;
  }


  if(data.state == IDLE){
    handleBuzzer();
    deployParachute();
    if(launchTimer.hasPassed(2000)){
      data.state = LAUNCH_COMMANDED;
      buzzOff();
      launchTimer.restart();
    }
  }
  // if (finishedWriting)
  // {
  //   Serial.println("Writing to SD");
  //   transferToSD();
  //   buzzComplete();
  //   delay(200);
  //   buzzComplete();
  //   delay(200);
  //   buzzComplete();
  //   Serial.println("SD writing complete");
  //   while (1)
  //     ;
  // }
}



// void loop()
// {

//   handleEUI();
//   handleAltimeter();
//   handleChangeBaroNoise();

//   if (accelTimer.hasPassed(5))
//   {
//     getAccel();
//     getYPR();
//     updateAccel(data.worldAx);
//     accelTimer.restart();
//   }

//   if (isNewAltimeterData())
//   {
//     updateBaro(getAltitude());
//   }
// }