#include <Arduino.h>
#include "LED.h"
#include "Buzzer.h"
#include "Altimeter.h"
#include "Chrono.h"
#include "Nav.h"
#include "Data.h"
#include "GPS.h"
#include "SD.h"

Chrono testTimer;

enum RocketState
{
  INITIALIZING,
  PAD_IDLE,
  LAUNCHING,
  POWERED_ASCENT,
  UNPOWERED_ASCENT,
  DESCENT,
  PARACHUTE_DESCENT,
  LANDED,
  DATA_TRANSFER,
  COMPLETE,
  ERROR,
  ABORT,
} state;

Chrono altitudeTimer;

void initFailed(int errorCode)
{
  Serial.print("Init Failed. Code: ");
  Serial.println(errorCode);
  while (1)
  {
    handleLEDBlink(255, 0, 0);
    buzzerError();
  }
}

void setup()
{
  state = INITIALIZING;
  Serial.begin(115200);
  // while (!Serial)
  //   ;
  initLED();
  turnOffLED();
  initBuzzer();
  initFlash();
  //initData();

  Serial.println("Initializing Altimeter...");
  if (!initAltimeter())
  {
    Serial.println("Initializing Altimeter Failed");
    initFailed(1);
  }

  Serial.println("Initializing IMU...");
  if (!initIMU())
  {
    Serial.println("Initializing IMU Failed");
    initFailed(2);
  }

  // Serial.println("Initializing Flash...");
  // if (!initFlash)
  // {
  //   Serial.println("Initializing Flash Failed");
  //   initFailed(3);
  // }

  Serial.println("Initializing SD...");
  if (!initSD())
  {
    Serial.println("Initializing SD Failed");
    initFailed(4);
  }
  Serial.println("Initializing GPS");
  // initServos();
  initGPS();
  buzzStartup();
  delay(2000);
}

void loop()
{

  handleAltitude();
  getAccel();
  handleGPS();
  getYPR();
  if (testTimer.hasPassed(100))
  {
    Serial.print(state);
    Serial.print(" ");
    Serial.print(data.altitude);
    Serial.print(" ");
    Serial.println(data.ax);
    testTimer.restart();
  }
  switch (state)
  {
  case INITIALIZING:
    if (gpsReady == true)
    {
      Serial.println("Switching to PAD_IDLE");
      state = PAD_IDLE;
    }
    break;
  case PAD_IDLE:
    if (!handleWriteFlash())
    {
      Serial.println("Switching to DATA_TRANSFER");
      state = DATA_TRANSFER;
    }
    if (data.ax > 25.0f)
    {
      delay(100);
      state = DATA_TRANSFER;
    }
    break;
  case LAUNCHING:
    break;
  case POWERED_ASCENT:
    break;
  case UNPOWERED_ASCENT:
    break;
  case DESCENT:
    break;
  case PARACHUTE_DESCENT:
    break;
  case LANDED:
    break;
  case DATA_TRANSFER:
    transferToSD();
    state = COMPLETE;
    break;
  case COMPLETE:
    handleLEDBlink(0, 255, 0);
    buzzComplete();
    while (1)
      ;
    break;
  case ERROR:
    break;
  case ABORT:
    break;
  default:
    break;
  }
}
