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

Chrono navTimer;
Chrono batteryCheckTimer;

float accelMag = 0;
bool flashWriteStatus = false;
bool PIDStatus = false;
bool gyroZeroStatus = false;
unsigned long landingTimer = 0;
bool landingWait = false;
unsigned long landingDetectTime = 0;

bool finishedWriting = false;
unsigned long prevLoopTime;
unsigned long currentLoopTime;

bool firstLaunchLoop = true;
bool firstAbortLoop = true;
unsigned long abortLoopTime = 0;

bool isAnglePassedThreshold()
{
  if (ENABLE_ANGLE_CHECK == true)
  {
    if (abs(data.yaw) >= ABORT_ANGLE_THRESHOLD || abs(data.pitch) >= ABORT_ANGLE_THRESHOLD)
    {
      return true;
    }
  }
  return false;
}

void setup()
{
  initBuzzer();

  Serial.begin(115200);

  goToState(INITIALIZING);
  delay(4000);

  buzzStartup();

  initFlash();
  initBluetooth();
  initPIDs();
  initServos();
  initParachute();

  delay(2000);

  initNav();

  buzzFast();

  delay(2000);

  buzzStartup();
  delay(200);
  buzzStartup();

  initEUI();

  // Put state into IDLE when finished if all good.
  initPyro();

  prevLoopTime = micros();
  currentLoopTime = micros();
  goToState(IDLE);
}

Chrono altitudeTimer;
Chrono kalmanTimer;

void handleRunPID()
{

  handleAltimeter();

  if (kalmanTimer.hasPassed(1))
  {
    predict();
    kalmanTimer.restart();
  }

  if (navTimer.hasPassed(NAV_RATE))
  {

    getAccel();
    getYPR();
    updateAccel(data.worldAx);

    if (PIDStatus == true)
    {
      setZPIDInput(data.pitch);
      setYPIDInput(data.yaw);
      computeBothPIDs();
      moveZServo(int(round(data.Z_Servo_Center + data.servo_z)));
      moveYServo(int(round(data.Y_Servo_Center + data.servo_y)));
    }
    navTimer.restart();
  };

  if (isNewAltimeterData())
  {
    updateBaro(getAltitude());
  }
}

void handleWritingToFlash()
{
  if (flashWriteStatus == true)
  {
    if (!handleWriteFlash())
    {
      flashWriteStatus = false;
      finishedWriting = true;
      goToState(LANDED);
    }
  }
}

void loop()
{
  currentLoopTime = micros();
  data.loopTime = float(currentLoopTime - prevLoopTime) / 1000000.0f;
  prevLoopTime = currentLoopTime;

  if (batteryCheckTimer.hasPassed(50))
  {
    data.batteryVoltage = getBattVoltage();
    batteryCheckTimer.restart();
  }

  checkBTLE();

  handleRunPID();

  handleWritingToFlash();

  handleEUI();

  if (data.state != LAUNCH_COMMANDED)
  {
    analogWrite(PYRO1_PIN, 0);
  }

  if (data.state == LAUNCH_COMMANDED || data.state == POWERED_ASCENT)
  {
    if (isAnglePassedThreshold())
    {
      goToState(ABORT);
    }
  }

  switch (data.state)
  {

  case IDLE:

    handleSendTelemetry();

    // wait for zero gyros command
    if (nonLoggedData.zeroGyrosStatus == true)
    {
      nonLoggedData.zeroGyrosStatus = false;
      zeroGyroscope();
    }

    handleServoCentering();

    break;

  case LAUNCH_COMMANDED:

    flashWriteStatus = true;
    // Zero Gyros and other sensors as needed
    if (gyroZeroStatus == false)
    {
      gyroZeroStatus = true;
      zeroGyroscope();
    }

    handleFirePyro();

    PIDStatus = true;

    if (data.worldAx > LAUNCH_ACCEL_THRESHOLD || data.ax > LAUNCH_ACCEL_THRESHOLD)
    {
      analogWrite(PYRO2_PIN, 0);
      goToState(POWERED_ASCENT);
    }

    break;
  case POWERED_ASCENT:

    // Check if accel magnitude is less than thresh
    accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));
    if (accelMag < ACCEL_UNPOWERED_THRESHOLD)
    {
      goToState(UNPOWERED_ASCENT);
    }

    break;
  case UNPOWERED_ASCENT:
    // Center and turn off TVC
    PIDStatus = false;
    moveZServo(data.Z_Servo_Center);
    moveYServo(data.Y_Servo_Center);

    if (data.kal_X_vel <= -1.0f)
    {
      goToState(FREE_DESCENT);
    }

    break;
  case FREE_DESCENT:
    // Detect barometer min altitude for parachute
    if (data.altitude <= PARACHUTE_ALTITUDE_THRESHOLD)
    {
      // Write prachute launch to servo
      goToState(PARACHUTE_DESCENT);
      landingDetectTime = millis();
    }

    break;

  case PARACHUTE_DESCENT:

    deployParachute();
    handleBuzzer();

    if (millis() - landingDetectTime > LANDING_DETECT_DELAY)
    {
      goToState(LANDED);
    }

    break;

  case ABORT:

    deployParachute();
    handleBuzzer();

    if (firstAbortLoop)
    {
      abortLoopTime = millis();
      firstAbortLoop = false;
    }

    if (millis() - abortLoopTime > ABORT_TO_LANDED_DELAY)
    {
      goToState(LANDED);
    }

    break;
  case LANDED:
    delay(2000);
    // Dump all data to SD
    flashWriteStatus = false;
    Serial.println("Writing to SD");
    transferToSD();
    buzzComplete();
    Serial.println("SD writing complete");
    while (1)
    {
      delay(500);
      buzzComplete();
    };

    while (1)
      ;

    break;

  case ERROR:
    // Send error message to BTLE
    flashWriteStatus = false;
    while (1)
      ;
    break;

  default:
    break;
  }
}