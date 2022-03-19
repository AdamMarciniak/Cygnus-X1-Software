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
#include "GPS.h"
#include "dumpData.h"
#include "LED.h"
#include "BNOIMU.h"

BNOIMU bno;

Chrono navTimer;

float accelMag = 0;
bool flashWriteStatus = false;
bool PIDStatus = false;
bool gyroZeroStatus = false;
unsigned long landingDetectTime = 0;

bool finishedWriting = false;
unsigned long prevLoopTime;
unsigned long currentLoopTime;

unsigned long launchAbortTime = 0;
unsigned long PID_DelayTime = 0;

bool firstLaunchLoop = true;
bool firstAbortLoop = true;
unsigned long abortLoopTime = 0;

unsigned long powTime = 0;

float accelAtStage2Start = 0.0f;

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

unsigned long testTime = 0;

void handleDumpData()
{
  if (IS_DUMP_MODE)
  {
    // while (!Serial)
    //   ;
    dumpData();
    while (1)
    {
      delay(500);
      buzzComplete();
      handleLEDBlink(255, 0, 0);
    };
    ;
  }
}

void setup()
{

  initBuzzer();

  Serial.begin(115200);
  Serial1.begin(9600);

  handleDumpData();

  goToState(INITIALIZING);
  delay(10000);

  buzzStartup();

  initFlash();

  initBluetooth();
  initPIDs();
  initParachute();

  delay(2000);

  initNav();
  if (INIT_BNO)
  {
    bno.initBNO();
  }

  buzzFast();

  delay(2000);

  buzzStartup();
  delay(200);
  buzzStartup();

  initEUI();
  initServos();

  // Put state into IDLE when finished if all good.
  initPyro();

  data.max_altitude = 0.0f;

  prevLoopTime = micros();
  currentLoopTime = micros();
  if (IS_TEST_MODE)
  {
    goToState(TEST);
    testTime = millis();
  }
  else
  {
    goToState(IDLE);
  }
}

void handleRunNav()
{
  handleAltimeter();

  if (navTimer.hasPassed(NAV_RATE))
  {
    getAccel();
    getYPR();
    predict(data.worldAx);

    if (data.state == IDLE || data.state == TEST)
    {
      data.worldAxBias += getMovingAverageWorldXAccel(data.worldAx);
    }

    if (PIDStatus == true)
    {
      setZPIDInput(data.pitch);
      setYPIDInput(data.yaw);
      computeBothPIDs();
      moveZServo(int(round(data.Z_Servo_Center + data.servo_z)));
      // Negative since new airframe, axes are reversed
      moveYServo(int(round(data.Y_Servo_Center - data.servo_y)));
    }
    navTimer.restart();
  };

  if (isNewAltimeterData())
  {
    getAltitude();
    if (data.state == IDLE || data.state == TEST)
    {
      // Do this so that while idle, bias doesn't rise.
      data.biasAltitude += getMovingAverage(data.altitude);
    }
    updateBaro(data.altitude);
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

unsigned long powStart = 0;
bool firstPow = true;
int pitchAngleSetpoint = 0;

Chrono tvcPrintLoop;

void loop()
{
  currentLoopTime = micros();
  data.loopTime = float(currentLoopTime - prevLoopTime) / 1000000.0f;
  prevLoopTime = currentLoopTime;

  handleRunNav();
  checkBTLE();
  handleWritingToFlash();
  handleEUI();
  handleGPS();

  handleBatteryCheck();

  if (data.state == LAUNCH_COMMANDED || (data.state == POWERED_ASCENT && data.kal_X_pos < ANGLE_ABORT_MAX_ALT))
  {
    if (isAnglePassedThreshold())
    {
      goToState(ABORT);
    }
  }

  if (data.kal_X_pos > data.max_altitude)
  {
    data.max_altitude = data.kal_X_pos;
  }

  switch (data.state)
  {

  case TEST:
    handleTestServos();
    PIDStatus = true;
    handleServoCentering();

    break;

  case IDLE:

    handleSendTelemetry();

    handleGetContinuity();

    handleTestServos();

    if (ENABLE_TVC_IMU)
    {

      if (tvcPrintLoop.hasPassed(NAV_RATE))
      {
        getTVCIMUAccel();
        getTVCAttitude();
        // Serial.print("TVC_AX: ");
        // Serial.print(data.tvc_ax);
        // Serial.print("  TVC_AY: ");
        // Serial.print(data.tvc_ay);
        // Serial.print("  TVC_AZ: ");
        // Serial.print(data.tvc_az);
        //Serial.print("  TVC_YAW: ");
        Serial.print(data.tvc_yaw);
        Serial.print(" ");
        Serial.print(data.yaw);
        Serial.print(" ");
        Serial.print(data.tvc_pitch);
        Serial.print(" ");
        Serial.print(data.pitch);
        Serial.println();

        tvcPrintLoop.restart();
      }
    }

    // wait for zero gyros command
    if (nonLoggedData.zeroGyrosStatus == true)
    {
      nonLoggedData.zeroGyrosStatus = false;
      zeroGyroscope();
    }

    if (!SELF_FIRE)
    {
      // Trigger powered flight if launch happens without BTLE
      if (data.worldAx > LAUNCH_ACCEL_THRESHOLD)
      {
        flashWriteStatus = true;
        zeroGyroscope();
        zeroKalman();
        goToState(POWERED_ASCENT);
        PIDStatus = true;
      }
    }

    handleServoCentering();

    break;

  case LAUNCH_COMMANDED:

    // Zero Gyros and other sensors as needed
    if (firstLaunchLoop == true)
    {
      flashWriteStatus = true;
      firstLaunchLoop = false;
      zeroGyroscope();
      zeroKalman();
      launchAbortTime = millis();
      PID_DelayTime = millis();
    }

    // Pyro2 is first pyro to launch based on wiring
    handleFirePyro2();

    if (millis() - PID_DelayTime >= FIRE_TO_PID_DELAY)
    {
      PIDStatus = true;
    }

    if (millis() - launchAbortTime >= MOTOR_FAIL_DELAY)
    {
      stopPyro2();
      goToState(ABORT);
    }

    if (data.worldAx > LAUNCH_ACCEL_THRESHOLD)
    {
      stopPyro2();
      goToState(POWERED_ASCENT);
    }

    break;
  case POWERED_ASCENT:

    PIDStatus = true;
    if (firstPow == true)
    {
      firstPow = false;
      powStart = millis();
    }

    powTime = millis() - powStart;

    if (powTime >= TIME_TO_SECOND_STAGE)
    {
      accelAtStage2Start = data.ax;
      goToState(LAUNCH_COMMANDED_2);
    }

    data.accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));

    if (data.kal_X_vel <= -1.0f)
    {
      goToState(FREE_DESCENT);
    }

    break;

  case LAUNCH_COMMANDED_2:

    handleFirePyro1();
    data.accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));

    if (data.ax > accelAtStage2Start + 5.0f)
    {
      stopPyro1();
      goToState(POWERED_ASCENT_2);
    }

    if (data.kal_X_vel <= -2.0f)
    {
      stopPyro1();
      goToState(FREE_DESCENT);
    }

    break;

  case POWERED_ASCENT_2:

    data.accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));
    if (data.ax < ACCEL_UNPOWERED_THRESHOLD)
    {
      goToState(UNPOWERED_ASCENT);
    }

    if (data.kal_X_vel <= -2.0f)
    {
      goToState(FREE_DESCENT);
    }

    break;
  case UNPOWERED_ASCENT:
    // Center and turn off TVC
    data.accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));

    if (data.kal_X_vel <= -0.5f)
    {
      goToState(FREE_DESCENT);
    }

    break;
  case FREE_DESCENT:
    data.accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));

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

    data.accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));

    PIDStatus = false;

    if (millis() - landingDetectTime > LANDING_DETECT_DELAY)
    {
      goToState(LANDED);
    }

    break;

  case ABORT:

    deployParachute();
    stopPyro1();
    stopPyro2();

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
    stopPyro1();
    stopPyro2();
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
      buzzMaxAltitude(data.max_altitude);
      //buzzComplete();
    };

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