#include "./SdCard/SD.h"
#include "./eui/EUIMyLib.h"
#include "BTLE.h"
#include "Battery.h"
#include "Buzzer.h"
#include "Chrono.h"
#include "Config.h"
#include "Data.h"
#include "GPS.h"
#include "Kalman.h"
#include "Nav.h"
#include "PID.h"
#include "Parachute.h"
#include "Pyro.h"
#include "ServoControl.h"
#include "Telemetry.h"
#include "dumpData.h"
#include <Arduino.h>
#include <Servo.h>

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

bool isAnglePassedThreshold() {
  if (ENABLE_ANGLE_CHECK == true) {
    if (abs(data.yaw) >= ABORT_ANGLE_THRESHOLD ||
        abs(data.pitch) >= ABORT_ANGLE_THRESHOLD) {
      return true;
    }
  }
  return false;
}

unsigned long testTime = 0;

void handleDumpData() {
  if (IS_DUMP_MODE) {
    while (!Serial)
      ;
    dumpData();
    while (1) {
      delay(500);
      buzzComplete();
    };
    ;
  }
}

void setup() {
  initBuzzer();
  Serial.begin(115200);
  Serial1.begin(9600);

  // handleDumpData();

  goToState(INITIALIZING);
  delay(1000);

  buzzStartup();

  initFlash();

  initBluetooth();
  initPIDs();
  initParachute();

  delay(1000);

  initNav();

  buzzFast();

  delay(1000);

  buzzStartup();
  delay(200);
  buzzStartup();

  // initEUI();
  initServos();

  // Put state into IDLE when finished if all good.
  initPyro();

  prevLoopTime = micros();
  currentLoopTime = micros();
  if (IS_TEST_MODE) {
    goToState(TEST);
    testTime = millis();
  } else {
    goToState(IDLE);
  }
}

void handleRunNav() {
  handleAltimeter();

  if (navTimer.hasPassed(NAV_RATE)) {

    getAccel();
    getYPR();

    if (data.state == IDLE || data.state == TEST) {
      data.worldAxBias += getMovingAverageWorldXAccel(data.worldAx);
    }

    predict(data.worldAx);

    if (PIDStatus == true) {
      setZPIDInput(data.pitch);
      setYPIDInput(data.yaw);
      computeBothPIDs();
      moveZServo(int(round(data.Z_Servo_Center + data.servo_z)));
      moveYServo(int(round(data.Y_Servo_Center + data.servo_y)));
    }
    navTimer.restart();
  };

  if (isNewAltimeterData()) {
    getAltitude();
    if (data.state == IDLE || data.state == TEST) {
      // Do this so that while idle, bias doesn't rise.
      data.biasAltitude += getMovingAverage(data.altitude);
    }
    updateBaro(data.altitude);
  }
}

void handleWritingToFlash() {
  if (flashWriteStatus == true) {
    if (!handleWriteFlash()) {
      flashWriteStatus = false;
      finishedWriting = true;
      goToState(LANDED);
    }
  }
}

unsigned long powStart = 0;
bool firstPow = true;

void loop() {
  currentLoopTime = micros();
  data.loopTime = float(currentLoopTime - prevLoopTime) / 1000000.0f;
  prevLoopTime = currentLoopTime;

  handleRunNav();
  checkBTLE();
  handleWritingToFlash();
  handleEUI();
  handleGPS();

  handleBatteryCheck();

  if (data.state != LAUNCH_COMMANDED) {
    stopPyros();
  }

  if (data.state == LAUNCH_COMMANDED || data.state == POWERED_ASCENT) {
    if (isAnglePassedThreshold()) {
      goToState(ABORT);
    }
  }

  switch (data.state) {

  case TEST:
    handleTestServos();
    PIDStatus = true;
    flashWriteStatus = true;

    break;

  case IDLE:

    handleSendTelemetry();

    handleGetContinuity();
    handleTestServos();

    // wait for zero gyros command
    if (nonLoggedData.zeroGyrosStatus == true) {
      nonLoggedData.zeroGyrosStatus = false;
      zeroGyroscope();
    }

    if (!SELF_FIRE) {
      // Trigger powered flight if launch happens without BTLE
      if (data.worldAx > LAUNCH_ACCEL_THRESHOLD) {
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
    if (firstLaunchLoop == true) {
      flashWriteStatus = true;
      firstLaunchLoop = false;
      zeroGyroscope();
      zeroKalman();
      launchAbortTime = millis();
      PID_DelayTime = millis();
    }

    handleFirePyro();

    if (millis() - PID_DelayTime >= FIRE_TO_PID_DELAY) {
      PIDStatus = true;
    }

    if (millis() - launchAbortTime >= MOTOR_FAIL_DELAY) {
      goToState(ABORT);
      stopPyros();
    }

    if (data.worldAx > LAUNCH_ACCEL_THRESHOLD) {
      goToState(POWERED_ASCENT);
      stopPyros();
    }

    break;
  case POWERED_ASCENT:
    PIDStatus = true;
    // Check if accel magnitude is less than thresh
    if (firstPow == true) {
      firstPow = false;
      powStart = millis();
    }

    data.accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));
    if (data.ax < ACCEL_UNPOWERED_THRESHOLD)
    // Where accel unpowered threshold is less than +1.0
    {
      goToState(UNPOWERED_ASCENT);
    }

    if (data.kal_X_vel <= -0.5f) {
      goToState(FREE_DESCENT);
    }

    break;
  case UNPOWERED_ASCENT:
    // Center and turn off TVC
    PIDStatus = false;

    if (data.kal_X_vel <= -0.5f) {
      goToState(FREE_DESCENT);
    }

    break;
  case FREE_DESCENT:
    // Detect barometer min altitude for parachute
    if (data.altitude <= PARACHUTE_ALTITUDE_THRESHOLD) {
      // Write prachute launch to servo
      goToState(PARACHUTE_DESCENT);
      landingDetectTime = millis();
    }

    break;

  case PARACHUTE_DESCENT:

    deployParachute();

    if (millis() - landingDetectTime > LANDING_DETECT_DELAY) {
      goToState(LANDED);
    }

    break;

  case ABORT:

    deployParachute();

    if (firstAbortLoop) {
      abortLoopTime = millis();
      firstAbortLoop = false;
    }

    if (millis() - abortLoopTime > ABORT_TO_LANDED_DELAY) {
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
    while (1) {
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
