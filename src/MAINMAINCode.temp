#include <Arduino.h>
#include "Data.h"
#include "Nav.h"
#include "Chrono.h"
#include "./SdCard/SD.h"
#include "Data.h"
#include "Buzzer.h"
#include "Altimeter.h"
#include <Servo.h>
#include "BTLE.h"
#include "LED.h"
#include "States.h"
#include <avr/dtostrf.h>
#include "Kalman.h"
#include "Parachute.h"

#define PARACHUTE_ALTITUDE_THRESHOLD 40.0f      // meters
#define ABORT_ANGLE_THRESHOLD 30.0f             // degrees
#define ACCEL_UNPOWERED_THRESHOLD 2.0f          //m/s^2
#define TELEMETRY_RATE 50                       // ms
#define FIRE_ON_TIME 1000                       // ms
#define FIRE_TO_PID_DELAY 500                   //ms
#define POWERED_FLIGHT_SAFETY_TIME 3800         //ms
#define UNPOWERED_ASCENT_SAFETY_TIME_LIMIT 3000 // ms
#define FREE_DESCENT_SAFETY_TIME_LIMIT 3000     // ms

Chrono telemetryTimer;

float accelMag = 0;
bool flashWriteStatus = false;
unsigned long fireTime = 0; // millis
bool firingStatus = false;
float launchAccelThreshold = 13; // m/s^2
bool PIDStatus = false;
bool gyroZeroStatus = false;
unsigned long landingTimer = 0;
bool landingWait = false;

unsigned long poweredFlightSafetyTime = 0;
unsigned long unpoweredAscentSafetyTime = 0;
unsigned long freeDescentSafetyTime = 0;

int telemetryState = 0;

void handleSendTelemetry()
{
  if (telemetryTimer.hasPassed(TELEMETRY_RATE))
  {

    char message[20];
    char buff[10]; // Buffer big enough for 7-character float

    switch (telemetryState)
    {
    case 0:
      dtostrf(data.yaw, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Ox");
      telemetryState += 1;
      break;
    case 1:
      dtostrf(data.pitch, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Oy");
      telemetryState += 1;
      break;
    case 2:
      dtostrf(data.roll, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Oz");
      telemetryState += 1;
      break;
    case 3:
      sprintf(buff, "%d", data.state);
      strcpy(message, "S");
      telemetryState += 1;
      break;
    case 4:
      dtostrf(data.altitude, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "B");
      telemetryState += 1;
      break;
    case 5:
      dtostrf(data.ax, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Ax");
      telemetryState += 1;
      break;
    case 6:
      dtostrf(data.ay, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Ay");
      telemetryState += 1;
      break;
    case 7:
      dtostrf(data.az, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Az");
      telemetryState += 1;
      break;
    case 8:
      dtostrf(data.gx, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Gx");
      telemetryState += 1;
      break;
    case 9:
      dtostrf(data.gy, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Gy");
      telemetryState += 1;
      break;
    case 10:
      dtostrf(data.gz, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Gz");
      telemetryState = 0;
      break;
    }
    strcat(message, ",");
    strcat(message, buff);
    sendTelemetry(message);
    telemetryTimer.restart();
  }
}

void initServos()
{

  // Attach servos
  // Write centers to servos
}

void initStatusVariables()
{
  nonLoggedData.servoCentersAvailable = false;
  nonLoggedData.zeroGyrosStatus = false;
}

unsigned long timer = 0;

void checkAngleThreshold()
{
  if (abs(data.yaw) >= ABORT_ANGLE_THRESHOLD || abs(data.pitch) >= ABORT_ANGLE_THRESHOLD)
  {
    Serial.println("Angle Past Limit");
    data.state = PARACHUTE_DESCENT;
  }
}

void handleServoCentering()
{
  if (nonLoggedData.servoCentersAvailable == true)
  {
    nonLoggedData.servoCentersAvailable = false;
    Serial.print("Writing To servos");
    Serial.print("   Y: ");
    Serial.print(data.Y_Servo_Center);
    Serial.print("   Z: ");
    Serial.println(data.Z_Servo_Center);
    // Write new center values to servos
  }
}

bool writingMode = false;
bool finishedWriting = false;
unsigned long prevLoopTime;
unsigned long currentLoopTime;

void setup()
{
  data.state = INITIALIZING;
  Serial.begin(115200);
  initStatusVariables();
  // Attach servos and center them
  initServos();
  initBuzzer();
  delay(10);
  buzzStartup();
  initBluetooth();
  initFlash();
  initSD();
  pinMode(PYRO2_PIN, OUTPUT);

  // Initialize all sensors and stuff.
  // Put state into error if not all good.

  // Init IMU, Altimeter, Gyros, Buzzer,
  if (!initIMU())
  {
    Serial.println("Error init IMU");
    data.state = ERROR;
  };
  if (!initAltimeter())
  {
    Serial.println("Error init altimeter");
    data.state = ERROR;
  };

  // Initialize PID classes
  delay(1000);
  // Put state into IDLE when finished if all good.
  Serial.println("Setup Success. Entering Loop");
  prevLoopTime = micros();
  currentLoopTime = micros();
  data.state = IDLE;

  if (analogRead(PYRO2_DETECT_PIN) < 500)
  {
    while (1)
    {
      buzzerError();
      delay(500);
    }
  }
}

void loop()
{
  currentLoopTime = micros();
  data.loopTime = float(currentLoopTime - prevLoopTime) / 1000000.0f;
  prevLoopTime = currentLoopTime;
  checkBTLE();
  getYPR();
  getAccel();
  getAltitude();

  if (PIDStatus == true)
  {
    //compute PID and write to servos;
    // do this every 10 ms
  }

  // Handle writing to flash
  if (flashWriteStatus == true)
  {
    if (!handleWriteFlash())
    {
      flashWriteStatus = false;
      finishedWriting = true;
    }
  }

  handleSendTelemetry();
  switch (data.state)
  {

  case IDLE:
    // Wait for BTLE Launch command
    // Send periodic data to BTLE
    data.state = IDLE;

    // wait for zero gyros command
    if (nonLoggedData.zeroGyrosStatus == true)
    {
      nonLoggedData.zeroGyrosStatus = false;
      zeroGyroscope();
    }

    // listen to BTLE for TVC centering commands if needed. Update accordingly
    handleServoCentering();
    // if rocket goes to > angleThresh => abort
    checkAngleThreshold();

    break;

  case LAUNCH_COMMANDED:
    flashWriteStatus = true;
    // Zero Gyros and other sensors as needed
    if (gyroZeroStatus == false)
    {
      gyroZeroStatus = true;
      zeroGyroscope();
    }

    if (firingStatus == false && fireTime == 0)
    {
      //Fire pyro charge
      fireTime = millis();
      firingStatus = true;
      Serial.println("FIRE");
      analogWrite(PYRO2_PIN, 255);
      // if gimbal test switch to POWERED_ASCENT NOW;
    }

    if (firingStatus == true && millis() - fireTime >= FIRE_ON_TIME)
    {
      //Stop pyro charge
      Serial.println("FIRE OFF");
      analogWrite(PYRO2_PIN, 0);
      firingStatus = false;
    }

    if (PIDStatus == false && millis() - fireTime >= FIRE_TO_PID_DELAY)
    {
      //
      analogWrite(PYRO2_PIN, 0);
      PIDStatus = true;
    }

    if (data.worldAx > launchAccelThreshold)
    {
      // stop pyro charge
      poweredFlightSafetyTime = millis();
      analogWrite(PYRO2_PIN, 0);
      data.state = POWERED_ASCENT;
    }
    // Or wait 0.25 sec if gimbal test

    // If more than angle thresh => abort.
    checkAngleThreshold();

    break;
  case POWERED_ASCENT:
    // Turn on TVC
    // or check if 4 seconds passed.
    // If more than angle thresh => abort.
    checkAngleThreshold();

    // Check if accel magnitude is less than thresh
    accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));
    if (accelMag < ACCEL_UNPOWERED_THRESHOLD)
    {
      data.state = UNPOWERED_ASCENT;
    }

    if (millis() - poweredFlightSafetyTime >= POWERED_FLIGHT_SAFETY_TIME)
    {
      unpoweredAscentSafetyTime = millis();
      data.state = UNPOWERED_ASCENT;
    }

    break;
  case UNPOWERED_ASCENT:
    // Center and turn off TVC
    PIDStatus = false;
    // Center Servos

    if (data.altitude < PARACHUTE_ALTITUDE_THRESHOLD)
    {
      // Write prachute launch to servo
      data.state = PARACHUTE_DESCENT;
    }

    // Velocity X passes 0 or barometer detects apogee
    // or timer runs out
    if (millis() - unpoweredAscentSafetyTime >= UNPOWERED_ASCENT_SAFETY_TIME_LIMIT)
    {
      freeDescentSafetyTime = millis();
      data.state = FREE_DESCENT;
    }

    break;
  case FREE_DESCENT:
    // Detect barometer min altitude for parachute
    if (data.altitude < PARACHUTE_ALTITUDE_THRESHOLD)
    {
      // Write prachute launch to servo
      data.state = PARACHUTE_DESCENT;
    }

    // or timer runs out
    if (millis() - freeDescentSafetyTime >= FREE_DESCENT_SAFETY_TIME_LIMIT)
    {
      // Write prachute launch to servo
      data.state = PARACHUTE_DESCENT;
    }

    break;

  case PARACHUTE_DESCENT:
    // wait for accel, magnitude threshold for gravity or something
    // to detect landing
    deployParachute();
    accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));

    if (accelMag > 9.3f && accelMag < 10.5f && landingWait == false)
    {
      landingWait = true;
      landingTimer = millis();
      Serial.println("Waiting for Landing");
    }

    if ((accelMag < 9.3f || accelMag > 10.5f) && landingWait == true)
    {
      landingWait = false;
      Serial.println("Landing timer reset");
    }

    if (landingWait == true && millis() - landingTimer >= 4000)
    {
      Serial.println("vehicle Landed");
      data.state = LANDED;
    }

    break;
  case LANDED:
    // wait a second
    // Dump all data to SD card
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