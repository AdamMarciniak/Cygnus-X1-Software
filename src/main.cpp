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
#include <avr/dtostrf.h>
#include "Parachute.h"

#define TELEMETRY_RATE 50     // ms
#define FIRE_ON_TIME 1000     // ms
#define FIRE_TO_PID_DELAY 500 //ms

Servo yServo;
Servo zServo;

bool goMode = false;
bool writingMode = false;
bool finishedWriting = false;

Chrono writeTimer;
Chrono servoTimer;
Chrono pidTimer;
Chrono telemetryTimer;
Chrono printTimer;

int telemetryState = 0;

int servoFlag = 0;
int writeSecond = 0;

PID z_PID;
PID y_PID;

// Z YAW
// Y PITCH

#define SERVO_RANGE 20
#define TVC_TO_SERVO_SCALE 4
#define Y_CENTER 91
#define Y_MAX Y_CENTER + SERVO_RANGE
#define Y_MIN Y_CENTER - SERVO_RANGE

#define Z_CENTER 102
#define Z_MAX Z_CENTER + SERVO_RANGE
#define Z_MIN Z_CENTER - SERVO_RANGE

int y_max = Z_MAX;
int y_min = Z_MIN;

float z_val = 0;
float y_val = 0;

// #define Y_KP 0.9
// #define Y_KI 0.2
// #define Y_KD 0.5

// #define Z_KP 0.9
// #define Z_KI 0.2
// #define Z_KD 0.5
#define Y_KP 1.5
#define Y_KI 0.9
#define Y_KD 0.8

#define Z_KP 1.5
#define Z_KI 0.9
#define Z_KD 0.8

unsigned long passed = 0;
unsigned long t_ = 0;

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
      telemetryState += 1;
      break;
    case 11:
      dtostrf(data.Y_Servo_Center, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Sy");
      telemetryState += 1;
      break;
    case 12:
      dtostrf(data.Z_Servo_Center, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Sz");
      telemetryState = 0;
      break;
    }
    strcat(message, ",");
    strcat(message, buff);
    sendTelemetry(message);
    telemetryTimer.restart();
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
    yServo.write(data.Y_Servo_Center);
    zServo.write(data.Z_Servo_Center);
    writeTVCCenters();

    // Write new center values to servos
  }
}

void initServos()
{
  Serial.println("Attaching servos to pins");
  yServo.attach(SERVO1_PIN);
  zServo.attach(SERVO2_PIN);

  readTVCCenters();
  Serial.println("Centering Servos. Wait 2 seconds..");
  yServo.write(data.Y_Servo_Center);
  zServo.write(data.Z_Servo_Center);
}

void initPyro()
{
  pinMode(PYRO2_PIN, OUTPUT);

  // Check if pyro has continuity
  if (analogRead(PYRO2_DETECT_PIN) < 200)
  {
    while (1)
    {
      buzzerError();
      delay(200);
    }
  }
}

unsigned long fireTime = 0; // millis
bool firingStatus = false;

void handleFirePyro()
{

  if (firingStatus == false && fireTime == 0)
  {
    //Fire pyro charge
    fireTime = millis();
    firingStatus = true;
    Serial.println("FIRE");
    analogWrite(PYRO2_PIN, 255);
  }

  if (firingStatus == true && millis() - fireTime >= FIRE_ON_TIME)
  {
    //Stop pyro charge
    Serial.println("FIRE OFF");
    analogWrite(PYRO2_PIN, 0);
    firingStatus = false;
  }
}

void initExtraData()
{
  data.kp_y = Y_KP;
  data.ki_y = Y_KI;
  data.kd_y = Y_KD;
  data.kp_z = Z_KP;
  data.ki_z = Z_KI;
  data.kd_z = Z_KD;
}

void initPID()
{
  z_PID.setTunings(Z_KP, Z_KI, Z_KD);
  z_PID.setOutputLimits(-SERVO_RANGE, SERVO_RANGE);
  z_PID.setSetpoint(0);

  y_PID.setTunings(Y_KP, Y_KI, Y_KD);
  y_PID.setOutputLimits(-SERVO_RANGE, SERVO_RANGE);
  y_PID.setSetpoint(0);
}

void checkBatteryVoltage()
{
  if (getBattVoltage() < 11.5f)
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

  initPID();
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

  if (pidTimer.hasPassed(10))
  {
    //getAltitude();
    z_PID.setInput(data.pitch);
    y_PID.setInput(data.yaw);

    z_PID.compute();
    y_PID.compute();

    data.p_err_y = y_PID.getPError();
    data.p_err_z = z_PID.getPError();

    data.i_err_y = y_PID.getIError();
    data.i_err_z = z_PID.getIError();

    data.d_err_y = y_PID.getDError();
    data.d_err_z = z_PID.getDError();

    z_val = z_PID.getOutput();
    y_val = y_PID.getOutput();
    data.servo_z = z_val;
    data.servo_y = y_val;
    zServo.write(int(round(data.Z_Servo_Center + z_val)));
    yServo.write(int(round(data.Y_Servo_Center + y_val)));
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
  handleServoCentering();

  if (data.state == LAUNCH_COMMANDED)
  {
    t_ += (millis() - passed);
    passed = millis();

    if (t_ > 10000)
    {
      //y_PID.setSetpoint(0);
    }

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