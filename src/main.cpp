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
#include "Altimeter.h"

Servo yServo;
Servo zServo;

bool goMode = false;
bool writingMode = false;
bool finishedWriting = false;

Chrono writeTimer;
Chrono servoTimer;
Chrono pidTimer;
int servoFlag = 0;
int writeSecond = 0;



PID z_PID;
PID y_PID;

// Z YAW
// Y PITCH

#define SERVO_RANGE 20
#define TVC_TO_SERVO_SCALE 4
#define Y_CENTER 106
#define Y_MAX Y_CENTER + SERVO_RANGE
#define Y_MIN Y_CENTER - SERVO_RANGE


#define Z_CENTER 93
#define Z_MAX Z_CENTER + SERVO_RANGE
#define Z_MIN Z_CENTER - SERVO_RANGE

int y_max = Z_MAX;
int y_min = Z_MIN;

int z_val = 0;
int y_val = 0;



#define Y_KP 1.2
#define Y_KI 0.8
#define Y_KD 0.9

#define Z_KP 1.2
#define Z_KI 0.8
#define Z_KD 0.9

unsigned long passed = 0;
unsigned long t_ = 0;

void setup()
{
  Serial.println(115200);

  // while (!Serial)
  //   ;
  initBuzzer();
  data.state = 1;
  data.kp_y = Y_KP;
  data.ki_y = Y_KI;
  data.kd_y = Y_KD;
  data.kp_z = Z_KP;
  data.ki_z = Z_KI;
  data.kd_z = Z_KD;
  delay(5000);
  buzzStartup();
  initFlash();
  Serial.println("Flash Initialized");

  delay(100);
  Serial.println("Buzzer Initialized");

  Serial.print("Battery Voltage: ");
  Serial.println(getBattVoltage());

  Serial.println("Attaching servos to pins");
  yServo.attach(SERVO2_PIN);
  zServo.attach(SERVO1_PIN);

  Serial.println("Centering Servos. Wait 2 seconds..");
  yServo.write(Y_CENTER);
  zServo.write(Z_CENTER);

  delay(3000);

  Serial.println("Setting PID Tunings");
  z_PID.setTunings(Z_KP, Z_KI, Z_KD);
  z_PID.setOutputLimits(-SERVO_RANGE,SERVO_RANGE );
  z_PID.setSetpoint(0);

  y_PID.setTunings(Y_KP, Y_KI, Y_KD);
  y_PID.setOutputLimits(-SERVO_RANGE, SERVO_RANGE);
  y_PID.setSetpoint(30);


  initIMU();
  delay(1000);
  initAltimeter();
  
  Serial.println("IMU INITIALIZED");

  Serial.println("DONE SETUP");
  buzzStartup();
  delay(500);
  buzzStartup();
  delay(500);
  buzzStartup();

  delay(2000);
  buzzStartup();
  delay(200);
  buzzStartup();
  delay(1000);
  goMode = true;
  passed = millis();
  t_ = 0;
}

void loop()
{

  getYPR();

  data.p_err_y = y_PID.getPError();
  data.i_err_y = y_PID.getIError();
  data.d_err_y = y_PID.getDError();

  data.p_err_z = z_PID.getPError();
  data.i_err_z = z_PID.getIError();
  data.d_err_z = z_PID.getDError();
  data.servo_z = z_val;
  data.servo_y = y_val;

  
  yServo.write(y_val);
  zServo.write(z_val);
  
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
    t_ += (millis() - passed);
    passed = millis();
    writingMode = true;


    if(t_ > 10000){
      y_PID.setSetpoint(0);
    }
  
    if(pidTimer.hasPassed(10)){
      getAltitude();
      z_PID.setInput(data.pitch);
      y_PID.setInput(data.yaw);

      z_PID.compute();
      y_PID.compute();

      z_val = z_PID.Output;
      y_val = y_PID.Output;
      
    };
    

    zServo.write(Z_CENTER + z_val);
    yServo.write(Y_CENTER - y_val);
    
    

    

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

}