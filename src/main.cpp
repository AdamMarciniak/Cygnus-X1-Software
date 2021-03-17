#include <Arduino.h>
#include "Data.h"
#include "Nav.h"
#include "Chrono.h"
#include "./SdCard/SD.h"
#include "Data.h"
#include "Buzzer.h"
#include "Altimeter.h"
#include <Servo.h>

#define PARACHUTE_SERVO_DEPLOY 53
#define PARACHUTE_SERVO_INIT 95

bool writingMode = false;
bool finishedWriting = false;

Chrono writeTimer;
Chrono dataTimer;
Chrono gravityTimer;
int servoFlag = 0;
int writeSecond = 0;

float alt = 0;
boolean flight = false;
float aveAccel = 0;
unsigned long ind = 0;

Servo parachuteServo;

void setup()
{
  Serial.println(115200);

  initBuzzer();
 
  delay(3000);
  buzzStartup();
  initFlash();
  Serial.println("Flash Initialized");


 
  initIMU();
  delay(10);
  initAltimeter();
  delay(10);
  buzzStartup();
  delay(100);
  buzzStartup();
  delay(100);
  buzzStartup();
  delay(100);


  while(ind < 200){
  if(gravityTimer.hasPassed(10)){
      getAccel();
      getYPR();
      
      aveAccel += data.worldAx / 200;
      Serial.print(data.worldAx);
      Serial.print(" ");
      Serial.println(aveAccel);
      ind += 1;
      gravityTimer.restart();
    }
  }
  parachuteServo.attach(SERVO3_PIN);
  parachuteServo.write(PARACHUTE_SERVO_INIT);
  delay(10000);
  buzzLongs();


}


void loop()
{

    writingMode = true;

    getAltitude();


    if(data.worldVx > 2.0 && flight == false){
      flight = true;
    }


    if(flight == true){
      if(data.worldVx <= 0){
        buzzStartup();
        parachuteServo.write(PARACHUTE_SERVO_DEPLOY);
        flight = false;
      }
    }


    if(dataTimer.hasPassed(10)){
      getYPR();
      getAccel();
      getAltitude();

    
      data.worldVx += (data.worldAx - aveAccel) * 0.010;

    };

  
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
  

  if (finishedWriting)
  {
    Serial.println("Writing to SD");
    buzz1();
    transferToSD();
    buzzComplete();
    Serial.println("SD writing complete");
    Serial.println(aveAccel, 8);
    while (1)
      ;
  }

}