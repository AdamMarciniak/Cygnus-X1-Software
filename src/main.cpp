#include <Arduino.h>
#include "Data.h"
#include "Nav.h"
#include "Chrono.h"
#include "./SdCard/SD.h"
#include "Data.h"
#include "Buzzer.h"
#include "Altimeter.h"



bool writingMode = false;
bool finishedWriting = false;

Chrono writeTimer;
Chrono dataTimer;
int servoFlag = 0;
int writeSecond = 0;



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
  
}

float alt = 0;

void loop()
{

    writingMode = false;

    getAltitude();

    if(dataTimer.hasPassed(10)){
      getYPR();
      getAccel();
      alt = getAltitude();

      // Serial.print(data.worldAx);
      // Serial.print(" ");
      // Serial.print(data.worldAy);
      // Serial.print(" ");
      // Serial.println(data.worldAz);
      // Serial.print(" ");
      // Serial.print(data.roll);
      // Serial.print(" ");
      // Serial.print(data.pitch);
      // Serial.print(" ");
      // Serial.print(data.yaw);
      // Serial.print(" ");
      // Serial.print(data.altitude);
      // Serial.print(" ");
      // Serial.print(data.biasAltitude);
      // Serial.println(" ");

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
    transferToSD();
    buzzComplete();
    Serial.println("SD writing complete");
    while (1)
      ;
  }

}