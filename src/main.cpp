#include <Arduino.h>
#include "Data.h"
#include "Nav.h"
#include "Chrono.h"
#include "./SdCard/SD.h"
#include "Data.h"
#include "Buzzer.h"
#include "Altimeter.h"
#include <Servo.h>
#include "electricui.h"
#include "BTLE.h"


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
float measuredGravity = 0;
unsigned long ind = 0;
float gravMag = 0;
Servo parachuteServo;

float pos = 0;
float prevPos = 0;
float compPos = 0;
float prevCompPos = 0;
float compVel = 0;



void serial_write( uint8_t *data, uint16_t len )
{
  Serial.write( data, len ); //output on the main serial port
}



eui_interface_t serial_comms = EUI_INTERFACE(&serial_write);

void serial_rx_handler()
{
  // While we have data, we will pass those bytes to the ElectricUI parser
  while( Serial.available() > 0 )  
  {  
    eui_parse( Serial.read(), &serial_comms );  // Ingest a byte
  }
}



eui_message_t tracked_vars[] =
{
  EUI_FLOAT(  "worldVx",  data.worldVx ),
  EUI_FLOAT(  "worldAx",  data.worldAx ),
  EUI_FLOAT(  "baro",  data.altitude ),
  EUI_FLOAT(  "pos",  pos ),
  EUI_FLOAT(  "compPos",  compPos ),
  EUI_FLOAT(  "compVel",  compVel ),
};


void setup()
{
  Serial.println(115200);

  initBuzzer();
  data.btleCmd = 0;
 
  delay(3000);
  initBluetooth();

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
      measuredGravity += data.worldAx / 200;
      Serial.print(data.worldAx);
      Serial.print(" ");
      Serial.println(measuredGravity);
      ind += 1;
      gravityTimer.restart();
    }
  }
  parachuteServo.attach(SERVO3_PIN);
  //parachuteServo.write(PARACHUTE_SERVO_INIT);
  delay(10000);
  buzzLongs();

  eui_setup_interface(&serial_comms);

  // Provide the tracked variables to the library
  EUI_TRACK(tracked_vars);

  // Provide a identifier to make this board easy to find in the UI
  eui_setup_identifier("hello", 5);


}




void loop()
{
  checkBTLE();
    serial_rx_handler();  //check for new inbound data

    

    if(data.btleCmd == 1){
      data.worldAx = 0;
      data.worldAy = 0;
      data.worldAz = 0;
      data.worldVx = 0;
      pos = 0;
      compPos = 0;
      data.biasAltitude = data.altitude;
      compVel = 0;
      prevPos = 0;
      prevCompPos = 0;
      data.btleCmd = 0;
    }

    writingMode = false;

    getAltitude();


    if(data.worldVx > 2.0 && flight == false){
      flight = true;
    }


    if(flight == true){
      if(data.worldVx <= 0.2){
        buzzStartup();
        parachuteServo.write(PARACHUTE_SERVO_DEPLOY);
        flight = false;
      }
    }


    if(dataTimer.hasPassed(10)){
      getYPR();
      getAccel();
      getAltitude();


      data.worldVx += (data.worldAx - 9.74) * 0.010;
      pos = prevPos + data.worldVx * 0.010;

      compPos = 0.01 * pos + 0.99 * data.altitude;
      compVel = (compPos - prevCompPos) / 0.010;

      prevPos = pos;
      prevCompPos = compPos;
    

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
    Serial.println(measuredGravity, 8);
    while (1)
      ;
  }

}