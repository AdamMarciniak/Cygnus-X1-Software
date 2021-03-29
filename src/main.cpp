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
#include "electricui.h"

#define PARACHUTE_ALTITUDE_THRESHOLD 30.0f // meters
#define ABORT_ANGLE_THRESHOLD 30.0f // degrees
#define ACCEL_UNPOWERED_THRESHOLD 2.0f //m/s^2
#define TELEMETRY_RATE 50 // ms
#define FIRE_ON_TIME 2000 // ms
#define FIRE_TO_PID_DELAY 500 //ms
#define POWERED_FLIGHT_SAFETY_TIME 4000 //ms
#define UNPOWERED_ASCENT_SAFETY_TIME_LIMIT 2000 // ms
#define FREE_DESCENT_SAFETY_TIME_LIMIT 1000  // ms

Chrono telemetryTimer;
Chrono EUITimer;


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
  EUI_FLOAT("yaw", data.yaw),
  EUI_FLOAT("pitch", data.pitch),
  EUI_FLOAT("roll", data.roll),
  EUI_FLOAT("baro", data.altitude),
  EUI_FLOAT("baroBias", data.biasAltitude),
  EUI_FLOAT("accelX", data.ax),
  EUI_FLOAT("accelY", data.ay),
  EUI_FLOAT("accelZ", data.az),
  EUI_FLOAT("worldAx", data.worldAx),
  EUI_FLOAT("worldVx", data.worldVx),
  EUI_FLOAT("loopTime", data.loopTime),
  EUI_UINT32("state", data.state),
};

void initEUI() {
  eui_setup_interface(&serial_comms);
  // Provide the tracked variables to the library
  EUI_TRACK(tracked_vars);
  // Provide a identifier to make this board easy to find in the UI
  eui_setup_identifier("hello", 5);
}

void sendEUIVars(){
  // eui_send_tracked("yaw");
  // eui_send_tracked("pitch");
  // eui_send_tracked("roll");
  // eui_send_tracked("baro");
  // eui_send_tracked("barobias");
  // eui_send_tracked("accelX");
  // eui_send_tracked("accelY");
  // eui_send_tracked("accelZ");
  // eui_send_tracked("worldAx");
  // eui_send_tracked("worldVx");
  // eui_send_tracked("loopTime");
  // eui_send_tracked("state");
}

void handleEUI(){
  serial_rx_handler();  //check for new inbound data
  if(EUITimer.hasPassed(16)){
      sendEUIVars();
      EUITimer.restart();
    }
}

int telemetryState = 0;

void handleSendTelemetry() {
   if(telemetryTimer.hasPassed(TELEMETRY_RATE)){

    char message[20];
    char buff[10]; // Buffer big enough for 7-character float

    switch(telemetryState){
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
      itoa(data.state, buff, 2);
      strcpy(message, "S");
      telemetryState += 1;
      break;
      case 4:
      dtostrf(data.altitude, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "B");
      telemetryState = 0;
      break;
    
    }
    strcat(message, ",");
    strcat(message, buff);
    sendTelemetry(message);
    telemetryTimer.restart();
  }
}

void initServos() {

  // Attach servos
  // Write centers to servos
}

void initStatusVariables(){
    nonLoggedData.servoCentersAvailable = false;
    nonLoggedData.zeroGyrosStatus = false;
}

unsigned long timer = 0;

void checkAngleThreshold() {
  if(abs(data.yaw) >= ABORT_ANGLE_THRESHOLD || abs(data.pitch) >= ABORT_ANGLE_THRESHOLD){
    Serial.println("Angle Past Limit");
    data.state = ABORT;
  }

}




void handleServoCentering() {
  if(nonLoggedData.servoCentersAvailable == true){
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
void setup(){
  data.state = INITIALIZING;
  Serial.begin(115200);
  initEUI();
  initStatusVariables();
   // Attach servos and center them 
  initServos();
  initBuzzer();
  delay(10);
  buzzStartup();
  initBluetooth();
  initFlash();
  initSD();

 // Initialize all sensors and stuff.
 // Put state into error if not all good.

  // Init IMU, Altimeter, Gyros, Buzzer,
  if(!initIMU()){
    Serial.println("Error init IMU");
    data.state = ERROR;
  };
  if(!initAltimeter()){
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
}




void loop() {
  currentLoopTime = micros();
  data.loopTime = float(currentLoopTime - prevLoopTime) / 1000000.0f;
  prevLoopTime = currentLoopTime;
  checkBTLE();
  getYPR();
  getAccel();
  getAltitude();
  handleEUI();



  if(PIDStatus == true){
    //compute PID and write to servos;
    // do this every 10 ms
  }

// Handle writing to flash
  if(flashWriteStatus == true){
      if (!handleWriteFlash())
      {
        flashWriteStatus = false;
        finishedWriting = true;
      }
  }
  

switch (data.state) {

  case IDLE:
    // Wait for BTLE Launch command
    // Send periodic data to BTLE
    data.state = IDLE;
    handleSendTelemetry();

    // wait for zero gyros command
    if(nonLoggedData.zeroGyrosStatus == true){
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
    if(gyroZeroStatus == false){
      gyroZeroStatus = true;
      zeroGyroscope();
    }

    if(firingStatus == false && fireTime == 0){
      //Fire pyro charge
      fireTime = millis();
      firingStatus = true;
      Serial.println("FIRE");
      // if gimbal test switch to POWERED_ASCENT NOW;
    }

    if(firingStatus == true && millis() - fireTime >= FIRE_ON_TIME){
      //Stop pyro charge
      Serial.println("FIRE OFF");
      firingStatus = false;
    }

    if(PIDStatus == false && millis() - fireTime >= FIRE_TO_PID_DELAY){
      // 
      PIDStatus = true;
    }
  
    if(data.worldAx > launchAccelThreshold){
      // stop pyro charge 
      poweredFlightSafetyTime = millis();
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
    if(accelMag < ACCEL_UNPOWERED_THRESHOLD){
      data.state = UNPOWERED_ASCENT;
    }

    if(millis() - poweredFlightSafetyTime >= POWERED_FLIGHT_SAFETY_TIME){
      unpoweredAscentSafetyTime = millis();
      data.state = UNPOWERED_ASCENT;
    }
    
    break;
  case UNPOWERED_ASCENT:
    // Center and turn off TVC
    PIDStatus = false;
    // Center Servos


    // Velocity X passes 0 or barometer detects apogee
    // or timer runs out
    if(millis() - unpoweredAscentSafetyTime >= UNPOWERED_ASCENT_SAFETY_TIME_LIMIT){
      freeDescentSafetyTime = millis();
      data.state = FREE_DESCENT;
    }

    break;
  case FREE_DESCENT:
    // Detect barometer min altitude for parachute
    if(data.altitude < PARACHUTE_ALTITUDE_THRESHOLD){
      // Write prachute launch to servo
      data.state = PARACHUTE_DESCENT;
    }

    // or timer runs out
    if(millis() - freeDescentSafetyTime >= FREE_DESCENT_SAFETY_TIME_LIMIT){
      // Write prachute launch to servo
      data.state = PARACHUTE_DESCENT;
    }

    break;

  case PARACHUTE_DESCENT:
  // wait for accel, magnitude threshold for gravity or something
  // to detect landing
   accelMag = sqrt(sq(data.ax) + sq(data.ay) + sq(data.az));

  if(accelMag > 9.3f && accelMag < 10.5f && landingWait == false){
    landingWait = true;
    landingTimer = millis();
    Serial.println("Waiting for Landing");
  } 

  if((accelMag < 9.3f || accelMag > 10.5f) && landingWait == true){
    landingWait = false;
     Serial.println("Landing timer reset");
  } 


  if(landingWait == true && millis() - landingTimer >= 4000){
     Serial.println("vehicle Landed");
    data.state = LANDED;
  }

  break;
  case LANDED:
  // wait a second
  // Dump all data to SD card
  sendEUIVars();
  delay(2000);
  // Dump all data to SD
  flashWriteStatus = false;
  Serial.println("Writing to SD");
  transferToSD();
  buzzComplete();
  Serial.println("SD writing complete");
  while (1){
    delay(500);
    buzzComplete();
  };
  
  while(1);

  break;

  case ERROR:
  // Send error message to BTLE
  sendEUIVars();
  flashWriteStatus = false;
  while(1);
  break;

  case ABORT:
    Serial.println("ABORTED");
    // Launch parachute
    sendEUIVars();
    delay(10000);
    // Dump all data to SD
    flashWriteStatus = false;
    Serial.println("Writing to SD");
    transferToSD();
    buzzComplete();
    Serial.println("SD writing complete");
    while (1){
      delay(500);
      buzzComplete();
    };
  

    while(1);
    break;
  default:
  break;
}



}