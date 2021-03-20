#include <Arduino.h>
#include "Data.h"
#include "Nav.h"
#include "Chrono.h"
#include "./SdCard/SD.h"
#include "Data.h"
#include "Buzzer.h"
#include "Altimeter.h"
#include <Servo.h>


enum State {
  INITIALIZING,
  IDLE,
  LAUNCH_COMMANDED,
  POWERED_ASCENT,
  UNPOWERED_ASCENT,
  FREE_DESCENT,
  PARACHUTE_DESCENT,
  LANDED,
  
}