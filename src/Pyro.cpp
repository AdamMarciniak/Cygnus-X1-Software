#include "Pyro.h"
#include <Arduino.h>
#include "Buzzer.h"

#define PYRO_CONTINUITY_THRESHOLD 200;
#define FIRE_ON_TIME 1000     // ms


unsigned long fireTime = 0; // millis
bool firingStatus = false;
float analogVal;

void initPyro()
{
  pinMode(PYRO2_PIN, OUTPUT);
  // Check if pyro has continuity
  // if (analogRead(PYRO2_DETECT_PIN) < 200)
  // {
  //   while (1)
  //   {
  //     buzzerError();
  //     delay(200);
  //   }
  // }
  Serial.println("Pyro Continuity Good");
}

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