#include "Pyro.h"
#include "Config.h"

unsigned long fireTime = 0; // millis
bool firingStatus = false;
float analogVal;

void initPyro()
{
  if (ENGAGE_PYRO == true)
  {
    pinMode(PYRO2_PIN, OUTPUT);
    if (analogRead(PYRO2_DETECT_PIN) < 200)
    {
      while (1)
      {
        buzzerError();
        delay(200);
      }
    }
    Serial.println("Pyro Continuity Good");
  }
}

void handleFirePyro()
{
  if (ENGAGE_PYRO == true)
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
}