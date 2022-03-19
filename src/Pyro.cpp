#include "Pyro.h"

unsigned long fireTime1 = 0; // millis
unsigned long fireTime2 = 0; // millis
bool firingStatus1 = false;
bool firingStatus2 = false;

float analogVal;

Chrono continuityTimer;

void initPyro()
{
  if (ENGAGE_PYRO == true)
  {
    pinMode(PYRO1_PIN, OUTPUT);
    pinMode(PYRO2_PIN, OUTPUT);

    if (analogRead(PYRO1_DETECT_PIN) < 200 || analogRead(PYRO2_DETECT_PIN) < 200)
    {
      data.pyro1Continuity = 0.0;
    }
    else
    {
      data.pyro1Continuity = 1.0;
    }
  }
}

void handleGetContinuity()
{
  if (ENGAGE_PYRO == true)
  {
    if (continuityTimer.hasPassed(500))
    {

      if (analogRead(PYRO1_DETECT_PIN) < 200 || analogRead(PYRO2_DETECT_PIN) < 200)
      {
        data.pyro1Continuity = 0.0;
      }
      else
      {
        data.pyro1Continuity = 1.0;
      }

      continuityTimer.restart();
    }
  }
}

void handleFirePyro1()
{
  if (ENGAGE_PYRO == true)
  {
    if (firingStatus1 == false && fireTime1 == 0)
    {
      //Fire pyro charge
      fireTime1 = millis();
      firingStatus1 = true;
      analogWrite(PYRO1_PIN, 255);
    }

    if (firingStatus1 == true && millis() - fireTime1 >= FIRE_ON_TIME)
    {
      //Stop pyro charge
      analogWrite(PYRO1_PIN, 0);
      firingStatus1 = false;
    }
  }
}

void handleFirePyro2()
{
  if (ENGAGE_PYRO == true)
  {
    if (firingStatus2 == false && fireTime2 == 0)
    {
      //Fire pyro charge
      fireTime2 = millis();
      firingStatus2 = true;
      analogWrite(PYRO2_PIN, 255);
    }

    if (firingStatus2 == true && millis() - fireTime2 >= FIRE_ON_TIME)
    {
      //Stop pyro charge
      analogWrite(PYRO2_PIN, 0);
      firingStatus2 = false;
    }
  }
}

void stopPyro1()
{
  analogWrite(PYRO1_PIN, 0);
}

void stopPyro2()
{
  analogWrite(PYRO2_PIN, 0);
}