#include "Buzzer.h"

Chrono buzzOnTimer;
Chrono buzzOffTimer;

bool buzzerState = false;

void initBuzzer()
{
  pinMode(BUZZER_PIN, OUTPUT);
  buzzStartup();
}

void buzzOff()
{
  buzzerState = false;
  analogWrite(BUZZER_PIN, 0);
}

void handleBuzzer()
{
  if (buzzOnTimer.hasPassed(1000))
  {
    if (buzzerState == false)
    {
      buzzerState = true;
      analogWrite(BUZZER_PIN, 150);
    }

    else if (buzzerState == true)
    {
      buzzerState = false;
      analogWrite(BUZZER_PIN, 0);
    }

    buzzOnTimer.restart();
  }
}

void buzzStartup()
{
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
  delay(50);
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
}

void buzzLongs()
{
  analogWrite(BUZZER_PIN, 150);
  delay(1000);
  analogWrite(BUZZER_PIN, 0);
  delay(50);
  analogWrite(BUZZER_PIN, 150);
  delay(1000);
  analogWrite(BUZZER_PIN, 0);
}

void buzz1()
{
  analogWrite(BUZZER_PIN, 150);
  delay(10);
  analogWrite(BUZZER_PIN, 0);
  delay(10);
  analogWrite(BUZZER_PIN, 150);
  delay(10);
  analogWrite(BUZZER_PIN, 0);
}

void buzzComplete()
{
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
  delay(50);
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
  delay(50);
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
}

void buzzerError()
{
  if (buzzOnTimer.hasPassed(1000))
  {
    if (buzzerState == false)
    {
      buzzerState = true;
      analogWrite(BUZZER_PIN, 150);
    }

    else if (buzzerState == true)
    {
      buzzerState = false;
      analogWrite(BUZZER_PIN, 0);
    }

    buzzOnTimer.restart();
  }
}

void buzzFast()
{
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
  delay(50);
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
  delay(400);
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
  delay(50);
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
  delay(400);
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
  delay(50);
  analogWrite(BUZZER_PIN, 150);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
}

void buzzMaxAltitude(float maxAltitude)
{

  int tensBeeps = (int)maxAltitude / 10;
  int onesBeeps = (int)maxAltitude % 10;

  while (1)
  {
    for (int i = 0; i < tensBeeps; i += 1)
    {
      analogWrite(BUZZER_PIN, 150);
      delay(200);
      analogWrite(BUZZER_PIN, 0);
      delay(400);
    }

    delay(2000);

    for (int i = 0; i < onesBeeps; i += 1)
    {
      analogWrite(BUZZER_PIN, 150);
      delay(200);
      analogWrite(BUZZER_PIN, 0);
      delay(400);
    }

    delay(2000);

    buzzComplete();
  }
}