#include <Arduino.h>
#include "Buzzer.h"
#include "Chrono.h"

Chrono buzzOnTimer;
Chrono buzzOffTimer;

bool buzzerState = false;

void initBuzzer()
{
  pinMode(BUZZER_PIN, OUTPUT);
  buzzStartup();
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