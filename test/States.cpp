#include "Arduino.h"
#include "States.h"

RocketState currentState = CONFIG;

void setState(RocketState newState)
{
  currentState = newState;
}
