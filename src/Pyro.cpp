#include "Pyro.h"

unsigned long fireTime = 0; // millis
bool firingStatus = false;
float analogVal;

Chrono continuityTimer;

void initPyro() {
  if (ENGAGE_PYRO == true) {
    pinMode(PYRO2_PIN, OUTPUT);
    if (analogRead(PYRO2_DETECT_PIN) < 50) {
      data.pyro1Continuity = 0.0;
    } else {
      data.pyro1Continuity = 1.0;
    }
  }
}

void handleGetContinuity() {
  if (ENGAGE_PYRO == true) {
    if (continuityTimer.hasPassed(1000)) {
      if (analogRead(PYRO2_DETECT_PIN) < 50) {
        data.pyro1Continuity = 0.0;
      } else {
        data.pyro1Continuity = 1.0;
      }
      continuityTimer.restart();
    }
  }
}

void handleFirePyro() {
  if (ENGAGE_PYRO == true) {
    if (firingStatus == false && fireTime == 0) {
      // Fire pyro charge
      fireTime = millis();
      firingStatus = true;
      analogWrite(PYRO2_PIN, 255);
    }

    if (firingStatus == true && millis() - fireTime >= FIRE_ON_TIME) {
      // Stop pyro charge
      analogWrite(PYRO2_PIN, 0);
      firingStatus = false;
    }
  }
}

void stopPyros() {

  analogWrite(PYRO1_PIN, 0);
  analogWrite(PYRO2_PIN, 0);
}