#pragma once

#include <Arduino.h>
#include "Buzzer.h"
#include "Config.h"
#include "Data.h"

void initPyro();
void handleFirePyro1();
void handleFirePyro2();
void handleGetContinuity();
void stopPyro1();
void stopPyro2();