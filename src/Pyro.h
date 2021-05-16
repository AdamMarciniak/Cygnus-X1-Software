#pragma once

#include <Arduino.h>
#include "Buzzer.h"
#include "Config.h"
#include "Data.h"

void initPyro();
void handleFirePyro();
void handleGetContinuity();
void stopPyros();