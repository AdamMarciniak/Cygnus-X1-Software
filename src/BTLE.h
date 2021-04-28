#pragma once

#include "Data.h"
#include "States.h"

void initBluetooth();
void checkBTLE();
void sendTelemetry(char message[]);
