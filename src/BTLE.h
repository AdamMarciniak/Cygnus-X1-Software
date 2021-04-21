#ifndef BTLE_H
#define BTLE_H

#include "Data.h"
#include "States.h"

void initBluetooth();
void checkBTLE();
void sendTelemetry(char message[]);

#endif