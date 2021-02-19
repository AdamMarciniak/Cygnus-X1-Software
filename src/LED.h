
#pragma once
#ifndef LED_H
#define LED_H

// Initialize LED
#include "Chrono.h"

void initLED();

// Turn LED on and set color
void turnOnLED();

void turnOffLED();

void setLEDRed(int val);

void setLEDColor(int red, int green, int blue);

void handleLEDBlink(int red, int green, int blue);

void handleFailLED();
#endif