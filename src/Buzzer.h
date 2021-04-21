#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>
#include "Chrono.h"

void initBuzzer();
void buzzStartup();
void buzzerError();
void buzzComplete();
void buzz1();
void buzzLongs();
#endif