#include "Telemetry.h"

#include <avr/dtostrf.h>
#include "BTLE.h"
#include "Data.h"
#include "Chrono.h"

#define TELEMETRY_RATE 50     // ms


int telemetryState = 0;

Chrono telemetryTimer;

void handleSendTelemetry()
{
  if (telemetryTimer.hasPassed(TELEMETRY_RATE))
  {

    char message[20];
    char buff[10]; // Buffer big enough for 7-character float

    switch (telemetryState)
    {
    case 0:
      dtostrf(data.yaw, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Ox");
      telemetryState += 1;
      break;
    case 1:
      dtostrf(data.pitch, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Oy");
      telemetryState += 1;
      break;
    case 2:
      dtostrf(data.roll, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Oz");
      telemetryState += 1;
      break;
    case 3:
      sprintf(buff, "%d", data.state);
      strcpy(message, "S");
      telemetryState += 1;
      break;
    case 4:
      dtostrf(data.altitude, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "B");
      telemetryState += 1;
      break;
    case 5:
      dtostrf(data.ax, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Ax");
      telemetryState += 1;
      break;
    case 6:
      dtostrf(data.ay, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Ay");
      telemetryState += 1;
      break;
    case 7:
      dtostrf(data.az, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Az");
      telemetryState += 1;
      break;
    case 8:
      dtostrf(data.gx, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Gx");
      telemetryState += 1;
      break;
    case 9:
      dtostrf(data.gy, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Gy");
      telemetryState += 1;
      break;
    case 10:
      dtostrf(data.gz, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Gz");
      telemetryState += 1;
      break;
    case 11:
      dtostrf(data.Y_Servo_Center, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Sy");
      telemetryState += 1;
      break;
    case 12:
      dtostrf(data.Z_Servo_Center, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "Sz");
      telemetryState = 0;
      break;
    }
    strcat(message, ",");
    strcat(message, buff);
    sendTelemetry(message);
    telemetryTimer.restart();
  }
}
