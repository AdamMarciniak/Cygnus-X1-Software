#include "Telemetry.h"

int telemetryState = 0;

Chrono telemetryTimer;
Chrono stateTimer;

void handleSendState()
{

  if (stateTimer.hasPassed(500))
  {

    char message[20];
    char buff[10]; // Buffer big enough for 7-character float

    sprintf(buff, "%d", data.state);
    strcpy(message, "S");

    strcat(message, ",");
    strcat(message, buff);
    sendTelemetry(message);
    stateTimer.restart();
  }
}

void handleSendTelemetry()
{

  // if (data.state != IDLE || data.state != TEST)
  // {
  //   handleSendState();
  // }
  // else
  // {

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
      telemetryState += 1;
      break;
    case 13:
      dtostrf(data.pyro1Continuity, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "P");
      telemetryState += 1;
      break;
    case 14:
      dtostrf(data.kal_X_pos, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "KX");
      telemetryState += 1;
      break;
    case 15:
      dtostrf(data.kal_X_vel, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "KV");
      telemetryState += 1;
      break;
    case 16:
      dtostrf(data.hdop, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "HD");
      telemetryState += 1;
      break;

    case 17:
      dtostrf(data.sats, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "ST");
      telemetryState += 1;
      break;
    case 18:
      dtostrf(data.gpsAltitude, 6, 2, buff); // Leave room for too large numbers!
      strcpy(message, "GA");
      telemetryState = 0;
      break;
    }
    strcat(message, ",");
    strcat(message, buff);
    sendTelemetry(message);
    telemetryTimer.restart();
  }
  //}
}
