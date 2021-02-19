#include <Arduino.h>
#include "Data.h"
#include "Nav.h"
#include "electricui.h"

#include "Chrono.h"

Chrono timer;

float gyros[3];

eui_message_t tracked_vars[] =
    {
        EUI_FLOAT_ARRAY("g", gyros),

};

void serial_write(uint8_t *data, uint16_t len)
{
  Serial.write(data, len); //output on the main serial port
}

eui_interface_t serial_comms = EUI_INTERFACE(&serial_write);

void serial_rx_handler()
{
  // While we have data, we will pass those bytes to the ElectricUI parser
  while (Serial.available() > 0)
  {
    eui_parse(Serial.read(), &serial_comms); // Ingest a byte
  }
}

void setup()
{
  Serial.begin(115200);
  initIMU();

  eui_setup_interface(&serial_comms);

  // Provide the tracked variables to the library
  EUI_TRACK(tracked_vars);

  // Provide a identifier to make this board easy to find in the UI
  eui_setup_identifier("hello", 5);
}

void loop()
{
  getYPR();
  gyros[0] = data.pitch * DEG_TO_RAD;
  gyros[1] = data.roll * DEG_TO_RAD;
  gyros[2] = data.yaw * DEG_TO_RAD;

  if (timer.hasPassed(16))
  {
    eui_send_tracked("g"); // send the new value to the UI
    serial_rx_handler();
    timer.restart();
  }
}