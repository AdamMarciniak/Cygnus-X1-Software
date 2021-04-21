#include "EUIMyLib.h"

Chrono EUITimer;

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

eui_message_t tracked_vars[] =
    {
        EUI_FLOAT("baro", data.altitude),
        EUI_FLOAT("worldAx", data.worldAx),
        EUI_FLOAT("kal_X", data.kal_X),
        EUI_FLOAT("kal_V", data.kal_V),
        EUI_FLOAT("kal_A", data.kal_A),
        EUI_FLOAT("kal_XP", data.kal_XP),
        EUI_FLOAT("kal_VP", data.kal_VP),
        EUI_FLOAT("kal_AP", data.kal_AP),

};

void initEUI()
{
  eui_setup_interface(&serial_comms);
  // Provide the tracked variables to the library
  EUI_TRACK(tracked_vars);
  // Provide a identifier to make this board easy to find in the UI
  eui_setup_identifier("hello", 5);
}

void sendEUIVars()
{
  eui_send_tracked("baro");
  eui_send_tracked("worldAx");
  eui_send_tracked("kal_X");
  eui_send_tracked("kal_V");
  eui_send_tracked("kal_A");
  eui_send_tracked("kal_XP");
  eui_send_tracked("kal_VP");
  eui_send_tracked("kal_AP");
}

void handleEUI()
{
  serial_rx_handler(); //check for new inbound data
  if (EUITimer.hasPassed(16))
  {
    sendEUIVars();
    EUITimer.restart();
  }
}
