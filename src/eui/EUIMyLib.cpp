#include "EUIMyLib.h"
#include "Config.h"
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
        EUI_FLOAT("ax", data.bno_yaw),
        EUI_FLOAT("ay", data.bno_pitch),
        EUI_FLOAT("az", data.bno_roll),
        EUI_FLOAT("world_ax", data.yaw),
        EUI_FLOAT("world_ay", data.pitch),
        EUI_FLOAT("world_az", data.roll),
        EUI_FLOAT("yaw", data.bno_yaw),
        EUI_FLOAT("pitch", data.bno_pitch),
        EUI_FLOAT("roll", data.bno_roll),
        EUI_FLOAT("gx", data.bno_gx),
        EUI_FLOAT("gy", data.bno_gy),
        EUI_FLOAT("gz", data.bno_gz),
};

void initEUI()
{

  if (DO_EUI == true)
  {
    eui_setup_interface(&serial_comms);
    // Provide the tracked variables to the library
    EUI_TRACK(tracked_vars);
    // Provide a identifier to make this board easy to find in the UI
    eui_setup_identifier("hello", 5);
  }
}

void sendEUIVars()
{

  eui_send_tracked("ax");
  eui_send_tracked("ay");
  eui_send_tracked("az");
  eui_send_tracked("world_ax");
  eui_send_tracked("world_ay");
  eui_send_tracked("world_az");
  eui_send_tracked("yaw");
  eui_send_tracked("pitch");
  eui_send_tracked("roll");
  eui_send_tracked("gx");
  eui_send_tracked("gy");
  eui_send_tracked("gz");
}

void handleEUI()
{
  if (DO_EUI == true)
  {
    serial_rx_handler(); //check for new inbound data
    if (EUITimer.hasPassed(16))
    {
      sendEUIVars();
      EUITimer.restart();
    }
  }
}
