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
        EUI_FLOAT("baro", data.altitude),
        EUI_FLOAT("worldAx", data.worldAx),
        EUI_FLOAT("worldAy", data.worldAy),
        EUI_FLOAT("worldAz", data.worldAz),
        EUI_FLOAT("kal_X_pos", data.kal_X_pos),
        EUI_FLOAT("kal_X_vel", data.kal_X_vel),
        EUI_FLOAT("kal_X_gravity", data.kal_X_accel),
        EUI_FLOAT("kal_X_bias", data.kal_Z_bias),
        EUI_FLOAT("kal_X_posP", data.kal_X_p),
        EUI_FLOAT("kal_X_velP", data.kal_V_p),
        EUI_FLOAT("kal_X_gravP", data.kal_G_p),
        EUI_FLOAT("kal_X_biasP", data.kal_B_p),
        EUI_FLOAT("gpsAltitude", data.gpsAltitude),

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
  eui_send_tracked("baro");
  eui_send_tracked("worldAx");
  eui_send_tracked("worldAy");
  eui_send_tracked("worldAz");
  eui_send_tracked("kal_X_pos");
  eui_send_tracked("kal_X_vel");
  eui_send_tracked("kal_X_gravity");
  eui_send_tracked("kal_X_bias");
  eui_send_tracked("kal_X_posP");
  eui_send_tracked("kal_X_velP");
  eui_send_tracked("kal_X_gravP");
  eui_send_tracked("kal_X_biasP");
  eui_send_tracked("gpsAltitude");
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
