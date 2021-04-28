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
        EUI_FLOAT("kal_X_accel", data.kal_X_accel),
        EUI_FLOAT("kal_X_posP", data.kal_X_posP),
        EUI_FLOAT("kal_X_velP", data.kal_X_velP),
        EUI_FLOAT("kal_X_accelP", data.kal_X_accelP),
        EUI_FLOAT("kal_Y_pos", data.kal_Y_pos),
        EUI_FLOAT("kal_Y_vel", data.kal_Y_vel),
        EUI_FLOAT("kal_Y_accel", data.kal_Y_accel),
        EUI_FLOAT("kal_Z_pos", data.kal_Z_pos),
        EUI_FLOAT("kal_Z_vel", data.kal_Z_vel),
        EUI_FLOAT("kal_Z_accel", data.kal_Z_accel),
        EUI_FLOAT("kal_Z_bias", data.kal_Z_bias),
        EUI_FLOAT("kal_Y_bias", data.kal_Y_bias),
        EUI_FLOAT("gps_altitude", data.gpsAltitude),
        EUI_FLOAT("gps_hdop", data.hdop),
        EUI_FLOAT("gps_sats", data.sats),
        EUI_FLOAT("pBaro", data.p_baro),

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
  eui_send_tracked("kal_X_accel");
  eui_send_tracked("kal_X_posP");
  eui_send_tracked("kal_X_velP");
  eui_send_tracked("kal_X_accelP");
  eui_send_tracked("kal_Y_pos");
  eui_send_tracked("kal_Y_vel");
  eui_send_tracked("kal_Y_accel");
  eui_send_tracked("kal_Z_pos");
  eui_send_tracked("kal_Z_vel");
  eui_send_tracked("kal_Z_accel");
  eui_send_tracked("kal_Z_bias");
  eui_send_tracked("kal_Y_bias");
  eui_send_tracked("gps_altitude");
  eui_send_tracked("gps_hdop");
  eui_send_tracked("gps_sats");
  eui_send_tracked("pBaro");
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
