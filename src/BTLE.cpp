
#include <SPI.h>
#include <./bluetooth/lib_aci.h>
#include <./bluetooth/aci_setup.h>
#include "./bluetooth/uart_over_ble.h"
#include "./bluetooth/services.h"
#include "BTLE.h"
#include "Config.h"
#include "ServoControl.h"

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
static services_pipe_type_mapping_t
    services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
#define NUMBER_OF_PIPES 0
static services_pipe_type_mapping_t *services_pipe_type_mapping = NULL;
#endif

/* Store the setup for the nRF8001 in the flash of the AVR to save on RAM */
static hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

static struct aci_state_t aci_state;

/*
Temporary buffers for sending ACI commands
*/
static hal_aci_evt_t aci_data;
//static hal_aci_data_t aci_cmd;

/*
Timing change state variable
*/
static bool timing_change_done = false;

/*
Used to test the UART TX characteristic notification
*/
static uart_over_ble_t uart_over_ble;
static uint8_t uart_buffer[23];
static uint8_t uart_buffer_len = 0;
static uint8_t dummychar = 0;

bool stringComplete = false; // whether the string is complete
uint8_t stringIndex = 0;     //Initialize the index to store incoming chars

/*
Initialize the radio_ack. This is the ack received for every transmitted packet.
*/
//static bool radio_ack_pending = false;

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  // Serial.print("ERROR ");
  // Serial.print(file);
  // Serial.print(": ");
  // Serial.print(line);
  // Serial.print("\n");
  while (1)
    ;
}
/*
Description:
In this template we are using the BTLE as a UART and can send and receive packets.
The maximum size of a packet is 20 bytes.
When a command it received a response(s) are transmitted back.
Since the response is done using a Notification the peer must have opened it(subscribed to it) before any packet is transmitted.
The pipe for the UART_TX becomes available once the peer opens it.
See section 20.4.1 -> Opening a Transmit pipe
In the master control panel, clicking Enable Services will open all the pipes on the nRF8001.
The ACI Evt Data Credit provides the radio level ack of a transmitted packet.
*/

void initPins()
{
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  aci_state.aci_pins.reqn_pin = REQN_BT;         //SS for Nordic board, 9 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.rdyn_pin = RDYN_BT;         //3 for Nordic board, 8 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.mosi_pin = PIN_SPI1_MOSI;
  aci_state.aci_pins.miso_pin = PIN_SPI1_MISO;
  aci_state.aci_pins.sck_pin = PIN_SPI1_SCK;

  aci_state.aci_pins.spi_clock_divider = SPI_CLOCK_DIV8; //SPI_CLOCK_DIV8  = 2MHz SPI speed
                                                         //SPI_CLOCK_DIV16 = 1MHz SPI speed

  aci_state.aci_pins.reset_pin = RESET_BT; //4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.active_pin = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
  aci_state.aci_pins.interrupt_number = 1;
}

void initBluetooth()
{
  /**
  Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
  */
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs = setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs = NB_SETUP_MESSAGES;

  initPins();

  //We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
  //If the RESET line is not available we call the ACI Radio Reset to soft reset the nRF8001
  //then we initialize the data structures required to setup the nRF8001
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  lib_aci_init(&aci_state, false);
  //Serial.println(F("Set up done"));
}

void uart_over_ble_init(void)
{
  uart_over_ble.uart_rts_local = true;
}

bool uart_tx(uint8_t *buffer, uint8_t buffer_len)
{
  bool status = false;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) &&
      (aci_state.data_credit_available >= 1))
  {
    status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, buffer, buffer_len);
    if (status)
    {
      aci_state.data_credit_available--;
    }
  }

  return status;
}

bool uart_process_control_point_rx(uint8_t *byte, uint8_t length)
{
  bool status = false;
  aci_ll_conn_params_t *conn_params;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX))
  {
    //Serial.println(*byte, HEX);
    switch (*byte)
    {
    /*
      Queues a ACI Disconnect to the nRF8001 when this packet is received.
      May cause some of the UART packets being sent to be dropped
      */
    case UART_OVER_BLE_DISCONNECT:
      /*
        Parameters:
        None
        */
      lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
      status = true;
      break;

    /*
      Queues an ACI Change Timing to the nRF8001
      */
    case UART_OVER_BLE_LINK_TIMING_REQ:
      /*
        Parameters:
        Connection interval min: 2 bytes
        Connection interval max: 2 bytes
        Slave latency:           2 bytes
        Timeout:                 2 bytes
        Same format as Peripheral Preferred Connection Parameters (See nRFgo studio -> nRF8001 Configuration -> GAP Settings
        Refer to the ACI Change Timing Request in the nRF8001 Product Specifications
        */
      conn_params = (aci_ll_conn_params_t *)(byte + 1);
      lib_aci_change_timing(conn_params->min_conn_interval,
                            conn_params->max_conn_interval,
                            conn_params->slave_latency,
                            conn_params->timeout_mult);
      status = true;
      break;

    /*
      Clears the RTS of the UART over BLE
      */
    case UART_OVER_BLE_TRANSMIT_STOP:
      /*
        Parameters:
        None
        */
      uart_over_ble.uart_rts_local = false;
      status = true;
      break;

    /*
      Set the RTS of the UART over BLE
      */
    case UART_OVER_BLE_TRANSMIT_OK:
      /*
        Parameters:
        None
        */
      uart_over_ble.uart_rts_local = true;
      status = true;
      break;
    }
  }

  return status;
}

void checkBTLE()
{
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t *aci_evt;
    aci_evt = &aci_data.evt;
    switch (aci_evt->evt_opcode)
    {
    /**
      As soon as you reset the nRF8001 you will get an ACI Device Started Event
      */
    case ACI_EVT_DEVICE_STARTED:
    {
      aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
      switch (aci_evt->params.device_started.device_mode)
      {
      case ACI_DEVICE_SETUP:
        /**
            When the device is in the setup mode
            */
        //Serial.println(F("Evt Device Started: Setup"));
        setup_required = true;
        break;

      case ACI_DEVICE_STANDBY:
        //Serial.println(F("Evt Device Started: Standby"));
        //Looking for an iPhone by sending radio advertisements
        //When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
        if (aci_evt->params.device_started.hw_error)
        {
        }
        else
        {
          lib_aci_connect(0 /* in seconds : 0 means forever */, 0x0050 /* advertising interval 50ms*/);
        }

        break;
      }
    }
    break; //ACI Device Started Event

    case ACI_EVT_CMD_RSP:
      //If an ACI command response event comes with an error -> stop
      if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
      {
      }
      if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
      {
        //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
        lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
                               (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
      }
      break;

    case ACI_EVT_CONNECTED:
      uart_over_ble_init();
      timing_change_done = false;
      aci_state.data_credit_available = aci_state.data_credit_total;

      /*
        Get the device version of the nRF8001 and store it in the Hardware Revision String
        */
      lib_aci_device_version();
      break;

    case ACI_EVT_PIPE_STATUS:
      //Serial.println(F("Evt Pipe Status"));
      if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done))
      {
        lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                                          // Used to increase or decrease bandwidth
        timing_change_done = true;
      }
      break;

    case ACI_EVT_TIMING:
      //Serial.println(F("Evt link connection interval changed"));
      lib_aci_set_local_data(&aci_state,
                             PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET,
                             (uint8_t *)&(aci_evt->params.timing.conn_rf_interval), /* Byte aligned */
                             PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET_MAX_SIZE);
      break;

    case ACI_EVT_DISCONNECTED:
      lib_aci_connect(0 /* in seconds  : 0 means forever */, 0x0050 /* advertising interval 50ms*/);
      break;

    case ACI_EVT_DATA_RECEIVED:
      data.btleCmd = 1;
      //Serial.print(F("Pipe Number: "));
      //Serial.println(aci_evt->params.data_received.rx_data.pipe_number, DEC);

      if (PIPE_UART_OVER_BTLE_UART_RX_RX == aci_evt->params.data_received.rx_data.pipe_number)
      {

        for (int i = 0; i < aci_evt->len - 2; i++)
        {

          char dataChar = aci_evt->params.data_received.rx_data.aci_data[i];

          switch (dataChar)
          {
          case 'f':
            if (data.state == IDLE)
            {
              if (IS_TEST_MODE)
              {
                goToState(TEST);
              }
              else
              {
                if (data.pyro1Continuity == 1.0)
                {
                  goToState(LAUNCH_COMMANDED);
                }
                else
                {
                  goToState(ABORT);
                }
              }
            }

            break;
          case 'z':
            nonLoggedData.zeroGyrosStatus = true;
            break;
          case 'A':
            goToState(ABORT);
            break;
          case '>':
            data.Y_Servo_Center += 1;
            nonLoggedData.servoCentersAvailable = true;
            break;
          case '<':
            data.Y_Servo_Center -= 1;
            nonLoggedData.servoCentersAvailable = true;
            break;
          case ')':
            data.Z_Servo_Center += 1;
            nonLoggedData.servoCentersAvailable = true;
            break;
          case '(':
            data.Z_Servo_Center -= 1;
            nonLoggedData.servoCentersAvailable = true;
            break;
          case 'R':
            readTVCCenters();
            nonLoggedData.servoCentersAvailable = true;
            break;
          case 'S':
            if (data.state == IDLE)
            {
              startServoTest();
            }

            break;

          default:
            break;
          }
          uart_buffer[i] = aci_evt->params.data_received.rx_data.aci_data[i];
        }
        uart_buffer_len = aci_evt->len - 2;
        if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
        {
          /*Do this to test the loopback otherwise comment it out*/
          /*
              if (!uart_tx(&uart_buffer[0], aci_evt->len - 2))
              {
                Serial.println(F("UART loopback failed"));
              }
              else
              {
                Serial.println(F("UART loopback OK"));
              }
              */
        }
      }
      if (PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_RX == aci_evt->params.data_received.rx_data.pipe_number)
      {
        uart_process_control_point_rx(&aci_evt->params.data_received.rx_data.aci_data[0], aci_evt->len - 2); //Subtract for Opcode and Pipe number
      }
      break;

    case ACI_EVT_DATA_CREDIT:
      aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
      break;

    case ACI_EVT_PIPE_ERROR:
      //See the appendix in the nRF8001 Product Specication for details on the error codes
      //Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
      //Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
      //Serial.print(F("  Pipe Error Code: 0x"));
      //Serial.println(aci_evt->params.pipe_error.error_code, HEX);

      //Increment the credit available as the data packet was not sent.
      //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
      //for the credit.
      if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
      {
        aci_state.data_credit_available++;
      }
      break;

    case ACI_EVT_HW_ERROR:
      //Serial.print(F("HW error: "));
      //Serial.println(aci_evt->params.hw_error.line_num, DEC);

      for (uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
      {
        //Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
      }
      //Serial.println();
      lib_aci_connect(0 /* in seconds, 0 means forever */, 0x0050 /* advertising interval 50ms*/);
      //Serial.println(F("Advertising started. Tap Connect on the nRF UART app"));
      break;
    }
  }
  else
  {
    //Serial.println(F("No ACI Events available"));
    // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
  }

  /* setup_required is set to true when the device starts up and enters setup mode.
   * It indicates that do_aci_setup() should be called. The flag should be cleared if
   * do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
   */
  if (setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
    }
  }

  // print the string when a newline arrives:
  if (stringComplete)
  {
    //Serial.print(F("Sending: "));
    //Serial.println((char *)&uart_buffer[0]);

    uart_buffer_len = stringIndex + 1;

    if (!lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, uart_buffer, uart_buffer_len))
    {
      //Serial.println(F("Serial input dropped"));
    }

    // clear the uart_buffer:
    for (stringIndex = 0; stringIndex < 23; stringIndex++)
    {
      uart_buffer[stringIndex] = ' ';
    }

    // reset the flag and the index in order to receive more data
    stringIndex = 0;
    stringComplete = false;
  }
}

void sendTelemetry(char message[])
{
  stringIndex = 0;
  for (int i = 0; i < strlen(message); i += 1)
  {
    uart_buffer[i] = message[i];
    stringIndex += 1;
  }

  // Serial.print(F("Sending: "));
  // Serial.println((char *)&uart_buffer[0]);

  uart_buffer_len = stringIndex + 1;

  if (!lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, uart_buffer, uart_buffer_len))
  {
    //Serial.println(F("Serial input dropped"));
  }

  // clear the uart_buffer:
  for (int i = 0; i < stringIndex; i++)
  {
    uart_buffer[stringIndex] = ' ';
  }

  // reset the flag and the index in order to receive more data
  stringIndex = 0;
}

/*
 COMMENT ONLY FOR ARDUINO
 SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 Serial Event is NOT compatible with Leonardo, Micro, Esplora
 */
void serialEvent()
{

  while (Serial.available() > 0)
  {
    // get the new byte:
    dummychar = (uint8_t)Serial.read();
    if (!stringComplete)
    {
      if (dummychar == '\n')
      {
        // if the incoming character is a newline, set a flag
        // so the main loop can do something about it
        stringIndex--;
        stringComplete = true;
      }
      else
      {
        if (stringIndex > 19)
        {
          //Serial.println("Serial input truncated");
          stringIndex--;
          stringComplete = true;
        }
        else
        {
          // add it to the uart_buffer
          uart_buffer[stringIndex] = dummychar;
          stringIndex++;
        }
      }
    }
  }
}