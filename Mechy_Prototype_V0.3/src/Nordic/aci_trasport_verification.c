/* Copyright (c) 2014, Nordic Semiconductor ASA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * echo project to verify the SPI/ACI connectivity
 */

/** @defgroup my_project my_project
@{
@ingroup projects
@brief Echo project to loop data from the mcu to the nRF800 and back

@details
This project is a test project to verify the SPI/ACI.
The data in the ACI echo command send and the data
received in the ACI echo event should be the same.


 */

#define DEBUG_ENABLE CODED_TRACES

#include <LUFA/LUFA/Drivers/Peripheral/SPI.h>
#include <LUFA/LUFA/Drivers/Peripheral/Serial.h>
//#include <Nordic/Config/services.h>
#include <Nordic/BLE/lib_aci.h>
//#include <Nordic/BLE/aci_setup.h>
#include <avr/eeprom.h>

// aci_struct that will contain
// total initial credits
// current credit
// current state of the aci (setup/standby/active/sleep)
// open remote pipe pending
// close remote pipe pending
// Current pipe available bitmap
// Current pipe closed bitmap
// Current connection interval, slave latency and link supervision timeout
// Current State of the the GATT client (Service Discovery)
// Status of the bond (R) Peer address
static struct aci_state_t aci_state;

static hal_aci_evt_t aci_data;

static uint8_t echo_data[] = { 0x00, 0xaa, 0x55, 0xff, 0x77, 0x55, 0x33, 0x22, 0x11, 0x44, 0x66, 0x88, 0x99, 0xbb, 0xdd, 0xcc, 0x00, 0xaa, 0x55, 0xff };
static uint8_t aci_echo_cmd = 0;

#define NUM_ECHO_CMDS 3

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  Serial_SendString("ERROR ");
  Serial_SendString(file);
  Serial_SendString(": ");
  Serial_SendString((char*) line);
  Serial_SendString("\n");
  while(1);
}

void setup(void)
{
  //Serial.begin(115200);
  //Wait until the serial port is available (useful only for the Leonardo)
  //As the Leonardo board is not reseted every time you open the Serial Monitor
  #if defined (__AVR_ATmega32U4__)
    //while(!Serial_IsSendReady)
    //{}
    Delay_MS(5000);  //5 seconds Delay_MS for enabling to see the start up comments on the serial board
  #elif defined(__PIC32MX__)
    Delay_MS(1000);
  #endif
  Serial_SendString("Mechy setup\r\n");

  /*
  Tell the ACI library, the MCU to nRF8001 pin connections.
  The Active pin is optional and can be marked UNUSED
  */
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  aci_state.aci_pins.reqn_pin   = 28;	//PB4
  aci_state.aci_pins.rdyn_pin   = 8;	//PB0
  aci_state.aci_pins.mosi_pin   = 10;	//PB2
  aci_state.aci_pins.miso_pin   = 11;	//PB3
  aci_state.aci_pins.sck_pin    = 9;	//PB1

  //DEFINED IN SPI_INIT(); AHJ
  aci_state.aci_pins.spi_clock_divider      = 1;//SPI_CLOCK_DIV8  = 2MHz SPI speed  or SPI_CLOCK_DIV16 = 1MHz SPI speed

  aci_state.aci_pins.reset_pin              = 29;	//PB5
  aci_state.aci_pins.active_pin             = 30;	//PB6
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false;  //IS it ? AHJ
  aci_state.aci_pins.interrupt_number       = 1;

  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  hal_aci_tl_init(&(aci_state.aci_pins),true);
  Serial_SendString("nRF8001 Reset done\r\n");
}

void loop()
{
  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;
    switch(aci_evt->evt_opcode)
    {
      /**
      As soon as you reset the nRF8001 you will get an ACI Device Started Event
      */
      case ACI_EVT_DEVICE_STARTED:
      {
        aci_state.data_credit_available = aci_evt->params.device_started.credit_available;
        switch(aci_evt->params.device_started.device_mode)
        {
          case ACI_DEVICE_SETUP:
            Serial_SendString("Evt Device Started: Setup\r\n");
            lib_aci_test(ACI_TEST_MODE_DTM_UART);
            break;
          case ACI_DEVICE_STANDBY:
            Serial_SendString("Evt Device Started: Standby\r\n");
            break;
          case ACI_DEVICE_TEST:
          {
            uint8_t i = 0;
            Serial_SendString("Evt Device Started: Test\r\n");
            Serial_SendString("Started infinite Echo test\r\n");
            Serial_SendString("Repeat the test with all bytes in echo_data inverted.\r\n");
            Serial_SendString("Waiting 4 seconds before the test starts....\r\n");
            Delay_MS(4000);
            for(i=0; i<NUM_ECHO_CMDS; i++)
            {
              lib_aci_echo_msg(sizeof(echo_data), &echo_data[0]);
              aci_echo_cmd++;
            }
          }
            break;
        }
      }
        break; //ACI Device Started Event
      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          Serial_SendString("ACI Command 0x");
          Serial_SendString((char*)aci_evt->params.cmd_rsp.cmd_opcode);
          Serial_SendString("Evt Cmd respone: Error. Arduino is in an while(1); loop");
          while (1);
        }
        break;
      case ACI_EVT_ECHO:
        if (0 != memcmp(&echo_data[0], &(aci_evt->params.echo.echo_data[0]), sizeof(echo_data)))
        {
          Serial_SendString("Error: Echo loop test failed. Verify the SPI connectivity on the PCB.");
        }
        else
        {
          Serial_SendString("Echo OK");
        }
        if (NUM_ECHO_CMDS == aci_echo_cmd)
        {
          uint8_t i = 0;
          aci_echo_cmd = 0;
          for(i=0; i<NUM_ECHO_CMDS; i++)
          {
            lib_aci_echo_msg(sizeof(echo_data), &echo_data[0]);
            aci_echo_cmd++;
          }
        }
        break;
    }
  }
  else
  {
    // No event in the ACI Event queue
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
  }
}

void Aci_t_ver(void)
{
	setup();
	for(;;)//uint8_t i; i < 5; i++)
	{
		loop();	
	}
	
}