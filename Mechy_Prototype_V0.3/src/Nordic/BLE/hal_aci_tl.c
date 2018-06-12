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

/** @file
@brief Implementation of the ACI transport layer module
*/

#include <LUFA/LUFA/Drivers/Peripheral/SPI.h>
#include "hal_platform.h"
#include "hal_aci_tl.h"
#include "aci_queue.h"
#if ( !defined(__SAM3X8E__) && !defined(__PIC32MX__) )
#include <avr/sleep.h>
#endif
/*
PIC32 supports only MSbit transfer on SPI and the nRF8001 uses LSBit
Use the REVERSE_BITS macro to convert from MSBit to LSBit
The outgoing command and the incoming event needs to be converted
*/
//Board dependent defines
#if defined (__AVR__)
    //For Arduino add nothing
#elif defined(__PIC32MX__)
    //For ChipKit as the transmission has to be reversed, the next definitions have to be added
    #define REVERSE_BITS(byte) (((reverse_lookup[(byte & 0x0F)]) << 4) + reverse_lookup[((byte & 0xF0) >> 4)])
    static const uint8_t reverse_lookup[] = { 0, 8,  4, 12, 2, 10, 6, 14,1, 9, 5, 13,3, 11, 7, 15 };
#endif

static void m_aci_data_print(hal_aci_data_t *p_data);
static void m_aci_event_check(void);
//static void m_aci_isr(void);
static void m_aci_pins_set(aci_pins_t *a_pins_ptr);
static inline void m_aci_reqn_disable (void);
static inline void m_aci_reqn_enable (void);
static void m_aci_q_flush(void);
static bool m_aci_spi_transfer(hal_aci_data_t * data_to_send, hal_aci_data_t * received_data);

static uint8_t        spi_readwrite(uint8_t aci_byte);

static bool           aci_debug_print = false;

aci_queue_t    aci_tx_q;
aci_queue_t    aci_rx_q;

static aci_pins_t	 *a_pins_local_ptr;

void m_aci_data_print(hal_aci_data_t *p_data)
{
  const uint8_t length = p_data->buffer[0];
  uint8_t i;
  Serial_SendByte((length + 0x30));
  Serial_SendString(" :");
  for (i=0; i<=length; i++)
  {
    Serial_SendByte((p_data->buffer[i] + 0x30));
    Serial_SendString(", ");
  }
  Serial_SendString("\r\n");
}

/*
  Interrupt service routine called when the RDYN line goes low. Runs the SPI transfer.
*/

/* REMOVED INTERRUPT -AHJ
//static void m_aci_isr(void)
ISR(INT0_vect)
{
  hal_aci_data_t data_to_send;
  hal_aci_data_t received_data;

  //Receive from queue
  if (!aci_queue_dequeue_from_isr(&aci_tx_q, &data_to_send))
  {
    // queue was empty, nothing to send 
    data_to_send.status_byte = 0;
    data_to_send.buffer[0] = 0;
  }

  // Receive and/or transmit data
  m_aci_spi_transfer(&data_to_send, &received_data);

  if (!aci_queue_is_full_from_isr(&aci_rx_q) && !aci_queue_is_empty_from_isr(&aci_tx_q))
  {
    m_aci_reqn_enable();
  }

  // Check if we received data
  if (received_data.buffer[0] > 0)
  {
    if (!aci_queue_enqueue_from_isr(&aci_rx_q, &received_data))
    {
      // Receive Buffer full.
      //   Should never happen.
      //   Spin in a while loop.
      
      while(1);
    }

    // Disable ready line interrupt until we have room to store incoming messages
    if (aci_queue_is_full_from_isr(&aci_rx_q))
    {
      //detachInterrupt(a_pins_local_ptr->interrupt_number);
	  EIMSK &= ~(1 << INT0); 	  
    }
  }

  return;
}
*/
/*
  Checks the RDYN line and runs the SPI transfer if required.
*/
static void m_aci_event_check(void)
{
  hal_aci_data_t data_to_send;
  hal_aci_data_t received_data;

  // No room to store incoming messages
  if (aci_queue_is_full(&aci_rx_q))
  {
    return;
  }

  // If the ready line is disabled and we have pending messages outgoing we enable the request line
  if (PINB0 == 1) //(HIGH == digitalRead(a_pins_local_ptr->rdyn_pin))
  {
    if (!aci_queue_is_empty(&aci_tx_q))
    {
      m_aci_reqn_enable();
    }

    return;
  }

  // Receive from queue
  if (!aci_queue_dequeue(&aci_tx_q, &data_to_send))
  {
    /* queue was empty, nothing to send */
    data_to_send.status_byte = 0;
    data_to_send.buffer[0] = 0;
  }

  // Receive and/or transmit data
  m_aci_spi_transfer(&data_to_send, &received_data);

  /* If there are messages to transmit, and we can store the reply, we request a new transfer */
  if (!aci_queue_is_full(&aci_rx_q) && !aci_queue_is_empty(&aci_tx_q))
  {
    m_aci_reqn_enable();
  }

  // Check if we received data
  if (received_data.buffer[0] > 0)
  {
    if (!aci_queue_enqueue(&aci_rx_q, &received_data))
    {
      /* Receive Buffer full.
         Should never happen.
         Spin in a while loop.
      */
      while(1);
    }
  }

  return;
}

/** @brief Point the low level library at the ACI pins specified
 *  @details
 *  The ACI pins are specified in the application and a pointer is made available for
 *  the low level library to use
 */
static void m_aci_pins_set(aci_pins_t *a_pins_ptr)
{
  a_pins_local_ptr = a_pins_ptr;
}

static inline void m_aci_reqn_disable (void)
{
  //digitalWrite(a_pins_local_ptr->reqn_pin, 1);
  PORTB |= (1 << 4);
}

static inline void m_aci_reqn_enable (void)
{
  //digitalWrite(a_pins_local_ptr->reqn_pin, 0);
  PORTB &= ~(1 << 4);
}

static void m_aci_q_flush(void)
{
  //noInterrupts();
  //  GlobalInterruptDisable(); REMOVE INTERRUPTS
  /* re-initialize aci cmd queue and aci event queue to flush them*/
  aci_queue_init(&aci_tx_q);
  aci_queue_init(&aci_rx_q);
  //interrupts();
  //GlobalInterruptEnable(); REMOVE INTERRUPTS
}

static bool m_aci_spi_transfer(hal_aci_data_t * data_to_send, hal_aci_data_t * received_data)
{
  uint8_t byte_cnt;
  uint8_t byte_sent_cnt;
  uint8_t max_bytes;

  m_aci_reqn_enable();

  // Send length, receive header
  byte_sent_cnt = 0;
  received_data->status_byte = spi_readwrite(data_to_send->buffer[byte_sent_cnt++]);
  // Send first byte, receive length from slave
  received_data->buffer[0] = spi_readwrite(data_to_send->buffer[byte_sent_cnt++]);
  if (0 == data_to_send->buffer[0])
  {
    max_bytes = received_data->buffer[0];
  }
  else
  {
    // Set the maximum to the biggest size. One command byte is already sent
    max_bytes = (received_data->buffer[0] > (data_to_send->buffer[0] - 1))
                                          ? received_data->buffer[0]
                                          : (data_to_send->buffer[0] - 1);
  }

  if (max_bytes > HAL_ACI_MAX_LENGTH)
  {
    max_bytes = HAL_ACI_MAX_LENGTH;
  }

  // Transmit/receive the rest of the packet
  for (byte_cnt = 0; byte_cnt < max_bytes; byte_cnt++)
  {
    received_data->buffer[byte_cnt+1] =  spi_readwrite(data_to_send->buffer[byte_sent_cnt++]);
  }

  // RDYN should follow the REQN line in approx 100ns
  m_aci_reqn_disable();

  return (max_bytes > 0);
}

void hal_aci_tl_debug_print(bool enable)
{
	aci_debug_print = enable;
}

void hal_aci_tl_pin_reset(void)
{
    if (UNUSED != a_pins_local_ptr->reset_pin)
    {
        //pinMode(a_pins_local_ptr->reset_pin, OUTPUT);
		DDRB |= (1 << 5);
		//PORTB|= (1 << 5);

        if ((REDBEARLAB_SHIELD_V1_1     == a_pins_local_ptr->board_name) ||
            (REDBEARLAB_SHIELD_V2012_07 == a_pins_local_ptr->board_name))
        {
            //The reset for the Redbearlab v1.1 and v2012.07 boards are inverted and has a Power On Reset
            //circuit that takes about 100ms to trigger the reset
            //digitalWrite(a_pins_local_ptr->reset_pin, 1);
			PORTB |= (1 << 5);
            Delay_MS(100);
            //digitalWrite(a_pins_local_ptr->reset_pin, 0);
			PORTB &= ~(1 << 5);
        }
        else
        {
            Serial_SendString("Resetting nRF8001\r\n"); 
			//digitalWrite(a_pins_local_ptr->reset_pin, 1);
			DDRB  |= (1 << 5);  //Reset Pin Set to output, can be removed AHJ
			PORTB |= (1 << 5);
            //digitalWrite(a_pins_local_ptr->reset_pin, 0);
			PORTB &= ~(1 << 5);
            //digitalWrite(a_pins_local_ptr->reset_pin, 1);
			PORTB |= (1 << 5);
        }
    }
}

bool hal_aci_tl_event_peek(hal_aci_data_t *p_aci_data)
{
  // REMOVE INTERRUPT
  //if (!a_pins_local_ptr->interface_is_interrupt)
  //{
    m_aci_event_check();
  //}

  if (aci_queue_peek(&aci_rx_q, p_aci_data))
  {
    return true;
  }

  return false;
}

bool hal_aci_tl_event_get(hal_aci_data_t *p_aci_data)
{
  bool was_full;

  if (/*!a_pins_local_ptr->interface_is_interrupt && //REMOVE INTERRUPT*/ !aci_queue_is_full(&aci_rx_q))
  {
    m_aci_event_check();
  }

  was_full = aci_queue_is_full(&aci_rx_q);

  if (aci_queue_dequeue(&aci_rx_q, p_aci_data))
  {
		if (aci_debug_print)
		{
		  Serial_SendString("E");
		  m_aci_data_print(p_aci_data);
		}
		/* REMOVE INTERRUPT
		if (was_full && a_pins_local_ptr->interface_is_interrupt)
		{
		  //Enable RDY line interrupt again 
		  //attachInterrupt(a_pins_local_ptr->interrupt_number, m_aci_isr, LOW); 
	  
		   //EIMSK &= ~(1 << INT0);				//Disable to prevent interrupt trigger while changing bits AHJ
		   //EICRA &= ~(1 << (ISC10 | ISC11));	//Set interrupt INT0 to trigger on low 
		   EIMSK |= (1 << INT0 );				//Enable INT0 interrupt
		
		}
		*/
		/* Attempt to pull REQN LOW since we've made room for new messages */
		if (!aci_queue_is_full(&aci_rx_q) && !aci_queue_is_empty(&aci_tx_q))
		{
		  m_aci_reqn_enable();
		}

    return true;
  }

  return false;
}

void hal_aci_tl_init(aci_pins_t *a_pins, bool debug)
{
  aci_debug_print = debug;

  /* Needs to be called as the first thing for proper initialization*/
  m_aci_pins_set(a_pins);

  /*
  The SPI lines used are mapped directly to the hardware SPI
  MISO MOSI and SCK
  Change here if the pins are mapped differently

  The SPI library assumes that the hardware pins are used
  */
  //SPI.begin();
  SPI_Init(SPI_SPEED_FCPU_DIV_32 | SPI_ORDER_LSB_FIRST | SPI_SCK_LEAD_RISING | SPI_SAMPLE_LEADING | SPI_MODE_MASTER);
  //Board dependent defines
  //#if defined (__AVR__)
    //For Arduino use the LSB first
    //SPI.setBitOrder(LSBFIRST); DEFINED IN SPI_INIT(SPI_ORDER_LSB_FIRST) AHJ
  //#elif defined(__PIC32MX__)
    //For ChipKit use MSBFIRST and REVERSE the bits on the SPI as LSBFIRST is not supported
    //SPI.setBitOrder(MSBFIRST);  DEFINED IN SPI_INIT() AHJ
  //#endif
  //SPI.setClockDivider(a_pins->spi_clock_divider);  DEFINED IN SPI_INIT(SPI_SPEED_FCPU_DIV_8) AHJ
  //SPI.setDataMode(SPI_MODE0);  DEFINED IN SPI_INIT(SPI_SCK_LEAD_RISING | SPI_SAMPLE_LEADING) AHJ
  Delay_MS(1000); //AHJ test delay for ACI command queue.
  /* Initialize the ACI Command queue. This must be called after the delay above. */
  Serial_SendString("Initializing TX Queue\r\n");
  aci_queue_init(&aci_tx_q);
  Serial_SendString("Initializing RX Queue\r\n");
  aci_queue_init(&aci_rx_q);

  //Configure the IO lines
  //pinMode(a_pins->rdyn_pin,		INPUT_PULLUP);
  DDRB	&= ~(1 << 0);
  PORTB	|=  (1 << 0);
  
  //pinMode(a_pins->reqn_pin,		OUTPUT);
  DDRB	|=  (1 << 4);
  PORTB	&=  ~(1 << 4);  //Set high or low? AHJ

  if (UNUSED != a_pins->active_pin)
  {
    //pinMode(a_pins->active_pin,	INPUT);
	DDRB &= ~(1 << 6);
	PORTB |= (1 << 6);
  }
  /* Pin reset the nRF8001, required when the nRF8001 setup is being changed */
  hal_aci_tl_pin_reset();

  /* Set the nRF8001 to a known state as required by the data sheet*/
  //digitalWrite(a_pins->miso_pin, 0);
    PORTB	&= ~(1 << 3);
  //digitalWrite(a_pins->mosi_pin, 0);
	PORTB	&= ~(1 << 2);
  //digitalWrite(a_pins->reqn_pin, 1);
	PORTB	|=  (1 << 4);	
  //digitalWrite(a_pins->sck_pin,  0);
    PORTB	&= ~(1 << 1);

  Delay_MS(30); //Wait for the nRF8001 to get hold of its lines - the lines float for a few ms after the reset

  /* Attach the interrupt to the RDYN line as requested by the caller */
  /* REMOVE INTERRUPT
  if (a_pins->interface_is_interrupt)
  {
    // We use the LOW level of the RDYN line as the atmega328 can wakeup from sleep only on LOW
    //attachInterrupt(a_pins->interrupt_number, m_aci_isr, LOW);
	
	EIMSK &= ~(1 << INT0);				//Disable to prevent interrupt trigger while changing bits AHJ
	EICRA &= ~(1 << (ISC10 | ISC11));	//Set interrupt INT0 to trigger on low
	EIMSK |= (1 << INT0 );				//Enable INT0 interrupt
	GlobalInterruptEnable();
  }
  */
}

bool hal_aci_tl_send(hal_aci_data_t *p_aci_cmd)
{
  const uint8_t length = p_aci_cmd->buffer[0];
  bool ret_val = false;

  if (length > HAL_ACI_MAX_LENGTH)
  {
    return false;
  }

  ret_val = aci_queue_enqueue(&aci_tx_q, p_aci_cmd);
  if (ret_val)
  {
    if(!aci_queue_is_full(&aci_rx_q))
    {
      // Lower the REQN only when successfully enqueued
      m_aci_reqn_enable();
    }

    if (aci_debug_print)
    {
      Serial_SendString("C"); //ACI Command
      m_aci_data_print(p_aci_cmd);
    }
  }

  return ret_val;
}

static uint8_t spi_readwrite(const uint8_t aci_byte)
{
	//Board dependent defines
#if defined (__AVR__)
    //For Arduino the transmission does not have to be reversed
    return SPI_TransferByte(aci_byte);
#elif defined(__PIC32MX__)
    //For ChipKit the transmission has to be reversed
    uint8_t tmp_bits;
    tmp_bits = SPI_TransferByte(REVERSE_BITS(aci_byte));
	return REVERSE_BITS(tmp_bits);
#endif
}

bool hal_aci_tl_rx_q_empty (void)
{
  return aci_queue_is_empty(&aci_rx_q);
}

bool hal_aci_tl_rx_q_full (void)
{
  return aci_queue_is_full(&aci_rx_q);
}

bool hal_aci_tl_tx_q_empty (void)
{
  return aci_queue_is_empty(&aci_tx_q);
}

bool hal_aci_tl_tx_q_full (void)
{
  return aci_queue_is_full(&aci_tx_q);
}

void hal_aci_tl_q_flush (void)
{
  m_aci_q_flush();
}
