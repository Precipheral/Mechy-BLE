/*
             LUFA Library
     Copyright (C) Dean Camera, 2017.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2017  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Joystick demo. This file contains the main tasks of the demo and
 *  is responsible for the initial application hardware configuration.
 */

#include "Joystick.h"

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main(void)
{
	
	SetupHardware();
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	for (;;)
	{

		HID_Task();
		USB_USBTask();
		
		Serial_SendString("I am currently in state: ");  
		Serial_SendByte((USB_DeviceState + 0x30));
		Serial_SendString("\r\n");
		
		if (USB_DeviceState == DEVICE_STATE_Suspended)
		{
			Serial_SendString("I am about to start BLE stuff\r\n");
			BLE_main();
			//Aci_t_ver();
		}
		
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	Joystick_Init();
	LEDs_Init();
	Buttons_Init();
	USB_Init();
	Serial_Init(9600, false);

}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management and joystick reporting tasks.
 */
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the joystick reporting task started.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup HID Report Endpoint */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(JOYSTICK_EPADDR, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);

	/* Indicate endpoint configuration success or failure */
	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */
	switch (USB_ControlRequest.bRequest)
	{
		case HID_REQ_GetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				USB_JoystickReport_Data_t JoystickReportData;

				/* Create the next HID report to send to the host */
				GetNextReport(&JoystickReportData);

				Endpoint_ClearSETUP();

				/* Write the report data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(&JoystickReportData, sizeof(JoystickReportData));
				Endpoint_ClearOUT();
			}

			break;
	}
}

/** Fills the given HID report data structure with the next HID report to send to the host.
 *
 *  \param[out] ReportData  Pointer to a HID report data structure to be filled
 *
 *  \return Boolean \c true if the new report differs from the last report, \c false otherwise
 */
bool GetNextReport(USB_JoystickReport_Data_t* const ReportData)
{
	static uint8_t PrevJoyStatus    = 0;
	static uint8_t PrevButtonStatus = 0;
	uint8_t        JoyStatus_LCL    = Joystick_GetStatus();
	uint8_t        ButtonStatus_LCL = Buttons_GetStatus();
	bool           InputChanged     = false;
	int8_t			HatMask			= 0;

	/* Clear the report contents */
	memset(ReportData, 0, sizeof(USB_JoystickReport_Data_t));
	Serial_SendString("I made it to the start of the report collection\r\n");
	//Send Left Stick X-Axis ADC Reading to Y in Report Data, axis inverted with 9-bit for signed int 
	ReportData->X = (JOY_X ^ 0x3FF);
	
	//Send Left Stick Y-Axis ADC Reading to X in Report Data
	ReportData->Y = JOY_Y;
	
	//Send Right Stick X-Axis ADC Reading to RX in Report Data, axis inverted with 9-bit for signed int  
	ReportData->RX = (JOY_RX ^ 0x3FF);

	//Send Right Stick Y-Axis ADC Reading to RY in Report Data
	ReportData->RY = JOY_RY;
		
	//Left press joystick button (not working yet)
	if (JoyStatus_LCL & JOY_PRESS)
		ReportData->Button |= (1 << 7);
	//Check if Button X is pressed and add it to the report
	if (ButtonStatus_LCL & BUTTONS_BUTTON1)
		ReportData->Button |= (1 << 0); 
		
	if (ButtonStatus_LCL & BUTTONS_BUTTON2)
		ReportData->Button |= (1 << 1); 
	
	if (ButtonStatus_LCL & BUTTONS_BUTTON3)
		ReportData->Button |= (1 << 2); 

	if (ButtonStatus_LCL & BUTTONS_BUTTON4)
		ReportData->Button |= (1 << 3);
		
	
	// Map pressed D-pad / hat switch buttons to HatMask 
	if (ButtonStatus_LCL & BUTTONS_BUTTON5)
		HatMask |= (1 << 0);
		
	if (ButtonStatus_LCL & BUTTONS_BUTTON6)
		HatMask |= (1 << 1);
		
	if (ButtonStatus_LCL & BUTTONS_BUTTON7)
		HatMask |= (1 << 2);	

	if (ButtonStatus_LCL & BUTTONS_BUTTON8)
		HatMask |= (1 << 3);
		
	// Convert HatMask to direction and send to report data hat	(no direction if two opposing ordinals are pressed at the same time)
	switch(HatMask)
	{
		case 0x01:					 //Up
			ReportData->Hat = 0;
			break;
		
		case 0x02:					 //Down
			ReportData->Hat = 4;
			break;		
		
		case 0x04:					//Left
			ReportData->Hat = 6;
			break;	
				
		case 0x08:					//Right
			ReportData->Hat = 2;
			break;	
				
		case 0x09:					//Up - Right
			ReportData->Hat = 1;
			break;	
				
		case 0x0A:					//Down - Right
			ReportData->Hat = 3;
			break;		
			
		case 0x06:					//Down -Left
			ReportData->Hat = 5;
			break;	
				
		case 0x05:					//Up - Left
			ReportData->Hat = 7;
			break;
					
		default :					//Neutral position
			ReportData->Hat = -1;
	}
	
	/* Check if the new report is different to the previous report */
	InputChanged = (uint8_t)(PrevJoyStatus ^ JoyStatus_LCL) | (uint8_t)(PrevButtonStatus ^ ButtonStatus_LCL);

	/* Save the current joystick status for later comparison */
	PrevJoyStatus    = JoyStatus_LCL;
	PrevButtonStatus = ButtonStatus_LCL;
	Serial_SendString("I did the report\r\n");
	/* Return whether the new report is different to the previous report or not */
	return InputChanged;
	
}

/** Function to manage HID report generation and transmission to the host. */
void HID_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;
	
	/* Select the Joystick Report Endpoint */
	Endpoint_SelectEndpoint(JOYSTICK_EPADDR);

	/* Check to see if the host is ready for another packet */
	if (Endpoint_IsINReady())
	{
		USB_JoystickReport_Data_t JoystickReportData;

		/* Create the next HID report to send to the host */
		GetNextReport(&JoystickReportData);
		
		/* Write Joystick Report Data */
		Endpoint_Write_Stream_LE(&JoystickReportData, sizeof(JoystickReportData), NULL);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();

		/* Clear the report data afterwards */
		memset(&JoystickReportData, 0, sizeof(JoystickReportData));
	}
}

