/**
* This file is autogenerated by nRFgo Studio 1.21.2RC1.4
*/

#ifndef SETUP_MESSAGES_H__
#define SETUP_MESSAGES_H__

#include <Nordic/BLE/hal_platform.h>
#include <Nordic/BLE/aci.h>


#define SETUP_ID 26681
#define SETUP_FORMAT 3 /** nRF8001 D */
#define ACI_DYNAMIC_DATA_SIZE 193

/* Service: Gap - Characteristic: Device name - Pipe: SET */
#define PIPE_GAP_DEVICE_NAME_SET          1
#define PIPE_GAP_DEVICE_NAME_SET_MAX_SIZE 16

/* Service: Battery - Characteristic: Battery Level - Pipe: TX */
#define PIPE_BATTERY_BATTERY_LEVEL_TX          2
#define PIPE_BATTERY_BATTERY_LEVEL_TX_MAX_SIZE 1

/* Service: Battery - Characteristic: Battery Level - Pipe: SET */
#define PIPE_BATTERY_BATTERY_LEVEL_SET          3
#define PIPE_BATTERY_BATTERY_LEVEL_SET_MAX_SIZE 1

/* Service: Battery - Characteristic: Battery Power State - Pipe: SET */
#define PIPE_BATTERY_BATTERY_POWER_STATE_SET          4
#define PIPE_BATTERY_BATTERY_POWER_STATE_SET_MAX_SIZE 1

/* Service: HID Service - Characteristic: HID Control Point - Pipe: RX */
#define PIPE_HID_SERVICE_HID_CONTROL_POINT_RX          5
#define PIPE_HID_SERVICE_HID_CONTROL_POINT_RX_MAX_SIZE 1

/* Service: HID Service - Characteristic: HID Report ID1 - Pipe: TX */
#define PIPE_HID_SERVICE_HID_REPORT_ID1_TX          6
#define PIPE_HID_SERVICE_HID_REPORT_ID1_TX_MAX_SIZE 3

/* Service: HID Service - Characteristic: HID Report ID1 - Pipe: SET */
#define PIPE_HID_SERVICE_HID_REPORT_ID1_SET          7
#define PIPE_HID_SERVICE_HID_REPORT_ID1_SET_MAX_SIZE 3

/* Service: HID Service - Characteristic: HID Report ID2 - Pipe: TX */
#define PIPE_HID_SERVICE_HID_REPORT_ID2_TX          8
#define PIPE_HID_SERVICE_HID_REPORT_ID2_TX_MAX_SIZE 1

/* Service: HID Service - Characteristic: HID Report ID2 - Pipe: SET */
#define PIPE_HID_SERVICE_HID_REPORT_ID2_SET          9
#define PIPE_HID_SERVICE_HID_REPORT_ID2_SET_MAX_SIZE 1

/* Service: Scan Parameters Service - Characteristic: Scan Interval Window - Pipe: RX */
#define PIPE_SCAN_PARAMETERS_SERVICE_SCAN_INTERVAL_WINDOW_RX          10
#define PIPE_SCAN_PARAMETERS_SERVICE_SCAN_INTERVAL_WINDOW_RX_MAX_SIZE 4

/* Service: Scan Parameters Service - Characteristic: Scan Param Refresh Characteristic - Pipe: TX */
#define PIPE_SCAN_PARAMETERS_SERVICE_SCAN_PARAM_REFRESH_CHARACTERISTIC_TX          11
#define PIPE_SCAN_PARAMETERS_SERVICE_SCAN_PARAM_REFRESH_CHARACTERISTIC_TX_MAX_SIZE 1

/* Service: Device Information - Characteristic: Hardware Revision String - Pipe: SET */
#define PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET          12
#define PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET_MAX_SIZE 9


#define NUMBER_OF_PIPES 12

#define SERVICES_PIPE_TYPE_MAPPING_CONTENT {\
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
}

#define GAP_PPCP_MAX_CONN_INT 0x18 /**< Maximum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific value requested */
#define GAP_PPCP_MIN_CONN_INT  0x6 /**< Minimum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific value requested */
#define GAP_PPCP_SLAVE_LATENCY 6
#define GAP_PPCP_CONN_TIMEOUT 0x64 /** Connection Supervision timeout multiplier as a multiple of 10msec, 0xFFFF means no specific value requested */

#define NB_SETUP_MESSAGES 33
#define SETUP_MESSAGES_CONTENT {\
    {0x00,\
        {\
            0x07,0x06,0x00,0x00,0x03,0x02,0x42,0x07,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x00,0x39,0x68,0x00,0x00,0x02,0x00,0x09,0x00,0x0c,0x01,0x01,0x00,0x00,0x06,0x00,0x01,\
            0x81,0x12,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x40,0x12,0x00,0x00,0x00,0x10,0x03,0x90,0x00,0x64,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x38,0x02,0xff,0x02,0x58,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x05,0x06,0x10,0x54,0x01,0x01,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x00,0x04,0x04,0x02,0x02,0x00,0x01,0x28,0x00,0x01,0x00,0x18,0x04,0x04,0x05,0x05,0x00,\
            0x02,0x28,0x03,0x01,0x0e,0x03,0x00,0x00,0x2a,0x04,0x34,0x10,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x1c,0x10,0x00,0x03,0x2a,0x00,0x01,0x4d,0x65,0x63,0x68,0x79,0x20,0x43,0x6f,0x6e,0x74,\
            0x72,0x6f,0x6c,0x6c,0x65,0x72,0x04,0x04,0x05,0x05,0x00,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x38,0x28,0x03,0x01,0x02,0x05,0x00,0x01,0x2a,0x06,0x04,0x03,0x02,0x00,0x05,0x2a,0x01,\
            0x01,0xc3,0x03,0x04,0x04,0x05,0x05,0x00,0x06,0x28,0x03,0x01,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x54,0x02,0x07,0x00,0x04,0x2a,0x06,0x04,0x09,0x08,0x00,0x07,0x2a,0x04,0x01,0x06,0x00,\
            0x18,0x00,0x06,0x00,0x64,0x00,0x04,0x04,0x02,0x02,0x00,0x08,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x70,0x28,0x00,0x01,0x01,0x18,0x04,0x04,0x02,0x02,0x00,0x09,0x28,0x00,0x01,0x0f,0x18,\
            0x04,0x04,0x05,0x05,0x00,0x0a,0x28,0x03,0x01,0x12,0x0b,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x8c,0x19,0x2a,0x16,0x0c,0x02,0x01,0x00,0x0b,0x2a,0x19,0x01,0x32,0x46,0x34,0x03,0x02,\
            0x00,0x0c,0x29,0x02,0x01,0x00,0x00,0x04,0x04,0x05,0x05,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xa8,0x0d,0x28,0x03,0x01,0x02,0x0e,0x00,0x1a,0x2a,0x06,0x0c,0x02,0x01,0x00,0x0e,0x2a,\
            0x1a,0x01,0x2f,0x04,0x04,0x02,0x02,0x00,0x0f,0x28,0x00,0x01,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xc4,0x12,0x18,0x04,0x04,0x05,0x05,0x00,0x10,0x28,0x03,0x01,0x02,0x11,0x00,0x4a,0x2a,\
            0x06,0x0c,0x05,0x04,0x00,0x11,0x2a,0x4a,0x01,0x11,0x01,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xe0,0x01,0x04,0x04,0x05,0x05,0x00,0x12,0x28,0x03,0x01,0x04,0x13,0x00,0x4c,0x2a,0x46,\
            0x30,0x02,0x01,0x00,0x13,0x2a,0x4c,0x01,0x00,0x04,0x04,0x05,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xfc,0x05,0x00,0x14,0x28,0x03,0x01,0x02,0x15,0x00,0x4b,0x2a,0x04,0x0c,0x3d,0x3d,0x00,\
            0x15,0x2a,0x4b,0x01,0x05,0x01,0x09,0x04,0xa1,0x01,0x85,0x01,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x18,0xa1,0x02,0x09,0x32,0x09,0x31,0x09,0x30,0x15,0x00,0x26,0xff,0x00,0x35,0x00,0x46,\
            0xff,0x00,0x75,0x08,0x95,0x03,0x81,0x02,0xc0,0x85,0x02,0xa1,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x34,0x02,0x05,0x09,0x29,0x02,0x19,0x01,0x95,0x02,0x75,0x01,0x25,0x01,0x15,0x00,0x81,\
            0x02,0x95,0x01,0x75,0x06,0x81,0x01,0xc0,0xc0,0x04,0x04,0x05,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x50,0x05,0x00,0x16,0x28,0x03,0x01,0x12,0x17,0x00,0x4d,0x2a,0x14,0x0c,0x03,0x00,0x00,\
            0x17,0x2a,0x4d,0x01,0x00,0x00,0x00,0x46,0x34,0x03,0x02,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x6c,0x18,0x29,0x02,0x01,0x00,0x00,0x06,0x0c,0x03,0x02,0x00,0x19,0x29,0x08,0x01,0x01,\
            0x01,0x04,0x04,0x05,0x05,0x00,0x1a,0x28,0x03,0x01,0x12,0x1b,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x88,0x00,0x4d,0x2a,0x14,0x0c,0x01,0x00,0x00,0x1b,0x2a,0x4d,0x01,0x00,0x46,0x34,0x03,\
            0x02,0x00,0x1c,0x29,0x02,0x01,0x00,0x00,0x06,0x0c,0x03,0x02,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0xa4,0x00,0x1d,0x29,0x08,0x01,0x02,0x01,0x04,0x04,0x02,0x02,0x00,0x1e,0x28,0x00,0x01,\
            0x13,0x18,0x04,0x04,0x05,0x05,0x00,0x1f,0x28,0x03,0x01,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0xc0,0x20,0x00,0x4f,0x2a,0x46,0x30,0x05,0x04,0x00,0x20,0x2a,0x4f,0x01,0x00,0x00,0x00,\
            0x00,0x04,0x04,0x05,0x05,0x00,0x21,0x28,0x03,0x01,0x10,0x22,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0xdc,0x00,0x31,0x2a,0x14,0x00,0x01,0x00,0x00,0x22,0x2a,0x31,0x01,0x00,0x46,0x34,0x03,\
            0x02,0x00,0x23,0x29,0x02,0x01,0x00,0x00,0x04,0x04,0x02,0x02,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0xf8,0x00,0x24,0x28,0x00,0x01,0x0a,0x18,0x04,0x04,0x05,0x05,0x00,0x25,0x28,0x03,0x01,\
            0x02,0x26,0x00,0x50,0x2a,0x06,0x0c,0x08,0x07,0x00,0x26,0x2a,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x22,0x14,0x50,0x01,0x02,0x15,0x19,0xdd,0xdd,0x01,0x00,0x04,0x04,0x05,0x05,0x00,0x27,0x28,\
            0x03,0x01,0x02,0x28,0x00,0x27,0x2a,0x04,0x0c,0x09,0x01,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x11,0x06,0x22,0x30,0x28,0x2a,0x27,0x01,0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x00,0x2a,0x00,0x01,0x00,0x80,0x04,0x00,0x03,0x00,0x00,0x2a,0x19,0x01,0x00,0x82,0x04,\
            0x00,0x0b,0x00,0x0c,0x2a,0x1a,0x01,0x00,0x80,0x04,0x00,0x0e,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x1c,0x00,0x00,0x2a,0x4c,0x01,0x00,0x08,0x04,0x00,0x13,0x00,0x00,0x2a,0x4d,0x01,0x00,\
            0x82,0x04,0x00,0x17,0x00,0x18,0x2a,0x4d,0x01,0x00,0x82,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x38,0x00,0x1b,0x00,0x1c,0x2a,0x4f,0x01,0x00,0x08,0x04,0x00,0x20,0x00,0x00,0x2a,0x31,\
            0x01,0x00,0x02,0x04,0x00,0x22,0x00,0x23,0x2a,0x27,0x01,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x09,0x06,0x40,0x54,0x80,0x04,0x00,0x28,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1e,0x06,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x19,0x06,0x70,0x00,0x19,0x02,0xc3,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x06,0x06,0xf0,0x00,0x03,0xbd,0x51,\
        },\
    },\
}

#endif
