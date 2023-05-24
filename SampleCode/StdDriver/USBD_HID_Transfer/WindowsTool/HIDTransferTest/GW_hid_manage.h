#pragma once
#define PAGE_SIZE                       2048/*256*/
#define SECTOR_SIZE                     4096
#define HID_PACKET_SIZE                 64
#define HID_RESERVE_SIZE                3
#define HID_SUB_PACKET_SIZE             HID_PACKET_SIZE - (HID_RESERVE_SIZE - 1)
#define CMD_PAYLOAD_LEN                 0x12
#define SUB_CMD_PAYLOAD_LEN             (HID_PACKET_SIZE - HID_RESERVE_SIZE - 3)

#define AW5808_DUP_PAYLOAD_SIZE_LENGTH              0x04
#define AW5808_DUP_PAYLOAD_LENGTH                   0x10

#define VERSION_TYPE_PAYLOAD_VALID_LENGTH           0x04
#define VERSION_GET_STATUS_ERROR                    0xFF

typedef enum {
    AW5808_PAYLOAD_STATE_INDEX,
    AW5808_PAYLOAD_TYPE_INDEX,
    AW5808_PAYLOAD_DATA_INDEX
} AW5808_PAYLOAD_INDEX_E;

typedef enum {
    AW5808_PAYLOAD_STATE_IDLE,
    AW5808_PAYLOAD_STATE_START,
    AW5808_PAYLOAD_STATE_TRANSFER,
    AW5808_PAYLOAD_STATE_FINISH,
    AW5808_PAYLOAD_STATE_STOP
} AW5808_PAYLOAD_STATE_E;

typedef enum {
    AW5808_PAYLOAD_TYPE_IDLE,
    AW5808_PAYLOAD_TYPE_SHUT_OFF,
    AW5808_PAYLOAD_TYPE_RX,
    AW5808_PAYLOAD_TYPE_TX
} AW5808_PAYLOAD_TYPE_E;

typedef enum {
    AW5808_PROCESS_STATUS_SUCCESS,
    AW5808_PROCESS_STATUS_FILE_FAILED,
    AW5808_PROCESS_STATUS_START_FAILED,
    AW5808_PROCESS_STATUS_STOP_FAILED,
    AW5808_PROCESS_STATUS_SHUT_OFF,
    AW5808_PROCESS_STATUS_TIMEOUT,
    AW5808_PROCESS_STATUS_USB_FAILED
} AW5808_PROCESS_STATUS_E;

typedef enum {
    ATS3606_PROCESS_STATUS_SUCCESS,
    ATS3606_PROCESS_STATUS_FILE_FAILED,
    ATS3606_PROCESS_STATUS_START_FAILED,
    ATS3606_PROCESS_STATUS_STOP_FAILED,
    ATS3606_PROCESS_STATUS_SHUT_OFF,
    ATS3606_PROCESS_STATUS_TIMEOUT,
    ATS3606_PROCESS_STATUS_USB_FAILED
} ATS3606_PROCESS_STATUS_E;

typedef enum {
    VERSION_TYPE_MCU = 0x01,
    VERSION_TYPE_AW5808_RX,
    VERSION_TYPE_AW5808_TX,
} VERSION_TYPE_E;

typedef enum {
    VERSION_TYPE_INDEX,
    VERSION_TYPE_STATUS_INDEX,
    VERSION_TYPE_PAYLOAD_INDEX = (VERSION_TYPE_STATUS_INDEX + VERSION_TYPE_PAYLOAD_VALID_LENGTH)
} VERSION_TYPE_INDEX_E;

typedef enum {
    VERSION_GET_SUCCESS,
    VERSION_GET_FAILED
} VERSION_GET_STATUS_E;

typedef enum {
    HID_WRITE_IDLE_TYPE,
    HID_WRITE_BUFFER_TYPE,
    HID_WRITE_CMD_TYPE
} HID_WRITE_TYPE_E;

typedef struct
{
    UCHAR   cmd;
    UCHAR   reserved[HID_RESERVE_SIZE];
} DEV_HEADER_T;

typedef struct
{
    UCHAR   cmd;
    UCHAR   len;
    CHAR    payload[SUB_CMD_PAYLOAD_LEN];
} DEV_CMD_T;

typedef struct
{
    DEV_HEADER_T    cmd_t;
    DEV_CMD_T       sub_cmd_t;
} DEV_CMD_S;

typedef union
{
    DEV_CMD_S   cmd_std;
    CHAR        buffer[HID_PACKET_SIZE];
} HID_PAYLOAD_U;

typedef void (*HID_APP_EVT_CB)(DEV_CMD_S* app_evt_cmd_in, DEV_CMD_S* app_evt_cmd_out);

typedef struct
{
    UCHAR                   cmd;
    HID_APP_EVT_CB          cb;
} HID_APP_EVT_S;

void LogHeximal(PCHAR payload, UINT size);

void PrintHeximal(PCHAR payload, UINT size);

BOOL USBConnectionState(void);

BOOL HidBufferSend(PCHAR pcmd_dst, PCHAR pcmd_src, INT timeout_ms);

INT HidCmdSend(DEV_CMD_T* pcmd_dst, DEV_CMD_T* pcmd_src, INT timeout_ms);

INT HidCmdResponse(DEV_CMD_T* pcmd_src, INT timeout_ms);

UINT HidAppEventRegister(HID_APP_EVT_S params);

UINT HidAppEventUnregister(HID_APP_EVT_S params);

void UsbCmdTest(void);

void HidListenerInit(void);

void HidEventInit(void);

void HidListenerUninit(void);

BOOL VersionGet(DEV_CMD_T* pcmd_dst, UCHAR type);

BOOL ConfigurationSetSend(DEV_CMD_T* pcmd_dst, PCHAR pbuffer, BYTE length);

BOOL DeviceInfoGet(DEV_CMD_T* pcmd_dst, UCHAR type);

BOOL HidInterfaceStatus(void);

void HidParamsSet(UINT timeout_ms);

BOOL HidInterfaceOpen(void);

void HidInterfaceClose(void);
