#ifndef __GW_HID_APP_EVENT
#define __GW_HID_APP_EVENT

#include "NuMicro.h"
#include "GW_hid_cmd_table.h"

#define EP_MAX_PKT_SIZE										64
#define HID_CMD_PREFIX_LENGTH							0x12
#define HID_DEV_CMD_LENGTH  							(EP_MAX_PKT_SIZE - HID_CMD_PREFIX_LENGTH - 2)

#define HID_RESERVE_SIZE                	3

#define FLASH_BLOCK_SIZE									256
#define FLASH_RESERVE_PAGE								4
#define	FLASH_APPLICATION_START_INDEX			FLASH_BLOCK_SIZE*FLASH_RESERVE_PAGE

#define SUB_CMD_PAYLOAD_LEN             (EP_MAX_PKT_SIZE - HID_RESERVE_SIZE - 3)

typedef struct __attribute__((__packed__))
{
    uint8_t u8Cmd;
    uint8_t u8Size;
    uint32_t u32Arg1;
    uint32_t u32Arg2;
    uint32_t u32Signature;
    uint32_t u32Checksum;
} HID_CMD_T;

typedef struct __attribute__((__packed__))
{
    unsigned char   cmd;
    unsigned char   reserved[HID_RESERVE_SIZE];
} DEV_HEADER_T;

typedef struct __attribute__((__packed__))
{
    unsigned char   cmd;
    unsigned char   len;
    unsigned char   payload[SUB_CMD_PAYLOAD_LEN];
} DEV_CMD_T;

typedef struct __attribute__((__packed__))
{
    DEV_HEADER_T	cmd_t;
    DEV_CMD_T   sub_cmd_t;
} DEV_CMD_S;

typedef void (* hid_page_erase)(uint32_t start_page, uint32_t pages);
typedef void (* hid_page_write)(uint32_t start_page, uint32_t pages, uint8_t * payload);
typedef void (* hid_page_read)(uint32_t start_page, uint32_t pages, uint8_t * payload);
typedef uint8_t (* hid_page_process)(uint8_t * payload, uint8_t len);
typedef void (* hid_app_evt_cb)(DEV_CMD_S * msg_in, DEV_CMD_S * msg_out);

typedef struct{
		hid_page_erase							pageErase_cb;
		hid_page_write							pageWrite_cb;
		hid_page_read								pageRead_cb;
		hid_page_process						cmdProcess_cb;
} hid_event_cb;

typedef struct
{
    uint8_t					cmd;
		hid_app_evt_cb	cb;
} hid_app_evt_s;

#endif
