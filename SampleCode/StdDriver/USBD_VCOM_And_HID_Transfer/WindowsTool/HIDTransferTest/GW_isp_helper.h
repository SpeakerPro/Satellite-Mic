#pragma once
#include <stdio.h>

#define CMD_PRODUCT				0x10010000

#define CMD_UPDATE_APROM		(CMD_PRODUCT | 0x000000A0)
#define CMD_UPDATE_CONFIG		(CMD_PRODUCT | 0x000000A1)
#define CMD_READ_CONFIG			(CMD_PRODUCT | 0x000000A2)
#define CMD_ERASE_ALL			(CMD_PRODUCT | 0x000000A3)
#define CMD_SYNC_PACKNO			(CMD_PRODUCT | 0x000000A4)
#define CMD_GET_FWVER			(CMD_PRODUCT | 0x000000A6)
#define CMD_RUN_APROM			(CMD_PRODUCT | 0x000000AB)
#define CMD_RUN_LDROM			(CMD_PRODUCT | 0x000000AC)
#define CMD_RESET				(CMD_PRODUCT | 0x000000AD)
#define CMD_CONNECT				(CMD_PRODUCT | 0x000000AE)
#define CMD_GET_DEVICEID		(CMD_PRODUCT | 0x000000B1)
#define CMD_UPDATE_DATAFLASH	(CMD_PRODUCT | 0x000000C3)
#define CMD_RESEND_PACKET		(CMD_PRODUCT | 0x000000FF)

#define FLASH_BLOCK_SIZE													256
#define FLASH_RESERVE_PAGE													4
#define	FLASH_APPLICATION_START_INDEX										FLASH_BLOCK_SIZE*FLASH_RESERVE_PAGE

#define ISP_RETRY_COUNT			0x05

typedef enum {
	DEVICE_IDLE_MODE,
	DEVICE_LDROM_MODE,
	DEVICE_RESERVED_MODE,
	DEVICE_APROM_MODE
} DEVICE_RUNNING_MODE;

typedef enum {
	ISP_HELPER_SUCCESSFUL,
	ISP_HELPER_OPEN_FAILED,
	ISP_HELPER_FIRST_CHECK_NO_RESPONSE,
	ISP_HELPER_LDROM_ENTER_FAILED,
	ISP_HELPER_RUN_FAILED,
	ISP_HELPER_FINAL_OPEN_FAILED,
	ISP_HELPER_APROM_CHECK_FAILED,
	ISP_HELPER_APROM_ENTER_FAILED,
	ISP_HELPER_UFP_STATUS_CHECK_FAILED
} ISP_HELPER_MACHINE_CODE_E;

typedef void (*isp_data_flash_start_cb)(void);
typedef void (*isp_data_flash_stop_cb)(void);
typedef void (*isp_start_cb)(void);
typedef void (*isp_end_cb)(void);
typedef void (*isp_data_update_cb)(UINT address, PUCHAR buffer, UINT size);

typedef struct {
	isp_data_flash_start_cb		data_flash_start;
	isp_data_flash_stop_cb		data_flash_stop;
	isp_start_cb				isp_start;
	isp_end_cb					isp_stop;
	isp_data_update_cb			data_update;
} isp_event_s;

BOOL LdromModeEnter(UCHAR status);

BOOL ApromModeEnter(void);

UINT IspMcuUpdate(PCHAR isp_file, PCHAR data_file, BOOL isp_aprom, BOOL ufp_check);
