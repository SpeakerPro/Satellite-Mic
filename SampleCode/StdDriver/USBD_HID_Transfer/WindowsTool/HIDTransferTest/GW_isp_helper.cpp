#include "stdafx.h"
#include "string.h"
#include "GW_isp_helper.h"
#include "GW_usb_info.h"
#include "GW_debug.h"
#include "GW_hid_manage.h"
#include "GW_hid_cmd_table.h"
#include "HID.hpp"

#define PAGE_SIZE       512/*256*/

#define HID_CMD_SIGNATURE						0x43444948

#define HID_MAX_PACKET_SIZE_EP 					64
#define ISP_COMMAND_POSITION					0
#define ISP_INDEX_POSITION						4
#define ISP_PAYLOAD_POSITION					8

#define ISP_PAYLOAD_START_ADDRESS_POSITION		0
#define ISP_PAYLOAD_TOTAL_LENGTH_POSITION		4
#define ISP_PAYLOAD_BUFFER_POSITION				8

#define USB_TIME_OUT							100
#define USB_ISP_TIME_OUT						5000
#define USB_LDROM_SLEEP_TIME_OUT				1000
#define USB_APROM_SLEEP_TIME_OUT				5000
#define USB_REPLUG_TIME_OUT						20000

#define HID_ISP_MODE_ENTER_V1					{0xF0, 0x0E, 0x78, 0x56, 0x34, 0x12, 0x01, 0xEF, 0xCD, 0xAB, 0x48, 0x49, 0x44, 0x43, 0x92, 0x05, 0x00, 0x00, 0x0A, 0x71, }
#define HID_VERSION_MCU_V2						{0xF0, 0x0E, 0x78, 0x56, 0x34, 0x12, 0x01, 0xEF, 0xCD, 0xAB, 0x48, 0x49, 0x44, 0x43, 0x92, 0x05, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, }

#define HID_ISP_CONNECT_CMD						{0xAE, 0x00, 0x01, 0x10, 0x01, }

typedef enum {
	UFP_PC,
	UFP_TV,
	UFP_NONE
} UFP_STATUS_E;

typedef enum {
	DEVICE_CHECK_CONNECT,
	DEVICE_DATA_FLASH_READY,
	DEVICE_DATA_FLASH_RUNNING,
	DEVICE_UPDATE_READY,
	DEVICE_UPDATE_RUNNING,
	DEVICE_DATA_LOAD_FINISHED,
	DEVICE_APROM_UPDATE
} device_status_e;

PCHAR	data_flash_payload, firmware_payload, p_payload;
UINT	total_len, data_addr;
FILE	* f;

void IspEventDataFlashStartCb(void)
{
	UINT length, payload_length;
	f = fopen((CONST PCHAR)data_flash_payload, "rb");
	fseek(f, 0, SEEK_END);
	length = ftell(f);
	fseek(f, 0, SEEK_SET);

	payload_length = length % PAGE_SIZE == 0 ? length : (length / PAGE_SIZE + 1) * PAGE_SIZE;
	p_payload = (PCHAR)malloc(payload_length);
	if (p_payload == NULL)
	{
		fclose(f);
		return;
	}
	fread(p_payload, length, sizeof(CHAR), f);

	total_len = length;

	debug_log("\r\n\r\n");
	debug_log("ISP event: data flash start...\r\n");
	debug_log("Data flash package size %ld...\r\n", length);
}

void IspEventDataFlashStopCb(void)
{
	debug_log("\n\n");
	debug_log("ISP event: data flash stop...\r\n");
	free(p_payload);
	fclose(f);
}

void IspEventIspStartCb(void)
{
	UINT length, payload_length;
	f = fopen((CONST PCHAR)firmware_payload, "rb");
	fseek(f, 0, SEEK_END);
	length = ftell(f);
	fseek(f, 0, SEEK_SET);

	payload_length = length % PAGE_SIZE == 0 ? length : (length / PAGE_SIZE + 1) * PAGE_SIZE;
	p_payload = (PCHAR)malloc(payload_length);
	if (p_payload == NULL)
	{
		fclose(f);
		return;
	}
	fread(p_payload, length, sizeof(UCHAR), f);

	total_len = length;

	debug_log("\n\n");
	debug_log("ISP event: ISP start...\r\n");
	debug_printf("ISP task: Package Size %ld\r\n", length);
}

void IspEventIspStopCb(void)
{
	debug_log("\n\n");
	debug_log("ISP event: ISP stop...\r\n");
	free(p_payload);
	fclose(f);
}

void IspEventDataUpdateCb(UINT address, PUCHAR buffer, UINT size)
{
	data_addr = address;
	debug_log("\tPackage update address %08X\r\n", address);
	memcpy(buffer, p_payload + address, 64);
}

BOOL RunModeState(PUCHAR state)
{
	CHidCmd io;
	BOOL bRet = FALSE;
	HID_PAYLOAD_U	tBuffer_IspConn = HID_ISP_CONNECT_CMD, tBuffer_TestReserve = HID_VERSION_MCU_V2, rBuffer;

	*state = DEVICE_IDLE_MODE;
	if (HidBufferSend((PCHAR)&rBuffer, (PCHAR)&tBuffer_IspConn, USB_TIME_OUT))
	{
		bRet = VersionGet(&rBuffer.cmd_std.sub_cmd_t, VERSION_TYPE_MCU);
		if (!bRet)
		{
			bRet = HidBufferSend((PCHAR)&rBuffer, (PCHAR)&tBuffer_TestReserve, USB_TIME_OUT);
			if (!bRet)
			{
				debug_printf("ISP aprom- found!\r\n");
				*state = DEVICE_RESERVED_MODE;
			}
		}
		else
		{
			debug_printf("ISP aprom found!\r\n");
			*state = DEVICE_APROM_MODE;
		}
	}
	else
	{
		debug_printf("ISP ldrom found!\r\n");
		*state = DEVICE_LDROM_MODE;
	}

lexit:
	return *state == DEVICE_IDLE_MODE ? FALSE: TRUE;
}

BOOL LdromModeEnter(UCHAR status)
{
	BOOL bRes = 0, bRet = FALSE, is_ldrom_boot = FALSE;
	HID_PAYLOAD_U	rBuffer, txBuffer = HID_ISP_MODE_ENTER_V1;

	if (status == DEVICE_APROM_MODE)
	{
		memset(&rBuffer, 0, sizeof(HID_PAYLOAD_U));
		memset(&txBuffer, 0, sizeof(HID_PAYLOAD_U));
		txBuffer.cmd_std.sub_cmd_t.cmd = GW_HID_CMD_TEST;
		txBuffer.cmd_std.sub_cmd_t.len = 1;
		txBuffer.cmd_std.sub_cmd_t.payload[0] = 0x00;
		bRes = HidCmdSend(&rBuffer.cmd_std.sub_cmd_t, &txBuffer.cmd_std.sub_cmd_t, USB_TIME_OUT);
		if (bRes == 0 && rBuffer.cmd_std.sub_cmd_t.len == 1)
		{
			memset(&rBuffer, 0, sizeof(HID_PAYLOAD_U));
			memset(&txBuffer, 0, sizeof(HID_PAYLOAD_U));
			txBuffer.cmd_std.sub_cmd_t.cmd = GW_HID_CMD_ROM_MODE;
			txBuffer.cmd_std.sub_cmd_t.len = 1;
			txBuffer.cmd_std.sub_cmd_t.payload[0] = 0x00;
			bRes = HidCmdSend(&rBuffer.cmd_std.sub_cmd_t, &txBuffer.cmd_std.sub_cmd_t, USB_TIME_OUT);
			if (bRes == 0 && rBuffer.cmd_std.sub_cmd_t.payload[0] == 0)
			{
				is_ldrom_boot = TRUE;
			}
			else
				goto lexit;
		}
		else if (bRes == 0 && rBuffer.cmd_std.sub_cmd_t.len == 4)
		{
			is_ldrom_boot = TRUE;
		}

		if (bRes == 0 && is_ldrom_boot)
		{
			memset(&rBuffer, 0, sizeof(HID_PAYLOAD_U));
			memset(&txBuffer, 0, sizeof(HID_PAYLOAD_U));
			txBuffer.cmd_std.sub_cmd_t.cmd = GW_HID_CMD_ROM_REBOOT;
			txBuffer.cmd_std.sub_cmd_t.len = 1;
			txBuffer.cmd_std.sub_cmd_t.payload[0] = 0x00;
			bRes = HidCmdSend(&rBuffer.cmd_std.sub_cmd_t, &txBuffer.cmd_std.sub_cmd_t, USB_TIME_OUT);
			bRet = TRUE;
		}
	}
	else
	{
		bRes = HidBufferSend((PCHAR)&rBuffer, (PCHAR)&txBuffer, USB_TIME_OUT);
		bRet = TRUE;
	}
lexit:
	return bRet;
}

BOOL ApromModeEnter(void)
{
	BOOL bRet = 0;
	HID_PAYLOAD_U	rBuffer = { 0 }, txBuffer = { 0 };

	txBuffer.cmd_std.sub_cmd_t.cmd = GW_HID_CMD_ROM_MODE;
	txBuffer.cmd_std.sub_cmd_t.len = 1;
	txBuffer.cmd_std.sub_cmd_t.payload[0] = 0x01;
	bRet = HidCmdSend(&rBuffer.cmd_std.sub_cmd_t, &txBuffer.cmd_std.sub_cmd_t, USB_TIME_OUT);
	return bRet == 0 && rBuffer.cmd_std.sub_cmd_t.payload[0] == 0 ? TRUE : FALSE;
}

BOOL IspHelperRun(void)
{
	BOOL		bRet = 0;
	UCHAR		conn_retry = 0;
	UCHAR		cmd_in[HID_MAX_PACKET_SIZE_EP] = {0}, cmd_out[HID_MAX_PACKET_SIZE_EP] = {0}, flash_buffer[FLASH_BLOCK_SIZE] = {0}, isp_buff[HID_MAX_PACKET_SIZE_EP] = {0};
	UINT 		cur_addr, start_addr, app_task = CMD_CONNECT, cmd_index = 1, timeout_ms;
	DWORD		length;
	device_status_e	device_state = DEVICE_CHECK_CONNECT;

	debug_printf("ISP helper runs.\r\n");
	while (device_state != DEVICE_APROM_UPDATE)
	{
		memset(isp_buff, 0, sizeof(isp_buff));
		if (device_state != DEVICE_DATA_FLASH_RUNNING && device_state != DEVICE_UPDATE_RUNNING)
			memcpy(isp_buff + ISP_COMMAND_POSITION, &app_task, sizeof(app_task));

		memcpy(isp_buff + ISP_INDEX_POSITION, &cmd_index, sizeof(cmd_index));
		switch (app_task)
		{
			case CMD_CONNECT:
				debug_printf("ISP task: Connect\r\n");
				app_task = CMD_SYNC_PACKNO;
				break;

			case CMD_SYNC_PACKNO:
				debug_printf("ISP task: Sync Packno\r\n");
				memcpy(isp_buff + ISP_PAYLOAD_POSITION, &cmd_index, sizeof(cmd_index));
				if (device_state == DEVICE_CHECK_CONNECT)
				{
					app_task = CMD_GET_FWVER;
				}
				else if (device_state == DEVICE_DATA_FLASH_READY)
				{
					app_task = CMD_UPDATE_DATAFLASH;
					IspEventDataFlashStartCb();
					cur_addr = 0;
					start_addr = 0;
				}
				else if (device_state == DEVICE_UPDATE_READY)
				{
					app_task = CMD_UPDATE_APROM;
					IspEventIspStartCb();
					cur_addr = 0;
					start_addr = 0;
				}
				break;

			case CMD_GET_FWVER:
				debug_printf("ISP task: Firmware Get\r\n");
				app_task = CMD_GET_DEVICEID;
				break;

			case CMD_GET_DEVICEID:
				debug_printf("ISP task: Device Get\r\n");
				app_task = CMD_READ_CONFIG;
				break;

			case CMD_READ_CONFIG:
				debug_printf("ISP task: Config Read\r\n");
				app_task = CMD_SYNC_PACKNO;
				if (data_flash_payload != NULL)
					device_state = DEVICE_DATA_FLASH_READY;
				else if (firmware_payload != NULL)
					device_state = DEVICE_UPDATE_READY;
				else
					return FALSE;
				break;

			case CMD_UPDATE_DATAFLASH:
			case CMD_UPDATE_APROM:
				if (app_task == CMD_UPDATE_DATAFLASH)
				{
					app_task = CMD_UPDATE_DATAFLASH;
					device_state = DEVICE_DATA_FLASH_RUNNING;
				}
				else if (app_task == CMD_UPDATE_APROM)
				{
					app_task = CMD_UPDATE_APROM;
					device_state = DEVICE_UPDATE_RUNNING;
				}

				unsigned long	write_len, packet_len, flash_addr;
				write_len = packet_len = total_len - (cur_addr - start_addr);
				flash_addr = cur_addr;
				IspEventDataUpdateCb(cur_addr, flash_buffer, sizeof(flash_buffer));

				if (start_addr == cur_addr)
				{
					if (write_len > sizeof(isp_buff) - (ISP_PAYLOAD_POSITION + ISP_PAYLOAD_BUFFER_POSITION))
						write_len = sizeof(isp_buff) - (ISP_PAYLOAD_POSITION + ISP_PAYLOAD_BUFFER_POSITION);

					memcpy(isp_buff + ISP_PAYLOAD_POSITION + ISP_PAYLOAD_START_ADDRESS_POSITION, &start_addr, sizeof(start_addr));
					memcpy(isp_buff + ISP_PAYLOAD_POSITION + ISP_PAYLOAD_TOTAL_LENGTH_POSITION, &total_len, sizeof(start_addr));

					memcpy(isp_buff + ISP_PAYLOAD_POSITION + ISP_PAYLOAD_BUFFER_POSITION, flash_buffer, write_len);
					cur_addr += write_len;
				}
				else
				{
					if (write_len > sizeof(isp_buff) - ISP_PAYLOAD_POSITION)
						write_len = sizeof(isp_buff) - ISP_PAYLOAD_POSITION;

					memcpy(isp_buff + ISP_PAYLOAD_POSITION, flash_buffer, write_len);

					if (packet_len <= (HID_MAX_PACKET_SIZE_EP - ISP_PAYLOAD_POSITION))
					{
						if (app_task == CMD_UPDATE_DATAFLASH)
						{
							app_task = CMD_SYNC_PACKNO;
							device_state = DEVICE_UPDATE_READY;
							IspEventDataFlashStopCb();
						}
						else if (app_task == CMD_UPDATE_APROM)
						{
							app_task = CMD_RUN_APROM;
							device_state = DEVICE_DATA_LOAD_FINISHED;
							IspEventIspStopCb();
						}
					}
					cur_addr += write_len;
				}
				debug_printf("ISP task: Aprom Update. State: %3d %s", ((flash_addr + write_len)*100)/total_len, app_task == CMD_RUN_APROM ? "\r\n" : "\r");
				break;

			case CMD_RUN_APROM:
				debug_printf("ISP task: Aprom Run\r\n");
				device_state = DEVICE_APROM_UPDATE;
				break;
		}

		if (app_task == CMD_SYNC_PACKNO)
			cmd_index = 1;
		else
			cmd_index += 2;

		memcpy(cmd_in, isp_buff, sizeof(isp_buff));

		if (device_state == DEVICE_APROM_UPDATE)
			timeout_ms = 0;
		else
			timeout_ms = USB_ISP_TIME_OUT;

		bRet = HidBufferSend((PCHAR)&cmd_out, (PCHAR)&cmd_in, timeout_ms);
		if (bRet)
			break;
	}

	return bRet == 0 ? TRUE: FALSE;
}

UINT IspMcuUpdate(PCHAR isp_file, PCHAR data_file, BOOL isp_aprom, BOOL ufp_check)
{
	UCHAR			status;
	UINT			Ret = ISP_HELPER_SUCCESSFUL;

	firmware_payload = isp_file;
	data_flash_payload = data_file;

	debug_printf("ISP port open and ldrom/aprom check...\r\n");
	if (HidInterfaceOpen() == FALSE)
		return ISP_HELPER_OPEN_FAILED;

	if (!RunModeState(&status))
	{
		debug_printf("ISP No mode-check response, stop ISP process!\r\n");
		Ret = ISP_HELPER_FIRST_CHECK_NO_RESPONSE;
		goto lexit;
	}
	else
	{
		if (status != DEVICE_LDROM_MODE)
		{
			if (ufp_check)
			{
				DEV_CMD_T	msg;
				BOOL res = DeviceInfoGet(&msg, NULL);
				if (!res || msg.payload[0] != UFP_PC)
				{
					debug_printf("ISP ufp check failed, status %s \r\n", msg.payload[0] == UFP_TV? "TV": "None");
					Ret = ISP_HELPER_UFP_STATUS_CHECK_FAILED;
					goto lexit;
				}
				else
					debug_printf("ISP ufp check successful \r\n");
			}

			if (LdromModeEnter(status) == FALSE)
			{
				debug_printf("ISP ldrom enter failed!\r\n");
				Ret = ISP_HELPER_LDROM_ENTER_FAILED;
				goto lexit;
			}
			else
				debug_printf("ISP ldrom enter successful.\r\n");
		}
		else
			debug_printf("ISP ldrom status\r\n");
	}
	HidInterfaceClose();

	Sleep(USB_LDROM_SLEEP_TIME_OUT);

	if (HidInterfaceOpen() == FALSE)
		return ISP_HELPER_OPEN_FAILED;

	if (!IspHelperRun())
	{
		Ret = ISP_HELPER_RUN_FAILED;
		goto lexit;
	}
	HidInterfaceClose();

	debug_printf("ISP port re-open and aprom check...\r\n");
	Sleep(USB_APROM_SLEEP_TIME_OUT);

	HidParamsSet(USB_REPLUG_TIME_OUT);

	if (HidInterfaceOpen() == FALSE)
		return ISP_HELPER_OPEN_FAILED;

	RunModeState(&status);
	if (status != DEVICE_APROM_MODE)
	{
		debug_printf("ISP upgrade failed, no mode-check response. Mode: %s!\r\n", status == DEVICE_LDROM_MODE? "LDROM": "Reserved");
		Ret = ISP_HELPER_APROM_CHECK_FAILED;
		goto lexit;
	}
	else
	{
		if (isp_aprom)
		{
			if (!ApromModeEnter())
			{
				debug_printf("ISP aprom mode enter failed!\r\n");
				Ret = ISP_HELPER_APROM_ENTER_FAILED;
				goto lexit;
			}
			debug_printf("ISP aprom mode enter successful.\r\n");
		}
	}
lexit:
	HidInterfaceClose();

	return Ret;
}
