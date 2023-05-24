#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include "NuMicro.h"
#include "GW_hid_Transfer.h"
#include "GW_hid_manage.h"
#include "GW_hid_cmd_table.h"
#include "GW_main.h"

#define HID_CMD_NONE     			0x00

//#define IGNORE_PRINTF
#ifndef IGNORE_PRINTF
	#define printf(fmt, ...) (0)
#endif

static uint32_t	epno, epsize;
static uint32_t page_size;
static hid_app_evt_s app_evt_list[APP_EVENT_COUNT];

void evt_a_cb(DEV_CMD_S * msg_out, DEV_CMD_S * msg_in);
void evt_b_cb(DEV_CMD_S * msg_out, DEV_CMD_S * msg_in);
void evt_ldrom_boot_cb(DEV_CMD_S * msg_out, DEV_CMD_S * msg_in);
uint32_t PrintMsg(char *fmt, ...)
{
		return 0;
}

uint32_t PrintMsg__(char *fmt, ...) 
{
		return 0;

		char msg[512];
		uint32_t len;

    va_list args;
    va_start(args, fmt);
    len = vsprintf(msg, fmt, args);
		va_end(args);
		//printf("%s", msg);
		printf("%7d: %s", m_module.current_time_ms, msg);
    return len;
}

void PrintHeximal(const char* _header, unsigned char* _payload, unsigned long _size)
{
		return;

		printf(" - %s", _header);
		uint8_t i;
		for (i =0;i<_size;i++)
		{
				if ((i % 16) == 0)
						printf("\n  ");
						//printf("\n%08X:  ", i);
				printf("%02X ", _payload[i]);
		}
		printf("\r\n\r\n");
		//printf("\r\n\tMessage length: %ld\r\n\r\n", _size);
}

void hid_event_show(void)
{
		uint8_t i;
		for(i=0;i<APP_EVENT_COUNT;i++)
		{
				//printf("Check Check Cmd[%02d] %X\r\n", i, app_evt_list[i].cmd);
		}
}

static void page_erase(uint32_t start_page, uint32_t pages)
{
		uint8_t	 payload[FLASH_BLOCK_SIZE];
    uint32_t u32FlashAddress, u32PageNumber, u32FlashAddressIndex;
		uint32_t i;

		memset(payload, 0xFF, sizeof(payload));

		u32FlashAddress = FLASH_APPLICATION_START_INDEX + start_page*page_size;
		u32PageNumber		= pages*(page_size/FLASH_BLOCK_SIZE);
		//printf("Flash erase. Page %04X-%04X, Flash memory %08X-%08X erased\n", start_page, start_page + pages - 1, u32FlashAddress, u32FlashAddress + pages*page_size);

		for(i=0;i<u32PageNumber;i++)
		{
				//u32FlashAddressIndex = i*FLASH_BLOCK_SIZE;
				//SpiFlash_NormalPageProgram(u32FlashAddress + u32FlashAddressIndex, payload);
				//SpiFlash_WaitReady();
		}
}

static void page_write(uint32_t start_page, uint32_t pages, uint8_t * payload)
{
    uint32_t u32FlashAddress, u32PageNumber, u32FlashAddressIndex;
		uint32_t i;

		u32FlashAddress = FLASH_APPLICATION_START_INDEX + start_page*page_size;
		u32PageNumber		= pages*(page_size/FLASH_BLOCK_SIZE);
		//printf("Flash write. Page %04X-%04X, Flash memory %08X-%08X writen\n", start_page, start_page + pages - 1, u32FlashAddress, u32FlashAddress + pages*page_size);

		for(i=0;i<u32PageNumber;i++)
		{
				//u32FlashAddressIndex = i*FLASH_BLOCK_SIZE;
				//SpiFlash_NormalPageProgram(u32FlashAddress + u32FlashAddressIndex, payload + u32FlashAddressIndex);
				//SpiFlash_WaitReady();
		}

}

static void page_read(uint32_t start_page, uint32_t pages, uint8_t * payload)
{
    uint32_t u32FlashAddress, u32PageNumber, u32FlashAddressIndex;
		uint32_t i;

		u32FlashAddress = FLASH_APPLICATION_START_INDEX + start_page*page_size;
		u32PageNumber		= pages*(page_size/FLASH_BLOCK_SIZE);
		//printf("Flash read. Page %04X-%04X, Flash memory %08X-%08X read\n", start_page, start_page + pages - 1, u32FlashAddress, u32FlashAddress + pages*page_size);

		for(i=0;i<u32PageNumber;i++)
		{
				//u32FlashAddressIndex = i*FLASH_BLOCK_SIZE;
				//SpiFlash_NormalRead(u32FlashAddress + u32FlashAddressIndex, payload + u32FlashAddressIndex);
		}
}

bool        hid_ack_pend;
DEV_CMD_T   cmd_dst;
bool GW_hid_response(DEV_CMD_T * msg_in)
{
		DEV_CMD_S		payload = {0};

		payload.cmd_t.cmd = HID_CMD_EVENT_RESPONSE;
		memcpy(&payload.sub_cmd_t, msg_in, sizeof(payload.sub_cmd_t));

		USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(epno)), (void *)&payload, epsize);
		USBD_SET_PAYLOAD_LEN(epno, epsize);

		return true;
}

void GW_hid_send(DEV_CMD_T * msg_in, uint16_t timeout)
{
		DEV_CMD_S		payload = {0};
		uint16_t		timeout_count = timeout;
		uint32_t		_epno, _epsize;

		payload.cmd_t.cmd = HID_CMD_EVENT_REQUEST;
		memcpy(&payload.sub_cmd_t, msg_in, sizeof(payload.sub_cmd_t));

		USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(epno)), (void *)&payload, epsize);
		USBD_SET_PAYLOAD_LEN(epno, epsize);

		hid_ack_pend = true;
		cmd_dst.cmd = msg_in->cmd;
		//PrintHeximal("HidCmdRequest:", (uint8_t *)&payload, 64);
}

static uint8_t cmd_process(uint8_t * payload, uint8_t len)
{
		bool 				cmd_match = false;
		DEV_CMD_S		* msg_in = (DEV_CMD_S *)payload, msg_out = {0};
		uint8_t 		i, res = 0, cmd = msg_in->cmd_t.cmd;

		if(cmd == HID_CMD_EVENT_REQUEST || cmd == HID_CMD_EVENT_RESPONSE)
		{
			  #if 0
				//printf("\r\n %s event: %2X\r\n", cmd == HID_CMD_EVENT_REQUEST? "Request": "Ack", msg_in->sub_cmd_t.cmd);
				#endif		
				for(i=0;i<APP_EVENT_COUNT;i++)
				{
						if(app_evt_list[i].cmd == msg_in->sub_cmd_t.cmd || app_evt_list[i].cmd == GW_HID_CMD_WILDCARD)
						{
								app_evt_list[i].cb(&msg_out, msg_in);
								if(cmd == HID_CMD_EVENT_REQUEST)
										msg_out.cmd_t.cmd = HID_CMD_EVENT_RESPONSE;
								else
										msg_out.sub_cmd_t.len = 0;

								if(msg_out.sub_cmd_t.len > 0)
								{
									  #if 0 
										//PrintHeximal("HidCmdResponse:", (uint8_t *)&msg_out, 64);
									  #endif
										USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(epno)), (void *)&msg_out, epsize);
										USBD_SET_PAYLOAD_LEN(epno, epsize);
								}

								if(app_evt_list[i].cmd != GW_HID_CMD_WILDCARD)
								{
										cmd_match = true;
										res = 1;
										break;
								}
						}
				}
		}
#if 0
		if(!cmd_match)
				//printf("\r\n HID event: unknown command %X, sub cmd %X\r\n", cmd, msg_in->sub_cmd_t.cmd);
#endif
    return res;
}

uint8_t GW_hid_app_event_register(hid_app_evt_s params)
{
		uint8_t	i = 1, res = 2;

		if(params.cmd == 0)
				return 1;
		else if(params.cmd == GW_HID_CMD_WILDCARD)
			i = 0;

		for(;i<APP_EVENT_COUNT;i++)
		{
				if(app_evt_list[i].cmd == 0)
				{
						app_evt_list[i].cmd = params.cmd;
						app_evt_list[i].cb = params.cb;
						res = 0;
						break;
				}
		}
		return res;
}

uint8_t GW_hid_app_event_unregister(hid_app_evt_s params)
{
		uint8_t	i, res = 2;

		if(params.cmd == 0)
				return 1;

		for(i=0;i<APP_EVENT_COUNT;i++)
		{
				if(app_evt_list[i].cmd == params.cmd)
				{
						app_evt_list[i].cmd = 0;
						app_evt_list[i].cb = NULL;
						res = 0;
						break;
				}
		}
		return res;
}

void GW_hid_init(uint32_t _epno)
{
		uint8_t i;
		hid_event_cb	task_cb;
		hid_app_evt_s evt_a, evt_b, evt_c, evt_d;

		if(_epno == EP2)
		{
				epno = EP2;
				epsize = EP2_MAX_PKT_SIZE;
		}
		else if(_epno == EP5)
		{
				epno = EP5;
				epsize = EP5_MAX_PKT_SIZE;
		}
		
		USBD_Open(&gsInfo, HID_ClassRequest, NULL);
    HID_Init();
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);

		task_cb.pageErase_cb 	=	page_erase;
		task_cb.pageWrite_cb 	=	page_write;
		task_cb.pageRead_cb 	=	page_read;
		task_cb.cmdProcess_cb =	cmd_process;

		GW_hid_event_init(task_cb);
		page_size = GW_hid_page_size();

		memset((uint8_t *)app_evt_list, 0, sizeof(APP_EVENT_COUNT));
}
