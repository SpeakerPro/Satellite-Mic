#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include "NuMicro.h"
#include "GW_main.h"
#include "GW_hid_Transfer.h"
#include "GW_hid_manage.h"
#include "GW_hid_cmd_table.h"
#include "GW_hid_app.h"
#include "GW_aw5808.h"

#define GW_HID_TIMEOUT_AW5808_MS				100
#define ISP_APROM_BIT										0x80

#define IGNORE_PRINTF
#ifdef IGNORE_PRINTF
	#define printf(fmt, ...) (0)
#endif

//#define TIMER_SIMULATOR_ENABLE

uint8_t 	module_buffer[AW5808_DUP_PAYLOAD_LENGTH];
uint32_t 	file_index, file_size, module_state, process_state;

void aw5808_simulator_finish(void);
void aw5808_simulator_query(void);

void evt_echo_cb(DEV_CMD_S * msg_out, DEV_CMD_S * msg_in);
void evt_b_cb(DEV_CMD_S * msg_out, DEV_CMD_S * msg_in);
void evt_ldrom_boot_cb(DEV_CMD_S * msg_out, DEV_CMD_S * msg_in);
hid_app_cs_cb				hid_app_cs_evt;
hid_app_ats3607_cb	ats3607_r_evt, ats3607_w_evt;

bool GW_hid_app_cs_response(uint8_t * payload, uint8_t len)
{
		DEV_CMD_T msg = {.cmd = GW_HID_CMD_CONFIGURATION_SET, .len = len};
		memcpy(msg.payload, payload, len);
		return GW_hid_response(&msg);
}

void configuration_set_event(uint8_t * payload, uint8_t len)
{
// Ex. Data: prefix + payload
#if 1
		if((USB_raw_req_len == 0) && (len > 0) && (len <64)){
			memcpy(USB_raw_request, payload, len);
			USB_raw_req_len = len;
		}
		GW_response_USB_direct();

#else
		uint8_t buffer[64], prefix[] = {0x01, 0x81, 0x71, 0x61, 0x51, 0x41}, i;
		memcpy(buffer, prefix, sizeof(prefix));
		memcpy(buffer + sizeof(prefix), payload, len);
		GW_hid_app_cs_response(buffer, sizeof(prefix) + len);
#endif
}

bool GW_hid_app_ats3607_response(uint8_t * payload, uint8_t len)
{
		DEV_CMD_T msg = {.cmd = GW_HID_CMD_ATS3607_UPGRADE, .len = len + ATS3607_DUP_PAYLOAD_INDEX};
		memcpy(msg.payload + ATS3607_DUP_PAYLOAD_INDEX, payload, len);
		return GW_hid_response(&msg);
}

#define VCOM_USED
#define ATS3607_CMD_UPDATE_MAX_LEN							6
#define ATS3607_BUFFER_LENGTH										15
#define ATS3607_UART_BUFFER_LENGTH							64
#define ATS3607_UART_RX_TIMEOUT_MS							50
#define ATS3607_UART_RX_EVENT_TIMEOUT_MS				1000

#define ATS3607_VERSION_UPDATE_TIMEOUT_MS				100

#define ATS3607_UPDATE_IDLE              				0x00
#define ATS3607_UPDATE_VERSION              		0xFB
#define ATS3607_UPDATE_STATUS              			0xFD
#define ATS3607_UPDATE_START              			0xFE
#define ATS3607_UPDATE_STOP              				0xFF
#define ATS3607_UPDATE_OPEN                     0x55
#define ATS3607_UPDATE_TRANSFER                 0x56
#define ATS3607_UPDATE_CLOSE                    0x57
#define ATS3607_PAYLOAD_SUFFIX									0x59

bool							ats3607_uart_new, ats3607_update_enable, ats3607_event_enable;
volatile uint8_t 	ats3607_uart2usb_buffer[ATS3607_BUFFER_LENGTH], ats3607_usb2uart_buffer[EP3_MAX_PKT_SIZE];
volatile uint8_t 	ats3607_target_state, ats3607_update_state = ATS3607_UPDATE_IDLE;
volatile uint8_t 	ats3607_uart2usb_length, ats3607_uart2usb_last_index, ats3607_uart2usb_index, ats3607_uart_index, ats3607_uart_buffer[ATS3607_UART_BUFFER_LENGTH], ats3607_uart_length, ats3607_uart_code, ats3607_uart_crc;
volatile uint32_t ats3607_update_timeout_ms, ats3607_update_used_ms, ats3607_version_code;
volatile uint8_t 	ats3607_event_cmd_code, ats3607_event;

void ats3607_uart_char(uint8_t chx)
{
		while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk){}
		UART0->DAT = chx;
}

void ats3607_uart2usb_xfer(void)
{
#ifdef VCOM_USED
    if (g_u32TxSize == 0)
		{
				if(ats3607_uart2usb_length)
				{
						uint8_t len = USBD_GET_PAYLOAD_LEN(EP2);
						ats3607_uart2usb_length -= len;
						if(ats3607_uart2usb_length == 0)
						{
								if(ats3607_update_state == ATS3607_UPDATE_OPEN)
								{
								}
								else if(ats3607_update_state == ATS3607_UPDATE_TRANSFER)
								{
								}
								else if(ats3607_update_state == ATS3607_UPDATE_CLOSE)
								{
										ats3607_update_state = ATS3607_UPDATE_STOP;
								}
						}
				}
		}
#endif
}

void ats3607_usb2uart_xfer(void)
{
#ifdef VCOM_USED
    if (g_i8BulkOutReady)
		{
				uint8_t len = g_u32RxSize, i;
				memcpy((uint8_t *)ats3607_usb2uart_buffer, (uint8_t *)g_pu8RxBuf, len);
        g_u32RxSize = 0;
        g_i8BulkOutReady = 0;
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
				for(i=0;i<len;i++)
				{
						while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk){}
						UART0->DAT = ats3607_usb2uart_buffer[i];
				}
		}
#endif
}

bool ats3607_update_running(void)
{
		bool status = false;
		if(ats3607_update_enable)
		{
				if(ats3607_update_used_ms >= ats3607_update_timeout_ms)
				{
						GW_led_unlock();
						ats3607_update_enable = false;
						ats3607_uart2usb_length = 0;
						ats3607_uart2usb_index = 0;
						ats3607_update_state = ATS3607_UPDATE_IDLE;
				}
		}
		status =	ats3607_update_enable || ats3607_event_enable;

		return status;
}

bool ats3607_uart_parse_status(uint8_t Chx)
{
		if(!ats3607_update_enable)
		{
				if(ats3607_uart2usb_index == 0)
				{
						ats3607_uart_crc = 0;
						ats3607_uart2usb_index = Chx == DSP_MAGIC? ats3607_uart2usb_index + 1: 0;
				}
				else if(ats3607_uart2usb_index == 1)
				{
						ats3607_uart_new = false;
						ats3607_uart_code = 0;
						ats3607_uart2usb_index = Chx == DSP_EVENT_FLAG? ats3607_uart2usb_index + 1: 0;
				}
				else if(ats3607_uart2usb_index == 2)
				{
						ats3607_uart_length = Chx;
						ats3607_uart2usb_index++;
				}
				else if(ats3607_uart2usb_index == 3)
				{
						ats3607_uart_code = Chx;
						ats3607_uart2usb_index++;
				}
				else
				{
						ats3607_uart2usb_index++;
				}

				if(ats3607_uart2usb_index != 2 && ats3607_uart2usb_index == (ats3607_uart_length + 2))		// + 2 = magic code + event code
				{
						ats3607_uart_new = true;
						if(ats3607_uart_crc == Chx)
						{
								if(ats3607_uart_code == DSP_EVENT_ISP_FAIL)
								{
										ats3607_event = DSP_EVENT_ISP_FAIL;
								}
								else if(ats3607_uart_code == DSP_EVENT_ISP_SUCCESS)
								{
										ats3607_event = DSP_EVENT_ISP_SUCCESS;
								}
								else if(ats3607_uart_code == DSP_EVENT_VERSION)
								{
								}
								else if(ats3607_uart_code == DSP_EVENT_CMD_COMPLETE)
								{
										ats3607_event_cmd_code = ats3607_uart_buffer[4];
								}
						}
						
						return true;
				}
				else
						ats3607_uart_crc = ats3607_uart2usb_index == 1? DSP_MAGIC: ats3607_uart_crc + Chx;
		}
		else
		{
				if(ats3607_update_state == ATS3607_UPDATE_START ||
					ats3607_update_state == ATS3607_UPDATE_OPEN ||
					ats3607_update_state == ATS3607_UPDATE_TRANSFER)
				{
						ats3607_uart2usb_buffer[ats3607_uart2usb_index] = Chx;
						if(ats3607_uart2usb_index == 0)
						{
								ats3607_uart2usb_index = Chx == ATS3607_UPDATE_OPEN || Chx == ATS3607_UPDATE_TRANSFER || Chx == ATS3607_UPDATE_CLOSE? ats3607_uart2usb_index + 1: 0;
								ats3607_target_state = Chx;
						}
						else if(ats3607_uart2usb_index == 1)
						{
								ats3607_uart2usb_index = Chx == ATS3607_BUFFER_LENGTH? ats3607_uart2usb_index + 1: 0;
						}
						else if(ats3607_uart2usb_index == 14)
						{
								ats3607_uart2usb_index = Chx == ATS3607_PAYLOAD_SUFFIX? ats3607_uart2usb_index + 1: 0;
						}
						else
						{
								ats3607_uart2usb_index = ats3607_target_state == ATS3607_UPDATE_TRANSFER? ats3607_uart2usb_index + 1: Chx == 0? ats3607_uart2usb_index + 1: 0;
						}

						if(ats3607_uart2usb_index == ATS3607_BUFFER_LENGTH)
						{
//PrintHeximal(":", (uint8_t *)ats3607_uart2usb_buffer, ATS3607_BUFFER_LENGTH);
								ats3607_update_state = ats3607_target_state;
								ats3607_uart2usb_index = 0;
								ats3607_uart2usb_length = ATS3607_BUFFER_LENGTH;
#ifdef VCOM_USED
								g_u32TxSize = ATS3607_BUFFER_LENGTH;
#endif
								USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)ats3607_uart2usb_buffer, ATS3607_BUFFER_LENGTH);
								USBD_SET_PAYLOAD_LEN(EP2, ATS3607_BUFFER_LENGTH);

								if(ats3607_update_state == ATS3607_UPDATE_CLOSE)
										ats3607_update_enable = false;

								return true;
						}
				}
		}
		return false;
}

void ats3607_uart_rx_update(uint8_t Chx)
{
		ats3607_update_used_ms = 0;
		ats3607_uart_buffer[ats3607_uart_index] = Chx;
		if(++ats3607_uart_index >= ATS3607_UART_BUFFER_LENGTH)
				ats3607_uart_index = 0;
}

void ats3607_uart_rx_parse(void)
{
		bool uart_rx_init = false;

		if(ats3607_uart_index > ats3607_uart2usb_last_index)
		{
				uint16_t i;
				for(i=ats3607_uart2usb_last_index;i<ats3607_uart_index;i++)
						uart_rx_init = ats3607_uart_parse_status(ats3607_uart_buffer[i]);
				ats3607_uart2usb_last_index = ats3607_uart_index;
		}
		else if(ats3607_uart_index && ats3607_update_used_ms > ATS3607_UART_RX_TIMEOUT_MS)
		{
				uart_rx_init = true;
				//PrintMsg("Timeout!\r\n");
		}

		if(ats3607_event_enable)
		{
				if(ats3607_update_used_ms > ATS3607_UART_RX_EVENT_TIMEOUT_MS)
						ats3607_event_enable = false;
		}

		if(uart_rx_init)
		{
				//PrintHeximal(":", (uint8_t *)ats3607_uart_buffer, ats3607_uart_index);
				ats3607_uart2usb_index = 0;
				ats3607_uart_index = 0;
				ats3607_uart2usb_last_index = 0;
		}
}

void setback_file_index(void)
{
		file_index -= sizeof(module_buffer);
}
void aw5808_state_show(DEV_CMD_T * model)
{
		uint8_t		state_msg[5][10] = {"idle", "start", "transfer", "finish", "stop"};
		uint8_t		module_msg[4][10] = {"idle", "shut-off", "rx", "tx"};
		//printf("\r\n    State: %10s, mode: %10s\r\n", state_msg[model->payload[AW5808_PAYLOAD_STATE_INDEX]], module_msg[model->payload[AW5808_PAYLOAD_TYPE_INDEX]]);
}

#ifdef TIMER_SIMULATOR_ENABLE
void TMR0_IRQHandler(void)
{
    TIMER_ClearIntFlag(TIMER0);

		aw5808_simulator_query();
}
#endif

void aw5808_simulator_start(void)
{
		file_index = 0;
		//printf("    The redirection of AW5808 is initialized, binary size is %d\r\n", file_size);

#ifdef TIMER_SIMULATOR_ENABLE
    if (TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 500) != 1)
    {
        printf("Set the frequency different from the user\n");
    }
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    TIMER_Start(TIMER0);
#endif
}

void aw5808_simulator_transfer(uint8_t * payload, uint8_t len)
{
		//PrintHeximal("    Receiving data from HID and redirect to AW5808 TX/RX module", payload, len);

// Send finish command to PC if package full or force the process to stop
		if(file_index < file_size)
				aw5808_simulator_query();
		else
				aw5808_simulator_finish();
}

void aw5808_simulator_query(void)
{
		DEV_CMD_T msg_in = {.cmd = GW_HID_CMD_AW5808_UPGRADE};
		msg_in.len = AW5808_PAYLOAD_DATA_INDEX + sizeof(file_index);
		msg_in.payload[AW5808_PAYLOAD_STATE_INDEX] = AW5808_PAYLOAD_STATE_TRANSFER;
		msg_in.payload[AW5808_PAYLOAD_TYPE_INDEX] = module_state;
		memcpy(msg_in.payload + AW5808_PAYLOAD_DATA_INDEX, &file_index, sizeof(file_index));
		file_index += sizeof(module_buffer);
		aw5808_state_show(&msg_in);
		GW_hid_send(&msg_in, GW_HID_TIMEOUT_AW5808_MS);
}

// T(N+1)
void aw5808_simulator_finish(void)
{
		bool			res = true;
		uint8_t		ret_code = AW5808_PAYLOAD_ACK_FAILED;

		//printf("    The AW5808 finished the process of data redirection\r\n");
		//res =
		//ret_code =

		DEV_CMD_T msg_in = {.cmd = GW_HID_CMD_AW5808_UPGRADE};
		msg_in.len = AW5808_PAYLOAD_DATA_INDEX + AW5808_DUP_PAYLOAD_ACK_LENGTH;
		msg_in.payload[AW5808_PAYLOAD_STATE_INDEX] = AW5808_PAYLOAD_STATE_FINISH;
		msg_in.payload[AW5808_PAYLOAD_TYPE_INDEX] = module_state;
		msg_in.payload[AW5808_PAYLOAD_DATA_INDEX] = res? AW5808_PAYLOAD_ACK_SUCCESS: ret_code;
		aw5808_state_show(&msg_in);
		GW_hid_send(&msg_in, GW_HID_TIMEOUT_AW5808_MS);
}

void aw5808_simulator_stop(void)
{
		//printf("    The AW5808 is forced to closed!\r\n");
}

void evt_aw5808_cb(DEV_CMD_S * p_msg_out, DEV_CMD_S * p_msg_in)
{
		uint8_t	header_cmd = p_msg_in->cmd_t.cmd;
		DEV_CMD_T	* msg_in = &p_msg_in->sub_cmd_t, * msg_out;

		memcpy(p_msg_out, p_msg_in, sizeof(DEV_CMD_S));

		msg_out = &p_msg_out->sub_cmd_t;
		process_state = msg_in->payload[AW5808_PAYLOAD_STATE_INDEX];
		module_state = msg_in->payload[AW5808_PAYLOAD_TYPE_INDEX];
		if(process_state == AW5808_PAYLOAD_STATE_START)
		{
				if(header_cmd == HID_CMD_EVENT_REQUEST)
				{
						memcpy(&file_size, msg_in->payload + AW5808_PAYLOAD_DATA_INDEX, sizeof(file_size));
						msg_out->len = AW5808_PAYLOAD_DATA_INDEX + AW5808_DUP_PAYLOAD_ACK_LENGTH;
						msg_out->payload[AW5808_PAYLOAD_DATA_INDEX] = AW5808_PAYLOAD_ACK_SUCCESS;
						//aw5808_state_show(msg_in);
						aw5808_simulator_start();
						get_aw5808_hid_event_response=1;//start
						aw5808_dfu_channel=(msg_in->payload[AW5808_PAYLOAD_TYPE_INDEX]==2)?1:0;
					  aw5808_dfu_enable=1;
						aw5805_dfu_step=0xff;

						sys_evt.fw_upgrade_aw5808(YES);
				}
				else
						msg_out->len = 0;
		}
		else if(process_state == AW5808_PAYLOAD_STATE_TRANSFER)
		{
				msg_out->len = 0;
				if(header_cmd == HID_CMD_EVENT_RESPONSE)
				{
						memcpy((uint8_t *)module_buffer, msg_in->payload + AW5808_PAYLOAD_DATA_INDEX + AW5808_DUP_PAYLOAD_ADDRESS_LENGTH, sizeof(module_buffer));

						//aw5808_state_show(msg_in);
						//aw5808_simulator_transfer(module_buffer, sizeof(module_buffer));
					  save_hid_aw5808_data(&file_index,module_buffer,sizeof(module_buffer));
						get_aw5808_hid_event_response=2;//transfer

				}
		}
		else if(process_state == AW5808_PAYLOAD_STATE_FINISH)
		{
				msg_out->len = 0;
		}
		else if(process_state == AW5808_PAYLOAD_STATE_STOP)
		{
				if(header_cmd == HID_CMD_EVENT_REQUEST)
				{
						msg_out->len = AW5808_PAYLOAD_DATA_INDEX + AW5808_DUP_PAYLOAD_ACK_LENGTH;
						msg_out->payload[AW5808_PAYLOAD_DATA_INDEX] = AW5808_PAYLOAD_ACK_SUCCESS;
						//aw5808_state_show(msg_in);
						//aw5808_simulator_stop();
						get_aw5808_hid_event_response=3;//stop
				}
				else
						msg_out->len = 0;
		}
		else
		{
				msg_out->len = 0;
				//printf("    AW5808 receive unknown command %02X, len %2d, state %2X ==========\r\n", msg_in->cmd, msg_in->len, process_state);
		}
}

void evt_ats3607_cb(DEV_CMD_S * p_msg_out, DEV_CMD_S * p_msg_in)
{
		uint32_t	when_to_start_ms = 100, delay_ms = (p_msg_in->sub_cmd_t.payload[2] << 0x08) | p_msg_in->sub_cmd_t.payload[1];
		uint32_t	timeout_ms;

		memcpy(&timeout_ms, p_msg_in->sub_cmd_t.payload + 4, sizeof(timeout_ms));

		p_msg_out->sub_cmd_t.cmd = GW_HID_CMD_ATS3607_UPGRADE;
		if(p_msg_in->sub_cmd_t.payload[3] == ATS3607_HELPER_ENABLE)
		{
				GW_led_lock(YES);
				GW_USB_send_to_DSP_ISP_request();

				if(delay_ms)
						when_to_start_ms = delay_ms;
				p_msg_out->sub_cmd_t.payload[1] = (when_to_start_ms >> 0x00)&0xFF;
				p_msg_out->sub_cmd_t.payload[2] = (when_to_start_ms >> 0x08)&0xFF;
				p_msg_out->sub_cmd_t.payload[3] = ATS3607_HELPER_ENABLE;
				p_msg_out->sub_cmd_t.len = 0x04;

				ats3607_uart2usb_length = 0;
				ats3607_uart2usb_index = 0;
				ats3607_update_used_ms = 0;
				ats3607_update_state	= p_msg_in->sub_cmd_t.payload[8]? ATS3607_UPDATE_STOP: ATS3607_UPDATE_START;
				ats3607_update_enable	= p_msg_in->sub_cmd_t.len > 9? p_msg_in->sub_cmd_t.payload[9]? false: true: p_msg_in->sub_cmd_t.payload[8]? false: true;
				ats3607_update_timeout_ms = timeout_ms;
				ats3607_event = 0;
				sys_evt.fw_upgrade_ats3607(YES);
		}
		else if(p_msg_in->sub_cmd_t.payload[3] == ATS3607_HELPER_STATUS)			// Status
		{
				GW_led_unlock();
				p_msg_out->sub_cmd_t.payload[0] = ats3607_event == DSP_EVENT_ISP_SUCCESS? true: false;
				p_msg_out->sub_cmd_t.len = 0x01;
				ats3607_update_state = ATS3607_UPDATE_IDLE;
				sys_evt.fw_upgrade_ats3607(NO);
		}
		else if(p_msg_in->sub_cmd_t.payload[3] == ATS3607_HELPER_VERSION)			// Version
		{
				GW_led_lock(NO);
				ats3607_event_enable = true;
				ats3607_uart_length = 0;
				ats3607_uart2usb_length = 0;
				ats3607_uart2usb_index = 0;
				ats3607_update_used_ms = 0;
				ats3607_version_code = 0;
				uint8_t version[] = {DSP_MAGIC, DSP_CMD_FLAG, 0x03, DSP_CMD_VERSION, 0xB0};
				GW_send_cmd_to_3607D(version, sizeof(version));
				p_msg_out->sub_cmd_t.payload[0] = 0x00;
				p_msg_out->sub_cmd_t.len = 0x01;
		}
		else if(p_msg_in->sub_cmd_t.payload[3] == ATS3607_HELPER_ENABLE_STATUS)			// Enable status
		{
				uint8_t ats3607_status = ats3607_uart_new && ats3607_uart_code == DSP_EVENT_CMD_COMPLETE && ats3607_event_cmd_code == DSP_CMD_ISP? ATS3607_HELPER_ENABLE: ATS3607_HELPER_DISABLE;

				ats3607_event_cmd_code = 0x00;
				p_msg_out->sub_cmd_t.payload[3] = ats3607_status;
				p_msg_out->sub_cmd_t.len = 0x04;

				//PrintMsg("S: %X %X %X %X\r\n", ats3607_status, ats3607_uart_new, ats3607_uart_code, ats3607_event_cmd_code);
				if(ats3607_status == ATS3607_HELPER_ENABLE)
				{
						ats3607_uart2usb_length = 0;
						ats3607_uart2usb_index = 0;
						ats3607_update_used_ms = 0;
						ats3607_update_state	= p_msg_in->sub_cmd_t.payload[8]? ATS3607_UPDATE_STOP: ATS3607_UPDATE_START;
						ats3607_update_enable	= p_msg_in->sub_cmd_t.payload[9]? false: true;
						ats3607_update_timeout_ms = timeout_ms;
				}
		}
		else
		{
				p_msg_out->sub_cmd_t.payload[3] = 0xFF;
				p_msg_out->sub_cmd_t.len = 0x01;
		}
}

void evt_test_cb(DEV_CMD_S * p_msg_out, DEV_CMD_S * p_msg_in)
{
		//PrintHeximal("========== evt_test_cb Test ==========", p_msg_in->sub_cmd_t.payload, p_msg_in->sub_cmd_t.len);
		uint8_t		index = 0;
		uint32_t 	config0;

		SYS_UnlockReg();
		FMC_Open();
		FMC_ReadConfig(&config0, 1);
		FMC_Close();
		SYS_LockReg();

		p_msg_out->sub_cmd_t.cmd = GW_HID_CMD_TEST;
		p_msg_out->sub_cmd_t.payload[index++] = config0&ISP_APROM_BIT? 0: 1;
		p_msg_out->sub_cmd_t.len = index;
}

void evt_wildcard_cb(DEV_CMD_S * p_msg_out, DEV_CMD_S * msg_in)
{
		//printf("========== evt_wildcard_cb test ==========\n");
		//printf("evt_wildcard_cb, cmd %X, len %d\n", msg_in->sub_cmd_t.cmd, msg_in->sub_cmd_t.len);
}

bool rom_boot_set(bool ldrom)
{
		bool 			boot_mode_status = false;
		uint32_t 	config0, _config0;

		SYS_UnlockReg();
		FMC_Open();
		FMC_ReadConfig(&config0, 1);
		if(ldrom)
				config0 &= ~ISP_APROM_BIT;
		else
				config0 |= ISP_APROM_BIT;

		FMC_ENABLE_CFG_UPDATE();
		FMC_Erase(FMC_CONFIG_BASE);
		if(FMC_WriteConfig(&config0, 1) == 0)
		{
				config0 = 0;
				FMC_ReadConfig(&config0, 1);
				_config0 = config0&ISP_APROM_BIT;
				if(ldrom && _config0 == 0)
						boot_mode_status = true;
				else if(!ldrom && _config0 != 0)
						boot_mode_status = true;
		}
		FMC_DISABLE_CFG_UPDATE();

		FMC_Close();
		SYS_LockReg();

		return boot_mode_status;
}

void evt_echo_cb(DEV_CMD_S * p_msg_out, DEV_CMD_S * p_msg_in)
{
		//PrintHeximal("========== evt_echo_cb Test ==========", p_msg_in->sub_cmd_t.payload, p_msg_in->sub_cmd_t.len);
}

// Version event
void evt_version_get_cb(DEV_CMD_S * p_msg_out, DEV_CMD_S * p_msg_in)
{
		//PrintHeximal("========== evt_version_get_cb Test ==========", p_msg_in->sub_cmd_t.payload, p_msg_in->sub_cmd_t.len);

		DEV_CMD_T	* msg_in = &p_msg_in->sub_cmd_t, * msg_out;
		uint32_t	version = 0, length = VERSION_TYPE_PAYLOAD_VALID_LENGTH;
		uint8_t		* p_version = (uint8_t *)&version;

// If no version got, set it to 0. Valid length of version is 4
		if (p_msg_in->cmd_t.cmd == HID_CMD_EVENT_REQUEST)
		{
				switch(msg_in->payload[VERSION_TYPE_INDEX])
				{
						case VERSION_TYPE_MCU:
								version = (VERSION_MAJOR_ID << 0x08) | VERSION_MINOR_ID;
								break;
						case VERSION_TYPE_AW5808_RX:
								version = (aw5808_rx_data.version[0]<<0x08)|aw5808_rx_data.version[1];
								break;
						case VERSION_TYPE_AW5808_TX:
								version = (aw5808_tx_data.version[0]<<0x08)|aw5808_tx_data.version[1];
								break;
						case VERSION_TYPE_ATS3607:
								GW_led_unlock();
								if(ats3607_uart_length && ats3607_uart_new && ats3607_uart_code == DSP_EVENT_VERSION)
								{
										p_version = (uint8_t *)ats3607_uart_buffer + 4;
										length = ats3607_uart_length - 4;
								}
								else
										p_version = NULL;
								break;
						default:
								p_version = NULL;
								break;
				}
		}

		// Send error status to PC if version is 0
		p_msg_out->sub_cmd_t.cmd = GW_HID_CMD_VERSION_GET;
		p_msg_out->sub_cmd_t.len = VERSION_TYPE_PAYLOAD_INDEX + length;
		if (p_version != NULL)
		{
				p_msg_out->sub_cmd_t.payload[VERSION_TYPE_STATUS_INDEX] = VERSION_GET_SUCCESS;
				memcpy(p_msg_out->sub_cmd_t.payload + VERSION_TYPE_PAYLOAD_INDEX, p_version, length);
		}
		else
		{
				p_msg_out->sub_cmd_t.payload[VERSION_TYPE_STATUS_INDEX] = VERSION_GET_FAILED;
				memset(p_msg_out->sub_cmd_t.payload + VERSION_TYPE_PAYLOAD_INDEX, VERSION_GET_STATUS_ERROR, VERSION_TYPE_PAYLOAD_VALID_LENGTH);
		}
}

void evt_configuration_set_cb(DEV_CMD_S * p_msg_out, DEV_CMD_S * p_msg_in)
{
		//printf("========== evt_configuration_set_cb Test ==========\n");
		//printf("\x1b[%d;%dHevt_configuration_set_cb, cmd %X, len %d\n", SELECT_Y+20, SELECT_X  p_msg_in->sub_cmd_t.cmd, p_msg_in->sub_cmd_t.len);
		p_msg_out->sub_cmd_t.len = 0;
		hid_app_cs_evt(p_msg_in->sub_cmd_t.payload, p_msg_in->sub_cmd_t.len);
}

void evt_device_info_get_cb(DEV_CMD_S * p_msg_out, DEV_CMD_S * p_msg_in)
{
		p_msg_out->sub_cmd_t.cmd = GW_HID_CMD_DEVICE_INFO_GET;
		p_msg_out->sub_cmd_t.len = 0x01;
		p_msg_out->sub_cmd_t.payload[0] = UFP_NONE;
}

void evt_rom_reboot_cb(DEV_CMD_S * p_msg_out, DEV_CMD_S * msg_in)
{
		//printf("========== evt_rom_reboot_cb test ==========\n");
		//printf("evt_rom_reboot_cb, cmd %X, len %d\n", msg_in->sub_cmd_t.cmd, msg_in->sub_cmd_t.len);

		bool 			ldrom = msg_in->sub_cmd_t.payload[0]? false: true;
		uint8_t		status, mode = ldrom? 1: 0;

		SYS_ResetModule(USBD_RST);
		CLK_SysTickDelay(100000);
		SYS_UnlockReg();
		FMC_Open();
		FMC_SetBootSource(mode);
		FMC_Close();
		NVIC->ICPR[0] = 0xFFFFFFFF;
		SYS_ResetCPU();

		while(1);

		p_msg_out->sub_cmd_t.cmd = GW_HID_CMD_ROM_REBOOT;
		p_msg_out->sub_cmd_t.len = 0x01;
		p_msg_out->sub_cmd_t.payload[0] = status;
}

void evt_rom_mode_cb(DEV_CMD_S * p_msg_out, DEV_CMD_S * msg_in)
{
		//printf("========== evt_rom_boot_cb test ==========, mode %s\n", ldrom? "ldrom": "aprom");
		//printf("evt_rom_boot_cb, cmd %X, len %d\r\n", msg_in->sub_cmd_t.cmd, msg_in->sub_cmd_t.len);

		bool ldrom = msg_in->sub_cmd_t.payload[0]? false: true;
		if(ldrom != NO)
				sys_evt.fw_upgrade_mcu(NULL);
		p_msg_out->sub_cmd_t.cmd = GW_HID_CMD_ROM_MODE;
		p_msg_out->sub_cmd_t.len = 0x01;
		p_msg_out->sub_cmd_t.payload[0] = rom_boot_set(ldrom)? 0x00: 0x01;
}

void GW_hid_app_cs_register(hid_app_cs_cb evt)
{
		hid_app_cs_evt = evt;
}

void GW_hid_app_ats3607_register(hid_app_ats3607_cb r_evt, hid_app_ats3607_cb w_evt)
{
		ats3607_r_evt = r_evt;
		ats3607_w_evt = w_evt;
}

void GW_hid_app_init(uint32_t no)
{
		hid_app_evt_s evt;

		ats3607_update_enable = false;

		GW_hid_init(no);
		// USB to private command set example. maximum number of cmd is APP_EVENT_COUNT
/*
		evt.cmd = GW_HID_CMD_WILDCARD;
		evt.cb = evt_wildcard_cb;
		GW_hid_app_event_register(evt);
*/
		evt.cmd = GW_HID_CMD_TEST;
		evt.cb = evt_test_cb;
		GW_hid_app_event_register(evt);

		evt.cmd = GW_HID_CMD_ECHO;
		evt.cb = evt_echo_cb;
		GW_hid_app_event_register(evt);

		evt.cmd = GW_HID_CMD_VERSION_GET;
		evt.cb = evt_version_get_cb;
		GW_hid_app_event_register(evt);

		evt.cmd = GW_HID_CMD_CONFIGURATION_SET;
		evt.cb = evt_configuration_set_cb;
		GW_hid_app_event_register(evt);

		evt.cmd = GW_HID_CMD_DEVICE_INFO_GET;
		evt.cb = evt_device_info_get_cb;
		GW_hid_app_event_register(evt);

		evt.cmd = GW_HID_CMD_AW5808_UPGRADE;
		evt.cb = evt_aw5808_cb;
		GW_hid_app_event_register(evt);

		evt.cmd = GW_HID_CMD_ATS3607_UPGRADE;
		evt.cb = evt_ats3607_cb;
		GW_hid_app_event_register(evt);

		evt.cmd = GW_HID_CMD_ROM_REBOOT;
		evt.cb = evt_rom_reboot_cb;
		GW_hid_app_event_register(evt);

		evt.cmd = GW_HID_CMD_ROM_MODE;
		evt.cb = evt_rom_mode_cb;
		GW_hid_app_event_register(evt);

		GW_hid_app_cs_register(configuration_set_event);
}
