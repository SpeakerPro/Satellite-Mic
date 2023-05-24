#ifndef __GW_HID_MANAGE
#define __GW_HID_MANAGE

#include "NuMicro.h"
#include <stdbool.h>
#include "GW_hid_event.h"

void hid_event_show(void);

uint8_t GW_hid_app_event_register(hid_app_evt_s params);
uint8_t GW_hid_app_event_unregister(hid_app_evt_s params);
void GW_hid_init(uint32_t _epno);

bool GW_hid_response(DEV_CMD_T * msg_in);

void GW_hid_send(DEV_CMD_T * msg_in, uint16_t timeout);
uint32_t PrintMsg__(char *fmt, ...);
uint32_t PrintMsg(char *fmt, ...);
void PrintHeximal(const char* _header, unsigned char* _payload, unsigned long _size);
void GW_hid_cmd_rx(uint8_t * buffer, uint8_t len);
#endif
