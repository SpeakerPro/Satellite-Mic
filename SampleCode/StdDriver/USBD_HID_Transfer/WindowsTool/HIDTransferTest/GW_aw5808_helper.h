#pragma once
#ifndef __GW_AW5808_HELPER
#define __GW_AW5808_HELPER

#include "GW_hid_manage.h"
#include "GW_hid_cmd_table.h"

void AW5808StateShow(DEV_CMD_T* model);

UINT AW5808HelperRun(PCHAR filename, AW5808_PAYLOAD_TYPE_E type);

void AW5808HelperIdle(void);
#endif