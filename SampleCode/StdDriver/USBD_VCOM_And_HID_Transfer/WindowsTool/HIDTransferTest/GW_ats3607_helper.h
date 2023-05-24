#pragma once
#ifndef __GW_AST3607_HELPER
#define __GW_AST3607_HELPER

#include "GW_hid_manage.h"
#include "GW_hid_cmd_table.h"

#define ATS3607_PAYLOAD_HEADER_LENGTH		6
#define ATS3607_PAYLOAD_MAX_LENGTH			(SUB_CMD_PAYLOAD_LEN - ATS3607_PAYLOAD_HEADER_LENGTH)
#define ATS3607_UART_RX_DELAY_MS			200

typedef enum {
	ATS3607_HELPER_ENABLE,
	ATS3607_HELPER_DISABLE,
	ATS3607_HELPER_STATUS,
	ATS3607_HELPER_VERSION,
	ATS3607_HELPER_ENABLE_STATUS,
	ATS3607_HELPER_READ = 0x11,
	ATS3607_HELPER_WRITE
} ATS3607_HELPER_STATUS_CODE_E;

typedef enum {
	ATS3607_HELPER_SUCCESSFUL,
	ATS3607_HELPER_OPEN_FAILED,
	ATS3607_HELPER_ACTIVATE_FAILED,
	ATS3607_HELPER_ACTIVATE_ENABLE_FAILED,
	ATS3607_HELPER_UPDATE_FAILED
} ATS3607_HELPER_MACHINE_CODE_E;

void ATS3607VersionUpdate(void);

void ATS3607HelperInit(BOOL ats3607_helper);

UINT ATS3607HelperRun(PCHAR binary, BOOL version_check);

#endif