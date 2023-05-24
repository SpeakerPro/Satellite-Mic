#pragma once

#define PARSE_RAWDATA_ABBR_INDEX     1
#define PARSE_RAWDATA_DATA_INDEX     2
#define PARSE_RAWDATA_VALUE_SIZE     5

typedef struct
{
	char** isp_file;
	char** data_flash_file;
	char** awr_file;
	char** awt_file;
	char** ats_file;
	char** test_cmd;
	char** version_mcu;
	char** version_awr;
	char** version_awt;
	char** isp_aprom;
	char** ufp_check;
	char** timeout_ms;
	char** timeout_retry_count;
	char** debug_file;
	char** raw_config_set;
	char** usb_port_list;
	char** ats_vid;
	char** ats_pid;
	char** atss;
} cmd_parse_s;

bool cmd_parse(INT argc, PCHAR argv[], cmd_parse_s* parse_info);