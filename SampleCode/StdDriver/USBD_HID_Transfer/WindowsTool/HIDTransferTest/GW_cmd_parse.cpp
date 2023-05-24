#include "stdafx.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "GW_cmd_parse.h"

#define CLI_ABBR_MAX_LENGTH                     10

#define CLI_ABBR_FILE_ISP                       "isp"
#define CLI_ABBR_FILE_RESERVED                  "rsv"
#define CLI_ABBR_FILE_AW5808_RX                 "awr"
#define CLI_ABBR_FILE_AW5808_TX                 "awt"
#define CLI_ABBR_FILE_ATS3607                   "ats"

#define CLI_ABBR_VERSION_MCU                    "vm"
#define CLI_ABBR_VERSION_AW5808_RX              "vawr"
#define CLI_ABBR_VERSION_AW5808_TX              "vawt"

#define CLI_ABBR_UPGRADE_ISP_APROM              "uap"
#define CLI_ABBR_UPGRADE_ISP_UFP_CHECK          "ufp"
#define CLI_ABBR_UPGRADE_TIMEOUT_MS             "uto"
#define CLI_ABBR_UPGRADE_RETRY_COUNT            "uct"

#define CLI_ABBR_DEBUG_FILENAME                 "p"
#define CLI_ABBR_CONFIGURATION_SET              "raw"
#define CLI_ABBR_USB_PORT_LIST                  "pl"

#define CLI_ABBR_ATS3607_VID                    "atsv"
#define CLI_ABBR_ATS3607_PID                    "atsp"
#define CLI_ABBR_ATS3607_SERIAL                 "atss"

#define CLI_ABBR_TIP_ISP_FILE                   "[File]: APROM file"
#define CLI_ABBR_TIP_RESERVED_FILE              "[File]: Reserved"
#define CLI_ABBR_TIP_AW5808_RX                  "[File]: AW5808 RX file"
#define CLI_ABBR_TIP_AW5808_TX                  "[File]: AW5808 TX file"
#define CLI_ABBR_TIP_ATS3607                    "[File]: ATS3607 file"

#define CLI_ABBR_TIP_VERSION_MCU                "[Version]: Version of main MCU"
#define CLI_ABBR_TIP_VERSION_AW5808_RX          "[Version]: Version of AW5808 RX"
#define CLI_ABBR_TIP_VERSION_AW5808_TX          "[Version]: Version of AW5808 TX"

#define CLI_ABBR_TIP_UPGRADE_ISP_APROM          "[Upgrade]: APROM activated"
#define CLI_ABBR_TIP_UPGRADE_ISP_UFP_CHECK      "[Upgrade]: UFP check activated"
#define CLI_ABBR_TIP_UPGRADE_TIMEOUT            "[Upgrade]: Timeout of the HID connection . Unit: ms"
#define CLI_ABBR_TIP_UPGRADE_RETRY_COUNT        "[Upgrade]: Retry-count of the upgrade"

#define CLI_ABBR_TIP_DEBUG_FILENAME             "[Debug]: Debug file. Show on the screen if NULL"
#define CLI_ABBR_TIP_CONFIGURATION_SET          "[Config]: The raw data of configurations set."
#define CLI_ABBR_TIP_USB_PORT_LIST              "[Info]: The list of USB device"

#define CLI_ABBR_TIP_ATS3607_VIP                "[Config]: The USB VID for ATS3607"
#define CLI_ABBR_TIP_ATS3607_PIP                "[Config]: The USB PID for ATS3607"
#define CLI_ABBR_TIP_ATS3607_ATSS               "[Upgrade]: Update the ATS3607 through MCU over serial"

#define FILE_NAME(x) strrchr(x, '\\')? strrchr(x, '\\') + 1:x

#define CLI_CMD_COUNT                           20

typedef struct
{
    UCHAR   abbr[CLI_ABBR_MAX_LENGTH];
    BOOL    parameters;
    CHAR* pParam;
    CHAR* tip;
} cmd_parse_params_s;

PCHAR   raw_argv = NULL;
CHAR    params_yes = 1, * p_params_yes;
INT     cli_params_count = 0;
cmd_parse_params_s  cli_params[CLI_CMD_COUNT] = { 0 };

BOOL cmd_parse_raw(INT argc, PCHAR argv[], cmd_parse_s* parse_info)
{
    CHAR    abbr[] = CLI_ABBR_CONFIGURATION_SET;
    INT     len_abbr = strlen(CLI_ABBR_CONFIGURATION_SET), len_argv = strlen(argv[PARSE_RAWDATA_ABBR_INDEX]) - 1;

    parse_info->raw_config_set = &raw_argv;
    if (memcmp(&argv[PARSE_RAWDATA_ABBR_INDEX][1], abbr, len_argv) == 0 && len_argv == len_abbr)
    {
        raw_argv = (PCHAR)calloc(argc - PARSE_RAWDATA_DATA_INDEX, sizeof(CHAR) * PARSE_RAWDATA_VALUE_SIZE);
        if (raw_argv != NULL)
        {
            INT     count = 0;
            for (INT i = PARSE_RAWDATA_DATA_INDEX; i < argc; i++)
            {
                INT value;
                if (sscanf(argv[i], "0x%2X", &value) == 0)
                    break;
                else
                {
                    sprintf(raw_argv + PARSE_RAWDATA_VALUE_SIZE * (i - PARSE_RAWDATA_DATA_INDEX), "0x%02X ", value);
                    count++;
                }
            }

            INT     sof, len;
            (void)sscanf(argv[PARSE_RAWDATA_DATA_INDEX], "0x%2X", &sof);
            if (sof != 0xAA)
            {
                printf("Raw data: incorrect SOF \r\n");
                raw_argv = NULL;
            }

            (void)sscanf(argv[PARSE_RAWDATA_DATA_INDEX + 1], "0x%2X", &len);
            if (len != (count - 2))
            {
                printf("Raw data: incorrect length \r\n");
                raw_argv = NULL;
            }
            return TRUE;
        }
        else
        {
            printf("Raw data allocate failed \r\n");
            raw_argv = NULL;
        }
    }
    else
    {
        return FALSE;
    }
}

bool cmd_parse(INT argc, PCHAR argv[], cmd_parse_s* parse_info)
{
    BOOL    cmd_parse_status = FALSE;
    INT     index, sub_index, length;

    p_params_yes = &params_yes;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_FILE_ISP, strlen(CLI_ABBR_FILE_ISP));
    cli_params[cli_params_count].parameters = TRUE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_ISP_FILE;
    parse_info->isp_file = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_FILE_RESERVED, strlen(CLI_ABBR_FILE_RESERVED));
    cli_params[cli_params_count].parameters = TRUE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_RESERVED_FILE;
    parse_info->data_flash_file = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_FILE_AW5808_RX, strlen(CLI_ABBR_FILE_AW5808_RX));
    cli_params[cli_params_count].parameters = TRUE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_AW5808_RX;
    parse_info->awr_file = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_FILE_AW5808_TX, strlen(CLI_ABBR_FILE_AW5808_TX));
    cli_params[cli_params_count].parameters = TRUE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_AW5808_TX;
    parse_info->awt_file = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_VERSION_MCU, strlen(CLI_ABBR_VERSION_MCU));
    cli_params[cli_params_count].parameters = FALSE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_VERSION_MCU;
    parse_info->version_mcu = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_VERSION_AW5808_RX, strlen(CLI_ABBR_VERSION_AW5808_RX));
    cli_params[cli_params_count].parameters = FALSE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_VERSION_AW5808_RX;
    parse_info->version_awr = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_VERSION_AW5808_TX, strlen(CLI_ABBR_VERSION_AW5808_TX));
    cli_params[cli_params_count].parameters = FALSE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_VERSION_AW5808_TX;
    parse_info->version_awt = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_UPGRADE_ISP_APROM, strlen(CLI_ABBR_UPGRADE_ISP_APROM));
    cli_params[cli_params_count].parameters = FALSE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_UPGRADE_ISP_APROM;
    parse_info->isp_aprom = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_UPGRADE_ISP_UFP_CHECK, strlen(CLI_ABBR_UPGRADE_ISP_UFP_CHECK));
    cli_params[cli_params_count].parameters = FALSE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_UPGRADE_ISP_UFP_CHECK;
    parse_info->ufp_check = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_UPGRADE_TIMEOUT_MS, strlen(CLI_ABBR_UPGRADE_TIMEOUT_MS));
    cli_params[cli_params_count].parameters = TRUE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_UPGRADE_TIMEOUT;
    parse_info->timeout_ms = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_UPGRADE_RETRY_COUNT, strlen(CLI_ABBR_UPGRADE_RETRY_COUNT));
    cli_params[cli_params_count].parameters = TRUE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_UPGRADE_RETRY_COUNT;
    parse_info->timeout_retry_count = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_DEBUG_FILENAME, strlen(CLI_ABBR_DEBUG_FILENAME));
    cli_params[cli_params_count].parameters = TRUE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_DEBUG_FILENAME;
    parse_info->debug_file = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_USB_PORT_LIST, strlen(CLI_ABBR_USB_PORT_LIST));
    cli_params[cli_params_count].parameters = FALSE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_USB_PORT_LIST;
    parse_info->usb_port_list = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_FILE_ATS3607, strlen(CLI_ABBR_FILE_ATS3607));
    cli_params[cli_params_count].parameters = TRUE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_ATS3607;
    parse_info->ats_file = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_ATS3607_VID, strlen(CLI_ABBR_ATS3607_VID));
    cli_params[cli_params_count].parameters = TRUE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_ATS3607_VIP;
    parse_info->ats_vid = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_ATS3607_PID, strlen(CLI_ABBR_ATS3607_PID));
    cli_params[cli_params_count].parameters = TRUE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_ATS3607_PIP;
    parse_info->ats_pid = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    memcpy(cli_params[cli_params_count].abbr, CLI_ABBR_ATS3607_SERIAL, strlen(CLI_ABBR_ATS3607_SERIAL));
    cli_params[cli_params_count].parameters = FALSE;
    cli_params[cli_params_count].tip = CLI_ABBR_TIP_ATS3607_ATSS;
    parse_info->atss = &cli_params[cli_params_count].pParam;
    cli_params_count++;

    if (argc == 1)
        goto ParseList;

    if (cmd_parse_raw(argc, argv, parse_info))
    {
        if (*parse_info->raw_config_set == NULL)
        {
            printf("Usage:\r\n");
            printf("        Ex. %s -raw 0xAA 0x05 0x01 0x02 0x03 0x04 0x05\r\n", FILE_NAME(argv[0]));
            return FALSE;
        }
        return TRUE;
    }

    for (index = 1; index < argc; index++)
    {
        if (argv[index][0] == '-')
        {
            INT len_argv = strlen(argv[index]) - 1;
            cmd_parse_status = FALSE;
            for (sub_index = 0; sub_index < cli_params_count; sub_index++)
            {
                INT len_abbr = strlen((PCHAR)cli_params[sub_index].abbr);
                if (memcmp(&argv[index][1], cli_params[sub_index].abbr, len_argv) == 0 && len_argv == len_abbr)
                {
                    cmd_parse_status = TRUE;
                    if (cli_params[sub_index].parameters)
                        cli_params[sub_index].pParam = argv[++index];
                    else
                        cli_params[sub_index].pParam = (CHAR*)&p_params_yes;
                    break;
                }
            }

            if (!cmd_parse_status)
                goto ParseList;
        }
        else
            goto ParseList;
    }

ParseList:
    if (argc == 1 || !cmd_parse_status)
    {
        printf("Usage: %s\r\n", FILE_NAME(argv[0]));
        for (index = 0; index < cli_params_count; index++)
        {
            cli_params[index].pParam = NULL;
            printf("       [-%s] %s\r\n", cli_params[index].abbr, cli_params[index].tip);
        }
    }
    return cmd_parse_status;
}
