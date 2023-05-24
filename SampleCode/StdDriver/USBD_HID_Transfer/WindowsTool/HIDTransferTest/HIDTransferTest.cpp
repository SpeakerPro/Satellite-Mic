/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) 2010 Nuvoton Technology Corp. All rights reserved.                                        */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include "stdafx.h"
#include "HIDTransferTest.h"
#include "HID.hpp"

#include "GW_hid_manage.h"
#include "GW_isp_helper.h"
#include "GW_usb_info.h"
#include "GW_debug.h"
#include "GW_cmd_parse.h"
#include "GW_usbtree.h"
#include "GW_aw5808_helper.h"
#include "GW_ats3607_helper.h"
#include <windows.h>
#include <iostream>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

typedef enum {
    HID_TASK_VERSION_MCU_INDEX,
    HID_TASK_VERSION_AW5808_RX_INDEX,
    HID_TASK_VERSION_AW5808_TX_INDEX,
    HID_TASK_UPGRADE_MCU_INDEX,
    HID_TASK_UPGRADE_AW5808_RX_INDEX,
    HID_TASK_UPGRADE_AW5808_TX_INDEX,
    HID_TASK_CONFIGURATION_SET_MCU,
    HID_TASK_USBTREE_INFO_SHOW,
    HID_TASK_UPGRADE_ATS3607_INDEX,
} HID_TASK_STATUS_E;

#define HID_TASK_VERSION_MCU_MASK                       (1 << HID_TASK_VERSION_MCU_INDEX)
#define HID_TASK_VERSION_AW5808_RX_MASK                 (1 << HID_TASK_VERSION_AW5808_RX_INDEX)
#define HID_TASK_VERSION_AW5808_TX_MASK                 (1 << HID_TASK_VERSION_AW5808_TX_INDEX)
#define HID_TASK_UPGRADE_MCU_MASK                       (1 << HID_TASK_UPGRADE_MCU_INDEX)
#define HID_TASK_UPGRADE_AW5808_RX_MASK                 (1 << HID_TASK_UPGRADE_AW5808_RX_INDEX)
#define HID_TASK_UPGRADE_AW5808_TX_MASK                 (1 << HID_TASK_UPGRADE_AW5808_TX_INDEX)
#define HID_TASK_CONFIGURATION_SET_MCU_MASK             (1 << HID_TASK_CONFIGURATION_SET_MCU)
#define HID_TASK_USBTREE_INFO_SHOW_MASK                 (1 << HID_TASK_USBTREE_INFO_SHOW)
#define HID_TASK_UPGRADE_ATS3607_MASK                   (1 << HID_TASK_UPGRADE_ATS3607_INDEX)

#define HID_CONNECTION_RETRY_COUNT                      5
#define HID_CONNECTION_TIMEOUT_MS   			        10000

#define LOG_VERSION_GET                                 "Version_get.log"
#define LOG_FIRMWARE_UPGRADE                            "Firmware_upgrade.log"
#define LOG_TCONFIGURATION_SET                          "Configuration_set.log"

#define LOG_DEFAULT_VALUE                               0xFF
#define VERSION_MAJOR_ID                                0x00
#define VERSION_MINOR_ID                                0x0C

// 僅有的一個應用程式物件

CWinApp theApp;

using namespace std;

int main(int argc, char* argv[]);

int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
    int nRetCode = 0;

    // ���l�� MFC �é󥢱ѮɦC�L���~
    if (!AfxWinInit(::GetModuleHandle(NULL), NULL, ::GetCommandLine(), 0))
    {
        // TODO: �t�X�z���ݭn�ܧ����~�X
        _tprintf(_T("�Y�����~: MFC ���l�ƥ���\n"));
        nRetCode = 1;
    }
    else
    {
        // TODO: �b�����g���ε{���欰���{���X�C
        //main();
    }

    return nRetCode;
}

UINT            hid_retry_count = HID_CONNECTION_RETRY_COUNT;
void HidDebugInit(cmd_parse_s* params)
{
    debug_init(*params->debug_file);
    debug_printf("WindowsTool version: %02X.%02X\r\n", VERSION_MAJOR_ID, VERSION_MINOR_ID);
}

void HidDebugUninit(cmd_parse_s* params)
{
    debug_uninit();
}

void HidParamsConfig(cmd_parse_s* params)
{
    UINT hid_timeout_ms = HID_CONNECTION_TIMEOUT_MS;

    if (*params->timeout_ms != NULL)
        hid_timeout_ms = atoi(*params->timeout_ms) == 0 ? HID_CONNECTION_TIMEOUT_MS : atoi(*params->timeout_ms);

    if (*params->timeout_retry_count != NULL)
        hid_retry_count = atoi(*params->timeout_retry_count) == 0 ? HID_CONNECTION_RETRY_COUNT : atoi(*params->timeout_retry_count);

    HidParamsSet(hid_timeout_ms);
}

void UsbtreePortList(cmd_parse_s* params, UINT* status)
{
    if (*params->usb_port_list)
    {
        UINT    res;
        res = UsbtreeReload();
        if(res == USBTREE_SUCCESSFUL)
            UsbtreeListShow();
        else
        {
            debug_printf("Usbtree work failed, res %X\r\n", res);
            *status |= HID_TASK_USBTREE_INFO_SHOW_MASK;
        }
    }
}

void PeripheralVersionGet(cmd_parse_s * params, UINT* status)
{
    BOOL        Res_awr = TRUE, Res_awt = TRUE, Res_mcu = TRUE;
    BOOL        Res, Ret;
    PUINT       p_version;
    UINT        version_mcu = 0, version_awr = 0, version_awt = 0;
    DEV_CMD_T   MSG = {0};

    if (*params->version_mcu || *params->version_awr || *params->version_awt)
    {
        HidParamsConfig(params);
        if (!HidInterfaceOpen())
        {
            debug_printf("Task version get. Can't Open HID Device\n");
            *status |= (HID_TASK_VERSION_MCU_MASK | HID_TASK_VERSION_AW5808_RX_MASK | HID_TASK_VERSION_AW5808_TX_MASK);
            goto log_update;
        }

        if (*params->version_mcu)
        {
            p_version = (PUINT)&MSG.payload[VERSION_TYPE_PAYLOAD_INDEX];
            Res_mcu = VersionGet(&MSG, VERSION_TYPE_MCU);
            if (!Res_mcu)
            {
                debug_printf("Version MCU not found: %X\r\n", MSG.payload[VERSION_TYPE_STATUS_INDEX]);
                *status |= HID_TASK_VERSION_MCU_MASK;
            }
            else
            {
                debug_printf("Version MCU get: %X\r\n", *p_version);
                version_mcu = *p_version;
            }
        }

        if (*params->version_awr)
        {
            p_version = (PUINT)&MSG.payload[VERSION_TYPE_PAYLOAD_INDEX];
            Res_awr = VersionGet(&MSG, VERSION_TYPE_AW5808_RX);
            if (!Res_awr)
            {
                debug_printf("Version AW5808-RX not found: %X\r\n", MSG.payload[VERSION_TYPE_STATUS_INDEX]);
                *status |= HID_TASK_VERSION_AW5808_RX_MASK;
            }
            else
            {
                debug_printf("Version AW5808-RX get: %X\r\n", *p_version);
                version_awr = *p_version;
            }
        }

        if (*params->version_awt)
        {
            p_version = (PUINT)&MSG.payload[VERSION_TYPE_PAYLOAD_INDEX];
            Res_awt = VersionGet(&MSG, VERSION_TYPE_AW5808_TX);
            if (!Res_awt)
            {
                debug_printf("Version AW5808-TX not found: %X\r\n", MSG.payload[VERSION_TYPE_STATUS_INDEX]);
                *status |= HID_TASK_VERSION_AW5808_TX_MASK;
            }
            else
            {
                debug_printf("Version AW5808-TX get: %X\r\n", *p_version);
                version_awt = *p_version;
            }
        }
        HidInterfaceClose();
    log_update:
        FILE* fp = NULL;
        fp = fopen(LOG_VERSION_GET, "wb+");
        if (*params->version_mcu != NULL)
            fprintf(fp, "Version MCU: %X\r\n", Res_mcu ? version_mcu : 0);
        if (*params->version_awr != NULL)
            fprintf(fp, "Version AW5808_RX state: %X\r\n", Res_awr ? version_awr : 0);
        if (*params->version_awt != NULL)
            fprintf(fp, "Version AW5808_TX: %X\r\n", Res_awt ? version_awt : 0);
        fflush(fp);
        fclose(fp);
    }
}

void FirmwareUpgrade(cmd_parse_s* params, UINT* status)
{
    BOOL    isp_arom = *params->isp_aprom == NULL ? FALSE: TRUE, ufp_check = *params->ufp_check == NULL ? FALSE : TRUE, mcu_helper = *params->atss == NULL ? FALSE : TRUE;
    UINT    Res_awr = AW5808_PROCESS_STATUS_SUCCESS, Res_awt = AW5808_PROCESS_STATUS_SUCCESS, Res_mcu = ISP_HELPER_SUCCESSFUL, Res_ats = ATS3606_PROCESS_STATUS_SUCCESS;
    UINT    i;

    if (*params->awr_file != NULL || *params->awt_file != NULL || *params->isp_file != NULL || *params->ats_file != NULL)
    {
        if (*params->awr_file != NULL)
        {
            for (i = 0; i < hid_retry_count; i++)
            {
                HidParamsConfig(params);
                if (!HidInterfaceOpen())
                {
                    if (i == hid_retry_count - 1)
                    {
                        debug_printf("Task upgrade AW5808. Can't Open HID Device\n");
                        *status |= (HID_TASK_UPGRADE_AW5808_RX_MASK | HID_TASK_UPGRADE_AW5808_TX_MASK | HID_TASK_UPGRADE_MCU_MASK);
                        goto log_update;
                    }
                    continue;
                }

                Res_awr = AW5808HelperRun(*params->awr_file, AW5808_PAYLOAD_TYPE_RX);
                HidInterfaceClose();
                if (Res_awr != AW5808_PROCESS_STATUS_SUCCESS)
                {
                    debug_printf("Update AW5808-RX failed.\r\n");
                    if (i == hid_retry_count - 1)
                        *status |= HID_TASK_UPGRADE_AW5808_RX_MASK;
                }
                else
                {
                    debug_printf("Update AW5808-RX successful.\r\n");
                    break;
                }
            }
        }

        if (*params->awt_file != NULL)
        {
            for (i = 0; i < hid_retry_count; i++)
            {
                HidParamsConfig(params);
                if (!HidInterfaceOpen())
                {
                    if (i == hid_retry_count - 1)
                    {
                        debug_printf("Task upgrade AW5808. Can't Open HID Device\n");
                        *status |= (HID_TASK_UPGRADE_AW5808_RX_MASK | HID_TASK_UPGRADE_AW5808_TX_MASK | HID_TASK_UPGRADE_MCU_MASK);
                        goto log_update;
                    }
                    continue;
                }

                Res_awt = AW5808HelperRun(*params->awt_file, AW5808_PAYLOAD_TYPE_TX);
                HidInterfaceClose();
                if (Res_awt != AW5808_PROCESS_STATUS_SUCCESS)
                {
                    debug_printf("Update AW5808-TX failed.\r\n");
                    if (i == hid_retry_count - 1)
                        *status |= HID_TASK_UPGRADE_AW5808_TX_MASK;
                }
                else
                {
                    debug_printf("Update AW5808-TX successful.\r\n");
                    break;
                }
            }
        }

        if (*params->isp_file != NULL)
        {
            for (i = 0; i < hid_retry_count; i++)
            {
                HidParamsConfig(params);
                Res_mcu = IspMcuUpdate(*params->isp_file, *params->data_flash_file, isp_arom, ufp_check);
                if (Res_mcu != ISP_HELPER_SUCCESSFUL)
                {
                    debug_printf("Update MCU failed.\r\n");
                    if (i == hid_retry_count - 1)
                        *status |= HID_TASK_UPGRADE_MCU_MASK;
                }
                else
                {
                    debug_printf("Update MCU successful.\r\n");
                    break;
                }
            }
        }

        UINT    ats3607_hid_retry_count = 1;
        if (*params->ats_file != NULL)
        {
            for (i = 0; i < ats3607_hid_retry_count; i++)
            {
                UINT ats_vip = 0, ats_pid = 0;

                if (*params->ats_vid != NULL)
                    ats_vip = atoi(*params->ats_vid);

                if (*params->ats_pid != NULL)
                    ats_pid = atoi(*params->ats_pid);

                ATS3607HelperInit(ats_vip, ats_pid, mcu_helper);
                Res_ats = ATS3607HelperRun(*params->ats_file);
                if (Res_ats != ATS3606_PROCESS_STATUS_SUCCESS)
                {
                    debug_printf("Update ATS3607 failed.\r\n");
                    if (i == ats3607_hid_retry_count - 1)
                        *status |= HID_TASK_UPGRADE_ATS3607_MASK;
                }
                else
                {
                    debug_printf("Update ATS3607 successful.\r\n");
                    break;
                }
            }
        }

    log_update:
        FILE* fp = NULL;
        fp = fopen(LOG_FIRMWARE_UPGRADE, "wb+");
        if (*params->awr_file != NULL)
            fprintf(fp, "Upgrade AW5808_RX: %X\r\n", Res_awr);
        if (*params->awt_file != NULL)
            fprintf(fp, "Upgrade AW5808_TX: %X\r\n", Res_awt);
        if (*params->isp_file != NULL)
            fprintf(fp, "Upgrade MCU: %X\r\n", Res_mcu);
        if (*params->ats_file != NULL)
            fprintf(fp, "Upgrade ATS3607: %X\r\n", Res_ats);
        fflush(fp);
        fclose(fp);
    }
}

void ConfigurationSet(cmd_parse_s* params, UINT* status)
{
    if (*params->raw_config_set != NULL)
    {
        BOOL        res = FALSE;
        BYTE* raw_data;
        PCHAR       raw_log = NULL;
        DEV_CMD_T   msg;
        INT         count = 0, value, index;

        while (1)
        {
            if (sscanf(*params->raw_config_set + PARSE_RAWDATA_VALUE_SIZE * count, "0x%02X ", &value) == 0 || strlen(*params->raw_config_set + PARSE_RAWDATA_VALUE_SIZE * count) < PARSE_RAWDATA_VALUE_SIZE)
                break;
            count++;
        }

        raw_data = (BYTE*)calloc(count, sizeof(BYTE));
        if (raw_data != NULL)
        {
            index = 0;
            for (index = 0; index < count; index++)
            {
                if (sscanf(*params->raw_config_set + PARSE_RAWDATA_VALUE_SIZE * index, "0x%02X ", &value) == 0)
                    break;
                else
                    raw_data[index] = value;
            }

            HidParamsConfig(params);
            if (!HidInterfaceOpen())
            {
                debug_printf("Task configurations set. Can't Open HID Device\n");
                *status |= HID_TASK_CONFIGURATION_SET_MCU_MASK;
                goto log_update;
            }

            res = ConfigurationSetSend(&msg, (PCHAR)raw_data, count);

            HidInterfaceClose();
            if (res)
            {
                debug_printf("Configurations set successful.\r\n");
                raw_log = (PCHAR)calloc(PARSE_RAWDATA_VALUE_SIZE * msg.len, sizeof(CHAR));
                if (raw_log != NULL)
                {
                    for (UINT j = 0; j < msg.len; j++)
                    {
                        (void)sprintf(raw_log + (PARSE_RAWDATA_VALUE_SIZE * j), "0x%02X ", (BYTE)msg.payload[j]);
                        raw_log[PARSE_RAWDATA_VALUE_SIZE * msg.len - 1] = 0;
                    }
                }
            }
            else
            {
                debug_printf("Configurations set failed.\r\n");
                *status |= HID_TASK_CONFIGURATION_SET_MCU_MASK;
            }
            free(raw_data);
        }

    log_update:
        FILE* fp = NULL;
        fp = fopen(LOG_TCONFIGURATION_SET, "wb+");
        fprintf(fp, "Configuration set: %X\r\n", res ? 0 : 1);
        fprintf(fp, "Configuration set log: %s\r\n", raw_log == NULL ? "" : raw_log);
        fflush(fp);
        fclose(fp);
    }
}

void TasksStatusShow(cmd_parse_s* params, UINT status_code)
{
    BOOL    status;

    debug_printf("\r\nLogs file:\r\n");
    debug_printf(" - %s\r\n", LOG_VERSION_GET);
    debug_printf(" - %s\r\n", LOG_FIRMWARE_UPGRADE);
    debug_printf(" - %s\r\n", LOG_TCONFIGURATION_SET);
    debug_printf("\r\nTasks status:\r\n");
    if (*params->version_mcu)
    {
        status = status_code & HID_TASK_VERSION_MCU_MASK? FALSE: TRUE;
        debug_printf(" - Version get MCU: %s\r\n", status? "successful": "failed");
    }

    if (*params->version_awr)
    {
        status = status_code & HID_TASK_VERSION_AW5808_RX_MASK ? FALSE : TRUE;
        debug_printf(" - Version get AW5808-RX: %s\r\n", status ? "successful" : "failed");
    }

    if (*params->version_awt)
    {
        status = status_code & HID_TASK_VERSION_AW5808_TX_MASK ? FALSE : TRUE;
        debug_printf(" - Version get AW5808-TX: %s\r\n", status ? "successful" : "failed");
    }

    if (*params->awr_file)
    {
        status = status_code & HID_TASK_UPGRADE_AW5808_RX_MASK ? FALSE : TRUE;
        debug_printf(" - Upgrade AW5808-RX: %s\r\n", status ? "successful" : "failed");
    }

    if (*params->awt_file)
    {
        status = status_code & HID_TASK_UPGRADE_AW5808_TX_MASK ? FALSE : TRUE;
        debug_printf(" - Upgrade AW5808-TX: %s\r\n", status ? "successful" : "failed");
    }

    if (*params->ats_file)
    {
        status = status_code & HID_TASK_UPGRADE_ATS3607_MASK ? FALSE : TRUE;
        debug_printf(" - Upgrade ATS3607: %s\r\n", status ? "successful" : "failed");
    }

    if (*params->isp_file)
    {
        status = status_code & HID_TASK_UPGRADE_MCU_MASK ? FALSE : TRUE;
        debug_printf(" - Upgrade MCU: %s\r\n", status ? "successful" : "failed");
    }

    if (*params->raw_config_set)
    {
        status = status_code & HID_TASK_CONFIGURATION_SET_MCU_MASK ? FALSE : TRUE;
        debug_printf(" - Configuration set: %s\r\n", status ? "successful" : "failed");
    }

    if (*params->usb_port_list)
    {
        status = status_code & HID_TASK_USBTREE_INFO_SHOW_MASK ? FALSE : TRUE;
        debug_printf(" - Usbtree info show: %s\r\n", status ? "successful" : "failed");
    }
}

int main(int argc, char* argv[])
{
    UINT        status = 0;
    cmd_parse_s params = {0};

    if (!cmd_parse(argc, (CHAR**)argv, &params))
        return 0;

    HidDebugInit(&params);
    UsbtreePortList(&params, &status);
    PeripheralVersionGet(&params, &status);
    FirmwareUpgrade(&params, &status);
    ConfigurationSet(&params, &status);
    TasksStatusShow(&params, status);
    HidDebugUninit(&params);

    return 0;
}
