#include "stdafx.h"
#include "GW_ats3607_helper.h"
#include "GW_usb_info.h"
#include "GW_debug.h"
#include "GW_cmd_parse.h"
#include "HID.hpp"

#define ATS3607_LOG_FILENAME                    "ats3607.log"
#define USB_MAX_HEADER_TIMEOUT_MS               120000

#define USB_TIME_OUT_MS                         500

#define USBTREE_EXE_COMMAND                     L"tools\\dsp_update.exe"

BYTE    is_mcu_helper = false;
CHAR    binary_name[MAX_PATH];
UINT    ats_vid, ats_pid;
void ATS3607VersionUpdate(void)
{
    UINT        Res, len = 3;
    DEV_CMD_T   pcmd_dst, pcmd_src;

    pcmd_src.cmd = GW_HID_CMD_3607D_UPGRADE;
    pcmd_src.payload[len++] = ATS3607_HELPER_VERSION;
    pcmd_src.len = len;
    Res = HidCmdSend(&pcmd_dst, &pcmd_src, USB_TIME_OUT_MS);
    if (!Res)
    {
    }
}

void ATS3607HelperInit(BOOL ats3607_helper)
{
    if (ats3607_helper)
    {
        HidParamsIdGet(&ats_vid, &ats_pid);
    }
    else
    {
        ats_vid = 0;
        ats_pid = 0;
    }
}

INT ATS3607Update(void)
{
    TCHAR   cmd[MAX_PATH * 4] = L"";
    TCHAR   cwd[MAX_PATH] = L"", file_path[MAX_PATH] = L".\\";

    GetCurrentDirectory(MAX_PATH, cwd);
    PathAppend(file_path, USBTREE_EXE_COMMAND);

    if (ats_vid == 0 || ats_pid == 0)
        swprintf(cmd, MAX_PATH * 4, L"%ls -bin %hs ", file_path, binary_name);
    else
        swprintf(cmd, MAX_PATH * 4, L"%ls -bin %hs -vid %d -pid %d", file_path, binary_name, ats_vid, ats_pid);

    return _wsystem(cmd);
}

UINT ATS3607HelperRun(PCHAR binary, BOOL version_check)
{
    BOOL                            ats3607_enable_status = FALSE;
    ATS3607_HELPER_MACHINE_CODE_E   Ret = ATS3607_HELPER_UPDATE_FAILED;
    HID_APP_EVT_S                   evt;

    INT         Res, len;
    DEV_CMD_T   pcmd_dst, pcmd_src;

    if (!HidInterfaceOpen())
    {
        debug_log("ATS3607 Can't Open HID Device\n");
        Ret = ATS3607_HELPER_OPEN_FAILED;
    }
    else
    {
#define ATS3607_VERSION_GET_RETRY_COUNT     1
        // Check whether ack is support or not
        if (version_check)
        {
            for (UINT i = 0; i < ATS3607_VERSION_GET_RETRY_COUNT; i++)
            {
                BOOL        Res = TRUE;
                DEV_CMD_T   MSG = { 0 };
                PUINT       p_version = (PUINT)&MSG.payload[VERSION_TYPE_PAYLOAD_INDEX];
                ATS3607VersionUpdate();
                Sleep(ATS3607_UART_RX_DELAY_MS);// Waiting for version status event
                Res = VersionGet(&MSG, VERSION_TYPE_ATS3607);
                if (Res && MSG.payload[VERSION_TYPE_STATUS_INDEX] == VERSION_GET_SUCCESS && MSG.len != 9)
                {
                    ats3607_enable_status = TRUE;
                }
            }

            if (!ats3607_enable_status)
            {
                Ret = ATS3607_HELPER_ACTIVATE_FAILED;
                goto ats_update_end;
            }
        }

        memset(binary_name, 0, sizeof(binary_name));
        if (binary != NULL)
            memcpy(binary_name, binary, strlen(binary));
        else
            memcpy(binary_name, "ast3607_ota.bin", strlen("ast3607_ota.bin"));

        UINT    timeout_ms = USB_MAX_HEADER_TIMEOUT_MS;

        len = 0;
        pcmd_src.cmd = GW_HID_CMD_3607D_UPGRADE;
        pcmd_src.payload[len++] = 0x00;
        pcmd_src.payload[len++] = 0xB8;
        pcmd_src.payload[len++] = 0x0B;
        pcmd_src.payload[len++] = ATS3607_HELPER_ENABLE;
        memcpy(pcmd_src.payload + len, &timeout_ms, sizeof(timeout_ms));
        len += sizeof(timeout_ms);
        pcmd_src.payload[len++] = ats_vid == 0 || ats_pid == 0 ? 1 : 0;// 1: stop. 0: start.
        pcmd_src.payload[len++] = (ats_vid == 0 || ats_pid == 0) || ats3607_enable_status ? 1 : 0;// 1: dsp event. 0: update event
        pcmd_src.len = len;
        Res = HidCmdSend(&pcmd_dst, &pcmd_src, USB_TIME_OUT_MS);
        if (!Res)
        {
            Sleep(ATS3607_UART_RX_DELAY_MS);// Waiting for acknowledge

            if (ats3607_enable_status)// For ack used
            {
                len = 0;
                pcmd_src.cmd = GW_HID_CMD_3607D_UPGRADE;
                pcmd_src.payload[len++] = 0x00;
                pcmd_src.payload[len++] = 0xB8;
                pcmd_src.payload[len++] = 0x0B;
                pcmd_src.payload[len++] = ATS3607_HELPER_ENABLE_STATUS;
                memcpy(pcmd_src.payload + len, &timeout_ms, sizeof(timeout_ms));
                len += sizeof(timeout_ms);
                pcmd_src.payload[len++] = ats_vid == 0 || ats_pid == 0 ? 1 : 0;// 1: stop. 0: start.
                pcmd_src.payload[len++] = (ats_vid == 0 || ats_pid == 0) ? 1 : 0;// 1: dsp event. 0: update event
                pcmd_src.len = len;
                Res = HidCmdSend(&pcmd_dst, &pcmd_src, USB_TIME_OUT_MS);
            }

            if (!Res)
            {
                HidInterfaceClose();
                if (pcmd_dst.payload[3] == ATS3607_HELPER_ENABLE)
                {
                    UINT    sleep_ms = ((BYTE)pcmd_dst.payload[2] << 0x08) | (BYTE)pcmd_dst.payload[1];

                    Sleep(ATS3607_UART_RX_DELAY_MS);          // Waiting for dsp update event

                    Res = ATS3607Update();
                    if (Res)
                    {
                        debug_printf("ATS3607 update failed res: %X \r\n", Res);
                    }
                    else
                    {
                        if (HidInterfaceOpen())
                        {
                            Sleep(2000);      // Waiting for dsp status event
                            len = 3;
                            pcmd_src.cmd = GW_HID_CMD_3607D_UPGRADE;
                            pcmd_src.payload[len++] = ATS3607_HELPER_STATUS;
                            pcmd_src.len = len;
                            Res = HidCmdSend(&pcmd_dst, &pcmd_src, USB_TIME_OUT_MS);
                            if (!Res)
                            {
                                if (pcmd_dst.payload[0])
                                    Ret = ATS3607_HELPER_SUCCESSFUL;
                            }
                        }
                    }
                }
                else
                {
                    debug_printf("ATS3607 activate and enable failed \r\n");
                    Ret = ATS3607_HELPER_ACTIVATE_ENABLE_FAILED;
                }
            }
            else
            {
                debug_printf("ATS3607 activate failed \r\n");
                Ret = ATS3607_HELPER_ACTIVATE_FAILED;
            }
        }
        else
            Ret = ATS3607_HELPER_OPEN_FAILED;
    }
ats_update_end:
    HidInterfaceClose();
    return Ret;
}
