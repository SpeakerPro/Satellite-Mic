#include "stdafx.h"
#include "GW_ats3607_helper.h"
#include "GW_usb_info.h"
#include "GW_debug.h"
#include "HID.hpp"

#define ATS3607_LOG_FILENAME                    "ats3607.log"
#define USB_MAX_HEADER_TIMEOUT_MS               60000

#define USB_MAX_UART_RX_TIMEOUT_MS              5
#define USB_MIN_UART_RX_TIMEOUT_MS              1
#define USB_TIME_OUT_MS                         500
#define ATS3607_UPGRADE_CANCEL_TIMEOUT_MS       5000

#define PORT_VID                                0x0419
#define PORT_PID                                0x225

#define ATS3607_UPDATE_MAX_LENGTH               126

#define ATS3607_UPDATE_OPEN                     0x55
#define ATS3607_UPDATE_TRANSFER                 0x56
#define ATS3607_UPDATE_CLOSE                    0x57

#define ATS3607_UPDATE_OPERATION_INDEX          3
#define ATS3607_UPDATE_LINE_INDEX               4
#define ATS3607_UPDATE_PAGE_INDEX               5
#define ATS3607_UPDATE_CMD_INDEX                6
#define ATS3607_UPDATE_DATA_INDEX               8

#define USBTREE_EXE_COMMAND                     "tools\\dsp_update.exe"

static UINT     file_size;
static PCHAR    f_ats3607_payload;
static FILE* f_ats3607;

BYTE    is_mcu_helper = false;
CHAR    binary_name[MAX_PATH];
UINT    ats_vid, ats_pid;

FILE* fp_log = NULL;
FILE* LogRegister(const char* file)
{
    return file != NULL ? fopen(file, "wb+") : NULL;
}

int LogPrint(const char* format, ...)
{
    if (fp_log == NULL)
        return 0;

    char    buffer[1024] = {};
    int     len;
    va_list ap;

    va_start(ap, format);
    vsnprintf(buffer, sizeof(buffer), format, ap);
    va_end(ap);

    len = fprintf(fp_log, "%s", (const char*)buffer);
    fflush(fp_log);

    return len;
}

void LogPrintHex(PCHAR header, PCHAR payload, UINT size)
{
    CHAR    buffer[256];
    UINT    i, len;

    LogPrint(" - %s\r\n", header);
    for (i = 0; i < size; i++)
    {
        if ((i % 16) == 0)
        {
            len = 0;
            memset(buffer, 0, sizeof(buffer));
            len = sprintf(buffer, "%08X:  ", i);    //11
        }
        len += sprintf(buffer + len, "%02X ", (UCHAR)payload[i]);
        if (((i + 1) % 16) == 0 || (i + 1) == size)
        {
            len += sprintf(buffer + len, "\r\n");
            LogPrint(buffer);
        }
    }

    LogPrint("  Message length: %ld\r\n\r\n", size);
}

void LogUnregister(FILE* f)
{
    if (f != NULL)
        fclose(f);
}

UINT ATS3607BinaryOpen(PCHAR filename)
{
    f_ats3607 = fopen((CONST PCHAR)filename, "rb");
    if (f_ats3607 == NULL)
        return 0;

    fseek(f_ats3607, 0, SEEK_END);
    file_size = ftell(f_ats3607);
    if (file_size == 0)
        return 0;

    fseek(f_ats3607, 0, SEEK_SET);

    f_ats3607_payload = (PCHAR)malloc(file_size);
    if (f_ats3607_payload == NULL)
    {
        fclose(f_ats3607);
        return 0;
    }
    fread(f_ats3607_payload, sizeof(CHAR), file_size, f_ats3607);

    return file_size;
}

UINT ATS3607BinaryDataGet(PCHAR payload, UINT data_address, UINT data_index)
{
    if (f_ats3607_payload != NULL)
    {
        memcpy(payload, f_ats3607_payload + data_address, data_index);
        return data_index;
    }
    else
        return 0;
}

UINT ATS3607BinaryClose(void)
{
    if (f_ats3607_payload != NULL)
        free(f_ats3607_payload);

    if (f_ats3607 != NULL)
        fclose(f_ats3607);

    return 0;
}

void ATS3607HelperInit(UINT vid, UINT pid, BOOL serial)
{
    ats_vid = vid == 0 ? PORT_VID : vid;
    ats_pid = pid == 0 ? PORT_PID : pid;
    is_mcu_helper = serial;
}

INT ATS3607Read(BYTE* payload, BYTE* len, UINT timeout_ms)
{
    DEV_CMD_T   pcmd_dst = { 0 }, pcmd_src = { 0 };
    UINT        res;
    
    pcmd_src.cmd = GW_HID_CMD_3607D_UPGRADE;
    pcmd_src.payload[0] = 0x00;
    pcmd_src.payload[1] = 0x00;
    pcmd_src.payload[2] = 0x00;
    pcmd_src.payload[3] = ATS3607_HELPER_READ;
    pcmd_src.payload[6] = 0x0F;
    pcmd_src.len = 7;
    res = HidCmdSend(&pcmd_dst, &pcmd_src, timeout_ms);
    if (!res)
    {
        *len = pcmd_dst.len - 6;
        memcpy(payload, pcmd_dst.payload + 6, *len);

        CHAR        buffer[256] = { 0 };
        SYSTEMTIME  now;
        GetLocalTime(&now);
        sprintf(buffer, "ATS3607 read timestamp: %02d:%02d:%02d.%02d", now.wHour, now.wMinute, now.wSecond, now.wMilliseconds);
        LogPrintHex(buffer, (PCHAR)payload, *len);
    }
    else
        return 1;

    return 0;
}

INT ATS3607SectionWrite(BYTE* payload, BYTE len)
{
    UINT    target_index = 0, target_length;

    while (target_index != len)
    {
        DEV_CMD_T   pcmd_dst = { 0 }, pcmd_src = { 0 };
        UINT        res;

        target_length = len - target_index > ATS3607_PAYLOAD_MAX_LENGTH ? ATS3607_PAYLOAD_MAX_LENGTH : len - target_index;
        memcpy(pcmd_src.payload + ATS3607_PAYLOAD_HEADER_LENGTH, payload + target_index, target_length);
        target_index += target_length;

        pcmd_src.cmd = GW_HID_CMD_3607D_UPGRADE;
        pcmd_src.payload[0] = 0x00;
        pcmd_src.payload[1] = 0x00;
        pcmd_src.payload[2] = 0x00;
        pcmd_src.payload[3] = ATS3607_HELPER_WRITE;
        pcmd_src.payload[4] = target_index / ATS3607_PAYLOAD_MAX_LENGTH + (target_index % ATS3607_PAYLOAD_MAX_LENGTH == 0 ? 0 : 1);
        pcmd_src.payload[5] = len / ATS3607_PAYLOAD_MAX_LENGTH + (len % ATS3607_PAYLOAD_MAX_LENGTH == 0 ? 0 : 1);
        pcmd_src.len = target_length + ATS3607_PAYLOAD_HEADER_LENGTH;
        res = HidCmdSend(&pcmd_dst, &pcmd_src, USB_TIME_OUT_MS);
        if (res)
            return 1;
    }

    CHAR        buffer[256] = { 0 };
    SYSTEMTIME  now;
    GetLocalTime(&now);
    sprintf(buffer, "ATS3607 write timestamp: %02d:%02d:%02d.%02d", now.wHour, now.wMinute, now.wSecond, now.wMilliseconds);
    LogPrintHex(buffer, (PCHAR)payload, len);

    return 0;
}

INT ATS3607Update(void)
{
#define USB_RETRY_COUNT                         10
#define USB_SECTION_WRITE_TRY_COUNT             3
    CHAR    file_path[MAX_PATH] = { 0 }, cmd[MAX_PATH * 2] = { 0 };
    INT     key_space, ret = 1;

    BOOL	is_timeout = FALSE, ats3607_update_state_check = FALSE;
    UINT	now, last = GetTickCount();
    if (is_mcu_helper)
    {
        UINT    count = 0, count_write_failed = 0, timeout_ms = USB_MAX_UART_RX_TIMEOUT_MS;
        while (!is_timeout)
        {
            now = GetTickCount();
            BYTE    cmd = 0, payload[ATS3607_UPDATE_MAX_LENGTH] = { 0 }, len;
            UINT    offset = 0, res = 1;
            res = ATS3607Read(payload, (BYTE*)&len, timeout_ms);
            if (!res)
            {
                cmd = payload[0];
                if (cmd == ATS3607_UPDATE_OPEN)
                {
                    debug_printf("ATS3607 opened. \r\n");
                    payload[0] = ATS3607_UPDATE_OPEN;
                    payload[1] = 0x0F;
                    payload[2] = 0x01;
                    payload[14] = 0x59;
                    len = 15;
                    count = 0;
                }
                else if (cmd == ATS3607_UPDATE_TRANSFER)
                {
                    memcpy(&offset, payload + 2, sizeof(offset));
                    memcpy(&len, payload + 6, sizeof(len));

                    ATS3607BinaryDataGet((PCHAR)payload, offset, len);
                }
                else if (cmd == ATS3607_UPDATE_CLOSE)
                {
                    debug_printf("\r\n");
                    debug_printf("ATS3607 closed.\r\n");
                    continue;
                }

#define ATS3607_CMD_UPDATE_MAX_LEN			    6
#define ATS3607_CMD_UPDATE_SUCCESSFUL		    {0xA5, 0x04, 0x04, 0x08, 0x41, 0xF6}
                if (len == ATS3607_CMD_UPDATE_MAX_LEN)
                {
                    if (payload[0] == 0xA5 && payload[1] == 0x04 && payload[2] == 0x04)
                    {
                        CHAR successful_msg[] = ATS3607_CMD_UPDATE_SUCCESSFUL;
                        if (memcmp(successful_msg, payload, ATS3607_CMD_UPDATE_MAX_LEN) == 0)
                            ret = 0;
                        break;
                    }
                }

                for (UINT i = 0; i < USB_SECTION_WRITE_TRY_COUNT; i++)
                {
                    if (ATS3607SectionWrite(payload, len) == 0)
                    {
                        if (cmd == ATS3607_UPDATE_TRANSFER)
                        {
                            debug_printf("ATS3607 transfer. State: %3d\r", ((offset + len) * 100) / file_size);
                            ++count;
                        }
                        break;
                    }

                    if (i + 1 == USB_SECTION_WRITE_TRY_COUNT)
                        count_write_failed++;
                }
                timeout_ms = USB_MAX_UART_RX_TIMEOUT_MS;
                last = now;
            }
            else
            {
                timeout_ms = USB_MIN_UART_RX_TIMEOUT_MS;
            }

_ats3607_update_timeout_check:
            Sleep(3);
            if (now >= last)
                is_timeout = now - last > USB_MAX_HEADER_TIMEOUT_MS ? TRUE : FALSE;
            else
                is_timeout = now > USB_MAX_HEADER_TIMEOUT_MS ? TRUE : FALSE;
        }
        debug_printf("%sATS3607 write retry count: %d. Access count: %d. State: %s .\r\n", ret ? "\r\n" : "", count_write_failed, count, ret != 0? "timeout": "finished");
    }
    else
    {
        GetCurrentDirectoryA(sizeof(file_path), file_path);

        std::string str(file_path);
        key_space = str.find(' ');
        if (key_space != -1)
            PathCombineA(file_path, ".\\", USBTREE_EXE_COMMAND);
        else
            PathCombineA(file_path, file_path, USBTREE_EXE_COMMAND);
        sprintf(cmd, "%s -bin %s -vid %d -pid %d", file_path, binary_name, ats_vid, ats_pid);

        ret = system(cmd);
    }

    return ret;
}

UINT ATS3607HelperRun(PCHAR binary)
{
    ATS3607_HELPER_MACHINE_CODE_E   Ret = ATS3607_HELPER_SUCCESSFUL;
    HID_APP_EVT_S                   evt;

    INT         Res, len;
    DEV_CMD_T   pcmd_dst, pcmd_src;

    fp_log = LogRegister(ATS3607_LOG_FILENAME);
    if (!HidInterfaceOpen())
    {
        debug_log("ATS3607 Can't Open HID Device\n");
        Ret = ATS3607_HELPER_OPEN_FAILED;
    }
    else
    {
        memset(binary_name, 0, sizeof(binary_name));
        if (binary != NULL)
            memcpy(binary_name, binary, strlen(binary));
        else
            memcpy(binary_name, "ast3607_ota.bin", strlen("ast3607_ota.bin"));

        ATS3607BinaryOpen(binary);

        UINT    timeout_ms = USB_MAX_HEADER_TIMEOUT_MS;
        len = 0;
        pcmd_src.cmd = GW_HID_CMD_3607D_UPGRADE;
        pcmd_src.payload[len++] = 0x00;
        pcmd_src.payload[len++] = 0xB8;     // Delay for 3000 ms
        pcmd_src.payload[len++] = 0x0B;     // Delay for 3000 ms
        pcmd_src.payload[len++] = ATS3607_HELPER_ENABLE;
        memcpy(pcmd_src.payload + len, &timeout_ms, sizeof(timeout_ms));
        len += sizeof(timeout_ms);
        pcmd_src.len = len;
        Res = HidCmdSend(&pcmd_dst, &pcmd_src, USB_TIME_OUT_MS);
        if (!Res)
        {
            if (pcmd_dst.payload[3] == ATS3607_HELPER_ENABLE)
            {
                UINT    sleep_ms = ((BYTE)pcmd_dst.payload[2] << 0x08) | (BYTE)pcmd_dst.payload[1];

                Sleep(sleep_ms);

                Res = ATS3607Update();
                if (Res)
                {
                    debug_log("ATS3607 update failed res: %X \r\n", Res);
                    Ret = ATS3607_HELPER_UPDATE_FAILED;
                }
            }
            else
            {
                debug_log("ATS3607 activate and enable failed \r\n");
                Ret = ATS3607_HELPER_ACTIVATE_ENABLE_FAILED;
            }
        }
        else
        {
            debug_log("ATS3607 activate failed \r\n");
            Ret = ATS3607_HELPER_ACTIVATE_FAILED;
        }
        ATS3607BinaryClose();
        HidInterfaceClose();
    }
    LogUnregister(fp_log);
    return Ret;
}
