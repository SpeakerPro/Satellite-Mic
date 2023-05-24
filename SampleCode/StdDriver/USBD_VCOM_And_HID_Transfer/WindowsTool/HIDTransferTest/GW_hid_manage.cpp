#include "stdafx.h"
#include "GW_hid_manage.h"
#include "GW_usb_info.h"
#include "GW_debug.h"
#include "GW_isp_helper.h"
#include "GW_hid_cmd_table.h"
#include "HID.hpp"

#include <windows.h>
#include <stdio.h>

#define USB_VID_NATIVE							0x0416
#define USB_PID_NATIVE							0x3F00
#define USB_VID_GW								0x17EF
#define USB_PID_GW								0xA052

#define USB_VID_WIRED_MIC_CM100				    0x17EF
#define USB_PID_WIRED_MIC_CM100				    0xA064
#define USB_VID_WIRELESS_MIC_WM200				0x17EF
#define USB_PID_WIRELESS_MIC_WM200				0xA074

#define USB_HID_TIME_OUT						500
#define USB_TIME_OUT							100

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */

typedef struct
{
    unsigned char   cmd;
    unsigned char   len;
    unsigned int    arg1;
    unsigned int    arg2;
    unsigned int    signature;
    unsigned int    checksum;
} CMD_T;

#pragma pack(pop)   /* restore original alignment from stack */

BOOL            IOHidStatus = FALSE;
CHidCmd         IOHid;
HANDLE          ThreadHandle;
DWORD           ThreadId;
USHORT	        USB_VID = 0, USB_PID = 0, USB_VID_DEFAULT = 0, USB_PID_DEFAULT = 0;
UINT            hid_timeout_ms;
HANDLE          ghMutex;
BOOL UsbIspConnectionState(void)
{
    CHidCmd io;

    if (USB_VID_DEFAULT != 0 && USB_PID_DEFAULT != 0)
    {
        USB_VID = USB_VID_DEFAULT;
        USB_PID = USB_PID_DEFAULT;
        if (io.OpenDevice(USB_VID, USB_PID))
        {
            io.CloseDevice();
            return TRUE;
        }
        debug_log("HID VID/PID %04X/%04X trying...\r\n", USB_VID_GW, USB_PID_GW);
        return FALSE;
    }

    USB_VID = USB_VID_GW;
    USB_PID = USB_PID_GW;
    if (io.OpenDevice(USB_VID, USB_PID))
    {
        io.CloseDevice();
        return TRUE;
    }
    debug_log("HID VID/PID %04X/%04X trying...\r\n", USB_VID_GW, USB_PID_GW);

    USB_VID = USB_VID_WIRED_MIC_CM100;
    USB_PID = USB_PID_WIRED_MIC_CM100;
    if (io.OpenDevice(USB_VID, USB_PID))
    {
        io.CloseDevice();
        return TRUE;
    }
    debug_log("HID VID/PID %04X/%04X trying...\r\n", USB_VID_WIRED_MIC_CM100, USB_PID_WIRED_MIC_CM100);

    USB_VID = USB_VID_WIRELESS_MIC_WM200;
    USB_PID = USB_PID_WIRELESS_MIC_WM200;
    if (io.OpenDevice(USB_VID, USB_PID))
    {
        io.CloseDevice();
        return TRUE;
    }
    debug_log("HID VID/PID %04X/%04X trying...\r\n", USB_VID_WIRELESS_MIC_WM200, USB_PID_WIRELESS_MIC_WM200);

    USB_VID = USB_VID_NATIVE;
    USB_PID = USB_PID_NATIVE;
    if (io.OpenDevice(USB_VID, USB_PID))
    {
        io.CloseDevice();
        return TRUE;
    }
    debug_log("HID VID/PID %04X/%04X trying...\r\n", USB_VID_NATIVE, USB_PID_NATIVE);

    return FALSE;
}

BOOL HidTryOpen(UINT timeout)
{
    BOOL	is_timeout = FALSE;
    UINT	now, last;
    last = GetTickCount();
    do
    {
        now = GetTickCount();
        if (now >= last)
            is_timeout = now - last > timeout ? TRUE : FALSE;
        else
            is_timeout = now > timeout ? TRUE : FALSE;

        if (UsbIspConnectionState())
            break;
        Sleep(50);
    } while (!is_timeout);
    return is_timeout == FALSE ? TRUE : FALSE;
}

void LogHeximal(PCHAR payload, UINT size)
{
    CHAR    buffer[256];
    UINT    i, len;
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
            debug_log(buffer);
        }
    }

    debug_log("  Message length: %ld\r\n", size);
}

void PrintHeximal(PCHAR payload, UINT size)
{
    CHAR    buffer[256];
    UINT    i, len;
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
            debug_printf(buffer);
        }
    }

    debug_printf("  Message length: %ld\r\n", size);
}

PCHAR HidCmdShow(UCHAR cmd)
{
    if (cmd == GW_HID_CMD_TEST)
        return "Test";
    else if (cmd == GW_HID_CMD_ECHO)
        return "Echo";
    else if (cmd == GW_HID_CMD_AW5808_UPGRADE)
        return "Upgrade";
    else if (cmd == GW_HID_CMD_VERSION_GET)
        return "VersionGet";
    else if (cmd == GW_HID_CMD_ROM_REBOOT)
        return "RomReboot";
    else if (cmd == GW_HID_CMD_ROM_MODE)
        return "RomMode";
}

HID_WRITE_TYPE_E    hid_write_type = HID_WRITE_CMD_TYPE;
HID_PAYLOAD_U       hid_recv;
HID_APP_EVT_S       app_evt_list[APP_EVENT_COUNT];
void HidTimeShow(PCHAR func)
{
    SYSTEMTIME		now;

    WORD wHour;
    WORD wMinute;
    WORD wSecond;
    WORD wMilliseconds;
    GetLocalTime(&now);
    debug_log("%s: Time %02d:%02d:%02d:%02d\r\n", func, now.wHour, now.wMinute, now.wSecond, now.wMilliseconds);
}

INT HidWrite(PCHAR payload_out, PCHAR payload_in, HID_WRITE_TYPE_E type, INT timeout_ms)
{
    BOOL        bRet, is_timeout;
    INT         Ret = 0;
    UINT        now, last;
    DWORD       length, count = 0;
    DEV_CMD_S   * p_cmd = (DEV_CMD_S *)payload_in;

    if (WaitForSingleObject(ghMutex, INFINITE) != WAIT_OBJECT_0)
        return 1;

    LogHeximal((PCHAR)payload_in, sizeof(HID_PAYLOAD_U));
    hid_write_type = type;
    bRet = IOHid.WriteFile((PUCHAR)payload_in, sizeof(HID_PAYLOAD_U), &length, timeout_ms);

    if (!ReleaseMutex(ghMutex))
        debug_log("Hid write mutex unlease failed\r\n");

    if (!bRet)
    {
        if (timeout_ms == 0)
            return Ret;

        debug_log("Send verify command error!\r\n");
        Ret = 1;
    }
    else if (timeout_ms != 0)
    {
        hid_recv.cmd_std.sub_cmd_t.cmd = p_cmd->sub_cmd_t.cmd;
        hid_recv.cmd_std.sub_cmd_t.len = 0;

        is_timeout = FALSE;
        last = GetTickCount();
        do
        {
            now = GetTickCount();
            if(now >= last)
                is_timeout = now - last > timeout_ms ? TRUE : FALSE;
            else
                is_timeout = now > timeout_ms ? TRUE : FALSE;

            if (hid_write_type == HID_WRITE_CMD_TYPE && hid_recv.cmd_std.sub_cmd_t.len != 0)
                break;
            else if (hid_write_type == HID_WRITE_IDLE_TYPE)
                break;
        } while (is_timeout == FALSE);

        if (is_timeout)
            Ret = 2;
        else
            memcpy(payload_out, &hid_recv, sizeof(hid_recv));
    }

    memset(&hid_recv, 0, sizeof(hid_recv));
    hid_write_type = HID_WRITE_CMD_TYPE;
    return Ret;
}

BOOL HidBufferSend(PCHAR pcmd_dst, PCHAR pcmd_src, INT timeout_ms)
{
    BOOL            Ret = 0;
    DWORD           length, timeout_count;
    HID_PAYLOAD_U   cmd = {0}, msg = {0};

    memcpy(&cmd, pcmd_src, sizeof(HID_PAYLOAD_U));

    debug_log("\r\n<<< HidBufferRequest:\r\n");
    Ret = HidWrite((PCHAR)&msg, (PCHAR)&cmd, HID_WRITE_BUFFER_TYPE, timeout_ms);
    if (!Ret)
    {
        debug_log("\r\n>>> HidBufferResponse:\r\n");
        memcpy(pcmd_dst, &msg, sizeof(HID_PAYLOAD_U));
        LogHeximal(pcmd_dst, sizeof(HID_PAYLOAD_U));
    }
    else
        debug_log("Request buffer failed. Ret %X\r\n", Ret);

    return Ret;
}

INT HidCmdSend(DEV_CMD_T* pcmd_dst, DEV_CMD_T* pcmd_src, INT timeout_ms)
{
    BOOL            Ret = 0;
    DWORD           length, timeout_count;
    HID_PAYLOAD_U   cmd = {0}, msg = {0};

    cmd.cmd_std.cmd_t.cmd = HID_CMD_EVENT_REQUEST;
    memcpy(&cmd.cmd_std.sub_cmd_t, pcmd_src, sizeof(cmd.cmd_std.sub_cmd_t));

    debug_log("\r\n<<< HidCmdRequest: %s\r\n", HidCmdShow(cmd.cmd_std.sub_cmd_t.cmd));
    Ret = HidWrite((PCHAR)&msg.cmd_std, (PCHAR)&cmd.cmd_std, HID_WRITE_CMD_TYPE, timeout_ms);
    if(!Ret)
    {
        memcpy(pcmd_dst, &msg.cmd_std.sub_cmd_t, sizeof(DEV_CMD_T));
        LogHeximal(pcmd_dst->payload, pcmd_dst->len);
    }
    else
        debug_log("Request cmd[%02X] failed. Ret %X\r\n", cmd.cmd_std.sub_cmd_t.cmd, Ret);

    return Ret;
}

INT HidCmdResponse(DEV_CMD_T* pcmd_src, INT timeout_ms)
{
    BOOL        bRet;
    INT         Ret = 0;
    DWORD       length, timeout_count;
    DEV_CMD_S   cmd = {0};

    cmd.cmd_t.cmd = HID_CMD_EVENT_RESPONSE;
    memcpy(&cmd.sub_cmd_t, pcmd_src, sizeof(cmd.sub_cmd_t));

    debug_log("\r\n>>> HidCmdResponse: %s\r\n", HidCmdShow(cmd.sub_cmd_t.cmd));
    LogHeximal((PCHAR)&cmd.sub_cmd_t.payload, cmd.sub_cmd_t.len);
    bRet = IOHid.WriteFile((PUCHAR)&cmd, sizeof(cmd), &length, timeout_ms);
    if (!bRet)
    {
        debug_log("Send verify command error!\r\n");
        Ret = 1;
    }
    return Ret;
}

BOOL USBConnectionState(void)
{
    BOOL        Res;
    PUINT       p_version;
    DEV_CMD_T   MSG = { 0 };
    p_version = (PUINT)&MSG.payload[VERSION_TYPE_PAYLOAD_INDEX];
    Res = VersionGet(&MSG, VERSION_TYPE_MCU);
    if (!Res)
        debug_log("Version MCU not found: %X\r\n", MSG.payload[VERSION_TYPE_STATUS_INDEX]);
    else
        debug_log("Version MCU get: %X\r\n", *p_version);

    return TRUE;
}

DWORD WINAPI HidListener(LPVOID Param)
{
    BOOL            bRet;
    DWORD           i, length;
    DEV_CMD_S       cmd_in = {0}, cmd_out = {0};
    DEV_CMD_T       buffer;

    while (1)
    {
        memset(&cmd_in, 0x00, sizeof(cmd_in));
        memset(&cmd_out, 0x00, sizeof(cmd_out));
        bRet = IOHid.ReadFile((PUCHAR)&cmd_in, 256, &length, USB_TIME_OUT);
        if (!bRet)
            continue;

        if (WaitForSingleObject(ghMutex, INFINITE) != WAIT_OBJECT_0)
            continue;

        if (hid_write_type == HID_WRITE_BUFFER_TYPE)
        {
            hid_write_type = HID_WRITE_IDLE_TYPE;
            memcpy((UCHAR*)&hid_recv, (UCHAR*)&cmd_in, sizeof(cmd_in));
        }
        else
        {
            if (cmd_in.cmd_t.cmd == HID_CMD_EVENT_RESPONSE || cmd_in.cmd_t.cmd == HID_CMD_EVENT_REQUEST)
            {
                debug_log("\r\n");
                debug_log(">>> %s evt: %s\r\n", cmd_in.cmd_t.cmd == HID_CMD_EVENT_RESPONSE ? "Response" : "Request", HidCmdShow(cmd_in.sub_cmd_t.cmd));
                for (i = 0; i < APP_EVENT_COUNT; i++)
                {
                    if (cmd_in.sub_cmd_t.cmd == app_evt_list[i].cmd)// || app_evt_list[i].cmd == GW_HID_CMD_WILDCARD)
                    {
                        if (app_evt_list[i].cb != NULL)
                            app_evt_list[i].cb(&cmd_in, &cmd_out);
                        else
                            cmd_out.sub_cmd_t.len = 0;

                        if (cmd_in.cmd_t.cmd == HID_CMD_EVENT_RESPONSE)
                        {
                            if (cmd_in.sub_cmd_t.cmd == hid_recv.cmd_std.sub_cmd_t.cmd)
                                memcpy((UCHAR*)&hid_recv, (UCHAR*)&cmd_in, sizeof(cmd_in));
                        }
                        else
                            LogHeximal(cmd_in.sub_cmd_t.payload, cmd_in.sub_cmd_t.len);

                        if (app_evt_list[i].cmd != GW_HID_CMD_WILDCARD && cmd_out.sub_cmd_t.len)
                        {
                            HidCmdResponse(&cmd_out.sub_cmd_t, USB_TIME_OUT);
                            break;
                        }
                    }
                }
            }
            else
            {
                debug_log("Recv evt: %X %X\r\n", cmd_in.cmd_t.cmd, cmd_in.sub_cmd_t.cmd);
                LogHeximal(cmd_in.sub_cmd_t.payload, cmd_in.sub_cmd_t.len);
            }
        }

        if (!ReleaseMutex(ghMutex))
            debug_log("Hid read mutex unlease failed\r\n");
    }
    return 0;
}

UINT HidAppEventRegister(HID_APP_EVT_S params)
{
    UCHAR	i = 1, res = 2;

    if (params.cmd == 0)
        return 1;
    else if (params.cmd == GW_HID_CMD_WILDCARD)
        i = 0;

    for (; i < APP_EVENT_COUNT; i++)
    {
        if (app_evt_list[i].cmd == 0)
        {
            app_evt_list[i].cmd = params.cmd;
            app_evt_list[i].cb = params.cb;
            res = 0;
            break;
        }
    }
    return res;
}

UINT HidAppEventUnregister(HID_APP_EVT_S params)
{
    UCHAR	i, res = 2;

    if (params.cmd == 0)
        return 1;

    for (i = 0; i < APP_EVENT_COUNT; i++)
    {
        if (app_evt_list[i].cmd == params.cmd)
        {
            app_evt_list[i].cmd = 0;
            app_evt_list[i].cb = NULL;
            res = 0;
            break;
        }
    }
    return res;
}

void EvtLDromBootCb(DEV_CMD_S* app_evt_cmd_out, DEV_CMD_S* app_evt_cmd_in)
{
    debug_log("========== EvtLDromBootCb test ==========\n");
    debug_log("EvtLDromBootCb, cmd %X, len %d\n", app_evt_cmd_in->sub_cmd_t.cmd, app_evt_cmd_in->sub_cmd_t.len);

    app_evt_cmd_out->sub_cmd_t.cmd = GW_HID_CMD_ROM_REBOOT;
    app_evt_cmd_out->sub_cmd_t.len = 0x03;
    app_evt_cmd_out->sub_cmd_t.payload[0] = 0xE0;
    app_evt_cmd_out->sub_cmd_t.payload[1] = 0xE1;
    app_evt_cmd_out->sub_cmd_t.payload[2] = 0xE2;
}

void EvtWildcardCb(DEV_CMD_S* app_evt_cmd_out, DEV_CMD_S* app_evt_cmd_in)
{
    debug_log("========== EvtWildcardCb test ==========\n");
    debug_log("EvtWildcardCb, cmd %X, len %d\n", app_evt_cmd_in->sub_cmd_t.cmd, app_evt_cmd_in->sub_cmd_t.len);
}

void HidEventInit(void)
{
    HID_APP_EVT_S evt;
#if 0
    evt.cmd = GW_HID_CMD_WILDCARD;
    evt.cb = EvtWildcardCb;
    HidAppEventRegister(evt);
#endif
    // USB to private command set example. maximum number of cmd is APP_EVENT_COUNT
    evt.cmd = GW_HID_CMD_TEST;
    evt.cb = NULL;
    HidAppEventRegister(evt);

    // USB to private command set example. maximum number of cmd is APP_EVENT_COUNT
    evt.cmd = GW_HID_CMD_ECHO;
    evt.cb = NULL;
    HidAppEventRegister(evt);

    // USB to private command set example. maximum number of cmd is APP_EVENT_COUNT
    evt.cmd = GW_HID_CMD_VERSION_GET;
    evt.cb = NULL;
    HidAppEventRegister(evt);

    evt.cmd = GW_HID_CMD_CONFIGURATION_SET;
    evt.cb = NULL;
    HidAppEventRegister(evt);

    evt.cmd = GW_HID_CMD_DEVICE_INFO_GET;
    evt.cb = NULL;
    HidAppEventRegister(evt);

    evt.cmd = GW_HID_CMD_3607D_UPGRADE;
    evt.cb = NULL;
    HidAppEventRegister(evt);

    evt.cmd = GW_HID_CMD_ROM_REBOOT;
    evt.cb = NULL;
    HidAppEventRegister(evt);

    evt.cmd = GW_HID_CMD_ROM_MODE;
    evt.cb = NULL;
    HidAppEventRegister(evt);
}

void HidEventUninit(void)
{
    memset(app_evt_list, 0, sizeof(app_evt_list));
}

void HidListenerInit(void)
{
    ghMutex = CreateMutex(NULL, FALSE, NULL);
    if (ghMutex == NULL)
        debug_log("CreateMutex error: %d\n", GetLastError());
    ThreadHandle = CreateThread(NULL, 0, HidListener, NULL, 0, &ThreadId);
}

void HidListenerUninit(void)
{
    DWORD Res = 0;
    TerminateThread(ThreadHandle, Res);
    CloseHandle(ThreadHandle);
    CloseHandle(ghMutex);
}

void UsbCmdTest(void)
{
    INT i, Res;
    DEV_CMD_T    pcmd_dst, pcmd_src;

    if (!HidInterfaceOpen())
    {
        debug_printf("Can't Open HID Device\n");
        return;
    }
    pcmd_src.cmd = GW_HID_CMD_TEST;
    Res = HidCmdSend(&pcmd_dst, &pcmd_src, USB_TIME_OUT);
    HidInterfaceClose();

    debug_printf("----- USB Test command: \r\n");
    PrintHeximal(pcmd_dst.payload, pcmd_dst.len);
}

void UsbCmdEcho(void)
{
    INT i, Res;
    DEV_CMD_T    pcmd_dst, pcmd_src;

    if (!HidInterfaceOpen())
    {
        debug_log("Can't Open HID Device\n");
        return;
    }

    pcmd_src.cmd = GW_HID_CMD_ECHO;
    pcmd_src.len = 13;
    for (i = 0; i < sizeof(pcmd_src.payload); i++)
        pcmd_src.payload[i] = 0x11 + i * 0x10;

    Res = HidCmdSend(&pcmd_dst, &pcmd_src, USB_TIME_OUT);

    HidInterfaceClose();
}

BOOL VersionGet(DEV_CMD_T * pcmd_dst, UCHAR type)
{
    INT         Res;
    DEV_CMD_T   pcmd_src = { 0 };

    pcmd_src.cmd = GW_HID_CMD_VERSION_GET;
    pcmd_src.len = 1;
    pcmd_src.payload[VERSION_TYPE_INDEX] = type;

    Res = HidCmdSend(pcmd_dst, &pcmd_src, USB_TIME_OUT);
    return Res == 0 ? TRUE : FALSE;
}

BOOL ConfigurationSetSend(DEV_CMD_T* pcmd_dst, PCHAR pbuffer, BYTE length)
{
    INT         Res;
    DEV_CMD_T   pcmd_src = { 0 };

    pcmd_src.cmd = GW_HID_CMD_CONFIGURATION_SET;
    pcmd_src.len = length;
    memcpy(pcmd_src.payload, pbuffer, length);

    Res = HidCmdSend(pcmd_dst, &pcmd_src, USB_TIME_OUT);
    return Res == 0 ? TRUE : FALSE;
}

BOOL DeviceInfoGet(DEV_CMD_T* pcmd_dst, UCHAR type)
{
#define DEVICE_INFO_GET_LEN     1
    UCHAR       length = DEVICE_INFO_GET_LEN, buffer[DEVICE_INFO_GET_LEN] = { 0 };
    INT         Res;
    DEV_CMD_T   pcmd_src = { 0 };

    pcmd_src.cmd = GW_HID_CMD_DEVICE_INFO_GET;
    pcmd_src.len = length;
    memcpy(pcmd_src.payload, &buffer, length);

    Res = HidCmdSend(pcmd_dst, &pcmd_src, USB_TIME_OUT);
    return Res == 0 ? TRUE : FALSE;
}

BOOL HidInterfaceStatus(void)
{
    return IOHidStatus;
}

void HidParamsSet(UINT timeout_ms)
{
    hid_timeout_ms = timeout_ms;
}

void HidParamsIdSet(UINT vid, UINT pid)
{
    USB_VID_DEFAULT = vid;
    USB_PID_DEFAULT = pid;
}

void HidParamsIdGet(UINT * vid, UINT * pid)
{
    if (USB_VID == 0 || USB_PID == 0)
        HidTryOpen(50);
    *vid = USB_VID;
    *pid = USB_PID;
}

BOOL HidInterfaceOpen(void)
{
    if (IOHidStatus)
        return TRUE;

    if (!HidTryOpen(hid_timeout_ms))
    {
        debug_log("HID not found! Stopped!\r\n");
        return FALSE;
    }
    else
    {
        debug_log("HID VID/PID: %02X/%02X found.\r\n", USB_VID, USB_PID);
    }

    if (IOHid.OpenDevice(USB_VID, USB_PID))
    {
        HidEventInit();
        HidListenerInit();
        IOHidStatus = TRUE;
        return TRUE;
    }
    else
        return FALSE;
}

void HidInterfaceClose(void)
{
    if (!IOHidStatus)
        return;
    IOHidStatus = FALSE;
    HidEventUninit();
    HidListenerUninit();
    Sleep(10);
    IOHid.CloseDevice();
}
