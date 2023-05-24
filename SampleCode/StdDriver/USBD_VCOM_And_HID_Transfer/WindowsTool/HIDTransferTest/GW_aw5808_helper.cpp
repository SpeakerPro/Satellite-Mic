#include "stdafx.h"
#include "GW_aw5808_helper.h"
#include "GW_isp_helper.h"
#include "GW_usb_info.h"
#include "GW_debug.h"
#include "HID.hpp"

#define USB_TIME_OUT                            200
#define AW5808_UPGRADE_CANCEL_TIMEOUT_MS        5000

UINT                    file_size;
PCHAR                   f_aw5808_payload;
FILE*                   f_aw5808;
AW5808_PAYLOAD_TYPE_E   AW5808_DUP_type = AW5808_PAYLOAD_TYPE_IDLE;
AW5808_PAYLOAD_STATE_E  AW5808_DUP_state = AW5808_PAYLOAD_STATE_IDLE;
UINT	                now, last_time;
void AW5808StateShow(DEV_CMD_T * model)
{
    CHAR		state_msg[5][10] = { "idle", "start", "transfer", "finish", "stop" };
    CHAR		module_msg[4][10] = { "idle", "shut-off", "rx", "tx" };

    debug_log("    State:  %s, mode:  %s\r\n", state_msg[model->payload[AW5808_PAYLOAD_STATE_INDEX]], module_msg[model->payload[AW5808_PAYLOAD_TYPE_INDEX]]);
}

UINT AW5808BinaryOpen(PCHAR filename)
{
    UINT block_size;
    f_aw5808 = fopen((CONST PCHAR)filename, "rb");
    if (f_aw5808 == NULL)
        return 0;

    fseek(f_aw5808, 0, SEEK_END);
    file_size = ftell(f_aw5808);
    if (file_size == 0)
        return 0;

    fseek(f_aw5808, 0, SEEK_SET);

    block_size = file_size % AW5808_DUP_PAYLOAD_LENGTH == 0 ? file_size : (file_size / AW5808_DUP_PAYLOAD_LENGTH + 1) * AW5808_DUP_PAYLOAD_LENGTH;

    f_aw5808_payload = (PCHAR)malloc(block_size);
    if (f_aw5808_payload == NULL)
    {
        fclose(f_aw5808);
        return 0;
    }
    fread(f_aw5808_payload, sizeof(CHAR), file_size, f_aw5808);

    LogHeximal((PCHAR)f_aw5808_payload, file_size);
    return file_size;
}

UINT AW5808BinaryDataGet(PCHAR payload, INT data_address)
{
    if (f_aw5808_payload != NULL)
    {
        memcpy(payload, f_aw5808_payload + data_address, AW5808_DUP_PAYLOAD_LENGTH);
        return AW5808_DUP_PAYLOAD_LENGTH;
    }
    else
        return 0;
}

UINT AW5808BinaryClose(void)
{
    if (f_aw5808_payload != NULL)
        free(f_aw5808_payload);

    if (f_aw5808 != NULL)
        fclose(f_aw5808);

    return 0;
}

void AW5808UpdateEventCb(DEV_CMD_S* p_msg_in, DEV_CMD_S* p_msg_out)
{
    UINT            data_addr, block_size, state;
    DEV_HEADER_T    * head_msg_in = &p_msg_in->cmd_t;
    DEV_CMD_T* msg_in = &p_msg_in->sub_cmd_t, * msg_out;

    memcpy(p_msg_out, p_msg_in, sizeof(DEV_CMD_S));
    msg_out = &p_msg_out->sub_cmd_t;
    msg_out->len = 0;
    if (head_msg_in->cmd == HID_CMD_EVENT_RESPONSE)
    {
        debug_log("AW5808 update ack received... \r\n");
    }
    else if (head_msg_in->cmd == HID_CMD_EVENT_REQUEST)
    {
        if (msg_in->cmd == GW_HID_CMD_AW5808_UPGRADE)
        {
            state = msg_in->payload[AW5808_PAYLOAD_STATE_INDEX];
            if (state == AW5808_PAYLOAD_STATE_TRANSFER)
            {
                msg_out->cmd = GW_HID_CMD_AW5808_UPGRADE;
                msg_out->payload[AW5808_PAYLOAD_STATE_INDEX] = AW5808_PAYLOAD_STATE_TRANSFER;
                msg_out->payload[AW5808_PAYLOAD_TYPE_INDEX] = AW5808_DUP_type;
                memcpy(msg_out->payload + AW5808_PAYLOAD_DATA_INDEX, msg_in->payload + AW5808_PAYLOAD_DATA_INDEX, AW5808_DUP_PAYLOAD_SIZE_LENGTH);

                memcpy(&data_addr, msg_in->payload + AW5808_PAYLOAD_DATA_INDEX, AW5808_DUP_PAYLOAD_SIZE_LENGTH);
                block_size = file_size % AW5808_DUP_PAYLOAD_LENGTH == 0 ? file_size : (file_size / AW5808_DUP_PAYLOAD_LENGTH + 1) * AW5808_DUP_PAYLOAD_LENGTH;

                if (data_addr + AW5808_DUP_PAYLOAD_LENGTH <= block_size)
                {
                    msg_out->len = AW5808_PAYLOAD_DATA_INDEX + AW5808_DUP_PAYLOAD_SIZE_LENGTH + AW5808_DUP_PAYLOAD_LENGTH;
                    AW5808BinaryDataGet((PCHAR)msg_out->payload + AW5808_PAYLOAD_DATA_INDEX + AW5808_DUP_PAYLOAD_SIZE_LENGTH, data_addr);
                }
                else
                {
                    msg_out->len = AW5808_PAYLOAD_DATA_INDEX + AW5808_DUP_PAYLOAD_SIZE_LENGTH;
                }
                debug_printf("AW5808 transfer. State: %3d\r", ((data_addr + msg_out->len)*100)/file_size);
                AW5808StateShow(&p_msg_out->sub_cmd_t);

                last_time = GetTickCount();
            }
            else if (state == AW5808_PAYLOAD_STATE_FINISH)
            {
                msg_out->cmd = GW_HID_CMD_AW5808_UPGRADE;
                msg_out->payload[AW5808_PAYLOAD_STATE_INDEX] = AW5808_PAYLOAD_STATE_FINISH;
                msg_out->payload[AW5808_PAYLOAD_TYPE_INDEX] = AW5808_DUP_type;
                msg_out->payload[AW5808_PAYLOAD_DATA_INDEX] = AW5808_DUP_type;
                msg_out->len = AW5808_PAYLOAD_DATA_INDEX + 1;
                AW5808_DUP_type = AW5808_PAYLOAD_TYPE_IDLE;
                debug_printf("\r\n");
                AW5808StateShow(&p_msg_out->sub_cmd_t);

                last_time = GetTickCount();
            }
            else
            {
                debug_printf("AW5808 unknown state %02X\r\n", state);
                LogHeximal((PCHAR)p_msg_in, sizeof(DEV_CMD_S));
            }
        }
        else
        {
            debug_printf("AW5808 unknown command %02X\r\n", msg_in->cmd);
            LogHeximal((PCHAR)p_msg_in, sizeof(DEV_CMD_S));
        }
    }
}

UINT AW5808HelperRun(PCHAR filename, AW5808_PAYLOAD_TYPE_E type)
{
    BOOL	                    is_timeout = FALSE;
    INT                         Res;
    UINT                        data_size;
    DEV_CMD_T                   msg_out, msg_in = {0};
    AW5808_PROCESS_STATUS_E     Ret = AW5808_PROCESS_STATUS_SUCCESS;
    HID_APP_EVT_S               evt;

    data_size = AW5808BinaryOpen(filename);
    if (data_size == 0)
    {
        debug_printf("AW5808 initialized failed, %s closed\r\n", filename);
        Ret = AW5808_PROCESS_STATUS_FILE_FAILED;
        goto AW5808Close;
    }
    else
        debug_printf("AW5808 initialized successful, %s opened, binary-size %d\r\n", filename, data_size);

    evt.cmd = GW_HID_CMD_AW5808_UPGRADE;
    evt.cb = AW5808UpdateEventCb;
    HidAppEventRegister(evt);

    AW5808_DUP_type = type;
    AW5808_DUP_state = AW5808_PAYLOAD_STATE_START;
    msg_in.cmd = GW_HID_CMD_AW5808_UPGRADE;
    msg_in.len = 6;
    msg_in.payload[AW5808_PAYLOAD_STATE_INDEX] = AW5808_DUP_state;
    msg_in.payload[AW5808_PAYLOAD_TYPE_INDEX] = type;
    memcpy(msg_in.payload + AW5808_PAYLOAD_DATA_INDEX, &data_size, sizeof(UINT));
    AW5808StateShow(&msg_in);
    Res = HidCmdSend(&msg_out, &msg_in, USB_TIME_OUT);
    if (Res)
    {
        debug_printf("AW5808 update start failed\r\n");
        Ret = AW5808_PROCESS_STATUS_START_FAILED;
        goto AW5808Close;
    }
    else
        debug_printf("AW5808 update start successful.\r\n");

    last_time = GetTickCount();
    AW5808_DUP_state = AW5808_PAYLOAD_STATE_TRANSFER;
    while (AW5808_DUP_type != AW5808_PAYLOAD_TYPE_IDLE && AW5808_DUP_type != AW5808_PAYLOAD_TYPE_SHUT_OFF)
    {
        Sleep(1);
        UINT now = GetTickCount();
        if (now >= last_time)
            is_timeout = now - last_time > AW5808_UPGRADE_CANCEL_TIMEOUT_MS ? TRUE : FALSE;
        else
            is_timeout = now > AW5808_UPGRADE_CANCEL_TIMEOUT_MS ? TRUE : FALSE;
        if (is_timeout)
        {
            debug_printf("AW5808 update timeout, no query received\r\n");
            Ret = AW5808_PROCESS_STATUS_TIMEOUT;
            goto AW5808Close;
        }
    }

    if (AW5808_DUP_type == AW5808_PAYLOAD_TYPE_SHUT_OFF)
    {
        debug_printf("AW5808 update is forced to shut off\r\n");
        Ret = AW5808_PROCESS_STATUS_SHUT_OFF;
        goto AW5808Close;
    }
    else
        debug_printf("AW5808 update is finished\r\n");

    Sleep(5);
    AW5808_DUP_state = AW5808_PAYLOAD_STATE_STOP;
    msg_in.cmd = GW_HID_CMD_AW5808_UPGRADE;
    msg_in.len = 2;
    msg_in.payload[AW5808_PAYLOAD_STATE_INDEX] = AW5808_DUP_state;
    msg_in.payload[AW5808_PAYLOAD_TYPE_INDEX] = type;
    AW5808StateShow(&msg_in);
    Res = HidCmdSend(&msg_out, &msg_in, USB_TIME_OUT);
    if (Res)
    {
        debug_printf("AW5808 update stopped failed\r\n");
        Ret = AW5808_PROCESS_STATUS_STOP_FAILED;
        goto AW5808Close;
    }
    else
        debug_printf("AW5808 update stopped successful.\r\n");

    AW5808_DUP_state = AW5808_PAYLOAD_STATE_IDLE;

AW5808Close:
    evt.cmd = GW_HID_CMD_AW5808_UPGRADE;
    HidAppEventUnregister(evt);

    AW5808BinaryClose();
    Sleep(5);

    return Ret;
}

void AW5808HelperIdle(void)
{
    debug_printf("AW5808 was forced to shut off\r\n");

    if(AW5808_DUP_type != AW5808_PAYLOAD_TYPE_IDLE)
        AW5808_DUP_type = AW5808_PAYLOAD_TYPE_SHUT_OFF;
}
