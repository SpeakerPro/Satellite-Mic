#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "stdafx.h"
#include "GW_usbtree.h"
#include "GW_debug.h"

#define USBTREE_FILE_NAME           "usbtree.txt"
#define USBTREE_SUB_COMMAND         " /q /f /saveall:"
#define USBTREE_EXE_COMMAND         "tools\\usbview.exe"

typedef struct _USBTREE_MSG_S
{
    PCHAR   pmsg;
    INT     length;
} USBTREE_MSG_S;

typedef struct _USBTREE_S
{
    INT             port_id;
    INT             root_index;
    USBTREE_ID_S    id;
} USBTREE_S;

USBTREE_S   * port_info_list;
INT         total_hubs, total_ports;

INT UsbtreeSubtreeGet(INT index)
{
    INT count = 0;
    while (1)
    {
        if (port_info_list[index].root_index == 0)
            break;
        index = port_info_list[index].root_index - 1;
        count++;
    }
    return count;
}

INT UsbtreeFileGenerate(void)
{
    CHAR    file_path[MAX_PATH] = { 0 }, cmd[MAX_PATH * 2] = { 0 };
    INT     key_space, res;
    GetCurrentDirectoryA(sizeof(file_path), file_path);

    std::string str(file_path);
    key_space = str.find(' ');
    if (key_space != -1)
        PathCombineA(file_path, ".\\", USBTREE_EXE_COMMAND);
    else
        PathCombineA(file_path, file_path, USBTREE_EXE_COMMAND);
    sprintf(cmd, "%s%s%s", file_path, USBTREE_SUB_COMMAND, USBTREE_FILE_NAME);

    res = system(cmd);
    return res;
}

void UsbtreeListShow(void)
{
    debug_printf("USBTREE info: %d hubs, %d ports \r\n", total_hubs, total_ports);
    if (port_info_list != NULL)
    {
        for (INT index = 0; index < total_ports; index++)
        {
            INT sub_count = UsbtreeSubtreeGet(index);
            for (INT i = 0; i < sub_count; i++)
                debug_printf("      ");
            debug_printf("- [Port %d]", port_info_list[index].port_id);
            if (port_info_list[index].id.iVendor && port_info_list[index].id.iProduct)
            {
                debug_printf(": Vid/Pid: %04X/%04X. ", port_info_list[index].id.iVendor, port_info_list[index].id.iProduct);
                if (strlen(port_info_list[index].id.product_name))
                    debug_printf("Product name: %s", port_info_list[index].id.product_name);
                else
                    debug_printf("Port Name: %s", port_info_list[index].id.port_name);
            }

            debug_printf("\r\n");
        }
    }
}

BOOL UsbtreeObjectGet(USBTREE_MSG_S* msg, PCHAR message, PCHAR keyword, INT range)
{
    std::string str(message);

    INT     start, stop, index = 0;

    start = str.find(keyword);
    stop = str.find('\n', start + 1);

    if (start == std::string::npos || stop == std::string::npos || start >= range)
        return false;

    while (1)
    {
        if (start++ >= stop)
        {
            msg->pmsg = NULL;
            msg->length = 0;
            return false;
        }

        if (message[start] == ':' && index == 0)
        {
            index++;
            start++;
        }

        if (message[start] != ' ' && index == 1)
        {
            msg->pmsg = message + start;
            msg->length = stop - start;
            return true;
        }
    }
}

INT UsbtreeReload(void)
{
#define     SAMPLE_ARRAY    10

    FILE    * f;
    PCHAR   pdata, psection;
    INT     file_size, section_size, total_sub_hubs = 0;
    INT     root_hub_arr[SAMPLE_ARRAY] = { 0 }, root_hub_port_arr[SAMPLE_ARRAY] = { 0 };
    INT     index_l = 0, index_h, res;

    USBTREE_MSG_S   msg;

    total_hubs = 0;
    total_ports = 0;

    res = UsbtreeFileGenerate();
    if (res != 0)
    {
        debug_log("Usbtree reload failed, err %X\r\n", res);
        return USBTREE_EXECUTE_FAILED;
    }

    if (port_info_list != NULL)
        free(port_info_list);

    f = fopen((CONST PCHAR)USBTREE_FILE_NAME, "rb");
    if (f != NULL)
    {
        fseek(f, 0, SEEK_END);
        file_size = ftell(f);
        fseek(f, 0, SEEK_SET);

        pdata = (PCHAR)calloc(file_size, sizeof(CHAR));
        if (pdata != NULL)
        {
            bool    root_hub = true;
            fread(pdata, sizeof(CHAR), file_size, f);

            std::string str(pdata);
            index_l = str.find("RootHub", index_l);
            while (1)
            {
                index_h = str.find("[Port", index_l + 1);
                if (index_h == std::string::npos)
                    index_h = file_size;

                section_size = index_h - index_l + 1;
                psection = (PCHAR)calloc(section_size, sizeof(CHAR));
                if (psection != NULL)
                {
                    INT     index = 0;
                    memcpy(psection, pdata + index_l, section_size);
                    psection[section_size - 1] = 0;

                    if (!root_hub)
                    {
                        USBTREE_S* port_info_buffer = (USBTREE_S*)calloc(total_ports, sizeof(USBTREE_S));
                        if (port_info_buffer != NULL)
                        {
                            if (port_info_list != NULL)
                            {
                                memcpy(port_info_buffer, port_info_list, sizeof(USBTREE_S) * total_ports);
                                free(port_info_list);
                            }
                        }
                        else
                            return USBTREE_MEMORY_ALLOCATE_FAILED;

                        port_info_list = (USBTREE_S*)calloc(total_ports + 1, sizeof(USBTREE_S));
                        if (port_info_list != NULL)
                        {
                            if (port_info_buffer != NULL)
                            {
                                memcpy(port_info_list, port_info_buffer, sizeof(USBTREE_S) * total_ports);
                                free(port_info_buffer);
                            }

                            (void)sscanf(psection, "[Port%d]", &port_info_list[total_ports].port_id);
                            port_info_list[total_ports].root_index = root_hub_arr[0];

                            if (UsbtreeObjectGet(&msg, psection, (PCHAR)"idVendor", section_size))
                                (void)sscanf(msg.pmsg, "0x%X", &port_info_list[total_ports].id.iVendor);

                            if (UsbtreeObjectGet(&msg, psection, (PCHAR)"idProduct", section_size))
                                (void)sscanf(msg.pmsg, "0x%X", &port_info_list[total_ports].id.iProduct);

                            if (UsbtreeObjectGet(&msg, psection, (PCHAR)"[Port", section_size))
                            {
                                memcpy(port_info_list[total_ports].id.port_name, msg.pmsg, msg.length);
                                port_info_list[total_ports].id.port_name[msg.length] = 0;
                            }

                            if (UsbtreeObjectGet(&msg, psection, (PCHAR)"product name", section_size))
                            {
                                memcpy(port_info_list[total_ports].id.product_name, msg.pmsg, msg.length);
                                port_info_list[total_ports].id.product_name[msg.length] = 0;
                            }
                        }
                        else
                            return USBTREE_MEMORY_ALLOCATE_FAILED;
                    }

                    if (UsbtreeObjectGet(&msg, psection, (PCHAR)"Number of Ports", section_size))
                    {
                        INT number_ports;
                        (void)sscanf(msg.pmsg, "%d", &number_ports);

                        memcpy(root_hub_arr + 1, root_hub_arr, sizeof(root_hub_arr[0]) * (SAMPLE_ARRAY - 1));
                        root_hub_arr[0] = total_ports;

                        memcpy(root_hub_port_arr + 1, root_hub_port_arr, sizeof(root_hub_arr[0]) * (SAMPLE_ARRAY - 1));
                        root_hub_port_arr[0] = number_ports;

                        if (!root_hub)
                        {
                            total_hubs++;
                            total_sub_hubs++;
                            root_hub_arr[0]++;
                        }
                    }
                    else
                    {
                        if (port_info_list != NULL)
                        {
                            if (root_hub_port_arr[0] == port_info_list[total_ports].port_id)
                            {
                                INT sub_tree_index = total_sub_hubs;
                                for (INT x = 0; x < sub_tree_index; x++)
                                {
                                    memcpy(root_hub_arr, root_hub_arr + 1, sizeof(root_hub_arr[0]) * (SAMPLE_ARRAY - 1));
                                    root_hub_arr[SAMPLE_ARRAY - 1] = 0;

                                    memcpy(root_hub_port_arr, root_hub_port_arr + 1, sizeof(root_hub_arr[0]) * (SAMPLE_ARRAY - 1));
                                    root_hub_port_arr[SAMPLE_ARRAY - 1] = 0;
                                    total_sub_hubs--;
                                }
                            }
                        }
                    }

                    if (root_hub)
                        root_hub = false;
                    else
                        total_ports++;

                    free(psection);
                }
                else
                    return USBTREE_MEMORY_ALLOCATE_FAILED;

                if (index_h == file_size)
                    break;
                else
                    index_l = index_h;
            }
        }
        else
            return USBTREE_MEMORY_ALLOCATE_FAILED;

        free(pdata);
        fclose(f);
    }
    else
    {
        debug_log("Usbtree log not found!\r\n");
        return USBTREE_LOG_OPEN_FAILED;
    }

    debug_log("USBTREE info: %d hubs, %d ports \r\n", total_hubs, total_ports);
    if (port_info_list != NULL)
    {
        for (INT index = 0; index < total_ports; index++)
        {
            INT sub_count = UsbtreeSubtreeGet(index);
            for (INT i = 0; i < sub_count; i++)
                debug_log("      ");
            debug_log("- [Port %d]", port_info_list[index].port_id);
            if (port_info_list[index].id.iVendor && port_info_list[index].id.iProduct)
            {
                debug_log(": Vid/Pid: %04X/%04X. ", port_info_list[index].id.iVendor, port_info_list[index].id.iProduct);
                if (strlen(port_info_list[index].id.product_name))
                    debug_log("Product name: %s", port_info_list[index].id.product_name);
                else
                    debug_log("Port Name: %s", port_info_list[index].id.port_name);
            }

            debug_log("\r\n");
        }
    }

    return USBTREE_SUCCESSFUL;
}

BOOL UsbtreeSingleIndexGet(USBTREE_ID_S source, INT* index_buffer)
{
    INT     index;
    for (index = 0; index < total_ports; index++)
    {
        if (port_info_list[index].id.iVendor == source.iVendor && port_info_list[index].id.iProduct == source.iProduct)
        {
            *index_buffer = index;
            break;
        }
    }
    return index >= total_ports ? FALSE : TRUE;
}

BOOL UsbtreeMultiIndexGet(USBTREE_ID_S source, INT* index_buffer, INT* index_count)
{
    INT     index, count = 0;
    for (index = 0; index < total_ports; index++)
    {
        if (port_info_list[index].id.iVendor == source.iVendor && port_info_list[index].id.iProduct == source.iProduct)
            index_buffer[count++] = index;
    }
    *index_count = count;
    return *index_count == 0 ? FALSE : TRUE;
}

BOOL UsbtreeSourceCheck(USBTREE_ID_S source, USBTREE_ID_S target)
{
    INT     index, index_buffer[10], index_count;
    BOOL    ret = TRUE;

    if (!UsbtreeSingleIndexGet(source, &index))
        return FALSE;

    while (1)
    {
        if (port_info_list[index].id.iVendor == target.iVendor && port_info_list[index].id.iProduct == target.iProduct)
            ret = FALSE;
        if (port_info_list[index].root_index == 0)
            break;
        index = port_info_list[index].root_index - 1;
    }

    return ret;
}
