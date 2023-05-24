#include "stdafx.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "GW_cmd_parse.h"

#define CLI_ABBR_AW5808_RX              "awr"
#define CLI_ABBR_AW5808_TX              "awt"

#define FILE_NAME(x) strrchr(x, '\\')? strrchr(x, '\\') + 1:x

bool cmd_parse(int argc, char* argv[], cmd_parse_s* parse_info)
{
    bool    parse_status = false;
    int     index, length;

    for (index = 1; index < argc; index++)
    {
        parse_status = false;
        if (argv[index][0] == '-')
        {
            length = strlen(argv[index]) - 1;
            switch (argv[index][1])
            {
            case 'a':
                if (length != 3)
                    goto ParsemdList;
                printf("AW5808 %s, %s, %s, %d, ", &argv[index][1], CLI_ABBR_AW5808_RX, CLI_ABBR_AW5808_TX, strlen(CLI_ABBR_AW5808_RX));
                if (memcmp(&argv[index][1], CLI_ABBR_AW5808_RX, strlen(CLI_ABBR_AW5808_RX)) == 0)
                {
                    printf("RX ");
                    parse_info->awr_file = argv[++index];
                }
                else if (memcmp(&argv[index][1], CLI_ABBR_AW5808_TX, strlen(CLI_ABBR_AW5808_TX)) == 0)
                {
                    printf("TX ");
                    parse_info->awt_file = argv[++index];
                }
                else
                    goto ParsemdList;

                break;
            case 'u':
                if (length != 1)
                    goto ParsemdList;
                parse_info->isp_file = argv[++index];
                break;
            case 'f':
                if (length != 1)
                    goto ParsemdList;
                parse_info->data_flash_file = argv[++index];
                break;
            case 'v':
                if (length != 1)
                    goto ParsemdList;
                parse_info->version = true;
                break;
            case 't':
                if (length != 1)
                    goto ParsemdList;
                parse_info->test_cmd = argv[++index];
                break;
            default:
                goto ParsemdList;
                break;
            }
        }
        else
            goto ParsemdList;
        parse_status = true;
    }

ParsemdList:
    if (argc == 1 || !parse_status)
    {
        printf("Usage: %s\r\n", FILE_NAME(argv[0]));
        printf("       [-u] [file] : Update APROM with new file\r\n");
        printf("       [-f] [file] : Update Data Flash with file\r\n");
        printf("       [-v] : Get firmware version. Not supported now\r\n");
        printf("       [-t] [Test command]: Test command\r\n");
    }

    return parse_status;
}
