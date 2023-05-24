/****************************************************************************//**
 * @file     APROM_main.c
 * @version  V1.00
 * @brief    Show how to reboot to LDROM functions from APROM.
 *           This sample code set VECMAP to LDROM and reset to re-boot to LDROM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "GW_main.h"

typedef void (FN_FUNC_PTR)(void);

extern uint32_t  loaderImage1Base, loaderImage1Limit;

static int  LoadImage(uint32_t u32ImageBase, uint32_t u32ImageLimit, uint32_t u32FlashAddr, uint32_t u32MaxSize)
{
    uint32_t   u32i, u32j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = u32MaxSize;
	
		if((u32ImageLimit - u32ImageBase) > u32MaxSize)
			return -1;

    //printf("Program image to flash address 0x%x...", u32FlashAddr);
    pu32Loader = (uint32_t *)u32ImageBase;

    for (u32i = 0; u32i < u32ImageSize; u32i += FMC_FLASH_PAGE_SIZE)
    {
        FMC_Erase(u32FlashAddr + u32i);

        for (u32j = 0; u32j < FMC_FLASH_PAGE_SIZE; u32j += 4)
        {
            FMC_Write(u32FlashAddr + u32i + u32j, pu32Loader[(u32i + u32j) / 4]);
        }
    }

    //printf("OK.\n");

    //printf("Verify ...");

    /* Verify loader */
    for (u32i = 0; u32i < u32ImageSize; u32i += FMC_FLASH_PAGE_SIZE)
    {
        for (u32j = 0; u32j < FMC_FLASH_PAGE_SIZE; u32j += 4)
        {
            u32Data = FMC_Read(u32FlashAddr + u32i + u32j);

            if (u32Data != pu32Loader[(u32i + u32j) / 4])
            {
                //printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", u32FlashAddr + u32i + u32j, u32Data, pu32Loader[(u32i + u32j) / 4]);
                return -2;
            }

            if (u32i + u32j >= u32ImageSize)
                break;
        }
    }

    //printf("OK.\n");
    return 0;
}

void GW_write_LDROM(void)
{
		int r_code ;

    /* Unlock protected registers to operate SYS_Init and FMC ISP function */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

		FMC_EnableLDUpdate();

		r_code = LoadImage((uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit, FMC_LDROM_BASE, FMC_LDROM_SIZE);
		printf("\x1b[%d;%dHWrite LDROM %s(%d)!\n", SELECT_Y+1, SELECT_X, r_code == 0 ? "success" : "fail", r_code);

		FMC_DisableLDUpdate();


    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
