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

uint32_t LDROM_check_count = 0, LDROM_check_err = 0;

void check_LDROM_need_update(uint32_t u32ImageBase, uint32_t u32ImageLimit) {
	
		uint32_t   u32i, u32Data, u32ImageSize, *pu32Loader;
	
		if(LDROM_check_count >= 10) {
			if(LDROM_check_err >= 10) {
				LDROM_check_err = 0;
				GW_write_LDROM();
			}
			return;
		}

	
    /* Unlock protected registers to operate SYS_Init and FMC ISP function */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

		FMC_EnableLDUpdate();	
	
		pu32Loader = (uint32_t *)u32ImageBase;
		LDROM_check_count ++;
	
		
	
    for (u32i = 0; u32i < (u32ImageLimit - u32ImageBase); u32i += 4)
    {
				u32Data = FMC_Read(FMC_LDROM_BASE + u32i);

				if (u32Data != pu32Loader[u32i/4]) {
					LDROM_check_err ++ ;
					break;
				}
    }	
		
		FMC_DisableLDUpdate();

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();		

}

static int  LoadImage(uint32_t u32ImageBase, uint32_t u32ImageLimit, uint32_t u32FlashAddr, uint32_t u32MaxSize)
{
    uint32_t   u32i, u32j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = u32MaxSize;
	
		if((u32ImageLimit - u32ImageBase) > u32MaxSize)
			return -1;

    pu32Loader = (uint32_t *)u32ImageBase;

    for (u32i = 0; u32i < u32ImageSize; u32i += FMC_FLASH_PAGE_SIZE)
    {
        FMC_Erase(u32FlashAddr + u32i);

        for (u32j = 0; u32j < FMC_FLASH_PAGE_SIZE; u32j += 4)
        {
            FMC_Write(u32FlashAddr + u32i + u32j, pu32Loader[(u32i + u32j) / 4]);
        }
    }

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
		printf("\x1b[%d;%dHWrite LDROM %s(%d)!\n", GPIO_START_Y+2, GPIO_START_X, r_code == 0 ? "success" : "fail", r_code);

		FMC_DisableLDUpdate();


    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

}

uint32_t APROM_checksum = 0x00;
uint32_t GW_read_ROM_checksum(uint32_t u32FlashAddr, uint32_t u32ImageSize) {
	
    uint32_t   u32i, checksum;
	
		checksum = 0;

    /* Unlock protected registers to operate SYS_Init and FMC ISP function */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

		FMC_EnableLDUpdate();

    for (u32i = 0; u32i < u32ImageSize; u32i += 4)
			checksum += FMC_Read(u32FlashAddr + u32i);
		
		FMC_DisableLDUpdate();

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();
		
		return checksum;
	
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
