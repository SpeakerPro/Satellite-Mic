/***************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to update chip flash data through USB HID interface
             between chip USB device and PC.
             Nuvoton NuMicro ISP Programming Tool is also required in this
             sample code to connect with chip USB device and assign update file
             of Flash.
 * @version  0x32
 * @date     14, June, 2017
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "hid_transfer.h"
#include "GW_main.h"

#define PLL_CLOCK               48000000

#if 0
/* This is a dummy implementation to replace the same function in clk.c for size limitation. */
uint32_t CLK_GetPLLClockFreq(void)
{
    return PLL_CLOCK;
}
#endif

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for external XTAL clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */

    SystemCoreClock = __HIRC;             // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()
    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk | CLK_AHBCLK_GPACKEN_Msk | CLK_AHBCLK_EXSTCKEN_Msk;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA.12 ~ PA.14 to input mode */
    PA->MODE &= ~(GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk | GPIO_MODE_MODE14_Msk);
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
}

void USBD_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/

//#define use_BOOT_TIME
//int32_t isp_connect = 0;

void GW_delay1ms(uint32_t ms) {
	SysTick->LOAD = ms * 1000 * CyclesPerUs;
	SysTick->VAL  = (0x00);
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
}

void GW_GPIO_init(void) {
	
	CLK_EnableModuleClock(GPA_MODULE);
	CLK_EnableModuleClock(GPB_MODULE);
	CLK_EnableModuleClock(GPD_MODULE);	

	GPIO_SetMode(PA, BIT10, GPIO_MODE_INPUT);
	if(GPO_PA11_DSP_ONOFF == LOW) {
		GPO_PD02_MCU_PWREN = LOW;
		GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT);		//MCU_PWREN,					Dock power,		Hi:on		
		
		GW_delay1ms(1000);
		GPO_PD02_MCU_PWREN = HIGH;
		
		GW_delay1ms(10);
		GPO_PA11_DSP_ONOFF = HIGH;
		GPIO_SetMode(PA, BIT11, GPIO_MODE_OUTPUT);	//DSP_PWREN,					DSP power,		Hi:on		
		/*
		GW_delay1ms(50);
		
		GPIO_SetMode(PB, BIT12, GPIO_MODE_OUTPUT);	//USBC_UDP_SEL1,			HUB UFP switch to TV or PC
		GPIO_SetMode(PB, BIT13, GPIO_MODE_OUTPUT);	//USBC_UDP_SEL0,			HUB UFP switch to TV or PC	
		GPO_PB13_USBC_UDP_SEL0 = 0;
		GPO_PB12_USBC_UDP_SEL1 = 1;		
		*/
	}

}

uint32_t APROM_checksum = 0x00, APROM_cal_checksum = 0x00, config0 = 0x00;
void GW_read_APROM_checksum(uint32_t u32FlashAddr, uint32_t u32ImageSize) {
	
    uint32_t   u32i;

    /* Enable FMC ISP function */
    FMC_Open();
	
		FMC_ReadConfig(&config0, 1);

		FMC_EnableLDUpdate();
	
    for (u32i = 0; u32i < (u32ImageSize-4); u32i += 4)
			APROM_cal_checksum += FMC_Read(u32FlashAddr + u32i);
	
		APROM_checksum = FMC_Read(u32FlashAddr + u32ImageSize - 4);
		
		FMC_DisableLDUpdate();

    /* Disable FMC ISP function */
    FMC_Close();
	
}

int32_t main(void)
{
		int32_t goto_aprom = YES;
	
    /* Unlock write-protected registers to operate SYS_Init and FMC ISP function */
    SYS_UnlockReg();

    /* Init system and multi-function I/O */
    SYS_Init();
		
		GW_read_APROM_checksum(FMC_APROM_BASE, FMC_APROM_SIZE);	

    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    g_apromSize = FMC_APROM_SIZE;
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
	
		if(APROM_checksum != APROM_cal_checksum)
			goto_aprom = NO;
		
		if((config0 & 0xFF) == 0x7F)
			goto_aprom = NO;

    if(goto_aprom == NO)
    {
				GW_GPIO_init();
			
				if(GPO_PA10_TYPEC1_SW) {
						gsInfo.gu8DevDesc[10] = WIRE_USBD_PID&0x00FF;
						gsInfo.gu8DevDesc[11] = (WIRE_USBD_PID&0xFF00) >> 8;
				}
				else {
						gsInfo.gu8DevDesc[10] = WIRELESS_USBD_PID&0x00FF;
						gsInfo.gu8DevDesc[11] = (WIRELESS_USBD_PID&0xFF00) >> 8;
				}

        /* Open USB controller */
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);

        /*Init Endpoint configuration for HID */
        HID_Init();

        /* Start USB device */
        USBD_Start();

        /* Using polling mode and Removed Interrupt Table to reduce code size for M251 */
		
        while (1)
        {
            // polling USBD interrupt flag
            USBD_IRQHandler();

            if (bUsbDataReady == TRUE)
            {
                ParseCmd((uint8_t *)usb_rcvbuf, 64);
                EP2_Handler();
                bUsbDataReady = FALSE;
            }
        }
    }

_APROM:
    outpw(&SYS->RSTSTS, SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk); //clear bit
    outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
    NVIC_SystemReset();

    /* Trap the CPU */
    while (1);
}
/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
