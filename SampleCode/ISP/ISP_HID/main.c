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

void set_system_tick(uint32_t ms) {
				SysTick->LOAD = ms * 1000 * CyclesPerUs;
				SysTick->VAL  = (0x00);
				SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;	
}

void USBD_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/

#define use_BOOT_TIME
int32_t isp_connect = 0;

void GW_GPIO_init(void) {
	
	CLK_EnableModuleClock(GPA_MODULE);
	CLK_EnableModuleClock(GPB_MODULE);
	//CLK_EnableModuleClock(GPC_MODULE);
	CLK_EnableModuleClock(GPD_MODULE);	
	//CLK_EnableModuleClock(GPF_MODULE);

#ifdef EN_UART0
#else
	//GPIO_SetMode(PA, BIT6, GPIO_MODE_INPUT);
	//GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
#endif	
	
	//GPIO_SetMode(PF, BIT15, GPIO_MODE_INPUT);		//SYS_FW_UPDATE, 			3607D, 				Request FW update from PC
	//GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT);		//DOA_LED_SEL,				LED select
	//GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT);		//MIC_LED_SEL,				LED select
	//GPIO_SetMode(PD, BIT15, GPIO_MODE_OUTPUT);	//TOUCH_LED_SEL,			LED select
	//GPIO_SetMode(PC, BIT6, GPIO_MODE_INPUT);		//DSP_INT1,						3607D,				TBD
	//GPIO_SetMode(PC, BIT7, GPIO_MODE_OUTPUT);		//TYPEC_SPDIF2_EN,		DFP2(SPDIF),	Control PCA9306
	//GPIO_SetMode(PF, BIT2, GPIO_MODE_OUTPUT);		//5GTRX_SW,						switch I2S,		Lo:5G(Tx) / Hi:5G(Rx)
	//GPIO_SetMode(PF, BIT3, GPIO_MODE_OUTPUT);		//WWL_SW,							switch I2S, 	Lo:5G / Hi:CS8422
	//GPIO_SetMode(PA, BIT10, GPIO_MODE_OUTPUT);	//TYPEC1_SW,					switch C,			UART/SPDIF switch
	//GPIO_SetMode(PA, BIT11, GPIO_MODE_OUTPUT);	//DSP_ONOFF,					3607D,				Lo:reset / Hi:normal
	//GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);		//GL3525_C2_VBUS_DET,	UFP(GL9510),	Hi:TV USB connect
	//GPIO_SetMode(PB, BIT1, GPIO_MODE_INPUT);		//GL3525_C1_VBUS_DET,	UFP(GL9510),	Hi:PC USB connect
	//GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);		//USBC2_DFP_DET,			DFP(GL9510),	Hi:DFP USB connect
	//GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT);		//USBC1_DFP_DET,			DFP(GL9510),	Hi:DFP USB connect
	//GPIO_SetMode(PB, BIT6, GPIO_MODE_OUTPUT);		//TYPEC2_SW,					switch C,			UART/SPDIF switch
	//GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);		//USBC2_DFP_ID,				DFP(GL9510),	Hi:Cable correct
	//GPIO_SetMode(PB, BIT8, GPIO_MODE_INPUT);		//USBC1_DFP_ID,				DFP(GL9510),	Hi:Cable correct
	//GPIO_SetMode(PB, BIT9, GPIO_MODE_INPUT);		//ADP_ID,							power ID,			TBD
	//GPIO_SetMode(PB, BIT10, GPIO_MODE_INPUT);		//Wireless_SW#,				Pair button,	Lo:press
	//GPIO_SetMode(PB, BIT11, GPIO_MODE_INPUT);		//PWR_SW#,						Power button,	Lo:press
	GPIO_SetMode(PB, BIT12, GPIO_MODE_OUTPUT);	//USBC_UDP_SEL1,			HUB UFP switch to TV or PC
	GPIO_SetMode(PB, BIT13, GPIO_MODE_OUTPUT);	//USBC_UDP_SEL0,			HUB UFP switch to TV or PC
	GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);	//USB2SW,							MCU USB switch to TV or PC
	//GPIO_SetMode(PB, BIT15, GPIO_MODE_INPUT);		//USBC2_UFP_ID,				TV cable correct, Hi:correct
	//GPIO_SetMode(PC, BIT14, GPIO_MODE_INPUT);		//USBC1_UFP_ID,				PC cable correct, Hi:correct
	GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT);		//MCU_PWREN,					Dock power,		Hi:on
	//GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);		//AMP_EN,							Audio amp,		Hi:on
	//GPIO_SetMode(PC, BIT2, GPIO_MODE_INPUT);		//5GTX_INT1,					5G(Tx) CCH int, Lo:interrupt
	//GPIO_SetMode(PC, BIT3, GPIO_MODE_INPUT);		//5GRX_INT1,					5G(Tx) CCH int, Lo:interrupt
	//GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);		//TYPEC_SPDIF1_EN,		DFP1(SPDIF),	Control PCA9306
	//GPIO_SetMode(PC, BIT5, GPIO_MODE_INPUT);		//MCU_I2C0_INT,				CY8CMBR3108,	status change
	//GPIO_SetMode(PF, BIT4, GPIO_MODE_OUTPUT);		//TYPEC_UART1_EN,			DFP1(UART),		Control PCA9306
	//GPIO_SetMode(PF, BIT5, GPIO_MODE_OUTPUT);		//TYPEC_UART2_EN,			DFP2(UART),		Control PCA9306
	
	GPIO_SetMode(PA, BIT10, GPIO_MODE_INPUT);		//GPO_PA10_TYPEC1_SW		0: Wireless; 1: Wired
	//GPO_PD03_AMP_EN = 1;
	//GPO_PA10_TYPEC1_SW = 0;
	//GPO_PB06_TYPEC2_SW = 0;
	//GPO_PF04_TYPEC_UART1_EN = 0;
	//GPO_PF05_TYPEC_UART2_EN = 0;
	//GPO_PC04_TYPEC_SPDIF1_EN = 0;
	//GPO_PC07_TYPEC_SPDIF2_EN = 0;
	//GPO_PA11_DSP_ONOFF = 0;
	
	//to PC side for DFU
	
	GPO_PB14_USB2SW = 1;
	GPO_PB13_USBC_UDP_SEL0 = 0;
	GPO_PB12_USBC_UDP_SEL1 = 1;
	GPO_PD02_MCU_PWREN = 1;
}


int32_t main(void)
{
		int32_t i;
	
    /* Unlock write-protected registers to operate SYS_Init and FMC ISP function */
    SYS_UnlockReg();

    /* Init system and multi-function I/O */
    SYS_Init();
		GW_GPIO_init();

    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

#if !defined(use_BOOT_TIME)
    while (DetectPin == 0)
#endif
    {
				if(GPO_PA10_TYPEC1_SW) {
						gsInfo.gu8DevDesc[10]=WIRE_USBD_PID&0x00FF;
						gsInfo.gu8DevDesc[11]=(WIRE_USBD_PID&0xFF00)>>8;
				}
				else {
						gsInfo.gu8DevDesc[10]=WIRELESS_USBD_PID&0x00FF;
						gsInfo.gu8DevDesc[11]=(WIRELESS_USBD_PID&0xFF00)>>8;
				}
        /* Open USB controller */
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);

        /*Init Endpoint configuration for HID */
        HID_Init();

        /* Start USB device */
        USBD_Start();

        /* Using polling mode and Removed Interrupt Table to reduce code size for M251 */
#if defined(use_BOOT_TIME)

				for(i=0; i<50; i++) {		// wait 2 sec.  Test in ASUS EXPertBook , can detect ISP for 1.5 sec
					
						set_system_tick(100);
						/* Waiting for down-count to zero */
					
						while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
						{
										// polling USBD interrupt flag
										USBD_IRQHandler();

										if (bUsbDataReady == TRUE)
										{
												ParseCmd((uint8_t *)usb_rcvbuf, 64);
												EP2_Handler();
												bUsbDataReady = FALSE;
										}			
						};
						
						if(isp_connect == 1)
							i = 0;
			}
#else			
        while (DetectPin == 0)
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

        goto _APROM;
#endif				
    }

    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

_APROM:
    outpw(&SYS->RSTSTS, SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk); //clear bit
    outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
    NVIC_SystemReset();

    /* Trap the CPU */
    while (1);
}
/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
