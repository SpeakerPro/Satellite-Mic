/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Demonstrate how to implement a composite device.(VCOM and HID Transfer)
 *           It supports one virtual COM port and transfers data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with a USB device.
 *
 *           Windows tool: User need to input the specific PID for the USB HID device connected to PC.
 *                         PID format with hexadecimal.
 *
 *           -> PID is 0xDC00 in this sample.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "GW_hid_Transfer.h"

//#define IGNORE_PRINTF
#ifdef IGNORE_PRINTF
	#define printf(fmt, ...) (0)
#endif

#define CRYSTAL_LESS    1 /* CRYSTAL_LESS must be 1 if USB clock source is HIRC */
#define TRIM_INIT           (SYS_BASE+0x118)

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING g_sLineCoding = {3000000, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t g_u16CtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           256 /* RX buffer size */
#define TXBUFSIZE           256 /* RX buffer size */

#define TX_FIFO_SIZE        16  /* TX Hardware FIFO size */


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t hw_current_version=0; //0:wireless 1:wire
uint16_t usb_current_vid=WIRELESS_USBD_PID;

/* UART0 */
volatile uint8_t  g_au8ComRbuf[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes = 0;
volatile uint16_t g_u16ComRhead = 0;
volatile uint16_t g_u16ComRtail = 0;

volatile uint8_t g_au8ComTbuf[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes = 0;
volatile uint16_t g_u16ComThead = 0;
volatile uint16_t g_u16ComTtail = 0;

uint8_t g_au8RxBuf[64] = {0};
volatile uint8_t *g_pu8RxBuf = 0;
volatile uint32_t g_u32RxSize = 0;
volatile uint32_t g_u32TxSize = 0;

volatile int8_t g_i8BulkOutReady = 0;

const uint8_t gu8ProductStringDesc_Wired[] =
{
    38,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'I', 0, 'M', 0, ' ', 0, 'W', 0, 'i', 0, 'r', 0, 'e', 0, 'd', 0, ' ', 0, 'M', 0, 'i', 0, 'c', 0, ' ', 0, 'C', 0, 'M', 0, '1', 0, '0', 0, '0', 0
};

void LDROM_sys_init(void) {
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

/*--------------------------------------------------------------------------*/
void GW_timer_delay_ms(uint32_t ms)
{
		timer_ms_delay_count = 0;

    CLK_EnableModuleClock(TMR0_MODULE);
		CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    if (TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000) != 1)
    {
    }

    TIMER_EnableInt(TIMER0);
		NVIC_EnableIRQ(TMR0_IRQn);
		TIMER_Start(TIMER0);
		while(timer_ms_delay_count < ms);
		TIMER_Close(TIMER0);
		TIMER_Stop(TIMER0);
		NVIC_DisableIRQ(TMR0_IRQn);
		TIMER_DisableInt(TIMER0);
		CLK_DisableModuleClock(TMR0_MODULE);
}

void GW_Power_on_preconfig(void)
{
		CLK_EnableModuleClock(GPD_MODULE);
		CLK_EnableModuleClock(GPB_MODULE);
	
		GPO_PD02_MCU_PWREN = HIGH;
		GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
		LDROM_sys_init();
		GW_Power_on_preconfig();

    /* Enable module clock */
#ifdef EN_UART0
    CLK_EnableModuleClock(UART0_MODULE);
#endif
		CLK_EnableModuleClock(UART1_MODULE);
		CLK_EnableModuleClock(UART2_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select module clock source */
#ifdef EN_UART0
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
#endif
		CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
		CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    //Uart0DefaultMPF();

		SYS_LED_Init();
		SYS_Touch_Init();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
		uint8_t new_end;

    u32IntStatus = UART0->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while ((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART0->DAT;
						if(!ats3607_update_running())
						{
								new_end = ats3607d_in_end;
								new_end ++ ;
								if(new_end >= MAX_DSP_BUF)
									new_end = 0;
						
								if(new_end == ats3607d_in_start) {
									//FIFO full
								} else {
									ats3607d_in_buf[ats3607d_in_end] = bInChar;
									ats3607d_in_end = new_end;
								}
						}
						
#if 0
            /* Check if buffer full */
            if (g_u16ComRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_au8ComRbuf[g_u16ComRtail++] = bInChar;

                if (g_u16ComRtail >= RXBUFSIZE)
                    g_u16ComRtail = 0;

                g_u16ComRbytes++;
            }
            else
            {
                /* FIFO over run */
            }
#else
						ats3607_uart_rx_update(bInChar);
#endif					
        }
    }

#if 1
    if (u32IntStatus & UART_INTSTS_THREIF_Msk)
    {
				#if 0
        if (g_u16ComTbytes && (UART0->INTEN & UART_INTEN_THREIEN_Msk))
        {
            int32_t size;

						
            /* Fill the Tx FIFO */
            size = g_u16ComTbytes;

            if (size >= TX_FIFO_SIZE)
            {
                size = TX_FIFO_SIZE;
            }

            while (size)
            {
                if (g_u16ComThead >= TXBUFSIZE)
                    g_u16ComThead = 0;

                bInChar = g_au8ComTbuf[g_u16ComThead++];
                UART0->DAT = bInChar;

                g_u16ComTbytes--;
                size--;
            }
						
        }
        else
				#endif
        {
            /* No more data, just stop Tx (Stop work) */
            UART0->INTEN &= (~UART_INTEN_THREIEN_Msk);
        }
    }
#endif		
}

extern volatile uint32_t PWOER_OFF_TMRINT_count;
extern uint8_t db_print_out_loc_2;
	
#define DOA_START_LOCATION 14
#define LED_RING_PWR_OFF_S 2
	
void GPB_IRQHandler(void)
{
    /* To check if PB.11 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PB, BIT11))
    {
        GPIO_CLR_INT_FLAG(PB, BIT11);
        //printf("PB.11 INT occurred.\n");
			  //printf("\x1b[%d;90HPB.11 INT occurred.\n",db_print_out_loc_2++);

				//GPIO_DisableInt(PB, 11);
				//GPIO_DISABLE_DEBOUNCE(PB, BIT11);
				//NVIC_DisableIRQ(GPB_IRQn);		
    }
    else if (GPIO_GET_INT_FLAG(PB, BIT2))
    {
        GPIO_CLR_INT_FLAG(PB, BIT2);
				//GPIO_DisableInt(PB, 2);
				//GPIO_DISABLE_DEBOUNCE(PB, BIT2);
				//NVIC_DisableIRQ(GPB_IRQn);		
    }		
    else
{
        uint32_t u32Status;
        u32Status = PB->INTSRC;
        /* Un-expected interrupt. Just clear all PB interrupts */
        PB->INTSRC = u32Status;
        //printf("Un-expected interrupts.\n");
    }
}

void VCOM_TransferData(void)
{
#if 0
    int32_t i;
    /* Check whether USB is ready for next packet or not */
    if (g_u32TxSize == 0)
    {
        int32_t i32Len;

        /* Check whether we have new COM Rx data to send to USB or not */
        if (g_u16ComRbytes)
        {
            i32Len = g_u16ComRbytes;

            if (i32Len > EP2_MAX_PKT_SIZE)
                i32Len = EP2_MAX_PKT_SIZE;

            for (i = 0; i < i32Len; i++)
            {
                if (g_u16ComRhead >= RXBUFSIZE)
                    g_u16ComRhead = 0;

                g_au8RxBuf[i] = g_au8ComRbuf[g_u16ComRhead++];
            }

            __set_PRIMASK(1);
            g_u16ComRbytes -= i32Len;
            __set_PRIMASK(0);

            g_u32TxSize = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)g_au8RxBuf, i32Len);
            USBD_SET_PAYLOAD_LEN(EP2, i32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP2);

            if (i32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if (g_i8BulkOutReady && (g_u32RxSize <= TXBUFSIZE - g_u16ComTbytes))
    {
        for (i = 0; i < g_u32RxSize; i++)
        {
            g_au8ComTbuf[g_u16ComTtail++] = g_pu8RxBuf[i];

            if (g_u16ComTtail >= TXBUFSIZE)
                g_u16ComTtail = 0;
        }

        __set_PRIMASK(1);
        g_u16ComTbytes += g_u32RxSize;
        __set_PRIMASK(0);

        g_u32RxSize = 0;
        g_i8BulkOutReady = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    /* Process the software Tx FIFO */
    if (g_u16ComTbytes)
    {
        /* Check if Tx is working */
        if ((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            if (g_u16ComThead >= TXBUFSIZE)
            {
                g_u16ComThead = 0;
            }

            /* Send one bytes out */
            UART0->DAT = g_au8ComTbuf[g_u16ComThead++];

            g_u16ComTbytes--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#else
		ats3607_uart_rx_parse();
		ats3607_uart2usb_xfer();
		ats3607_usb2uart_xfer();
#endif		
}

void PowerDown()
{
		//CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_DPD);
		SYS_UnlockReg();
    GPIO_SetMode(PB, BIT11, GPIO_MODE_INPUT);		//PowerDown button
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);		//VBUS
    GPIO_SetMode(PC, BIT3, GPIO_MODE_INPUT);	//5G RX module CCH interrupt

		GPIO_EnableInt(PB, 11, GPIO_INT_FALLING);
		GPIO_EnableInt(PB, 2, GPIO_INT_BOTH_EDGE);
    GPIO_DisableInt(PC, 3);

		NVIC_EnableIRQ(GPB_IRQn);			
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);	
    GPIO_ENABLE_DEBOUNCE(PB, BIT11);
    GPIO_ENABLE_DEBOUNCE(PB, BIT2);	

    NVIC_DisableIRQ(GPC_IRQn);

    GPIO_CLR_INT_FLAG(PB, BIT11);
    GPIO_CLR_INT_FLAG(PB, BIT2);
    GPIO_CLR_INT_FLAG(PC, BIT3);

		//GW_power_down_interface();
		//--------------------------------------------------------------------------
		GPO_PA11_DSP_ONOFF = HIGH; //dsp 	
		GPO_PD03_AMP_EN = NO;
		GPO_PD02_MCU_PWREN = LOW;
#if 1
		TIMER_Stop(TIMER0);
		TIMER_DisableInt(TIMER0);
		NVIC_DisableIRQ(TMR0_IRQn);

		UART_DisableInt(UART0, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		UART_DisableInt(UART1, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		UART_DisableInt(UART2, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);

		UART_Close(UART0);
		UART_Close(UART1);
		UART_Close(UART2);

		GW_AW5808_Tx_On_Off(0);
		//wireless
		if(hw_current_version == 0)
				GW_ADC_close();

		UI2C_Close(UI2C0);
		UI2C_DISABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk | UI2C_PROTIEN_ARBLOIEN_Msk |UI2C_PROTIEN_ERRIEN_Msk));
    UI2C_DisableInt(UI2C0, UI2C_TO_INT_MASK | UI2C_STAR_INT_MASK | UI2C_STOR_INT_MASK | UI2C_NACK_INT_MASK | UI2C_ARBLO_INT_MASK | UI2C_ERR_INT_MASK | UI2C_ACK_INT_MASK);		
		NVIC_DisableIRQ(USCI0_IRQn);		

		PDMA_Close(PDMA);

		CLK_DisableModuleClock(UART0_MODULE);
		CLK_DisableModuleClock(UART1_MODULE);
		CLK_DisableModuleClock(UART2_MODULE);
		CLK_DisableModuleClock(USCI0_MODULE);
		CLK_DisableModuleClock(I2C0_MODULE);
		CLK_DisableModuleClock(SPI0_MODULE);
		CLK_DisableModuleClock(PDMA_MODULE);
		CLK_DisableModuleClock(USBD_MODULE);
		CLK_DisableModuleClock(EADC_MODULE);	

    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA6MFP_Msk);
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA7MFP_Msk);

    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA8MFP_Msk);
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA9MFP_Msk);
	
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk);	

		//GW_free_gpio();
		//-----------------------------------------------------------
		//release USB interface
		PA->MODE &= ~(GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk | GPIO_MODE_MODE14_Msk);
		SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);	

		GPIO_SetMode(PA, BIT11, GPIO_MODE_INPUT);     //dsp
		GPIO_SetMode(PA, BIT4, GPIO_MODE_INPUT);		//DOA_LED_SEL,				LED select
		GPIO_SetMode(PA, BIT5, GPIO_MODE_INPUT);		//MIC_LED_SEL,				LED select
		GPIO_SetMode(PD, BIT15, GPIO_MODE_INPUT);	//TOUCH_LED_SEL,			LED select
		GPIO_SetMode(PC, BIT7, GPIO_MODE_INPUT);		//TYPEC_SPDIF2_EN,		DFP2(SPDIF),	Control PCA9306
		GPIO_SetMode(PF, BIT2, GPIO_MODE_INPUT);		//5GTRX_SW,						switch I2S,		Lo:5G(Tx) / Hi:5G(Rx)
		GPIO_SetMode(PF, BIT3, GPIO_MODE_INPUT);		//WWL_SW,							switch I2S, 	Lo:5G / Hi:CS8422
		GPIO_SetMode(PA, BIT10, GPIO_MODE_INPUT);	//TYPEC1_SW,					switch C,			UART/SPDIF switch
		GPIO_SetMode(PA, BIT11, GPIO_MODE_INPUT);	//DSP_ONOFF,					3607D,				Lo:reset / Hi:normal
		GPIO_SetMode(PB, BIT6, GPIO_MODE_INPUT);		//TYPEC2_SW,					switch C,			UART/SPDIF switch
		GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);		//5GTX_WWL_SW,				U35,	Hi:Wireless Lo:wire
		GPIO_SetMode(PB, BIT8, GPIO_MODE_INPUT);		//5GRX_WWL_SW,				U37,	Hi:Wireless Lo:wire	
		GPIO_SetMode(PB, BIT9, GPIO_MODE_INPUT);		//5GRX_MS_SEL

		GPIO_SetMode(PB, BIT12, GPIO_MODE_INPUT);	//USBC_UDP_SEL1,			HUB UFP switch to TV or PC
		GPIO_SetMode(PB, BIT13, GPIO_MODE_INPUT);	//USBC_UDP_SEL0,			HUB UFP switch to TV or PC
		GPIO_SetMode(PB, BIT14, GPIO_MODE_INPUT);	//USB2SW,							MCU USB switch to TV or PC

		GPIO_SetMode(PD, BIT3, GPIO_MODE_INPUT);		//AMP_EN,							Audio amp,		Hi:on
		GPIO_SetMode(PC, BIT4, GPIO_MODE_INPUT);		//TYPEC_SPDIF1_EN,		DFP1(SPDIF),	Control PCA9306
		GPIO_SetMode(PF, BIT4, GPIO_MODE_INPUT);		//TYPEC_UART1_EN,			DFP1(UART),		Control PCA9306
		GPIO_SetMode(PF, BIT5, GPIO_MODE_INPUT);		//TYPEC_UART2_EN,			DFP2(UART),		Control PCA9306	

		CLK_DisableModuleClock(GPA_MODULE);
		//CLK_DisableModuleClock(GPB_MODULE);	
		CLK_DisableModuleClock(GPC_MODULE);
		CLK_DisableModuleClock(GPD_MODULE);	//ned control dock power
		CLK_DisableModuleClock(GPF_MODULE);			
		//-------------------------------------
#else
		TIMER_Stop(TIMER0);
		TIMER_DisableInt(TIMER0);
		NVIC_DisableIRQ(TMR0_IRQn);
	
		UART_DisableInt(UART0, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		UART_DisableInt(UART1, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		UART_DisableInt(UART2, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		
		UART_Close(UART0);
		UART_Close(UART1);
		UART_Close(UART2);
			
		GW_AW5808_Tx_On_Off(0);		
		
		//wireless
		if(hw_current_version==0)
		{
			GW_ADC_close();
		}
		
		UI2C_Close(UI2C0);
		UI2C_DISABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk | UI2C_PROTIEN_ARBLOIEN_Msk |UI2C_PROTIEN_ERRIEN_Msk));
    UI2C_DisableInt(UI2C0, UI2C_TO_INT_MASK | UI2C_STAR_INT_MASK | UI2C_STOR_INT_MASK | UI2C_NACK_INT_MASK | UI2C_ARBLO_INT_MASK | UI2C_ERR_INT_MASK | UI2C_ACK_INT_MASK);		
		NVIC_DisableIRQ(USCI0_IRQn);		
		PDMA_Close(PDMA);
			

		CLK_DisableModuleClock(UART0_MODULE);
		CLK_DisableModuleClock(UART1_MODULE);
		CLK_DisableModuleClock(UART2_MODULE);
		CLK_DisableModuleClock(USCI0_MODULE);
		CLK_DisableModuleClock(I2C0_MODULE);
		CLK_DisableModuleClock(SPI0_MODULE);
		CLK_DisableModuleClock(PDMA_MODULE);
		CLK_DisableModuleClock(USBD_MODULE);
		CLK_DisableModuleClock(EADC_MODULE);	
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA6MFP_Msk);
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA7MFP_Msk);
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA8MFP_Msk);
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA9MFP_Msk);
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk);	
		//------------------------------------------------------------------
		#if 0
		PA->MODE &= ~(GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk | GPIO_MODE_MODE14_Msk);
		SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);	
		GPIO_SetMode(PA, BIT11, GPIO_MODE_INPUT);     //dsp
		GPIO_SetMode(PA, BIT4, GPIO_MODE_INPUT);		//DOA_LED_SEL,				LED select
		GPIO_SetMode(PA, BIT5, GPIO_MODE_INPUT);		//MIC_LED_SEL,				LED select
		GPIO_SetMode(PD, BIT15, GPIO_MODE_INPUT);	//TOUCH_LED_SEL,			LED select
		GPIO_SetMode(PC, BIT7, GPIO_MODE_INPUT);		//TYPEC_SPDIF2_EN,		DFP2(SPDIF),	Control PCA9306
		GPIO_SetMode(PF, BIT2, GPIO_MODE_INPUT);		//5GTRX_SW,						switch I2S,		Lo:5G(Tx) / Hi:5G(Rx)
		GPIO_SetMode(PF, BIT3, GPIO_MODE_INPUT);		//WWL_SW,							switch I2S, 	Lo:5G / Hi:CS8422
		GPIO_SetMode(PA, BIT10, GPIO_MODE_INPUT);	//TYPEC1_SW,					switch C,			UART/SPDIF switch
		GPIO_SetMode(PA, BIT11, GPIO_MODE_INPUT);	//DSP_ONOFF,					3607D,				Lo:reset / Hi:normal
		GPIO_SetMode(PB, BIT6, GPIO_MODE_INPUT);		//TYPEC2_SW,					switch C,			UART/SPDIF switch
		GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);		//5GTX_WWL_SW,				U35,	Hi:Wireless Lo:wire
		GPIO_SetMode(PB, BIT8, GPIO_MODE_INPUT);		//5GRX_WWL_SW,				U37,	Hi:Wireless Lo:wire	
		GPIO_SetMode(PB, BIT9, GPIO_MODE_INPUT);		//5GRX_MS_SEL
		GPIO_SetMode(PB, BIT12, GPIO_MODE_INPUT);	//USBC_UDP_SEL1,			HUB UFP switch to TV or PC
		GPIO_SetMode(PB, BIT13, GPIO_MODE_INPUT);	//USBC_UDP_SEL0,			HUB UFP switch to TV or PC
		GPIO_SetMode(PB, BIT14, GPIO_MODE_INPUT);	//USB2SW,							MCU USB switch to TV or PC
		GPIO_SetMode(PD, BIT3, GPIO_MODE_INPUT);		//AMP_EN,							Audio amp,		Hi:on
		GPIO_SetMode(PC, BIT4, GPIO_MODE_INPUT);		//TYPEC_SPDIF1_EN,		DFP1(SPDIF),	Control PCA9306
		GPIO_SetMode(PF, BIT4, GPIO_MODE_INPUT);		//TYPEC_UART1_EN,			DFP1(UART),		Control PCA9306
		GPIO_SetMode(PF, BIT5, GPIO_MODE_INPUT);		//TYPEC_UART2_EN,			DFP2(UART),		Control PCA9306	
		CLK_DisableModuleClock(GPA_MODULE);
		//CLK_DisableModuleClock(GPB_MODULE);	
		CLK_DisableModuleClock(GPC_MODULE);
		//CLK_DisableModuleClock(GPD_MODULE);	//ned control dock power
		CLK_DisableModuleClock(GPF_MODULE);
		#endif
			
#endif			
    /* Enter to Power-down mode */
		//CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_PD);
    CLK_PowerDown();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
		GW_LED_init();

		hw_current_version = get_hardware_version();
    //wire	
	  if(hw_current_version==1)
		{
			gsInfo.gu8DevDesc[10]=WIRE_USBD_PID&0x00FF;
			gsInfo.gu8DevDesc[11]=(WIRE_USBD_PID&0xFF00)>>8;
			gsInfo.gu8StringDesc[2]=(uint8_t *)gu8ProductStringDesc_Wired;
			satellite_mode = WIRE_MODE;			
		}

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();
    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim value */
    u32TrimInit = M32(TRIM_INIT);

    /* Clear SOF */
    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
#endif

		GW_menu_init();
		GW_hid_app_init(EP5);
		GW_system_event_init();

		//wireless
		if(hw_current_version == 0) {
				SYS_ADC_Init();
				GW_battery_adc_init();
				GW_Touch_Init();
		}
		else {
				GW_AW5808_Init();
		}

		printf("\x1b[13;61HMIC HW=%d[%x %x/%x %x]\n",hw_current_version,gsInfo.gu8DevDesc[8],gsInfo.gu8DevDesc[9], gsInfo.gu8DevDesc[10],gsInfo.gu8DevDesc[11]);	

    while (SYS->PDID)
    {
				GW_LED_loop();
		
#if CRYSTAL_LESS

        /* Start USB trim function if it is not enabled. */
        if ((SYS->HIRCTRIMCTL & SYS_HIRCTRIMCTL_FREQSEL_Msk) != 0x1)
        {
            /* Start USB trim only when USB signal arrived */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /*
                    USB clock trim function:
                    HIRC Trimming with boundary function enhances robustility
                    and keeps HIRC in right frequency while receiving unstable USB signal
                */
                SYS->HIRCTRIMCTL = (0x1 << SYS_HIRCTRIMCTL_REFCKSEL_Pos)
                                   | (0x1 << SYS_HIRCTRIMCTL_FREQSEL_Pos)
                                   | (0x0 << SYS_HIRCTRIMCTL_LOOPSEL_Pos)
                                   | (0x1 << SYS_HIRCTRIMCTL_BOUNDEN_Pos)
                                   | (10  << SYS_HIRCTRIMCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when any error found */
        if (SYS->HIRCTRIMSTS & (SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable USB clock trim function */
            SYS->HIRCTRIMCTL = 0;

            /* Clear trim error flags */
            SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
        }

#endif

        /* Enter power down when USB suspend */
        if (g_u8Suspend) {
            //PowerDown();
				}
				else
						VCOM_TransferData();
    }
}



/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
