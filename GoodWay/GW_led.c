#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "GW_main.h"
#include "GW_aw5808.h"
#include "GW_touch.h"
#include "GW_hid_Transfer.h"

//#define IGNORE_PRINTF
#ifdef IGNORE_PRINTF
	#define printf(fmt, ...) (0)
#endif

//LED need 800KHz , 300ns for 0 , 900ns for 1
//Use 3.2MHz SPI clk
// 0 : 1000
// 1 : 1110
// SPI 1 byte only can display 2 bit
// so 1 RGB LED = 24bit/2 = 12 bytes
// R7..0 G7..0 B7..0

#define true  1
#define false 0

#define RING_R_F	0
#define RING_G_F	1
#define RING_B_F	2
#define RING_R_B	3
#define RING_G_B	4
#define RING_B_B	5
#define RING_STEP 6
#define RING_FPS	7
#define IDLE_R_F	8
#define IDLE_G_F	9
#define IDLE_B_F	10
#define CALL_R_F	11
#define CALL_G_F	12
#define CALL_B_F	13
#define VOL_R_F		14
#define VOL_G_F		15
#define VOL_B_F		16
#define MUTE_R_F	17
#define MUTE_G_F	18
#define MUTE_B_F	19
#define WARRING_R_F 20
#define WARRING_G_F 21
#define WARRING_B_F	22
#define PAIR_R_F		23
#define PAIR_G_F		24
#define PAIR_B_F		25
#define DOA_R_F			26
#define DOA_G_F			27
#define DOA_B_F			28
#define DOA_R_B			29
#define DOA_G_B			30
#define DOA_B_B			31
#define PAIR_FAIL_R	32
#define PAIR_FAIL_G	33
#define PAIR_FAIL_B	34
#define PAIR_OK_R		35
#define PAIR_OK_G		36
#define PAIR_OK_B		37
#define DOA_R_OFF		38
#define DOA_G_OFF		39
#define DOA_B_OFF		40
#define FW_FLASH_R	20
#define FW_FLASH_G	21
#define FW_FLASH_B	422

#define DOA_R_AMBER		0

#define DOA_BG_VALUE 16

#define VOL_DISPLAY_TIME	2		//volume dislay time when volume change

#define SPI_TX_DMA_CH                     (0)

uint8_t LED_DOA_source[DOA_size];

//0  power on/off ring frontend
//3  power on/off ring background
//6  ring step , FPS
//8  power on , not under call
//11 power on , during call and DOA
//14 volume adjust
//17 mute
//20 wairing
//23 pairing
uint8_t DOA_RGB[] = { 
192, 	192, 	192, 	\
64, 	64, 	64, 	\
DOA_TOTAL_STEP, DOA_FPS,	\
64,		64,		64, 	\
0,		128,	0,		\
0,		128,	0,		\
128,	0,		0,		\
255,	191,	0,		\
0,		0,		128,	\
0,		128,	0,		\
0,		DOA_BG_VALUE,		0,		\
255,	191,	0,		\
0,		0,		128,		\
0,		0,		0		\
};

uint8_t factory_led_cross;
uint8_t factory_led_set[8] = { 64, 64, 64, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // R(8b) , G(8b) , B(8b) , LED 1...36b(LSB) , default use idle RGB

volatile uint32_t PWOER_OFF_TMRINT_count = 0;
volatile uint8_t wait_PWR_btn_release = NO;
volatile uint8_t wait_PAIR_btn_release = NO;
volatile uint8_t wait_HOST_btn_release = NO;
volatile uint8_t wireless_mux_sw = SLAVE_5G;
volatile uint8_t wire_mux_sw = SLAVE_5G;
volatile uint8_t satellite_mode = WIRELESS_MODE;

volatile uint8_t wireless_who = WHO_IS_NO;
volatile uint8_t wireless_level = WHO_IS_NO , DFP1_level = WHO_IS_NO , DFP2_level = WHO_IS_NO;
volatile uint8_t factory_mode = NO;
volatile uint8_t dsp_mode_ready = NO;

volatile uint8_t ats3607d_in_buf[MAX_DSP_BUF];
volatile uint8_t ats3607d_in_start = 0, ats3607d_in_end = 0;
volatile uint8_t DSP_event_buf[64];
volatile uint8_t DSP_event_ptr = 0;
volatile uint32_t g_u32EadcInt0Flag, g_u32EadcInt1Flag, g_u32EadcInt2Flag;

uint32_t adc_count = 0;
uint32_t adc_temp_count = 0;

uint8_t USB_raw_request[64];
uint8_t USB_raw_response[64];
uint8_t USB_raw_req_len = 0, USB_raw_res_len = 0;
uint8_t RF_module_set[6] = {0, 0, 0, 0, 0, 0}; //TX fix , Tx save , Tx power, Rx fix, Rx save, Rx power

led_status_s       m_led;
cch_module_s       m_cch_rx;
vdm_module_s       m_vdm;
module_status_s    m_module;
debug_s            m_debug;
system_evt_cb_s    sys_evt;

void send_atten_req(void);
void GW_usb_adc_init(void);
void GW_usb_adc_uninit(void);
uint8_t GW_check_DFP_level(uint8_t connect);
uint8_t GW_host_check(uint8_t status);
void GW_5G_CCH_pop_all(void);
void GW_5G_CCH_push(uint8_t type);
void GW_system_event_init(void);
void GW_led_lock(uint8_t long_lock)
{
		m_module.led_lock_timeout_ms = long_lock? 3600000: 2000;
		m_module.led_lock_time_ms = m_module.current_time_ms;
		m_module.led_lock = YES;
}

void GW_led_unlock(void)
{
		m_module.led_lock = NO;
}

//  The section which mark with "Unknown!!!" should fixed in the future

//void SYS_LED_Init(void)
void set_LED_RGB(uint8_t *source, uint8_t LED_num , uint8_t R , uint8_t G, uint8_t B);
void GW_led_color_update(void);
void system_state_update(uint8_t state);

void GW_led_constant_set(uint8_t * p_color);
void GW_led_blinky_set(uint8_t hz);
void GW_led_breeze_set(uint8_t hz);

uint8_t time_to_work(uint32_t * time_ms, uint32_t time_period_ms)
{
		if(*time_ms >= time_period_ms) {
				*time_ms = 0;
				return YES;
		}
		return NO;
}

void aw5808_rx_init(void)
{
		aw5808_rx_data.old_online = 0xFF,
		aw5808_rx_data.old_status = 0xFF,
		memset(aw5808_rx_data.old_id, CCH_PAIR_ID_OLD, sizeof(aw5808_rx_data.old_id));
		memcpy(aw5808_rx_data.old_version, "\x00\x00", sizeof(aw5808_rx_data.old_version));
}

void provide_LED_data(void);
void UART0_Init(void);
void UART1_Init(void);
void UART2_Init(void);
void timer0_init(void);

void system_standby_restart(void)
{
		m_module.standby_timeout_ms = 0;
		m_module.standby_timeout_s = 0;
}

void system_unpair_restart(void)
{
		m_cch_rx.pair.pairing_timeout_ticks_ms = 0;
}

void power_down_debug_uart2(void)
{
    /* Set GPB multi-function pins for UART2 RXD and TXD */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB4MFP_Msk) | SYS_GPB_MFPL_PB4MFP_UART2_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk) | SYS_GPB_MFPL_PB5MFP_UART2_TXD;

		CLK_EnableModuleClock(UART2_MODULE);
		CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
		UART2_Init();
}

void GW_standby_module_disable(void)
{
// Make interrupt of USB and button actived begin
		GPO_PD02_MCU_PWREN = LOW;
		GPO_PA11_DSP_ONOFF = HIGH;

    GPIO_CLR_INT_FLAG(PB, BIT11);
    GPIO_SetMode(PB, BIT11, GPIO_MODE_INPUT);                //PowerDown button
		GPIO_EnableInt(PB, 11, GPIO_INT_FALLING);
    GPIO_ENABLE_DEBOUNCE(PB, BIT11);

    GPIO_CLR_INT_FLAG(PB, BIT2);
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);                 //VBUS
		GPIO_EnableInt(PB, 2, GPIO_INT_BOTH_EDGE);
    GPIO_ENABLE_DEBOUNCE(PB, BIT2);

		NVIC_EnableIRQ(GPB_IRQn);
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);	

    GPIO_CLR_INT_FLAG(PC, BIT3);
    GPIO_SetMode(PC, BIT3, GPIO_MODE_INPUT);                 //5G RX module CCH interrupt
    GPIO_DisableInt(PC, 3);
    NVIC_DisableIRQ(GPC_IRQn);
// Make interrupt of USB and button actived end

// GPIO disabled begin
		CLK_DisableModuleClock(GPA_MODULE);
		//CLK_DisableModuleClock(GPB_MODULE);
		CLK_DisableModuleClock(GPC_MODULE);
		CLK_DisableModuleClock(GPD_MODULE);	        //ned control dock power
		CLK_DisableModuleClock(GPF_MODULE);

		GPIO_SetMode(PA, BIT10, GPIO_MODE_INPUT);   //TYPEC1_SW,          switch C, UART/SPDIF switch
		GPIO_SetMode(PA, BIT11, GPIO_MODE_INPUT);   //DSP_ONOFF,          3607D, Lo:reset / Hi:normal
		GPIO_SetMode(PB, BIT6, GPIO_MODE_INPUT);    //TYPEC2_SW,          switch C, UART/SPDIF switch
		GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);    //5GTX_WWL_SW,        U35, Hi:Wireless Lo:wire
		GPIO_SetMode(PB, BIT8, GPIO_MODE_INPUT);    //5GRX_WWL_SW,        U37, Hi:Wireless Lo:wire
		GPIO_SetMode(PB, BIT9, GPIO_MODE_INPUT);    //5GRX_MS_SEL
		GPIO_SetMode(PC, BIT4, GPIO_MODE_INPUT);    //TYPEC_SPDIF1_EN,    DFP1(SPDIF), Control PCA9306
		GPIO_SetMode(PC, BIT7, GPIO_MODE_INPUT);    //TYPEC_SPDIF2_EN,    DFP2(SPDIF), Control PCA9306
		GPIO_SetMode(PF, BIT2, GPIO_MODE_INPUT);    //5GTRX_SW,           switch I2S, Lo:5G(Tx) / Hi:5G(Rx)
		GPIO_SetMode(PF, BIT4, GPIO_MODE_INPUT);    //TYPEC_UART1_EN,     DFP1(UART), Control PCA9306
		GPIO_SetMode(PF, BIT5, GPIO_MODE_INPUT);    //TYPEC_UART2_EN,     DFP2(UART), Control PCA9306
// GPIO disabled end

// TIMER0 disabled begin
		TIMER_Stop(TIMER0);
		TIMER_DisableInt(TIMER0);
		NVIC_DisableIRQ(TMR0_IRQn);
		CLK_DisableModuleClock(TMR0_MODULE);
// TIMER0 disabled end

// I2C0 disabled begin
    NVIC_DisableIRQ(I2C0_IRQn);
    I2C_DisableInt(I2C0);
    I2C_Close(I2C0);
		CLK_DisableModuleClock(I2C0_MODULE);
// I2C0 disabled begin

// UI2C0 disabled begin
		UI2C_Close(UI2C0);
		UI2C_DISABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk | UI2C_PROTIEN_ARBLOIEN_Msk |UI2C_PROTIEN_ERRIEN_Msk));
    UI2C_DisableInt(UI2C0, UI2C_TO_INT_MASK | UI2C_STAR_INT_MASK | UI2C_STOR_INT_MASK | UI2C_NACK_INT_MASK | UI2C_ARBLO_INT_MASK | UI2C_ERR_INT_MASK | UI2C_ACK_INT_MASK);		
		NVIC_DisableIRQ(USCI0_IRQn);
		CLK_DisableModuleClock(USCI0_MODULE);
// UI2C0 disabled end

// EADC disabled begin
		EADC_Close(EADC);
		NVIC_DisableIRQ(EADC_INT0_IRQn);
		NVIC_DisableIRQ(EADC_INT1_IRQn);
		CLK_DisableModuleClock(EADC_MODULE);
// EADC disabled end

// USBD disabled begin
    NVIC_DisableIRQ(USBD_IRQn);
		CLK_DisableModuleClock(USBD_MODULE);
		PA->MODE &= ~(GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk | GPIO_MODE_MODE14_Msk);
		SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);	
// USBD disabled end

// UART 0~2 disabled begin
		UART_DisableInt(UART0, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		UART_DisableInt(UART1, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		UART_DisableInt(UART2, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);

		UART_Close(UART0);
		UART_Close(UART1);
		UART_Close(UART2);

		CLK_DisableModuleClock(UART0_MODULE);
		CLK_DisableModuleClock(UART1_MODULE);
		CLK_DisableModuleClock(UART2_MODULE);
// UART 0~2 disabled end

// SPI0 disabled begin
		CLK_DisableModuleClock(SPI0_MODULE);
		SPI_Close(SPI0);
		SPI_DisableAutoSS(SPI0);
// SPI0 disabled end

// PDMA disabled begin
		CLK_DisableModuleClock(PDMA_MODULE);
		PDMA_Close(PDMA);
// PDMA disabled end
}

void GW_standby_module_enable(void)
{
		GPIO_DisableInt(PB, 11);
    GPIO_CLR_INT_FLAG(PB, BIT11);

		GPIO_DisableInt(PB, 2);
    GPIO_CLR_INT_FLAG(PB, BIT2);

// GPIO enabled begin
		CLK_EnableModuleClock(GPA_MODULE);
		//CLK_DisableModuleClock(GPB_MODULE);	
		CLK_EnableModuleClock(GPC_MODULE);
		CLK_EnableModuleClock(GPD_MODULE);	        //ned control dock power
		CLK_EnableModuleClock(GPF_MODULE);

		GPIO_SetMode(PA, BIT10, GPIO_MODE_INPUT);   //TYPEC1_SW,          switch C, UART/SPDIF switch
		GPIO_SetMode(PA, BIT11, GPIO_MODE_OUTPUT);   //DSP_ONOFF,          3607D, Lo:reset / Hi:normal
		GPIO_SetMode(PB, BIT6, GPIO_MODE_OUTPUT);    //TYPEC2_SW,          switch C, UART/SPDIF switch
		GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);    //5GTX_WWL_SW,        U35, Hi:Wireless Lo:wire
		GPIO_SetMode(PB, BIT8, GPIO_MODE_INPUT);    //5GRX_WWL_SW,        U37, Hi:Wireless Lo:wire
		GPIO_SetMode(PB, BIT9, GPIO_MODE_OUTPUT);    //5GRX_MS_SEL
		GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);    //TYPEC_SPDIF1_EN,    DFP1(SPDIF), Control PCA9306
		GPIO_SetMode(PC, BIT7, GPIO_MODE_OUTPUT);    //TYPEC_SPDIF2_EN,    DFP2(SPDIF), Control PCA9306
		GPIO_SetMode(PF, BIT2, GPIO_MODE_OUTPUT);    //5GTRX_SW,           switch I2S, Lo:5G(Tx) / Hi:5G(Rx)
		GPIO_SetMode(PF, BIT4, GPIO_MODE_OUTPUT);    //TYPEC_UART1_EN,     DFP1(UART), Control PCA9306
		GPIO_SetMode(PF, BIT5, GPIO_MODE_OUTPUT);    //TYPEC_UART2_EN,     DFP2(UART), Control PCA9306
// GPIO enabled end

// GPIO initialized begin
		GPO_PD02_MCU_PWREN = HIGH;
		GPO_PA11_DSP_ONOFF = LOW;
		GPI_PB09_5G_RX_MS_SEL = HIGH;
		GPO_PC04_PD_DFP_INT = HIGH;
		GPO_PF04_TYPEC_UART1_EN = HIGH;
		GPO_PF05_TYPEC_UART2_EN = HIGH;
		GPO_PC07_Daisy_Chain1_EN = HIGH;
		GPO_PF02_Daisy_Chain2_EN = HIGH;

    GPIO_CLR_INT_FLAG(PC, BIT3);
    GPIO_SetMode(PC, BIT3, GPIO_MODE_INPUT);                 //5G RX module CCH interrupt
    GPIO_EnableInt(PC, 3, GPIO_INT_FALLING);		
    NVIC_EnableIRQ(GPC_IRQn);
// GPIO initialized end

// TIMER0 enabled begin
		CLK_EnableModuleClock(TMR0_MODULE);
		CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
		if (TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, TIMER0_FREQ) != 1) {
		}
		TIMER_EnableInt(TIMER0);
		NVIC_EnableIRQ(TMR0_IRQn);
		TIMER_Start(TIMER0);
// TIMER0 enabled end

// I2C0 enabled begin
		CLK_EnableModuleClock(I2C0_MODULE);
    NVIC_EnableIRQ(I2C0_IRQn);
    I2C_EnableInt(I2C0);
    I2C_Open(I2C0, 100000);
// I2C0 enabled begin

// UI2C0 enabled begin
		CLK_EnableModuleClock(USCI0_MODULE);
    UI2C_Open(UI2C0, 400000);
    UI2C_SetSlaveAddr(UI2C0, 0, 0x28, UI2C_GCMODE_DISABLE);
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk | UI2C_PROTIEN_ARBLOIEN_Msk |UI2C_PROTIEN_ERRIEN_Msk));	
    NVIC_EnableIRQ(USCI0_IRQn);
// UI2C0 enabled end

// EADC enabled begin
		CLK_EnableModuleClock(EADC_MODULE);
		EADC_Open(EADC, 0);
		NVIC_EnableIRQ(EADC_INT0_IRQn);
// EADC enabled end

// USBD enabled begin
		CLK_EnableModuleClock(USBD_MODULE);
    USBD_Open(&gsInfo, HID_ClassRequest, NULL);
    HID_Init();
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);
// USBD enabled end

// UART 0~2 enabled begin
    CLK_EnableModuleClock(UART0_MODULE);
		CLK_EnableModuleClock(UART1_MODULE);
		CLK_EnableModuleClock(UART2_MODULE);

#ifdef EN_UART0
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
#endif
		CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
		CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));

#ifdef EN_UART0
		UART0_Init();
#endif
		UART1_Init();
		UART2_Init();
// UART 0~2 enabled end

// SPI0 enabled begin
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
		CLK_EnableModuleClock(SPI0_MODULE);
		SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, 3389830);
		SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);
// SPI0 enabled end

// PDMA enabled begin
		SYS_ResetModule(PDMA_RST);
		CLK_EnableModuleClock(PDMA_MODULE);
    PDMA_Open(PDMA, (1 << SPI_TX_DMA_CH));
		PDMA_CLR_TD_FLAG(PDMA, (1 << SPI_TX_DMA_CH));
    PDMA_SetTransferMode(PDMA, SPI_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
    PDMA_SetBurstType(PDMA, SPI_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    PDMA->DSCT[SPI_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
    SPI_TRIGGER_TX_RX_PDMA(SPI0);
// PDMA enabled end
}

void GW_standby_mode_enter(void)
{
		while(GPI_PB11_PWR_SW == LOW);
		PrintMsg__("Standby power-down! \r\n");

		SYS_UnlockReg();

		uint8_t   sleep_now = YES, keep_check = YES;
		uint8_t   usb_connection_last = GPI_PB02_VBUS_DET, button_int_source;
		uint32_t  adc_accum, adc_count;
		do {
				if(sleep_now) {
						GW_standby_module_disable();
						CLK_PowerDown();
						m_module.power_on_timeout_ms = 0;
						timer0_init();

						button_int_source = GPI_PB11_PWR_SW == LOW? YES: NO;
						if(button_int_source == NO && usb_connection_last == LOW) {
								adc_accum = 0;
								adc_count = 0;
								GW_usb_adc_init();
						}
				}

				if(button_int_source) {
						keep_check = NO;
				}
				else {
						if(usb_connection_last == LOW) {
								sleep_now = NO;
								if(m_module.power_on_timeout_ms >= USB_PLUG_IN_ADC_TIME_MS) {
										if((adc_accum/adc_count) >= USB_PLUG_IN_ADC_LEVEL)
												keep_check = NO;
										else
												sleep_now = YES;
										PrintMsg__(":%4d \r\n", (adc_accum/adc_count));
										GW_usb_adc_uninit();
								}
								else {
										if(g_u32EadcInt2Flag == 1) {
												g_u32EadcInt2Flag = 0;
												uint32_t adc = EADC_GET_CONV_DATA(EADC, 2);
												adc_accum += adc;
												adc_count++;
												EADC_START_CONV(EADC, BIT2);
										}
								}
						}
						else if(GPI_PB02_VBUS_DET == LOW)
								keep_check = NO;
				}
		} while(keep_check);
		GW_standby_module_enable();

		SYS_LockReg();

		system_standby_restart();

		PrintMsg__("Standby wakeup \r\n");
		m_module.wakeup_timeout_ms = 0;
		while(time_to_work(&m_module.wakeup_timeout_ms, TIME_S_TO_MS(STANDBY_WAKEUP_TIME_S)));

		sys_evt.paired_awake(AWAKE_CHARGING);
}

void GW_standby_mode_watcher(void)
{
		if(factory_mode == YES)
				return;

		if(time_to_work(&m_module.standby_timeout_ms, TIMER0_FREQ)) {
				m_module.standby_timeout_s++;
		}

		if(m_module.standby == STATE_STANDBY_IDLE) {
				uint8_t standby_evt = STANDBY_IDLE;
				if(m_module.wireless_pair == PAIR_STATE_SUCCESS) {
						if(m_module.usb_connected == YES && 0) {           // Disable the standby-2 mode
								if(m_module.off_range == YES) {
										// STANBY_MODE 2
										if(m_module.standby_timeout_s >= STANBY_MODE2_PAIRED_AND_CHARGING_WITHOUT_CONNECTION_S)
												standby_evt = STANBY_PAIRED_AND_CHARGING_WITHOUT_CONNECTION;
								}
						}
						else {
								if(m_module.off_range == YES) {
										// STANBY_MODE 1
										if(m_module.standby_timeout_s >= STANBY_MODE1_PAIRED_WITHOUT_CONNECTION_S)
												standby_evt = STANDBY_PAIRED_WITHOUT_CONNECTION;
								}
								else if(m_module.system_state != SYSTEM_STATE_IN_CALL) {
										// STANBY_MODE 3
										if(m_module.standby_timeout_s >= STANBY_MODE3_PAIRED_AND_CONNECTED_WITHOUT_CALL_S)
												standby_evt = STANBY_PAIRED_AND_CONNECTING_WITHOUT_CALL;
								}
						}
				}

				if(standby_evt != STANDBY_IDLE) {
						system_standby_restart();
						m_module.standby_evt = standby_evt;
						system_state_update(STATE_PAIR_STANDBY_LED_BREEZE);
				}
		}
		else if(m_module.standby == STATE_STANDBY_LED_BREEZE) {
				if(m_module.standby_timeout_s >= STANBY_MODE_LED_BREEZE_TIMEOUT_S) {
						system_standby_restart();
						system_state_update(STATE_PAIR_STANDBY_LED_POWER_OFF);
						//system_state_update(STATE_PAIR_STANDBY_LED_OFF);
				}
		}
#if 0
		else if(m_module.standby == STATE_STANDBY_LED_OFF) {
				if(m_module.standby_timeout_s >= STANBY_MODE_LED_OFF_TIMEOUT_S) {
						system_standby_restart();
						system_state_update(STATE_PAIR_STANDBY_LED_POWER_OFF);
				}
		}
#endif
}

void led_state_push(uint8_t state)
{
		uint8_t i;

		for(i=0;i<sizeof(m_led.state_buffer);i++)
				if(m_led.state_buffer[i] == state)
						return;

		for(i=0;i<sizeof(m_led.state_buffer);i++) {
				if(m_led.state_buffer[i] == LED_IDLE) {
						m_led.state_buffer[i] = state;
						m_led.state_cycle[i] = 0;
						break;
				}
		}
}

void led_state_pop(uint8_t state)
{
		uint8_t i;

		for(i=0;i<sizeof(m_led.state_buffer);i++) {
				if(m_led.state_buffer[i] == state) {
						memmove(m_led.state_buffer + i, m_led.state_buffer + i + 1, sizeof(m_led.state_buffer) - i);
						memmove(m_led.state_cycle + i, m_led.state_cycle + i + 1, sizeof(m_led.state_cycle) - i);
						break;
				}
		}
}

void led_state_pop_all(void)
{
		uint8_t i;
		for(i=LED_POWER_ON_PEND;i<LED_STATE_COUNT;i++)
				led_state_pop(i);
}

void GW_led_state_machine(void)
{
		uint8_t state = m_led.state_buffer[0];

		switch(state) {
				case LED_IDLE:
						break;
				case LED_POWER_ON_PEND:
						if(m_led.state_cycle[0] >= POWER_ON_BLINKY_TIME_S) {        // Blinky three times to power-up
								led_state_pop(LED_POWER_ON_PEND);
								system_state_update(STATE_POWER_ON_E);
						}
						break;
				case LED_POWER_OFF_PEND:
						if(m_led.state_cycle[0] >= POWER_OFF_BLINKY_TIME_S) {       // Blinky two times to power-down
								led_state_pop(LED_POWER_OFF_PEND);
								system_state_update(STATE_POWER_OFF_E);
						}
						break;
				case LED_POWER_ULTRALOW_BLINKY:
						if(m_led.state_cycle[0] >= POWER_ULTRALOW_PERIOD_S) {       // Blinky two times to power-down
								led_state_pop(LED_POWER_ULTRALOW_BLINKY);
								system_state_update(STATE_ULTRA_LOW_POWER_OFF);
						}
						break;
				case LED_POWER_LOW_BLINKY:
						if(m_module.low_power_led == POWER_LOW_BLINKY) {
								if(m_led.state_cycle[0] >= POWER_LOW_PERIOD_1_S)
								{
										m_led.state_cycle[0] = 0;
										m_module.low_power_led = POWER_LOW_CONSTANT_OFF;
										GW_led_color_update();
								}
						}
						else if(m_module.low_power_led == POWER_LOW_CONSTANT_OFF) {
								if(m_led.state_cycle[0] >= POWER_LOW_PERIOD_2_S)
								{
										m_led.state_cycle[0] = 0;
										m_module.low_power_led = POWER_LOW_BLINKY;
										GW_led_color_update();
								}
						}
						break;
				case LED_CONNECTED_SUCCESS_PEND:
						if(m_led.state_cycle[0] >= POWER_CONNECTED_SUCCESS_TIME_S) {       // Blinky three times to enter paired-idle
								led_state_pop(LED_CONNECTED_SUCCESS_PEND);
								system_state_update(STATE_PAIR_SUCCESS_E);
						}
						break;
				case LED_PAIR_FAIL_BLINKY:
						if(m_led.state_cycle[0] >= POWER_PAIR_FAIL_TIME_S) {       // Blinky three times to enter paired-idle
								led_state_pop(LED_PAIR_FAIL_BLINKY);
								system_state_update(STATE_PAIR_FAIL_E);
						}
						break;
				case LED_STANDBY_AWAKE:
						if(m_led.state_cycle[0] >= POWER_STANDBY_AWAKE_TIME_S) {       // Blinky three times to enter paired-idle
								led_state_pop(LED_STANDBY_AWAKE);
								system_state_update(STATE_PAIR_AWAKE_E);
						}
						break;
		}
}

void system_state_update(uint8_t state)
{
		if(state == STATE_EVENT_UPDATED)
				m_module.state_last = STATE_EVENT_UPDATED;
		else
				m_module.state = state;

		if(m_module.state_last == m_module.state)
				return;

		uint8_t state_last = m_module.state, color_update = YES;

		switch(m_module.state)
		{
				case STATE_ULTRA_LOW_POWER_OFF:
						m_module.power_down = POWER_DOWN_NOW;
						break;
				case STATE_POWER_ON_S:
						m_module.off_range = YES;
						m_module.power_init = YES;
						led_state_push(LED_POWER_ON_PEND);
						state_last = STATE_POWER_ON_W;             // Move to next state and do nothing
						break;
				case STATE_POWER_ON_W:
						break;
				case STATE_POWER_ON_E:
						m_module.first_boot = YES;//m_module.wired_pair? NO: YES;
						//m_module.off_range = m_module.wired_pair? NO: YES;
						m_module.power_init = NO;
						GW_host_check(NULL);
						break;
				case STATE_POWER_OFF_S:
						m_module.power_init = YES;
						m_module.power_down = POWER_DOWN_READY_LED;
						state_last = STATE_POWER_OFF_W;            // Move to next state and do nothing
						led_state_pop_all();
						led_state_push(LED_POWER_OFF_PEND);
						break;
				case STATE_POWER_OFF_W:
						break;
				case STATE_POWER_OFF_E:
						m_module.power_init = NO;
						m_module.power_down = POWER_DOWN_NOW;
						break;
				case STATE_PAIR_RX_INIT:
						m_module.off_range = YES;
						m_module.mute = NO;
						m_module.standby_work = NO;
						m_module.standby = STATE_STANDBY_IDLE;
						m_module.wireless_pair = PAIR_STATE_RETRY;
				case STATE_PAIR_RX_START:
				case STATE_PAIR_RX_WAIT:
						break;
				case STATE_PAIR_SUCCESS_S:
						aw5808_rx_init();
						//m_module.off_range = NO;
						m_module.wireless_pair = PAIR_STATE_TRYING;
						led_state_push(LED_CONNECTED_SUCCESS_PEND);
						system_standby_restart();
						state_last = STATE_PAIR_SUCCESS_W;         // Move to next state and do nothing
						break;
				case STATE_PAIR_SUCCESS_W:
						break;
				case STATE_PAIR_SUCCESS_E:
						m_module.wireless_pair = PAIR_STATE_SUCCESS;
						break;
				case STATE_PAIR_FAIL_S:
						if(m_module.charging == NO) {
								if(m_module.low_power == YES) {            // Keep and try pair till ultra-low power
										m_module.power_down = POWER_DOWN_READY;
										color_update = NO;
								}
								else {
										m_module.wireless_pair = PAIR_STATE_FAIL;
										led_state_push(LED_PAIR_FAIL_BLINKY);
								}
								state_last = STATE_PAIR_FAIL_W;            // Move to next state and do nothing
						}
						break;
				case STATE_PAIR_FAIL_W:
						break;
				//wait for the DSP play tone is done
				case STATE_PAIR_FAIL_E:
						m_module.pair_fail_power_down = YES;
						if(m_module.fw_upgrade_ats3607 == NO && m_module.fw_upgrade_aw5808 == NO)
								m_module.power_down = POWER_DOWN_READY;
						break;
				case STATE_USB_CONNECTED:
						if(m_module.usb_connected == NO) {
								system_standby_restart();
								system_unpair_restart();
						}
						m_module.usb_connected = YES;
						break;
				case STATE_USB_DISCONNECT:
						if(m_module.usb_connected == YES) {
								system_standby_restart();
								system_unpair_restart();
						}
						m_module.usb_connected = NO;
						break;
				case STATE_BATTERY_CHARGING:
						m_module.charge_status = BATTERY_EVENT_CHARGED;
						m_module.charging = YES;
						system_standby_restart();
						break;
				case STATE_BATTERY_DISCHARGE:
						m_module.charge_status = BATTERY_EVENT_DISCHARGE;
						m_module.charging = NO;
						break;
				case STATE_BATTERY_FULL_CHARGE:
						m_module.charge_status = BATTERY_EVENT_FULL_CHARGED;
						if(m_module.charging == YES)
								system_standby_restart();
						m_module.charging = NO;
						m_module.ultralow_power = NO;
						m_module.low_power = NO;
						break;
				case STATE_BATTERY_ULTRALOW_POWER:
						// Ultra low power force to shut down if no charging
						if(++m_module.battery_level_count > POWER_ULTRALOW_CHECK_COUNT) {
								m_module.battery_level_count = 0;
								if(m_module.charging == NO) {
										if(m_module.ultralow_power == NO) {
												if(m_module.system_state == SYSTEM_STATE_IN_CALL && m_module.off_range == NO && m_module.mute == YES) {
														led_state_pop_all();
														led_state_push(LED_POWER_ULTRALOW_BLINKY);
												}
												else {
														m_module.power_down = POWER_DOWN_NOW;
												}
										}
										m_module.ultralow_power = YES;
										m_module.low_power = YES;
								}
						}
						else
								color_update = NO;
						break;
				case STATE_BATTERY_LOW_POWER:
						if(++m_module.battery_level_count > POWER_LOW_CHECK_COUNT) {
								m_module.power_check = YES;
								m_module.battery_level_count = 0;
								m_module.ultralow_power = NO;
								m_module.low_power = YES;
						}
						else
								color_update = NO;
						break;
				case STATE_BATTERY_MEDIUM_POWER:
				case STATE_BATTERY_HIGH_POWER:
						//led_state_pop(LED_POWER_LOW_BLINKY);
						m_module.battery_level_count = 0;
						m_module.ultralow_power = NO;
						m_module.low_power = NO;
						break;
				case STATE_CONNECTED_DEVICE_TYPE:
						color_update = GW_host_check(CCH_SYNC_DEVICE_TYPE_SYNC_TYPE_BIT_FIELD);
						break;
				case STATE_CONNECTED_SYSTEM_NORMAL:
						if(m_module.system_state == SYSTEM_STATE_IN_CALL)
								system_standby_restart();
						m_module.system_state = SYSTEM_STATE_NORMAL;
						color_update = GW_host_check(CCH_SYNC_SYSTEM_COMMAND_TYPE_BIT_FIELD);
						break;
				case STATE_CONNECTED_SYSTEM_STANDBY:
						m_module.system_state = SYSTEM_STATE_STANDBY;
						color_update = GW_host_check(CCH_SYNC_SYSTEM_COMMAND_TYPE_BIT_FIELD);
						break;
				case STATE_CONNECTED_SYSTEM_INACTIVE:
						if(m_module.system_state != SYSTEM_STATE_INACTIVE)
								m_debug.inactive_count_test++;
						m_module.system_state = SYSTEM_STATE_INACTIVE;
						color_update = GW_host_check(CCH_SYNC_SYSTEM_COMMAND_TYPE_BIT_FIELD);
						break;
				case STATE_CONNECTED_SYSTEM_IN_CALL:
						m_module.standby = STATE_STANDBY_IDLE;
						m_module.system_state = SYSTEM_STATE_IN_CALL;
						system_standby_restart();
						color_update = GW_host_check(CCH_SYNC_SYSTEM_COMMAND_TYPE_BIT_FIELD);
						break;
				case STATE_CONNECTED_MUTE:
						if(m_module.mute == NO) {
								uint8_t dsp_mute[] = {DSP_MAGIC, DSP_CMD_FLAG, 0x04, DSP_CMD_SLAVE_MIC_MUTE, 0x01, 0xB3};
								GW_send_cmd_to_3607D(dsp_mute, sizeof(dsp_mute));
						}
						m_module.mute = YES;
						color_update = GW_host_check(CCH_SYNC_AUDIO_VOLUME_SYNC_TYPE_BIT_FIELD);
						break;
				case STATE_CONNECTED_UNMUTE:
						if(m_module.mute == YES) {
								uint8_t dsp_unmute[] = {DSP_MAGIC, DSP_CMD_FLAG, 0x04, DSP_CMD_SLAVE_MIC_MUTE, 0x00, 0xB2};
								GW_send_cmd_to_3607D(dsp_unmute, sizeof(dsp_unmute));
						}
						m_module.mute = NO;
						color_update = GW_host_check(CCH_SYNC_AUDIO_VOLUME_SYNC_TYPE_BIT_FIELD);
						break;
				case STATE_PAIR_STANDBY_LED_BREEZE:
						m_module.standby = STATE_STANDBY_LED_BREEZE;
						break;
				case STATE_PAIR_STANDBY_LED_OFF:
						m_module.standby = STATE_STANDBY_LED_OFF;
						break;
				case STATE_PAIR_STANDBY_LED_POWER_OFF:
						m_module.standby_work = YES;
						GW_standby_mode_enter();
						break;
				case STATE_PAIR_AWAKE_S:
						if(m_module.standby != STATE_STANDBY_LED_AWAKE) {
								aw5808_rx_init();
								led_state_pop_all();
								led_state_push(LED_STANDBY_AWAKE);
						}
						m_module.standby = STATE_STANDBY_LED_AWAKE;
						m_module.standby_work = NO;
						m_module.battery_level_count = 0;
						m_module.low_power = NO;
						m_module.ultralow_power = NO;
						m_module.power_check = NO;
						state_last = STATE_PAIR_AWAKE_W;           // Move to next state and do nothing
						aw5808_rx_data.online = 0xFF;
						break;
				case STATE_PAIR_AWAKE_W:
						break;
				case STATE_PAIR_AWAKE_E:
						m_module.standby = STATE_STANDBY_IDLE;
						if(m_cch_rx.pair.steps == CCH_PAIR_SUCCESS) {
								m_module.dsp_mode = DSP_MODE_SINGLE;
								GW_send_DSP_tone(DSP_TONE_PAIR_OK_CN);
						}
						break;
				case STATE_CONNECTED_OUT_RANGE:
						m_module.system_state = SYSTEM_STATE_IDLE;
						m_debug.range_out_count++;
						if(m_cch_rx.connected == YES)
								system_standby_restart();
						m_cch_rx.connected = NO;
						m_cch_rx.status = 0x00;
						m_module.off_range = YES;

						wireless_who = m_module.who = WHO_IS_NO;
						m_module.dsp_mode = DSP_MODE_INIT;
						aw5808_rx_init();
						//GW_5G_CCH_pop_all();
						break;
				case STATE_CONNECTED_IN_RANGE:
						m_module.system_state = SYSTEM_STATE_IDLE;
						m_debug.range_in_count++;
						m_cch_rx.connected = YES;
						if(m_module.off_range == YES) {
								color_update = NO;
								m_cch_rx.state_init = 0;
								system_standby_restart();
								GW_5G_CCH_pop_all();
								GW_5G_CCH_push(CCH_SYNC_DEVICE_TYPE_SYNC_TYPE);
								GW_5G_CCH_push(CCH_SYNC_SYSTEM_COMMAND_TYPE);
								GW_5G_CCH_push(CCH_SYNC_AUDIO_VOLUME_SYNC_TYPE);
						}
						m_module.off_range = NO;
						if(m_module.standby != STATE_STANDBY_IDLE)
								sys_evt.paired_awake(AWAKE_CALL);

						if(m_module.standby != STATE_STANDBY_LED_AWAKE)
								m_module.standby = STATE_STANDBY_IDLE;
						break;
				case STATE_UP_PORT_CONNECTED:
						m_module.off_range = NO;
						break;
				case STATE_UP_PORT_DISCONNECT:
						m_module.off_range = YES;			// Reserve for test
						break;
				case STATE_FIRMWARE_UPGRADE_AW5808_START:
						m_module.fw_upgrade_aw5808 = YES;
						break;
				case STATE_FIRMWARE_UPGRADE_AW5808_STOP:
						m_module.fw_upgrade_aw5808 = NO;
						break;
				case STATE_FIRMWARE_UPGRADE_ATS3607_START:
						m_module.fw_upgrade_ats3607 = YES;
						break;
				case STATE_FIRMWARE_UPGRADE_ATS367_STOP:
						m_module.fw_upgrade_ats3607 = NO;
						break;
				case STATE_FIRMWARE_UPGRADE_MCU_START:
						m_module.fw_upgrade_mcu = YES;
						break;
				case STATE_FIRMWARE_UPGRADE_MCU_STOP:
						m_module.fw_upgrade_mcu = NO;
						break;
				default:
						color_update = NO;
						break;
		}

		m_module.state_last = state_last;
		if(color_update)
				GW_led_color_update();
}

void GW_led_color_update(void)
{
		uint8_t * p_side_color = DOA_RGB + IDLE_R_F, * p_center_color = DOA_RGB + IDLE_R_F, led_mode, hz = 1, color_state_mode = LED_STATE_IDLE;

		if(m_module.power_down == POWER_DOWN_NOW) {
				p_side_color = DOA_RGB + DOA_R_OFF;
				p_center_color = DOA_RGB + DOA_R_OFF;
				led_mode = LED_STATUS_CONSTANT_MODE;
				color_state_mode = LED_STATE_POWER_DOWN;
		}
		else if(m_module.ultralow_power == YES && m_module.low_power == YES) {
				hz = 3;
				p_side_color = DOA_RGB + MUTE_R_F;
				p_center_color = DOA_RGB + MUTE_R_F;
				led_mode = LED_STATUS_BLINKY_MODE;
				color_state_mode = LED_STATE_ULTRALOW_POWER_BLINKY;
		}
		else {
				if(m_module.power_init /*|| m_module.first_boot*/) {
						p_side_color = DOA_RGB + IDLE_R_F;
						if(m_module.power_init) {
								led_mode = LED_STATUS_BLINKY_MODE;
								color_state_mode = LED_STATE_POWER_ON;
						}
						else {
								led_mode = LED_STATUS_CONSTANT_MODE;
								color_state_mode = LED_STATE_POWER_ON_FIRST_BOOT;
						}
				}
				else {
						if(m_module.standby != STATE_STANDBY_IDLE) {
								if(m_module.standby_work == YES) {
										p_side_color = DOA_RGB + DOA_R_OFF;
										p_center_color = DOA_RGB + DOA_R_OFF;
										led_mode = LED_STATUS_CONSTANT_MODE;
										color_state_mode = LED_STATE_STANDBY_POWER_DOWN;
								}
								else {
										if(m_module.standby == STATE_STANDBY_LED_AWAKE) {
												p_side_color = DOA_RGB + IDLE_R_F;
												led_mode = LED_STATUS_BLINKY_MODE;
												color_state_mode = LED_STATE_STANDBY_POWER_ON;
										}
										else if(m_module.standby == STATE_STANDBY_LED_OFF) {
												p_side_color = DOA_RGB + DOA_R_OFF;
												p_center_color = DOA_RGB + DOA_R_OFF;
												led_mode = LED_STATUS_CONSTANT_MODE;
												color_state_mode = LED_STATE_STANDBY_BREEZE;
										}
										else if(m_module.standby == STATE_STANDBY_LED_BREEZE || m_module.standby_evt == STANBY_PAIRED_AND_CHARGING_WITHOUT_CONNECTION) {
												hz = LED_COLOR_UPDATE_RATE_MASK | 2;
												p_side_color = DOA_RGB + IDLE_R_F;
												led_mode = LED_STATUS_BREEZE_MODE;
												color_state_mode = LED_STATE_STANDBY_OFF;
										}
								}
						}
						else {
								if(m_module.fw_upgrade_aw5808 || m_module.fw_upgrade_ats3607 || m_module.fw_upgrade_mcu) {
										p_side_color = DOA_RGB + FW_FLASH_R;
										led_mode = LED_STATUS_CONSTANT_MODE;
										color_state_mode = LED_STATE_FIRMWARE_UPGRADE;
								}
								else {
										if(m_module.wireless_pair == PAIR_STATE_FAIL) {
												if(m_module.pair_fail_power_down == YES) {
														p_side_color = DOA_RGB + IDLE_R_F;
														led_mode = LED_STATUS_CONSTANT_MODE;
														color_state_mode = LED_STATE_PAIR_FAIL_POWER_DOWN;
												}
												else {
														p_side_color = DOA_RGB + PAIR_FAIL_R;
														led_mode = LED_STATUS_CONSTANT_MODE;
														color_state_mode = LED_STATE_PAIR_FAIL;
												}
										}
										else if(m_module.wireless_pair == PAIR_STATE_SUCCESS || m_module.wired_module == YES) {
												if(m_module.off_range) {
														p_side_color = DOA_RGB + WARRING_R_F;
														led_mode = LED_STATUS_BLINKY_MODE;
														color_state_mode = LED_STATE_CONNECTION_FAIL;
												}
												else {
														if(m_module.system_state == SYSTEM_STATE_NORMAL) {
																p_side_color = DOA_RGB + IDLE_R_F;
																led_mode = m_module.charging || m_module.low_power? LED_STATUS_BLINKY_MODE: LED_STATUS_CONSTANT_MODE;
																color_state_mode = LED_STATE_CONNECTION_IDLE;
														}
														else if(m_module.system_state == SYSTEM_STATE_STANDBY) {
																p_side_color = DOA_RGB + PAIR_R_F;
																led_mode = LED_STATUS_CONSTANT_MODE;
																color_state_mode = LED_STATE_CONNECTION_STANDBY;
														}
														else if(m_module.system_state == SYSTEM_STATE_INACTIVE) {
																p_side_color = DOA_RGB + WARRING_R_F;
																led_mode = m_module.charging || m_module.low_power? LED_STATUS_BLINKY_MODE: LED_STATUS_CONSTANT_MODE;
																color_state_mode = LED_STATE_CONNECTION_INACTIVE;
														}
														else if(m_module.system_state == SYSTEM_STATE_IN_CALL) {
																if(m_module.mute) {
																		p_side_color = DOA_RGB + MUTE_R_F;
																		p_center_color = DOA_RGB + MUTE_R_F;
																		led_mode = m_module.charging? LED_STATUS_BLINKY_MODE: LED_STATUS_CONSTANT_MODE;
																		color_state_mode = LED_STATE_CONNECTION_MUTE;
																}
																else {
																		hz = m_module.charging? LED_COLOR_UPDATE_RATE_MASK | 2: 1;
																		p_side_color = DOA_RGB + CALL_R_F;
																		led_mode = m_module.charging? LED_STATUS_BREEZE_MODE: m_module.low_power? LED_STATUS_BLINKY_MODE: LED_STATUS_CONSTANT_MODE;
																		color_state_mode = LED_STATE_CONNECTION_CALL;
																}
														}
												}
										}
										else {
												if(m_module.wireless_pair == PAIR_STATE_TRYING) {
														led_mode = LED_STATUS_CONSTANT_MODE;
														p_side_color = m_module.off_range? DOA_RGB + WARRING_R_F: DOA_RGB + PAIR_R_F;
														color_state_mode = LED_STATE_PAIR_SUCCESS;
												}
												else {
														led_mode = LED_STATUS_BLINKY_MODE;
														p_side_color = DOA_RGB + PAIR_R_F;
														color_state_mode = LED_STATE_PAIR_RETRY;
												}
										}
								}
						}
				}
		}

		if(led_mode != m_led.led_mode || color_state_mode != m_led.color_state) {
				uint8_t _hz = hz&(~LED_COLOR_UPDATE_RATE_MASK);
				memcpy(m_led.side_colors, p_side_color, sizeof(m_led.side_colors));
				memcpy(m_led.center_colors, p_center_color, sizeof(m_led.center_colors));
				m_led.mode = led_mode;
				m_led.hz = _hz;
				m_led.cycle_step = LED_HIGH_LOW_SAMPLE_RATE/m_led.hz;
				m_led.init = YES;
				m_led.init_step_ms = led_mode == LED_STATUS_CONSTANT_MODE? 1: hz&LED_COLOR_UPDATE_RATE_MASK? (LED_HIGH_LOW_SAMPLE_PERIOD_MS*_hz): (LED_HIGH_LOW_SAMPLE_PERIOD_MS/_hz);
				m_led.step_ms = 0;

				m_debug.led_init_time_ms = m_module.current_time_ms;
				m_debug.led_begin_time_ms = 0;

				m_debug.led_count[color_state_mode]++;
				if(color_state_mode == LED_STATE_POWER_DOWN)                  PrintMsg__("LED: Power-down! \r\n");
				else if(color_state_mode == LED_STATE_LOW_POWER_BLINKY)       PrintMsg__("LED: Low-power blinky! \r\n");
				else if(color_state_mode == LED_STATE_LOW_POWER_CONSTANT)     PrintMsg__("LED: Low-power blinky-off! \r\n");
				else if(color_state_mode == LED_STATE_POWER_ON)               PrintMsg__("LED: Power-init! \r\n");
				else if(color_state_mode == LED_STATE_POWER_ON_FIRST_BOOT)    PrintMsg__("LED: First-boot! \r\n");
				else if(color_state_mode == LED_STATE_STANDBY_POWER_DOWN)     PrintMsg__("LED: Standby power-down! \r\n");
				else if(color_state_mode == LED_STATE_STANDBY_POWER_ON)       PrintMsg__("LED: Standby awake! \r\n");
				else if(color_state_mode == LED_STATE_STANDBY_BREEZE)         PrintMsg__("LED: Standby off! \r\n");
				else if(color_state_mode == LED_STATE_STANDBY_OFF)            PrintMsg__("LED: Standby breeze! \r\n");
				else if(color_state_mode == LED_STATE_FIRMWARE_UPGRADE)       PrintMsg__("LED: Upgrading! \r\n");
				else if(color_state_mode == LED_STATE_PAIR_FAIL_POWER_DOWN)   PrintMsg__("LED: Pair-fail power-down\r\n");
				else if(color_state_mode == LED_STATE_PAIR_FAIL)              PrintMsg__("LED: Pair-fail \r\n");
				else if(color_state_mode == LED_STATE_CONNECTION_FAIL)        PrintMsg__("LED: Off-range! \r\n");
				else if(color_state_mode == LED_STATE_CONNECTION_IDLE)        PrintMsg__("LED: Paired idle%s \r\n", m_module.charging? " & charging": "!");
				else if(color_state_mode == LED_STATE_CONNECTION_STANDBY)     PrintMsg__("LED: Standby%s \r\n", m_module.charging? " & charging": "!");
				else if(color_state_mode == LED_STATE_CONNECTION_INACTIVE)    PrintMsg__("LED: Inactivae! \r\n");
				else if(color_state_mode == LED_STATE_CONNECTION_MUTE)        PrintMsg__("LED: Mute%s \r\n", m_module.charging? " & charging": "!");
				else if(color_state_mode == LED_STATE_CONNECTION_CALL)        PrintMsg__("LED: In-call%s \r\n", m_module.charging? " & charging": "!");
				else if(color_state_mode == LED_STATE_PAIR_SUCCESS)           PrintMsg__("LED: Pair-trying \r\n");
				else if(color_state_mode == LED_STATE_PAIR_RETRY)             PrintMsg__("LED: Pair-retry \r\n");
		}
		m_led.led_mode = led_mode;
		m_led.color_state = color_state_mode;
}

void GW_led_constant_set(uint8_t * p_color)
{
		m_led.cycle_step = 0;
		m_led.init = YES;
		m_led.mode = LED_STATUS_CONSTANT_MODE;
		m_led.update_timeout_ms = 0;
		m_led.step_ms = 0;
		m_led.init_step_ms = 1;
		memcpy(m_led.side_colors, p_color, sizeof(m_led.side_colors));
}

uint8_t GW_led_manager(void)
{
		uint8_t	i, lde_adjust = NO, breeze_colors[3], * p_side_color, * p_center_colors = m_led.center_colors;

		if(factory_mode == YES) {
				for(i=1;i<=DOA_LED_total;i++) {
						if(factory_led_cross == 0)
								set_LED_RGB(LED_DOA_source, i, factory_led_set[0], factory_led_set[1], factory_led_set[2]);
						else if(factory_led_cross == 1) {
								if(i%2)
										set_LED_RGB(LED_DOA_source, i, DOA_RGB[DOA_R_OFF], DOA_RGB[DOA_R_OFF], DOA_RGB[DOA_R_OFF]);
								else
										set_LED_RGB(LED_DOA_source, i, DOA_RGB[IDLE_R_F], DOA_RGB[IDLE_R_F], DOA_RGB[IDLE_R_F]);
						}
						else if(factory_led_cross == 2) {
								if(i%2)
										set_LED_RGB(LED_DOA_source, i, DOA_RGB[IDLE_R_F], DOA_RGB[IDLE_R_F], DOA_RGB[IDLE_R_F]);
								else
										set_LED_RGB(LED_DOA_source, i, DOA_RGB[DOA_R_OFF], DOA_RGB[DOA_R_OFF], DOA_RGB[DOA_R_OFF]);
						}
				}
				return YES;
		}

		if(m_led.init) {
				m_led.step_ms = m_led.init_step_ms;
				m_led.init = NO;
				m_led.cycle_index = 0;
				m_led.second_index = 0;
				m_led.update_timeout_ms = 0;
				p_side_color = DOA_RGB + DOA_R_OFF;
				lde_adjust = YES;
				goto led_manager_exit;
		}

		m_led.second_index++;
		m_led.cycle_index++;
		if(m_led.cycle_index == 1 && m_debug.led_begin_time_ms == 0)
				m_debug.led_begin_time_ms = m_module.current_time_ms - m_debug.led_init_time_ms;

		if(m_led.mode == LED_STATUS_CONSTANT_MODE) {
				p_side_color = m_led.side_colors;
				lde_adjust = YES;
				m_led.mode = LED_STATUS_CONSTANT_RETAIN_MODE;
				m_led.step_ms = 100;
				m_led.second_index = 0;
		}
		else if(m_led.mode == LED_STATUS_BLINKY_MODE) {
				p_side_color = &DOA_RGB[DOA_R_OFF];
				if(m_led.cycle_index == 1) {
						p_side_color = m_led.side_colors;
						lde_adjust = YES;
				}
				if(m_led.cycle_index == ((LED_HIGH_LOW_SAMPLE_RATE*LED_BLINKY_ON_DUTY)/LED_BLINKY_ONOFF_DUTY)) {
						p_side_color = &DOA_RGB[DOA_R_OFF];
						lde_adjust = YES;
				}

//PrintMsg__(": %3d %3d %3d %3d %3d %3d\r\n", m_led.cycle_index, ((LED_HIGH_LOW_SAMPLE_RATE*LED_BLINKY_ON_DUTY)/LED_BLINKY_ONOFF_DUTY), p_side_color[0], lde_adjust, m_led.second_index, m_led.step_ms);
				if(m_led.cycle_index == LED_HIGH_LOW_SAMPLE_RATE)
						m_led.cycle_index = 0;

				if(m_led.second_index*m_led.step_ms >= TIMER0_FREQ) {
						m_led.second_index = 0;
						sys_evt.led_second_timeout_cb(LED_STATUS_BLINKY_MODE);
				}
		}
		else if(m_led.mode == LED_STATUS_BREEZE_MODE) {
#define BREEZE_COLOR_RECALC(step, x)			((5*step*x) >> 7)
				uint8_t recalc_cycle_index = m_led.cycle_index <= DOA_FPS? m_led.cycle_index: LED_HIGH_LOW_SAMPLE_RATE - m_led.cycle_index;

				breeze_colors[0] = BREEZE_COLOR_RECALC(recalc_cycle_index, m_led.side_colors[0]);
				breeze_colors[1] = BREEZE_COLOR_RECALC(recalc_cycle_index, m_led.side_colors[1]);
				breeze_colors[2] = BREEZE_COLOR_RECALC(recalc_cycle_index, m_led.side_colors[2]);
				p_side_color = breeze_colors;
//PrintMsg__(": %3d %3d %3d %3d \r\n", m_led.cycle_index, recalc_cycle_index, breeze_colors[0], m_led.second_index);

				lde_adjust = YES;

				if(m_led.cycle_index == LED_HIGH_LOW_SAMPLE_RATE)
						m_led.cycle_index = 0;

				if(m_led.second_index*m_led.step_ms >= TIMER0_FREQ) {
						m_led.second_index = 0;
						sys_evt.led_second_timeout_cb(LED_STATUS_BREEZE_MODE);
				}
		}
		else if(m_led.mode == LED_STATUS_CONSTANT_RETAIN_MODE) {
				if(m_led.second_index*m_led.step_ms >= TIMER0_FREQ) {
						m_led.second_index = 0;
						sys_evt.led_second_timeout_cb(LED_STATUS_CONSTANT_RETAIN_MODE);
				}
		}

led_manager_exit:
		if(lde_adjust) {
				for(i=1;i<=DOA_LED_total;i++) {
						if(i >= 7) {
								m_led.center_color_now = p_center_colors[0];
								set_LED_RGB(LED_DOA_source, i, p_center_colors[0], p_center_colors[1], p_center_colors[2]);
						}
						else {
								m_led.side_color_now = p_side_color[0];
								set_LED_RGB(LED_DOA_source, i, p_side_color[0], p_side_color[1], p_side_color[2]);
						}
				}
		}
		return lde_adjust;
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                        SPI0 DMA transfer                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void provide_LED_data(void)
{
		if(GPO_PD02_MCU_PWREN == 0)
				return;

		m_led.update_count++;

		/* Set transfer width (8 bits) and transfer count */
		PDMA_SetTransferCnt(PDMA, SPI_TX_DMA_CH, PDMA_WIDTH_8, DOA_size);
		/* Set source/destination address and attributes */
		PDMA_SetTransferAddr(PDMA, SPI_TX_DMA_CH, (uint32_t)LED_DOA_source, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);	
}

void GW_LED_dma_transfer(void)
{
		uint8_t		i;
		uint32_t u32RegValue;

		if(m_module.led_lock == YES)
				return;

#ifdef LED_MANAGER_TEST
		if(GW_led_manager() == NO)
				return;
#endif
		/* Get interrupt status */
		u32RegValue = PDMA_GET_INT_STATUS(PDMA);

		/* Check the PDMA transfer done interrupt flag */
		if (u32RegValue & PDMA_INTSTS_TDIF_Msk) {
				/* Check the PDMA transfer done flags */
				if ((PDMA_GET_TD_STS(PDMA) & (1 << SPI_TX_DMA_CH)) == (1 << SPI_TX_DMA_CH)) {
						/* Clear the PDMA transfer done flags */
						PDMA_CLR_TD_FLAG(PDMA, (1 << SPI_TX_DMA_CH));

						/* Disable SPI0 master's PDMA transfer function */
						SPI_DISABLE_RX_PDMA(SPI0);
						provide_LED_data();

						/* Set request source; set basic mode. */
						PDMA_SetTransferMode(PDMA, SPI_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);

						/* Enable SPI slave DMA function */
						SPI_TRIGGER_TX_RX_PDMA(SPI0);
				}
		}
}

/*---------------------------------------------------------------------------------------------------------*/
/*                           Timer handle                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t PAIR_WAIT_TMRINT_count = 0;
volatile uint32_t DSP_READY_TMRINT_count = 0;

volatile uint32_t I2C_READY_TMRINT_count = 0;
volatile uint32_t UI2C_READY_TMRINT_count = 0;

volatile uint32_t RESET_TMRINT_count = 0;

uint32_t aw5808_auto_test_count=0;
uint32_t usci_check_value=0;
uint32_t aw5808TX_TMRINTCount=0;
uint32_t aw5808RX_TMRINTCount=0;

uint32_t i2c0_check_start=0;
uint32_t i2c0_check_value=0;
uint32_t timer_ms_delay_count=0;
uint32_t i2c0_margin_value=50;

uint32_t check_pair_wait_count=0;
uint32_t unpair_wait_count=0;
volatile uint8_t get_usb_cmd = NO;
volatile uint32_t BATTERY_TMRINT_count = 0;
volatile uint32_t BATTERY_TMRINT_status_count = 0;
volatile uint32_t dfu_scan_time_count_2=0;
volatile uint32_t check_auto_connection_wait_count=0;

volatile uint32_t TEMPERATURE_TMRINT_count = 0;
volatile uint32_t wire_mic_sendout_count=0;

void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0))
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

				adc_count ++;
				adc_temp_count++;

        m_led.update_timeout_ms++;
				if(time_to_work(&m_led.update_timeout_ms, m_led.step_ms))
						GW_LED_dma_transfer();

				m_vdm.send_timeout_ms++;
				m_cch_rx.send_period_time_ticks_ms++;
				m_cch_rx.rx_access_timeout_ms++;
				m_cch_rx.time_ticks_ms++;
				m_cch_rx.pair.steps_timeout_ticks_ms++;
				m_cch_rx.pair.mode_check_timeout_ticks_ms++;
				m_cch_rx.pair.pairing_timeout_ticks_ms++;
				m_module.current_time_ms++;
				m_module.standby_timeout_ms++;
				m_module.sync_retry_timeout_ms++;
				m_module.button_power_timeout_ms++;
				m_module.button_mute_timeout_ms++;
				m_module.button_unpair_timeout_ms++;
				m_module.power_on_timeout_ms++;
				m_module.i2c_timeout_ms++;
				ats3607_update_used_ms++;
				timer_ms_delay_count++;
				g_au32AW5808INTCount[0]++;
				aw5808TX_TMRINTCount++;
				aw5808RX_TMRINTCount++;
				aw5808_auto_test_count++;
				PWOER_OFF_TMRINT_count ++;
				DSP_READY_TMRINT_count++;
				check_pair_wait_count++;
				unpair_wait_count++;
				I2C_READY_TMRINT_count ++;
				BATTERY_TMRINT_count++;
				BATTERY_TMRINT_status_count++;
				dfu_scan_time_count_2++;
				UI2C_READY_TMRINT_count++;
				check_auto_connection_wait_count++;
				TEMPERATURE_TMRINT_count++;
				RESET_TMRINT_count++;
				wire_mic_sendout_count++;

				if(aw5808_dfu_enable == 0)
				{
						if(usci_check_start == 1)
						{
								usci_check_value++;
								if(usci_check_value > 50)
								{
										usci_check_start = 0;
										stop_usci_i2c();
								}
						}	

						if(i2c0_check_start)
						{
								i2c0_check_value++;
								i2c0_margin_value = (enable_touch_update_process == 0)? 50: 130;
								if(i2c0_check_value > i2c0_margin_value)
								{
										i2c0_check_start = 0;
										stop_i2c0();
                }						
						}
				}	
    }
}

void UART1_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
		uint8_t new_end;

    u32IntStatus = UART1->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while ((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART1->DAT;
#ifdef for_console_menu
						//transfer data to menun key buffer
							new_end = menu_key.end;
							new_end ++;
							if(new_end == sizeof(menu_key.buf))
								new_end = 0;
							
							if(new_end == menu_key.start) {
								//buffer full
							} else {
								menu_key.buf[menu_key.start] = bInChar;
								menu_key.end = new_end;
							}
#endif
        }
    }

    if (u32IntStatus & UART_INTSTS_THREIF_Msk)
    {
            /* No more data, just stop Tx (Stop work) */
            UART1->INTEN &= (~UART_INTEN_THREIEN_Msk);
    }

}

void UART2_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
		uint8_t new_end;

    u32IntStatus = UART2->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while ((UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART2->DAT;
#if 0
						//transfer data to menun key buffer
							new_end = menu_key.end;
							new_end ++;
							if(new_end == sizeof(menu_key.buf))
								new_end = 0;
							
							if(new_end == menu_key.start) {
								//buffer full
							} else {
								menu_key.buf[menu_key.start] = bInChar;
								menu_key.end = new_end;
							}
#endif
        }
    }

    if (u32IntStatus & UART_INTSTS_THREIF_Msk)
    {
            /* No more data, just stop Tx (Stop work) */
            UART2->INTEN &= (~UART_INTEN_THREIEN_Msk);
    }

}

void Uart_MPF(void)
{
#ifdef EN_UART0	
    /* Set GPA multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA6MFP_Msk) | SYS_GPA_MFPL_PA6MFP_UART0_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA7MFP_Msk) | SYS_GPA_MFPL_PA7MFP_UART0_TXD;
#endif
    /* Set GPB multi-function pins for UART1 RXD and TXD */

    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA8MFP_Msk) | SYS_GPA_MFPH_PA8MFP_UART1_RXD;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA9MFP_Msk) | SYS_GPA_MFPH_PA9MFP_UART1_TXD;
	
		//GPIO_SetPullCtl(PA, BIT8, GPIO_PUSEL_PULL_UP);
		//PA->SMTEN |= GPIO_SMTEN_SMTEN8_Msk;	
	
    /* Set GPB multi-function pins for UART2 RXD and TXD */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB4MFP_Msk) | SYS_GPB_MFPL_PB4MFP_UART2_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk) | SYS_GPB_MFPL_PB5MFP_UART2_TXD;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, g_sLineCoding.u32DTERate);

    /* Enable UART0 RX Time-Out Interrupt and RX Data Available Interrupt */
    UART_EnableInt(UART0, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART1_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART1, 115200);

    /* Enable UART0 RX Time-Out Interrupt and RX Data Available Interrupt */
    UART_EnableInt(UART1, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		NVIC_EnableIRQ(UART1_IRQn);
}

void UART2_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART2_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART2, 115200);

    /* Enable UART0 RX Time-Out Interrupt and RX Data Available Interrupt */
    UART_EnableInt(UART2, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		NVIC_EnableIRQ(UART2_IRQn);
}

void SYS_LED_Init(void)
{
    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    
    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);	

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
		CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL = SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS ;

		/* Setup UART 0/1/2 multi-function pins */
		Uart_MPF();

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

		UART0_Init();
		UART1_Init();
		UART2_Init();
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
		SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, 3389830);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);
}

void set_LED_RGB(uint8_t *source, uint8_t LED_num , uint8_t R , uint8_t G, uint8_t B) {
  
  uint32_t ptr;
  uint8_t i;

  if((LED_num == 0) || (LED_num > DOA_LED_total)) {
    printf("Warring LED number :%d\n", LED_num);
    return;
  }

	// 0 : 100
	// 1 : 110
	// 800KHz, GRB , 3 * 8 * 3 = 72 bits = 3*3 bytes
	uint32_t value; //24bits for every R/G/B
		
	ptr = (LED_num - 1) * 9;
	
	//G
	value = 0;
	for(i=0; i<8; i++) {
		value <<= 3;
		value |= G & 0x80 ? 0x06 : 0x04;
		G <<= 1;
	}
	
	source[ptr+0] = (value & 0x00FF0000) >> 16;
	source[ptr+1] = (value & 0x0000FF00) >> 8;
	source[ptr+2] = (value & 0x000000FF);

	//R
	value = 0;
	for(i=0; i<8; i++) {
		value <<= 3;
		value |= R & 0x80 ? 0x06 : 0x04;
		R <<= 1;
	}
	
	source[ptr+3] = (value & 0x00FF0000) >> 16;
	source[ptr+4] = (value & 0x0000FF00) >> 8;
	source[ptr+5] = (value & 0x000000FF);	
	
	//B
	value = 0;
	for(i=0; i<8; i++) {
		value <<= 3;
		value |= B & 0x80 ? 0x06 : 0x04;
		B <<= 1;
	}
	
	source[ptr+6] = (value & 0x00FF0000) >> 16;
	source[ptr+7] = (value & 0x0000FF00) >> 8;
	source[ptr+8] = (value & 0x000000FF);	
	
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_INT0_IRQHandler(void)
{
    g_u32EadcInt0Flag = 1;
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);
}

void EADC_INT1_IRQHandler(void)
{
    g_u32EadcInt1Flag = 1;
    /* Clear the A/D ADINT1 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);
}

void EADC_INT2_IRQHandler(void)
{
    g_u32EadcInt2Flag = 1;
    /* Clear the A/D ADINT2 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);
}

void SYS_ADC_Init(void) {

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is PCLK1, set divider to 8, ADC clock is PCLK1/8 MHz */
    /* Note: The EADC_CLK speed should meet datasheet spec (<16MHz) and rules in following table.   */
    /* +--------------+------------------+                                                          */
    /* | PCLK divider | EADC_CLK divider |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 1            | 1, 2, 3, 4, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    /* | 2, 4, 8, 16  | 2, 4, 6, 8, ...  |                                                          */
    /* +--------------+------------------+                                                          */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

		/* Set PB.0 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk);
    //PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk);
    //PB->MODE &= ~(GPIO_MODE_MODE1_Msk);

    /* Configure the EADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB0MFP_Msk) | SYS_GPB_MFPL_PB0MFP_EADC0_CH0;
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB1MFP_Msk) | SYS_GPB_MFPL_PB1MFP_EADC0_CH1;

    /* Disable the digital input path to avoid the leakage current for EADC analog input pins. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0);  /* Disable PB0*/
    //GPIO_DISABLE_DIGITAL_PATH(PB, (uint32_t)(BIT0| BIT1));  /* Disable PB0 PB1*/
    //GPIO_DISABLE_DIGITAL_PATH(PB, BIT1);  /* Disable PB1*/
}

void GW_GPIO_init(void)
{	
		CLK_EnableModuleClock(GPA_MODULE);
		CLK_EnableModuleClock(GPB_MODULE);
		CLK_EnableModuleClock(GPC_MODULE);
		// Power-on pre-configure
		//CLK_EnableModuleClock(GPD_MODULE);
		CLK_EnableModuleClock(GPF_MODULE);
//#if 0	

#ifdef EN_UART0
#else
		GPIO_SetMode(PA, BIT6, GPIO_MODE_INPUT);
		GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
#endif	

		GPIO_SetMode(PC, BIT6, GPIO_MODE_INPUT);		//DSP_INT1,						3607D,				TBD
		GPIO_SetMode(PC, BIT7, GPIO_MODE_OUTPUT);		//Daisy_Chain1_EN,		high: enable
		GPIO_SetMode(PF, BIT2, GPIO_MODE_OUTPUT);		//Daisy_Chain2_EN,		high: enable
		GPIO_SetMode(PA, BIT10, GPIO_MODE_INPUT);	//TYPEC1_SW,					switch C,			UART/SPDIF switch

		GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);		//battery
		//GPIO_SetMode(PB, BIT1, GPIO_MODE_INPUT);		//temperature

		GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);		//USBC2_DFP_DET,			DFP(GL9510),	Hi:DFP USB connect
		GPIO_SetMode(PB, BIT6, GPIO_MODE_OUTPUT);		//TYPEC2_SW,					switch C,			UART/SPDIF switch
		GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);
		//for hardware version detection:  0(low): wireless   1(high):wire
		GPIO_SetMode(PA, BIT10, GPIO_MODE_INPUT);	
		PB9 = HIGH;
		GPIO_SetMode(PB, BIT9, GPIO_MODE_OUTPUT);		//5GRX_MS_SEL,				need high or float
		GPIO_SetMode(PB, BIT10, GPIO_MODE_INPUT);		//Wireless_SW#,				Pair button,	Lo:press
		GPIO_SetMode(PB, BIT11, GPIO_MODE_INPUT);		//PWR_SW#,						Power button,	Lo:press
		GPIO_SetMode(PA, BIT11, GPIO_MODE_OUTPUT);
		GPO_PA11_DSP_ONOFF = LOW;

		GPIO_SetMode(PC, BIT3, GPIO_MODE_INPUT);		//5GRX_INT1,					5G(Tx) CCH int, Lo:interrupt

		GPO_PC04_PD_DFP_INT = 0;
		GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);

		GPIO_SetMode(PC, BIT5, GPIO_MODE_INPUT);		//MCU_I2C0_INT,				CY8CMBR3108,	status change

		GPO_PF04_TYPEC_UART1_EN = 1;
		GPO_PF05_TYPEC_UART2_EN = 1;
		GPIO_SetMode(PF, BIT4, GPIO_MODE_OUTPUT);		//TYPEC_UART1_EN,			DFP1(UART),		Control PCA9306
		GPIO_SetMode(PF, BIT5, GPIO_MODE_OUTPUT);		//TYPEC_UART2_EN,			DFP2(UART),		Control PCA9306

		GPO_PC07_Daisy_Chain1_EN = 1;
		GPO_PF02_Daisy_Chain2_EN = 1;

		GW_mcu_switch();
		GW_host_switch();
}

void GW_message_update(void)
{
		uint8_t i;
		static uint32_t msg_last_time_ms = 0;
#if 0
		if(m_module.tx.lengh > 0) {
				uint8_t dsp_msg[64] = {0};
				for(i=0;i<m_module.tx.lengh;i++)
						sprintf((char *)dsp_msg + i*3, " %02X ", m_module.tx.buffer[i]);

				printf("\x1b[%d;%dHDSP Tx(%9d):%s\n", DSP_START_Y + m_debug.dsp_tx_log_index, DSP_START_X, m_module.current_time_ms, dsp_msg);
				m_debug.dsp_tx_log_index = ++m_debug.dsp_tx_log_index >= DSP_START_Y_COUNT? 0: m_debug.dsp_tx_log_index;

				PrintMsg__("DSP tx: %s\n", dsp_msg);
				m_module.tx.lengh = 0;
		}

		if(m_module.rx.lengh > 0) {
				uint8_t dsp_msg[256] = {0};
				for(i=0;i<m_module.rx.lengh;i++)
						sprintf((char *)dsp_msg + i*3, " %02X ", DSP_event_buf[i]);

				printf("\x1b[%d;%dH  Rx(%9d):%s\n", DSP_START_Y + m_debug.dsp_rx_log_index, DSP_START_X + 100, m_module.current_time_ms, dsp_msg);
				m_debug.dsp_rx_log_index = ++m_debug.dsp_rx_log_index >= DSP_START_Y_COUNT? 0: m_debug.dsp_rx_log_index;

				PrintMsg__("DSP rx: %s\n", dsp_msg);
				m_module.rx.lengh = 0;
		}
#endif
		if(m_module.current_time_ms > (msg_last_time_ms + 200)) {
				msg_last_time_ms = m_module.current_time_ms;
				printf("\x1b[%d;%dH[Who:%02d][DSP:%02d][Module:%s/%d St:%c %6s][Md:%02d][DOA m:%d s:%02X c:%02X n:%02X; t:%3d][Fwu:%d/%d][Sb:%d %4d s %8d ms][Bt evt:%d p:%4d m:%4d d:%4d]\n", \
				SELECT_Y + 3, GPIO_START_X , \
				m_module.who, m_module.dsp_mode, \
				m_module.wired_module? "Wd": "Wl", m_module.wired_module? m_module.off_range == NO: m_module.wireless_pair, \
				m_module.system_state == SYSTEM_STATE_NORMAL? 'N': m_module.system_state == SYSTEM_STATE_STANDBY? 'S': m_module.system_state == SYSTEM_STATE_INACTIVE? 'I':'C', \
				m_module.mute? "Mute": "Unmute", \
				m_led.color_state, \
				m_led.mode, m_led.side_color_now, m_led.center_color_now, m_led.update_count, m_debug.led_begin_time_ms, \
				m_module.fw_upgrade_aw5808, m_module.fw_upgrade_ats3607, \
				m_module.standby, m_module.standby_timeout_s, m_module.standby_timeout_ms, \
				m_module.button_evt, m_module.wired_module? 0: m_module.button_unpair_timeout_ms, m_module.button_mute_timeout_ms, m_module.wired_module? 0: m_module.button_power_timeout_ms \
				);

				if(m_module.wired_module == YES) {
						printf("\x1b[%d;%dH[Dbg:vdm s:%5d t:%5d rs:%5d to:%5d] ", \
						SELECT_Y + 4, GPIO_START_X , \
						m_vdm.vdm_send_count, m_vdm.vdm_spend_time_ms, m_vdm.vdm_resend_count, m_vdm.vdm_timeout_count \
						);
				}
				else {
						printf("\x1b[%d;%dH[%s St:%02X Cnt:%3d ID:%02X%02X%02X%02X | losync:%6d][Sz:%d Sn:%d Sp:%d][Dbg:cch t:%8d i:%5d r:%5d e:%5d][Ri:%5d Ro:%5d Rc:%5d Ic:%5d][Bat(%4d/%4d/%4d):%4d(%c) C:%2d St:%d/%c] ", \
						SELECT_Y + 4, GPIO_START_X , \
						m_cch_rx.connected? "Sync  ": "NoSync", aw5808_rx_data.status, m_cch_rx.status_count, m_cch_rx.pair.id[0], m_cch_rx.pair.id[1], m_cch_rx.pair.id[2], m_cch_rx.pair.id[3], m_cch_rx.cch_lose_sync_count, \
						m_cch_rx.size, m_cch_rx.seq_num, m_cch_rx.pair.steps, \
						m_debug.cch_int_time_ms, m_debug.cch_int_count, m_debug.cch_read_count, m_debug.cch_recv_count, \
						m_debug.range_in_count, m_debug.range_out_count, m_debug.reconnect_count, m_debug.inactive_count_test, \
						BAT_ULTRALOW_THRESHOLD_ADC, BAT_LOW_THRESHOLD_ADC, BAT_HIGH_THRESHOLD_ADC, m_module.battery_level, m_module.ultralow_power? 'U': m_module.low_power? 'L': 'N', m_module.battery_level_count, \
						m_module.usb_connected, m_module.charge_status == BATTERY_EVENT_CHARGED? 'C': m_module.charge_status == BATTERY_EVENT_DISCHARGE? 'D': 'F' \
						);
				}

				uint8_t i, led_count_msg[LED_STATE_NUMBER*5 + 1] = {0};
				for(i=0;i<LED_STATE_NUMBER;i++)
						sprintf((char *)led_count_msg + i*5, " %4d", m_debug.led_count[i]);

				printf("\x1b[%d;%dH[Led count:%s][Sys N:%5d S:%5d I:%5d C:%5d Q:%5d]\n", \
				SELECT_Y + 5, GPIO_START_X, \
				led_count_msg, \
				m_debug.normal_count, m_debug.standby_count, m_debug.inactive_count, m_debug.call_count, m_debug.err_seq_num_count \
				);
		}

		if(m_module.power_init == NO)
				check_LDROM_need_update((uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit);
#if 0
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 0, GPIO_START_X + 4, GPO_PD02_MCU_PWREN == 1 ? 'H' : 'L');	//MCU_PWREN
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 1, GPIO_START_X + 4, GPO_PD03_AMP_EN == 1 ? 'H' : 'L');	//AMP_EN
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 2, GPIO_START_X + 4, GPO_PC04_PD_DFP_INT == 1 ? 'H' : 'L');	//PD DFP INT
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 3, GPIO_START_X + 4, GPO_PC07_Daisy_Chain1_EN == 1 ? 'H' : 'L');
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 4, GPIO_START_X + 4, GPO_PF02_Daisy_Chain2_EN == 1 ? 'H' : 'L');
	
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 5, GPIO_START_X + 4, GPO_PF03_WWL_SW == 1 ? 'H' : 'L');	//WWL_SW
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 6, GPIO_START_X + 4, GPO_PA10_TYPEC1_SW == 1 ? 'H' : 'L');	//TYPEC1_SW
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 7, GPIO_START_X + 4, GPO_PB06_TYPEC2_SW == 1 ? 'H' : 'L');	//TYPEC2_SW
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 8, GPIO_START_X + 4, GPO_PB13_USBC_UDP_SEL0 == 1 ? 'H' : 'L');	//USBC_UDP_SEL0
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 9, GPIO_START_X + 4, GPO_PB12_USBC_UDP_SEL1 == 1 ? 'H' : 'L');	//USBC_UDP_SEL1
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 10, GPIO_START_X + 4, GPO_PB14_USB2SW == 1 ? 'H' : 'L');	//USB2SW
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 11, GPIO_START_X + 4, GPO_PA11_DSP_ONOFF == 1 ? 'H' : 'L');	//DSP_ONOFF
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 12, GPIO_START_X + 4, GPO_PF04_TYPEC_UART1_EN == 1 ? 'H' : 'L');		//TYPEC_UART1_EN
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 13, GPIO_START_X + 4, GPO_PF05_TYPEC_UART2_EN == 1 ? 'H' : 'L');		//TYPEC_UART2_EN
	
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 0, GPIO_START_X + 30, GPI_PB01_GL3525_C1_VBUS_DET == 1 ? 'H' : 'L');	//GL3525_C1_VBUS_DET
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 1, GPIO_START_X + 30, GPI_PB00_GL3525_C2_VBUS_DET == 1 ? 'H' : 'L');	//GL3525_C2_VBUS_DET
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 2, GPIO_START_X + 30, GPI_PC14_USBC1_UFP_ID == 1 ? 'H' : 'L');	//USBC1_UFP_ID

	printf("\x1b[%d;%dH%c", GPIO_START_Y + 4, GPIO_START_X + 30, PB8 == 1 ? 'H' : 'L');
	//printf("\x1b[%d;%dH%c", GPIO_START_Y + 5, GPIO_START_X + 30, PB7 == 1 ? 'H' : 'L');
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 5, GPIO_START_X + 30, GPO_PA11_DSP_ONOFF == 1 ? 'H' : 'L');

	printf("\x1b[%d;%dH%c", GPIO_START_Y + 3, GPIO_START_X + 30, GPO_PB15_5GTX_MS_SEL == 1 ? 'H' : 'L');	//5GTX_MS_SEL	

	printf("\x1b[%d;%dH%c", GPIO_START_Y + 6, GPIO_START_X + 30, GPI_PB03_USBC1_DFP_DET == 1 ? 'H' : 'L');	//USBC1_DFP_DET
	//printf("\x1b[%d;%dH%c", GPIO_START_Y + 7, GPIO_START_X + 30, GPI_PB02_USBC2_DFP_DET == 1 ? 'H' : 'L');	//USBC2_DFP_DET
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 8, GPIO_START_X + 30, GPI_PB09_5G_RX_MS_SEL == 1 ? 'H' : 'L');	//GPI_PB09_5G_RX_MS_SEL
	
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 9, GPIO_START_X + 30, GPI_PB10_Wireless_SW == 1 ? 'H' : 'L');	//Wireless_SW#
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 10, GPIO_START_X + 30, GPI_PB11_PWR_SW == 1 ? 'H' : 'L');	//PWR_SW#
	
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 0, GPIO_START_X + 55, GPI_PC02_5GTX_INT1 == 1 ? 'H' : 'L');	//5GTX_INT1
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 1, GPIO_START_X + 55, GPI_PC03_5GRX_INT1 == 1 ? 'H' : 'L');	//5GRX_INT1
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 2, GPIO_START_X + 55, GPI_PC05_MCU_I2C0_INT == 1 ? 'H' : 'L');	//MCU_I2C0_INT
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 3, GPIO_START_X + 55, GPI_PF15_SYS_FW_UPDATE == 1 ? 'H' : 'L');	//SYS_FW_UPDATE
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 4, GPIO_START_X + 55, GPI_PC06_DSP_INT1 == 1 ? 'H' : 'L');	//DSP_INT1
	
	printf("\x1b[%d;%dH%c", GPIO_START_Y + 5, GPIO_START_X + 55, GPI_PA05_WM8804B_I2C_INT == 1 ? 'H' : 'L');	//WM8804B_I2C_INT

	printf("\x1b[%d;%dH[%10d][%10d][%3d/%3d/%3d/%02x][%s][%3d]", \
	SELECT_Y + 0, GPIO_START_X , \
	m_module.button_power_timeout_ms, \
	PAIR_WAIT_TMRINT_count,  \
	ats3607d_in_start, \
	ats3607d_in_end, \
	DSP_event_ptr, \
	ats3607d_in_buf[ats3607d_in_start], \
	m_module.mute? " mute " : "unmute" \
	);

	printf("\x1b[%d;%dH[statellite:%s][W:%s][L:%s][DFP1 0x%02x:%c/%c/%c/%c/%c/%c][DFP2 0x%02x:%c/%c/%c/%c/%c/%c]", \
	SELECT_Y + 1, GPIO_START_X , \
	satellite_mode == WIRELESS_MODE ? "wireless" : "wire", \
	wireless_mux_sw == MASTER_5G ? "M" : "S", \
	wire_mux_sw == MASTER_WIRE ? "M" : "S", \
	DFP1_connect, \
	DFP1_connect & DFP_CONNECT ? 		'C':'-', \
	DFP1_connect & DFP_CABLE_PASS ? 'P':'-', \
	DFP1_connect & DFP_PD_IS_SPK ? 	'S':'-', \
	DFP1_connect & DFP_PD_IS_G2 ? 	'G':'-', \
	DFP1_connect & DFP_PD_IS_MIC ? 	'M':'-', \
	DFP1_connect & DFP_PD_IS_M ? 		'M':'S', \
	DFP2_connect, \
	DFP2_connect & DFP_CONNECT ? 		'C':'-', \
	DFP2_connect & DFP_CABLE_PASS ? 'P':'-', \
	DFP2_connect & DFP_PD_IS_SPK ? 	'S':'-', \
	DFP2_connect & DFP_PD_IS_G2 ? 	'G':'-', \
	DFP2_connect & DFP_PD_IS_MIC ? 	'M':'-', \
	DFP2_connect & DFP_PD_IS_M ? 		'M':'S' \
 );

		printf("\x1b[%d;%dH[DSP:%02d][%s][VDM_btn:%02x][who:%d][W:%d/D1:%d/D2:%d][USB:%d/%d/%d][bat:%5d/%d/%d][inactive:%d][check=%d/enable=%d]\n", \
		SELECT_Y + 2, GPIO_START_X , \
		m_module.dsp_mode , \
		check_5G_sync_status() == YES ? "5G" : "0G", \
		wire_vdm_button, \
		wireless_who, \
		wireless_level, \
		DFP1_level, \
		DFP2_level, \
		ats3607d_in_start, \
		ats3607d_in_end, \
		DSP_event_ptr, \
		gw_battery.battery_value, \
		gw_battery.status, \
		gw_battery.voltage_level,	\
		m_module.system_state == SYSTEM_STATE_INACTIVE? 1: 0, \
		wire_check_need_ack_done, \
		enable_check_wire_mic_sendout_time
		);
#endif
}

void GW_host_switch(void)
{
}

void GW_mcu_switch(void)
{
}

void GW_monitor_5G_I2S_switch(void) {

	if(satellite_mode == WIRE_MODE)
		return;
	
		wireless_mux_sw = SLAVE_5G;
}

void GW_monitor_wire_I2S_switch(void) {

	if(satellite_mode == WIRELESS_MODE)
		return;
	
		wire_mux_sw = SLAVE_WIRE;
}

void GW_gpio_control(uint8_t key) {

		switch(key) {
			case 'E':
			case 'e':
				break;
			case 'F':
			case 'f':
				GPO_PF03_WWL_SW = GPO_PF03_WWL_SW == 1 ? 0 : 1;
				printf("\x1b[%d;%dH  PF3 set to %d", SELECT_Y+1, SELECT_X, GPO_PF03_WWL_SW);
				break;
			case 'G':
			case 'g':
				GPO_PA10_TYPEC1_SW = GPO_PA10_TYPEC1_SW == 1 ? 0 : 1;
				printf("\x1b[%d;%dH PA10 set to %d", SELECT_Y+1, SELECT_X, GPO_PA10_TYPEC1_SW);
				break;
			case 'H':
			case 'h':
				GPO_PB06_TYPEC2_SW = GPO_PB06_TYPEC2_SW == 1 ? 0 : 1;
				printf("\x1b[%d;%dH  PB6 set to %d", SELECT_Y+1, SELECT_X, GPO_PB06_TYPEC2_SW);
				break;
			case 'I':
			case 'i':
				GPO_PB13_USBC_UDP_SEL0 = GPO_PB13_USBC_UDP_SEL0 == 1 ? 0 : 1;
				printf("\x1b[%d;%dH PB13 set to %d", SELECT_Y+1, SELECT_X, GPO_PB13_USBC_UDP_SEL0);
				break;
			case 'J':
			case 'j':
				GPO_PB12_USBC_UDP_SEL1 = GPO_PB12_USBC_UDP_SEL1 == 1 ? 0 : 1;
				printf("\x1b[%d;%dH PB12 set to %d", SELECT_Y+1, SELECT_X, GPO_PB12_USBC_UDP_SEL1);
				break;
			case 'K':
			case 'k':
				GPO_PB14_USB2SW = GPO_PB14_USB2SW == 1 ? 0 : 1;
				printf("\x1b[%d;%dH PB14 set to %d", SELECT_Y+1, SELECT_X, GPO_PB14_USB2SW);
				break;
			case 'L':
			case 'l':
				GPO_PA11_DSP_ONOFF = GPO_PA11_DSP_ONOFF == 1 ? 0 : 1;
				printf("\x1b[%d;%dH GPO_PA11_DSP_ONOFF set to %d", SELECT_Y+1, SELECT_X, GPO_PA11_DSP_ONOFF);
				break;
			case 'M':
			case 'm':
				GPO_PF04_TYPEC_UART1_EN = GPO_PF04_TYPEC_UART1_EN == 1 ? 0 : 1;
				printf("\x1b[%d;%dH PF4 set to %d", SELECT_Y+1, SELECT_X, GPO_PF04_TYPEC_UART1_EN);
				break;		
			case 'N':
			case 'n':
				GPO_PF05_TYPEC_UART2_EN = GPO_PF05_TYPEC_UART2_EN == 1 ? 0 : 1;
				printf("\x1b[%d;%dH PF5 set to %d", SELECT_Y+1, SELECT_X, GPO_PF05_TYPEC_UART2_EN);
				break;
		}
}

//---------------------------------------------------------------
void GW_LED_init(void)
{
		SPI_Init();
		GW_GPIO_init();

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    if (TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, TIMER0_FREQ) != 1) {
    }

    TIMER_EnableInt(TIMER0);
		NVIC_EnableIRQ(TMR0_IRQn);
		TIMER_Start(TIMER0);

		/* Enable PDMA channels */
    PDMA_Open(PDMA, (1 << SPI_TX_DMA_CH));

    /*=======================================================================
      SPI slave PDMA TX channel configuration:
      -----------------------------------------------------------------------
        Word length = 8 bits
        Transfer Count = DOA_size
        Source = LED_DOA_source
        Source Address = Increasing
        Destination = SPI0->TX
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/

		provide_LED_data();
		GW_led_constant_set(DOA_RGB + DOA_R_OFF);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA, SPI_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA, SPI_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
		
    /* Enable SPI slave DMA function */
    SPI_TRIGGER_TX_RX_PDMA(SPI0);
}

void show_GPIO_menu_status(void)
{
		printf("\x1b[%d;1HA. [ ] MCU_PWREN",GPIO_START_Y);
		printf("\x1b[%d;1HB. [ ] AMP_EN", GPIO_START_Y+1);
		printf("\x1b[%d;1HC. [ ] GL9510_DFP_I2C_INT", GPIO_START_Y+2);
		printf("\x1b[%d;1HD. [ ] Daisy_Chain1_EN", GPIO_START_Y+3);
		printf("\x1b[%d;1HE. [ ] Daisy_Chain2_EN", GPIO_START_Y+4);
		printf("\x1b[%d;1HF. [ ] Daisy_Chain1_RST#", GPIO_START_Y+5);	

		printf("\x1b[%d;1HG. [ ] TYPEC1_SW", GPIO_START_Y+6);	
		printf("\x1b[%d;1HH. [ ] TYPEC2_SW", GPIO_START_Y+7);
		printf("\x1b[%d;1HI. [ ] USBC_UDP_SEL0", GPIO_START_Y+8);	
		printf("\x1b[%d;1HJ. [ ] USBC_UDP_SEL1", GPIO_START_Y+9);
		printf("\x1b[%d;1HK. [ ] Daisy_Chain2_RST#", GPIO_START_Y+10);
		printf("\x1b[%d;1HL. [ ] DSP_ON_OFF", GPIO_START_Y+11);

		printf("\x1b[%d;1HM. [ ] TYPEC_UART1_EN", GPIO_START_Y+12);
		printf("\x1b[%d;1HN. [ ] TYPEC_UART2_EN", GPIO_START_Y+13);

		printf("\x1b[%d;30H[ ] GL3525_C1_VBUS_DET", GPIO_START_Y);
		printf("\x1b[%d;30H[ ] GL3525_C2_VBUS_DET", GPIO_START_Y+1);
		printf("\x1b[%d;30H[ ] USBC1_UFP_ID", GPIO_START_Y+2);

		printf("\x1b[%d;30H[ ] PB15", GPIO_START_Y+3);
		printf("\x1b[%d;30H[ ] PBx", GPIO_START_Y+4);
		//printf("\x1b[%d;30H[ ] PB7", GPIO_START_Y+5);	
		printf("\x1b[%d;30H[ ] GPO_PA11_DSP_ONOFF", GPIO_START_Y+5);	

		printf("\x1b[%d;30H[ ] USBC1_DFP_DET", GPIO_START_Y+6);
		printf("\x1b[%d;30H[ ] USBC2_DFP_DET", GPIO_START_Y+7);
		printf("\x1b[%d;30H[ ] 5GRX_MS_SEL", GPIO_START_Y+8);
		printf("\x1b[%d;30H[ ] Wireless_SW#", GPIO_START_Y+9);
		printf("\x1b[%d;30H[ ] PWR_SW#", GPIO_START_Y+10);

		printf("\x1b[%d;55H[ ] 5GTX_INT1", GPIO_START_Y);
		printf("\x1b[%d;55H[ ] 5GRX_INT1",GPIO_START_Y+1);
		printf("\x1b[%d;55H[ ] MCU_I2C0_INT", GPIO_START_Y+2);
		printf("\x1b[%d;55H[ ] CS8842_I2C_INT", GPIO_START_Y+3);

		printf("\x1b[%d;55H[ ] DSP_INT1", GPIO_START_Y+4);
		printf("\x1b[%d;55H[ ] WM8804B_I2C_INT", GPIO_START_Y+5);
}

void GW_battery_adc_init(void)
{
		uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;
		int32_t  i32ConversionData;

    u32IntNum = 0;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 0;   /* Use Sample Module 0 */	
		u32ChannelNum = 0;
	
    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);	
	
		EADC_Open(EADC, 0);

		/* Configure the sample module for analog input channel and software trigger source. */
		EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

		/* Set sample module external sampling time to 10 */
		EADC_SetExtendSampleTime(EADC, u32ModuleNum, 10);

		/* Enable Accumulate feature */
		EADC_ENABLE_ACU(EADC, u32ModuleNum, EADC_MCTL1_ACU_8);

		/* Enable Average feature */
		EADC_ENABLE_AVG(EADC, u32ModuleNum);

		/* Clear the A/D ADINT0 interrupt flag for safe */
		EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

		/* Enable the sample module interrupt.  */
		EADC_ENABLE_INT(EADC, u32IntMask);
		EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
		NVIC_EnableIRQ(EADC_INT0_IRQn);
}

void GW_usb_adc_init(void)
{
    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    PB->MODE &= ~(GPIO_MODE_MODE2_Msk);
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB2MFP_Msk) | SYS_GPB_MFPL_PB2MFP_EADC0_CH2;
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2);   /* Disable PB2 */

		uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;
		int32_t  i32ConversionData;

    u32IntNum = 2;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 2;   /* Use Sample Module 1 */
		u32ChannelNum = 2;

    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);

		EADC_Open(EADC, 0);

		/* Configure the sample module for analog input channel and software trigger source. */
		EADC_ConfigSampleModule(EADC, u32ModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

		/* Set sample module external sampling time to 10 */
		EADC_SetExtendSampleTime(EADC, u32ModuleNum, 10);

		/* Enable Accumulate feature */
		EADC_ENABLE_ACU(EADC, u32ModuleNum, EADC_MCTL1_ACU_8);

		/* Enable Average feature */
		EADC_ENABLE_AVG(EADC, u32ModuleNum);

		/* Clear the A/D ADINT2 interrupt flag for safe */
		EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);

		/* Enable the sample module interrupt.  */
		EADC_ENABLE_INT(EADC, BIT2);
		EADC_ENABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, BIT0 << u32ModuleNum);

		NVIC_EnableIRQ(EADC_INT2_IRQn);

    g_u32EadcInt2Flag = 0;
    EADC_START_CONV(EADC, u32ModuleMask);
}

void GW_usb_adc_uninit(void)
{
		uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;
		int32_t  i32ConversionData;

    u32IntNum = 2;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 2;   /* Use Sample Module 2 */
		u32ChannelNum = 2;

		EADC_STOP_CONV(EADC, BIT2);
		EADC_DISABLE_INT(EADC, BIT2);
		EADC_DISABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, BIT2);
		EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);
		EADC_Close(EADC);
		NVIC_DisableIRQ(EADC_INT2_IRQn);
		CLK_DisableModuleClock(EADC_MODULE);

		GPIO_ENABLE_DIGITAL_PATH(PB, BIT2);   /* Enable PB2 */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);		//VBUS
}

int32_t GW_Battery_Charger_Voltage_check(void)
{
    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;
		int32_t  i32ConversionData;
		int id_ptr = 0;

    u32IntNum = 0;      /* Use EADC Interrupt 0 */
    u32ModuleNum = 0;   /* Use Sample Module 0 */	
		u32ChannelNum = 0;
	
    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);	

		/* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
		g_u32EadcInt0Flag = 0;
		EADC_START_CONV(EADC, u32ModuleMask);

		/* Wait EADC interrupt (g_u32EadcInt0Flag will be set at EADC_INT0_IRQHandler() function) */
		adc_count = 0;
			
		return i32ConversionData;	
}

int32_t GW_Battery_Temperature_check(void)
{
    uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
    uint32_t u32IntMask, u32ModuleMask;
		int32_t  i32ConversionData;
		int id_ptr = 0;

    u32IntNum = 1;
    u32ModuleNum = 1;
		u32ChannelNum = 1;
	
    u32IntMask = (BIT0 << u32IntNum);
    u32ModuleMask = (BIT0 << u32ModuleNum);		
		/* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
    g_u32EadcInt1Flag = 0;
    EADC_START_CONV(EADC, u32ModuleMask);
	
    /* Wait EADC interrupt (g_u32EadcInt0Flag will be set at EADC_INT0_IRQHandler() function) */
		adc_temp_count = 0;

		return 	i32ConversionData;		
}

void GW_ADC_close(void)
{
		uint32_t u32IntNum,  u32ModuleNum, u32ChannelNum;
		uint32_t u32IntMask, u32ModuleMask;

		u32IntNum = 0;      /* Use EADC Interrupt 0 */
		u32ModuleNum = 0;   /* Use Sample Module 0 */	
		u32ChannelNum = 0;
		u32IntMask = (BIT0 << u32IntNum);
		u32ModuleMask = (BIT0 << u32ModuleNum);	

		EADC_Close(EADC);
		EADC_DISABLE_INT(EADC, u32IntMask);
		EADC_DISABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
		NVIC_DisableIRQ(EADC_INT0_IRQn);	
	
		u32IntNum = 1;      /* Use EADC Interrupt 1 */
		u32ModuleNum = 1;   /* Use Sample Module 1 */	
		u32ChannelNum = 1;
		u32IntMask = (BIT0 << u32IntNum);
		u32ModuleMask = (BIT0 << u32ModuleNum);	
	
		EADC_DISABLE_INT(EADC, u32IntMask);
		EADC_DISABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
		NVIC_DisableIRQ(EADC_INT1_IRQn);
	
		u32IntNum = 2;      /* Use EADC Interrupt 1 */
		u32ModuleNum = 2;   /* Use Sample Module 1 */	
		u32ChannelNum = 2;
		u32IntMask = (BIT0 << u32IntNum);
		u32ModuleMask = (BIT0 << u32ModuleNum);	

		EADC_DISABLE_INT(EADC, u32IntMask);
		EADC_DISABLE_SAMPLE_MODULE_INT(EADC, u32IntNum, u32ModuleMask);
		NVIC_DisableIRQ(EADC_INT2_IRQn);
}

void GW_reboot_for_power_on(void) {
	
    outpw(&SYS->RSTSTS, SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk); //clear bit
    outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
    NVIC_SystemReset();

    /* Trap the CPU */
    while (1);	
}

void GW_monitor_POWER_btn(void)
{
		//if(factory_mode == YES)
		//		return;

		// Check button whether release over 0.1 second at the beginning
		if(m_module.button_init == NO) {
				if(GPI_PB11_PWR_SW == HIGH) {
						if(m_module.button_power_timeout_ms > POWER_UP_RELEASE_TIME_MS) {
								m_module.button_evt = BUTTON_EVENT_CLEAR_LOW;
								m_module.button_power_timeout_ms = 0;
								m_module.button_init = YES;
						}
				}
				else
						m_module.button_power_timeout_ms = 0;

				return;
		}

		if(GPI_PB11_PWR_SW == HIGH) {
				m_module.button_power_timeout_ms = 0;
				wait_PWR_btn_release = NO;
				return;
		}

		if(GPO_PB07_5GRX_WWL_SW == LOW)
				m_module.button_mute_timeout_ms = 0;

		if(GPI_PB10_Wireless_SW == HIGH)
				m_module.button_unpair_timeout_ms = 0;

		if(wait_PWR_btn_release == YES)
				return;

		if(GPI_PB11_PWR_SW == LOW && GPO_PB07_5GRX_WWL_SW == LOW) {
		//if(GPI_PB11_PWR_SW == LOW && GPI_PB10_Wireless_SW == HIGH) {
				if(m_module.button_power_timeout_ms > TIME_S_TO_MS(POWER_DOWN_PRESS_HOLD_TIME_S) && m_module.button_unpair_timeout_ms <= 100) {
						m_module.button_evt = BUTTON_EVENT_POWER_OFF;
						sys_evt.button_power_off_cb();
				}
				else if(m_module.button_power_timeout_ms >= 100) {
						if(m_module.standby != STATE_STANDBY_IDLE) {
								m_module.button_evt = BUTTON_EVENT_WAKE_UP;
								m_module.button_power_timeout_ms = 0;
								sys_evt.paired_awake(AWAKE_BUTTON);
						}
				}
		}
		else if(GPI_PB11_PWR_SW == LOW && GPO_PB07_5GRX_WWL_SW == HIGH) {
				if(m_module.button_power_timeout_ms > TIME_S_TO_MS(FACTORY_PRESS_HOLD_TIME_S) && m_module.button_mute_timeout_ms > TIME_S_TO_MS(FACTORY_PRESS_HOLD_TIME_S)) {
		/*else if(GPI_PB11_PWR_SW == 0 && GPI_PB10_Wireless_SW == 0) {
				if(m_module.button_power_timeout_ms > 5000 && m_module.button_unpair_timeout_ms > 5000) {*/
						m_module.button_evt = BUTTON_EVENT_REBOOT_RESET;
						GW_AW5808_RX_Unpair();
						RESET_TMRINT_count = 0;
						while(RESET_TMRINT_count > 1000);
						GW_reboot_for_power_on();
				}
		}
}

void GW_monitor_MUTE_btn(void)
{
		if(GPO_PB07_5GRX_WWL_SW == HIGH) {
				//if(m_module.button_mute_timeout_ms > 1000) {
				{
						if(m_module.button_lock.mute == NO) {
								m_module.button_lock.mute = YES;
								sys_evt.button_mute_cb();
						}
				}
		}
		else {
				m_module.button_lock.mute = NO;
				m_module.button_mute_timeout_ms = 0;
		}
}

void GW_monitor_PAIR_btn(void)
{
		if(factory_mode == YES)
				return;

		if(GPI_PB10_Wireless_SW == HIGH) {
				m_module.button_unpair_timeout_ms = 0;
				wait_PAIR_btn_release = NO;
				return;
		}

		if(wait_PAIR_btn_release == YES)
				return;

		if(GPI_PB10_Wireless_SW == LOW && GPI_PB11_PWR_SW == HIGH) {
				if(time_to_work(&m_module.button_unpair_timeout_ms, TIME_S_TO_MS(BUTTON_UNPAIR_TIMEOUT_S))) {
						m_module.button_evt = BUTTON_EVENT_MUTE;
						sys_evt.button_unpair_cb();
				}
		}
		else
				m_module.button_unpair_timeout_ms = 0;
}

uint8_t check_5G_sync_status(void)
{
		return aw5808_rx_data.status & 0x01? YES: NO;
}

void GW_USB_send_to_DSP_slave_vol_request(uint8_t volume)
{
}

volatile uint8_t  rx_pair_cch_count = 0;

uint8_t GW_host_check(uint8_t status)
{
		uint8_t init_status = YES;
		if(m_module.wired_module == YES) {
				m_module.off_range = m_module.system_state != SYSTEM_STATE_IDLE? NO: YES;
		}
		else {
				m_cch_rx.state_init |= status;
				if(m_cch_rx.state_init != CCH_SYNC_STATE_INIT_BIT_FIELD) {
						init_status = NO;
				}
		}
		return init_status;
}

void GW_5G_CCH_pop_all(void)
{
		uint8_t i;
		for(i=0;i<CCH_SYNC_STATUS_COUNT;i++) {
				m_cch_rx.req[i] = NO;
		}
		m_cch_rx.size = 0;
}

void GW_5G_CCH_push(uint8_t type)
{
		uint8_t push = YES;

		if(m_cch_rx.connected == NO)
				return;

		if(m_cch_rx.size > 0) {
				if(m_cch_rx.req[type] == YES)
						push = NO;
		}

		if(push) {
PrintMsg__("5G CCH pushed: %02X \r\n", type);
				m_cch_rx.req[type] = YES;
				m_cch_rx.buffer[m_cch_rx.size] = type;
				m_cch_rx.size++;
				m_cch_rx.time_ticks_ms = CCH_RX_RESEND_PERIOD_MS;       // CCH_RX_SENDER_PERIOD_MS used for sending cch message immediately
				m_cch_rx.retry_count = CCH_RX_SENDER_RETRY_COUNT + 1;   // CCH_RX_SENDER_RETRY_COUNT + 1 used for sending cch message immediately

				m_cch_rx.seq_num_last = CCH_RX_SEQUENCE_NUMBER_MAX;
				m_cch_rx.send_period_time_ticks_ms = 0;
		}
}

void GW_5G_monitor_pair(void)
{
#define ST_PAIR_EXIT  0x00
#define ST_UNDER_PAIR 0x30
#define ST_PAIR_OK    0x10
#define ST_PAIR_FAIL  0x20

		//if(m_module.first_boot == NO)
		//		return;

		if(aw5808_rx_data.online == 0xFF)
				return;

		if(time_to_work(&m_cch_rx.pair.steps_timeout_ticks_ms, CCH_PAIR_CHECK_TIMEOUT_MS)) {
				if(m_cch_rx.pair.steps == CCH_PAIR_SUCCESS) {
						uint8_t resync = NO;
						if(m_cch_rx.long_nosync) {
								if(++m_cch_rx.long_nosync_count >= (AW5808_NOSYNC_MAX_TIME_MS/CCH_PAIR_CHECK_TIMEOUT_MS)) {
										m_cch_rx.long_nosync = NO;
								}
						}

						if(m_cch_rx.cch_int_count == m_debug.cch_int_count) {
								if(m_cch_rx.connected == YES) {
										if(time_to_work(&m_module.sync_retry_timeout_ms, TIME_S_TO_MS(AW5808_LOSS_SYNC_TIME_S) + (m_cch_rx.long_nosync? AW5808_NOSYNC_MAX_TIME_MS: AW5808_NOSYNC_MIN_TIME_MS)))
												resync = YES;
								}
								else
										m_module.sync_retry_timeout_ms = 0;

								aw5808_rx_data.status = 0xFF;
								if(GW_AW5808_RX_Read_Status(&aw5808_rx_data) == 0) {
										if((aw5808_rx_data.status&0x01) == 0x00)
												m_cch_rx.cch_lose_sync_count++;
								}
						}
						else {
								m_cch_rx.cch_int_count = m_debug.cch_int_count;
								uint16_t sync_period_ms;
								if(m_cch_rx.long_sync == NO) {
										sync_period_ms = AW5808_FIRST_SYNC_TIME_MS;
								}
								else
										sync_period_ms = AW5808_SYNC_TIME_MS + (m_cch_rx.long_nosync? AW5808_NOSYNC_MAX_TIME_MS: AW5808_NOSYNC_MIN_TIME_MS);

								if(m_cch_rx.status != aw5808_rx_data.status) {
										if(time_to_work(&m_module.sync_retry_timeout_ms, sync_period_ms))
												resync = YES;
								}
								else
										m_module.sync_retry_timeout_ms = 0;
						}

						if(resync) {
								if(GW_AW5808_RX_Read_Status(&aw5808_rx_data) == 0) {
										m_cch_rx.status = aw5808_rx_data.status;
										sys_evt.sync_range_cb(m_cch_rx.status&0x01? NO: YES);
										if(m_cch_rx.long_sync == NO) {
												if(m_cch_rx.status&0x01)
														m_cch_rx.long_sync = YES;
										}

										if(m_cch_rx.status&0x01) {
												m_cch_rx.long_nosync = YES;
												m_cch_rx.long_nosync_count = 0;
										}
								}
						}
						return;
				}

				int8_t	ret1, ret2;

				AW5808_DATA data = {0};
				ret1 = GW_AW5808_RX_Get_ID(&data);
				ret2 = GW_AW5808_RX_Read_Status(&data);
				if(ret1 == 0 && ret2 == 0) {
						uint8_t status = data.status & 0x30;
						memcpy(m_cch_rx.pair.id, data.id, 4);

						if(m_cch_rx.pair.steps == CCH_PAIR_INIT_CHECK) {
								if(memcmp(m_cch_rx.pair.id, "\x00\x00\x00\x00", 4) == 0) {
										sys_evt.pair_init_cb(YES);
								}
								else {
										memcpy(aw5808_rx_data.id, m_cch_rx.pair.id, sizeof(m_cch_rx.pair.id));
										sys_evt.pair_success_cb(NO);
								}
						}
						else if(m_cch_rx.pair.steps == CCH_PAIR_INIT) {
								sys_evt.pair_init_cb(YES);
						}
						else if(m_cch_rx.pair.steps == CCH_PAIR_MODE_CHECK) {
								if(status == ST_UNDER_PAIR)
										sys_evt.pair_mode_check_cb();
								else if(time_to_work(&m_cch_rx.pair.mode_check_timeout_ticks_ms, TIME_S_TO_MS(CCH_PAIR_CHECK_ID_0_TIMEOUT_S)))
										sys_evt.pair_init_cb(NO);
						}
						else if(m_cch_rx.pair.steps == CCH_PAIR_STATUS_CHECK) {
								if(time_to_work(&m_cch_rx.pair.pairing_timeout_ticks_ms, TIME_S_TO_MS(UNPAIR_TIMEOUT_S))) {
										sys_evt.unpair_timeout_cb();
										return;
								}

								if(status == ST_PAIR_FAIL || status == ST_PAIR_EXIT) {
										sys_evt.pair_start_cb();
								}
								else if(status == ST_PAIR_OK) {
										if(memcmp(m_cch_rx.pair.id, "\x00\x00\x00\x00", 4) == 0)
												sys_evt.pair_init_cb(NO);
										else {
												memcpy(aw5808_rx_data.id, m_cch_rx.pair.id, sizeof(m_cch_rx.pair.id));
												sys_evt.pair_success_cb(YES);
										}
								}
						}
				}
		}
}

void GW_5G_monitor_CCH(void)
{
		uint8_t device_sync_res[] = {0x53, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};

		uint8_t new_vol , new_mute, seq_num = CCH_RX_SEQUENCE_NUMBER_MAX;

		if(rx_pair_cch_count != aw5808_rx_data.cch_read_data[9]) {
				m_debug.cch_recv_count++;

				rx_pair_cch_count = aw5808_rx_data.cch_read_data[9];
				printf("\x1b[%d;%dHCCH Rx(%3d Int:%8d Rcv:%8d Diff:%3d; Last:%5d):0x%02x 0x%02x 0x%02x 0x%02x\n", CCH_START_Y + m_debug.cch_rx_log_index, CCH_START_X, rx_pair_cch_count, \
				m_debug.cch_int_time_ms, m_debug.cch_read_time_ms, m_debug.cch_read_time_ms > m_debug.cch_int_time_ms? (m_debug.cch_read_time_ms - m_debug.cch_int_time_ms): 0, (m_debug.cch_read_time_ms - m_debug.cch_read_last_time_ms), \
				aw5808_rx_data.cch_read_data[0], aw5808_rx_data.cch_read_data[1], aw5808_rx_data.cch_read_data[2], aw5808_rx_data.cch_read_data[3]);
				m_debug.cch_rx_log_index = ++m_debug.cch_rx_log_index >= CCH_START_Y_COUNT? 0: m_debug.cch_rx_log_index;

				m_debug.cch_read_last_time_ms = m_debug.cch_read_time_ms;
				//REQ, Type=00b
				if((aw5808_rx_data.cch_read_data[0]&0x30) == CCH_REQ) {
						if(m_cch_rx.req_seq_num != aw5808_rx_data.cch_read_data[1]) {
								// Increase the speed for fast read
								m_cch_rx.rx_access_period_ms = CCH_RX_ACCESS_MIN_PERIOD_MS;
								m_cch_rx.rx_access_timeout_ms = 0;

								//Device type sync,00000b
								if(aw5808_rx_data.cch_read_data[2] == 0x00) {
										//check g2 0010b,spk-pro 0100b
										if(aw5808_rx_data.cch_read_data[3]==0x02 || aw5808_rx_data.cch_read_data[3]==0x04) {
												//Data1,Seq No
												device_sync_res[1]=aw5808_rx_data.cch_read_data[1];
												device_sync_res[3]=0x06;//wireless mic
												GW_AW5808_RX_Send_CCH_Data(device_sync_res, 8);
												wireless_who = m_module.who = aw5808_rx_data.cch_read_data[3] >> 1;
												sys_evt.sync_device_type_cb(m_module.who);
												m_cch_rx.send_period_time_ticks_ms = 0;
										}
								}
								//Audio Volume sync,00011b
								if(aw5808_rx_data.cch_read_data[2] == 0x18) {
										new_mute = aw5808_rx_data.cch_read_data[3] & 0x01;
										new_vol = aw5808_rx_data.cch_read_data[3] >> 1;

										GW_USB_send_to_DSP_slave_vol_request(new_vol);

										device_sync_res[0]=0x51;
										//Data1,Seq No
										device_sync_res[1]=aw5808_rx_data.cch_read_data[1];
										GW_AW5808_RX_Send_CCH_Data(device_sync_res, 8);
										device_sync_res[0]=0x53;
										sys_evt.sync_mute_volume_cb(new_mute, new_vol);
										//PrintMsg__("%s! \r\n", new_mute? "Muted": "Unmuted");
										m_cch_rx.send_period_time_ticks_ms = 0;
								}
								//System command,00101b
								if(aw5808_rx_data.cch_read_data[2] == 0x28) {
										uint8_t system_state_updated = YES;
										//enter inactive state,0000010b
										if(aw5808_rx_data.cch_read_data[3]==0x04) {
												//printf("\x1b[%d;110H--- enter inactive ---\n",debug_row++);						
												device_sync_res[0]=0x51;
												//Data1,Seq No
												device_sync_res[1]=aw5808_rx_data.cch_read_data[1];
												GW_AW5808_RX_Send_CCH_Data(device_sync_res, 8);

												device_sync_res[0]=0x53;
												//PrintMsg__("Inactive \r\n");
												m_cch_rx.send_period_time_ticks_ms = 0;
												m_debug.inactive_count++;
										}
										//exit inactive/in-call state,0000000b
										else if(aw5808_rx_data.cch_read_data[3]==0x00) {
												device_sync_res[0]=0x51;
												//Data1,Seq No
												device_sync_res[1]=aw5808_rx_data.cch_read_data[1];
												GW_AW5808_RX_Send_CCH_Data(device_sync_res, 8);
												//printf("\x1b[%d;110H--- exit inactive/in-call ---\n",debug_row++);						

												device_sync_res[0]=0x53;
												//PrintMsg__("Evt: No-call \r\n");
												m_cch_rx.send_period_time_ticks_ms = 0;
												m_debug.normal_count++;
										}
										//enter in-call state,0000011b
										else if(aw5808_rx_data.cch_read_data[3]==0x06) {
												//printf("\x1b[%d;110H--- enter in-call ---\n",debug_row++);
												device_sync_res[0]=0x51;
												//Data1,Seq No
												device_sync_res[1]=aw5808_rx_data.cch_read_data[1];
												GW_AW5808_RX_Send_CCH_Data(device_sync_res, 8);

												device_sync_res[0]=0x53;
										
												//PrintMsg__("Evt: In-call \r\n");
												m_cch_rx.send_period_time_ticks_ms = 0;
												m_debug.call_count++;
										}
										else if(aw5808_rx_data.cch_read_data[3]==0x02) {
												m_debug.standby_count++;
										}
										else
												system_state_updated = NO;

										if(system_state_updated)
												sys_evt.sync_system_state_cb((aw5808_rx_data.cch_read_data[3] >> 1)&0x03);
								}
								else {
										sys_evt.sync_unknown_cb(aw5808_rx_data.cch_read_data[2], aw5808_rx_data.cch_read_data[3]);
								}
						}
						else {
								m_debug.err_seq_num_count++;
								printf("\x1b[%d;%dH[Err sn:0x%02x 0x%02x 0x%02x 0x%02x]\n", \
								SELECT_Y + 6, GPIO_START_X, \
								aw5808_rx_data.cch_read_data[0], aw5808_rx_data.cch_read_data[1], aw5808_rx_data.cch_read_data[2], aw5808_rx_data.cch_read_data[3] \
								);
						}
						m_cch_rx.req_seq_num = aw5808_rx_data.cch_read_data[1];
				}
				//ACK, Type=01b
				else if((aw5808_rx_data.cch_read_data[0]&0x30) == CCH_ACK) {
						if((aw5808_rx_data.cch_read_data[0]&0x03) == 0x03) {
								m_cch_rx.seq_num_last = aw5808_rx_data.cch_read_data[1];
						}
				}
		}

		if(m_cch_rx.size > 0 && m_cch_rx.connected == YES) {
				uint8_t send_now = NO, buffer_updated = NO, ack_get = NO;
				if(m_cch_rx.send_period_time_ticks_ms >= CCH_RX_SEND_PERIOD_MIN_MS) {
						if(m_cch_rx.seq_num_last != m_cch_rx.seq_num) {
								uint8_t times_up = time_to_work(&m_cch_rx.time_ticks_ms, CCH_RX_RESEND_PERIOD_MS);
								if(times_up) {
										send_now = YES;
										m_cch_rx.retry_count--;
								}

								if(m_cch_rx.retry_count == 0) {
										send_now = NO;
										buffer_updated = YES;
								}
						}
						else {
								buffer_updated = YES;
								ack_get = YES;
						}

						if(buffer_updated) {
								if(m_cch_rx.buffer[0] < CCH_SYNC_STATUS_COUNT)
										m_cch_rx.req[m_cch_rx.buffer[0]] = NO;

								if(aw5808_rx_data.cch_read_data[2] == 0x00) {
										//check g2 0010b,spk-pro 0100b
										// Reserved for G2
										if(aw5808_rx_data.cch_read_data[3] == 0)
												aw5808_rx_data.cch_read_data[3] = 0x04;

										if(aw5808_rx_data.cch_read_data[3] == 0x02 || aw5808_rx_data.cch_read_data[3] == 0x04) {
												//Data1,Seq No
												wireless_who = m_module.who = aw5808_rx_data.cch_read_data[3] >> 1;
												sys_evt.sync_device_type_cb(m_module.who);
										}
								}
								else if(aw5808_rx_data.cch_read_data[2] == 0x18) {
										new_mute = aw5808_rx_data.cch_read_data[3] & 0x01;
										new_vol = aw5808_rx_data.cch_read_data[3] >> 1;
										GW_USB_send_to_DSP_slave_vol_request(new_vol);
										sys_evt.sync_mute_volume_cb(new_mute, new_vol);
								}
								else if(aw5808_rx_data.cch_read_data[2] == 0x28) {
										uint8_t system_state_updated = YES;
										if(aw5808_rx_data.cch_read_data[3] == 0x00) {
										}
										else if(aw5808_rx_data.cch_read_data[3] == 0x02) {
										}
										else if(aw5808_rx_data.cch_read_data[3] == 0x04) {
										}
										else if(aw5808_rx_data.cch_read_data[3] == 0x06) {
										}
										else
												system_state_updated = NO;
										if(system_state_updated)
												sys_evt.sync_system_state_cb((aw5808_rx_data.cch_read_data[3] >> 1)&0x03);
								}

								if(m_cch_rx.seq_num_last != m_cch_rx.seq_num) {
										m_debug.reconnect_count++;
										//system_state_update(STATE_CONNECTED_OUT_RANGE);
								}

								m_cch_rx.size--;
								memmove(m_cch_rx.buffer, m_cch_rx.buffer + 1, m_cch_rx.size);
								m_cch_rx.seq_num = m_cch_rx.seq_num < (CCH_RX_SEQUENCE_NUMBER_MAX - 1)? m_cch_rx.seq_num + 1: 0;
								if(m_cch_rx.size > 0) {
										send_now = YES;
										m_cch_rx.retry_count = CCH_RX_SENDER_RETRY_COUNT;
										//PrintMsg__("CCH[%02X] next! \r\n", m_cch_rx.seq_num);
								}
								else {
										send_now = NO;
										//PrintMsg__("CCH[%02X] stopped! \r\n", m_cch_rx.seq_num);
								}
						}
				}

				if(send_now) {
						uint8_t buffer[] = {0x43, m_cch_rx.seq_num, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
						switch(m_cch_rx.buffer[0]) {
								case CCH_SYNC_DEVICE_TYPE_SYNC_TYPE:
										m_cch_rx.req[CCH_SYNC_DEVICE_TYPE_SYNC_TYPE] = YES;
										buffer[2] = 0x00;
										buffer[3] = 0x06;
										break;
								case CCH_SYNC_SYSTEM_COMMAND_TYPE:
										m_cch_rx.req[CCH_SYNC_SYSTEM_COMMAND_TYPE] = YES;
										buffer[2] = 0x28;
										buffer[3] = 0x00;
										break;
								case CCH_SYNC_AUDIO_VOLUME_SYNC_TYPE:
										m_cch_rx.req[CCH_SYNC_AUDIO_VOLUME_SYNC_TYPE] = YES;
										buffer[2] = 0x18;
										buffer[3] = 0x00;
										break;
								case CCH_SYNC_BUTTON_EVENT_MUTE_TYPE:
										m_cch_rx.req[CCH_SYNC_BUTTON_EVENT_MUTE_TYPE] = YES;
										buffer[2] = 0x08;
										buffer[3] = 0x02;
										break;
								default:
										break;
						}

						printf("\x1b[%d;%dHTx[%02x]: 0x%02x 0x%02x 0x%02x 0x%02x.\n", CCH_START_Y + m_debug.cch_tx_log_index, CCH_START_X + 80, buffer[1], buffer[0], buffer[1], buffer[2], buffer[3]);
						m_debug.cch_tx_log_index = ++m_debug.cch_tx_log_index >= CCH_START_Y_COUNT? 0: m_debug.cch_tx_log_index;

						m_cch_rx.time_ticks_ms = 0;
						m_cch_rx.send_period_time_ticks_ms = 0;
						GW_AW5808_RX_Send_CCH_Data(buffer, sizeof(buffer));
				}
		}
		else if(m_cch_rx.size > 0 && m_cch_rx.connected == NO) {
				m_cch_rx.size = 0;
		}
		aw5808_rx_data.cch_read_data[2] = 0x00;
		aw5808_rx_data.cch_read_data[3] = 0x00;
}

void GW_monitor_VDM(void)
{
		if(m_vdm.attention == YES) {
				uint8_t send_now = NO;
				if(m_vdm.send_timeout_ms >= VDM_RESEND_PERIOD_MS) {
						if(m_vdm.send_retry_count == 0) {
								m_vdm.attention = NO;
								m_vdm.vdm_timeout_count++;
						}
						else {
								send_now = YES;
								m_vdm.vdm_resend_count++;
						}
						m_vdm.send_retry_count--;
				}

				if(send_now == YES) {
						m_vdm.send_timeout_ms = 0;
						send_atten_req();
				}
		}
}

void GW_send_cmd_to_3607D(uint8_t *buf, uint8_t len) {

		uint8_t i;

		m_module.tx.lengh = len;
		memcpy(m_module.tx.buffer, (uint8_t *)buf, len);

		for(i=0; i<len; i++) {
				while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk){}
				UART0->DAT = buf[i];
		}
}

volatile uint8_t need_wire_DFP1_ack = NO;
volatile uint8_t need_wire_DFP2_ack = NO;
volatile uint8_t need_wire_DFP1_req = NO;
									//len,Read Data type(0x11),port number, VDM Header(4 byte),VDO-3(4 Bytes),VDO-2(4 Bytes),VDO-1(4 Bytes) 	
uint8_t GW_vdm_req[19]={0x13,0x11,0x00,0x17,0xef,0xa0,VDM_HEAD_REQ,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t GW_vdm_ack[19]={0x13,0x11,0x00,0x17,0xef,0xa0,VDM_HEAD_ACK,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t GW_vdm_response[6]={0x03,0x04,0x00,0x00,0x00,0x00};

uint8_t GW_wire_process_vol_st_check(uint8_t* data, uint8_t _data_len)
{
		uint8_t new_vol , new_mute, need_ack = NO, system_cmd=0;

		switch(m_module.dsp_mode) {
				case DSP_MODE_WIRE_M_TO_SPK:
				case DSP_MODE_WIRE_M_TO_G2:
						break;
				default:
						return NO;
		}	

		if(data[6] == 0 && data[7] == 0 && data[8] == 0) {
				wire_check_need_ack_done = 0;
				//printf("\x1b[31;90H--- send mute---\n");
				//check_wire_button_timeout_enable=0;
				send_attention = 0;
				GW_vdm_ack[7] = VDM_BUTTON_MUTE;	//bit[31:29] 001b mute button 
				GW_vdm_ack[7] |= 0x10;    //bit28: button break event 
				GW_vdm_ack[8] |= 0x80;    //bit23: docking event
				return YES;
		}

		//0x13,0x11 Read_MCU_Data (Data Type VDM_Message) <-> Send VDM response
		//system event for system command changed (enter normal state/enter standby state/enter inactive state/enter in-call state)
		if((data[7] & 0x02) == 0x02) {
				system_cmd = data[7]&0x0C;
				//enter normal state,  (exit inactive, exit in-call)
				if(system_cmd == 0x00) {
				}
				//enter standby state
				else if(system_cmd == 0x04) {
						//standby_mode_start_count = 0;
						//pre_battery_mode = 0xFF;
				}
				//enter inactive state
				else if(system_cmd == 0x08) {
				}
				//enter in-call state
				else if(system_cmd == 0x0C) {
						wire_check_need_ack_done = 0;
						GW_vdm_ack[8] |= 0x01;    //bit16: docking ack
				}
				need_ack = YES;
				sys_evt.sync_system_state_cb((system_cmd >> 2)&0x03);
		}

		//system event for volume or mute event (Host Mute)
		if((data[9] & 0x80) == 0x80) {
				new_vol = data[8] >> 1;
				new_mute = data[8] & 0x01;
				wire_check_need_ack_done = 0;
				GW_vdm_ack[10] |= 0x40;    //bit6: docking event ack
				need_ack = YES;
				m_vdm.attention = NO;
				m_vdm.vdm_spend_time_ms = m_vdm.send_timeout_ms;
				sys_evt.sync_mute_volume_cb(new_mute, new_vol);
		}
		return need_ack;	
}

volatile uint8_t wire_vdm_button = 0x00;

void GW_wire_process_button_ack(uint8_t* data, uint8_t _data_len)
{
		wire_vdm_button = data[6]&0xE0;
}

void GW_wire_process_master_req(uint8_t* data, uint8_t _data_len)
{
		uint8_t need_ack = NO;
		if((data[0]&0x7F) == SEND_VDM_RESPONSE) {
				//REQ
				if((data[5]&VDM_HEADER_CMD_TYPE_BIT_MASK) == VDM_HEADER_CMD_TYPE_REQ) {
						if((data[5]&0x1F) != VDM_HEADER_GET_STATUS)
								return;

						if(GW_wire_process_vol_st_check(data, _data_len) == YES)
								need_ack = YES;

						if(need_ack == YES) {
								need_wire_DFP1_ack = YES;
						}
				}
				//ACK
				else if((data[5]&VDM_HEADER_CMD_TYPE_BIT_MASK) == VDM_HEADER_CMD_TYPE_ACK) {
						GW_wire_process_button_ack(data, _data_len);
				}
		}		
}

#ifndef FUNCTION_NOT_SUPPORT
void GW_wire_button_key_event(uint8_t button) {

		GW_vdm_ack[7] = button;
		need_wire_DFP1_ack = YES;
		need_wire_DFP2_ack = YES;	
}
#endif

void GW_wire_check_need_req(void)
{
		if(need_wire_DFP1_req == YES) {
				if(m_module.dsp_mode == DSP_MODE_WIRE_M_TO_SPK || m_module.dsp_mode == DSP_MODE_WIRE_M_TO_G2 ) {		
						//GW_vdm_req[2] = 0x80;
						//Request_To_SendOut_Data(GW_vdm_req,sizeof(GW_vdm_req));
						//GW_vdm_response[2] = 0;
						Request_To_SendOut_Data(GW_vdm_response, sizeof(GW_vdm_response));
				}
				need_wire_DFP1_req = NO;
				GW_vdm_response[5] = 0;
				//reset_VDM_REQ_DATA();
				return;
		}
}

uint8_t wire_check_need_ack_done=0;

void GW_wire_check_need_ack(void)
{
		if(need_wire_DFP1_ack == YES) {
				if(m_module.dsp_mode == DSP_MODE_WIRE_M_TO_SPK || m_module.dsp_mode == DSP_MODE_WIRE_M_TO_G2) {
						if(wire_check_need_ack_done == 0) {
								wire_check_need_ack_done = 1;
								//printf("\x1b[%d;110H--- send out VDM ---\n",debug_row++);
								enable_check_wire_mic_sendout_time = 1;
								GW_vdm_ack[2] = 0;
								Request_To_SendOut_Data(GW_vdm_ack, sizeof(GW_vdm_ack));
						}
				}
				need_wire_DFP1_ack = NO;
				reset_VDM_ACK_DATA();
				return;
		}
}

void GW_send_DSP_tone(uint8_t tone)
{
		int i;
		uint16_t checksum;

		uint8_t dsp_tone_set[] = { DSP_MAGIC, DSP_CMD_FLAG, 0x04, DSP_CMD_TONE, 0x00, 0x00};

		dsp_tone_set[4] = tone;

		for(i=0;i<5;i++)
				checksum += dsp_tone_set[i];

		dsp_tone_set[5] = checksum&0xFF;
		GW_send_cmd_to_3607D((uint8_t*)dsp_tone_set, sizeof(dsp_tone_set));
}

volatile uint8_t send_attention=0;
uint8_t wire_seq_num=0;
uint8_t debug_row=30;

void check_DSP_event_buf(void)
{
		int i;

		//mute status
		if(DSP_event_buf[3] == DSP_EVENT_MUTE)  {
				m_module.mute = DSP_event_buf[4]? YES: NO;
		}

		//volume value
		if(DSP_event_buf[3] == DSP_EVENT_RX_VOL)  {
				//dsp_mode_ready = YES;
		}

		//voice status
		if(DSP_event_buf[3] == DSP_EVENT_UAC_ST)  {
				m_module.system_state = (DSP_event_buf[4] == 0x02) || (DSP_event_buf[4] == 0x04) ? SYSTEM_STATE_IN_CALL : SYSTEM_STATE_NORMAL;
		}

		//bg sound detect
		if(DSP_event_buf[3] == DSP_EVENT_VOICE_DETECT) {
		}

		//dsp ready
		if(DSP_event_buf[3] == DSP_EVENT_READY) {
				dsp_mode_ready = YES;
		}

		if(DSP_event_buf[3] == DSP_EVENT_CMD_COMPLETE) {
				if(DSP_event_buf[4] == DSP_MODE_SINGLE_CMD) {
						m_module.dsp_mode = DSP_MODE_SINGLE;
						m_module.dsp_update_time_ms = 0;
				}
				else if(DSP_event_buf[4] == DSP_MODE_WIRED_CMD) {
						DFP1_level = GW_check_DFP_level(DFP1_connect);
						m_module.dsp_mode = DFP1_level == WHO_IS_G2? DSP_MODE_WIRE_M_TO_G2: DSP_MODE_WIRE_M_TO_SPK;
				}
				else if(DSP_event_buf[4] == DSP_MODE_WIRELESS_CMD) {
						if(m_module.who == WHO_IS_SPK)
								m_module.dsp_mode = DSP_MODE_WIRELESS_M_TO_SPK;
						else if(m_module.who == WHO_IS_G2)
								m_module.dsp_mode = DSP_MODE_WIRELESS_M_TO_G2;
						m_module.dsp_update_time_ms = 0;
				}
		}

		m_module.rx.lengh = DSP_event_ptr;
		memcpy(m_module.rx.buffer, (uint8_t *)DSP_event_buf, m_module.rx.lengh);

		DSP_event_ptr = 0;
}

void GW_check_3607D_in_buf(void) {
	
	//offset 0...5 : 0xa5 0x04 length event params checksum
	
	#define DSP_POINT_MOVE_NEXT {new_start ++; if(new_start >= MAX_DSP_BUF) new_start = 0; ats3607d_in_start = new_start;}
	
	uint8_t new_start;
	
	if(ats3607d_in_start == ats3607d_in_end)
		return;
	
	new_start = ats3607d_in_start;
	
	if((DSP_event_ptr == 0) && (ats3607d_in_buf[new_start] != DSP_MAGIC)) {
		DSP_POINT_MOVE_NEXT;
		return;
	}
/*
	if((DSP_event_ptr == 1) && (ats3607d_in_buf[new_start] != DSP_EVENT_FLAG)) {
		DSP_event_ptr = 0;
		DSP_POINT_MOVE_NEXT;
		return;
	}
*/
	DSP_event_buf[DSP_event_ptr] = ats3607d_in_buf[new_start];
	DSP_event_ptr ++;
	DSP_POINT_MOVE_NEXT;
	
	//work run
	if((DSP_event_ptr == 3) && (DSP_event_buf[2] == DSP_MAGIC))
	DSP_event_ptr = 1;

	
	if((DSP_event_ptr >= 4) && (DSP_event_buf[2] == (DSP_event_ptr - 2))) {
		if(DSP_event_buf[2] < 60) {
			memcpy(USB_raw_response+2, (void *)DSP_event_buf, DSP_event_buf[2]+2);
			USB_raw_response[0] = 0xAA;
			USB_raw_response[1] = DSP_event_buf[2] + 2;
			USB_raw_res_len = DSP_event_buf[2] + 4;
		}
		check_DSP_event_buf();
	}
	
}

extern uint8_t random_value_array[];

void GW_free_gpio(void) {
	
	//release USB interface
	PA->MODE &= ~(GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk | GPIO_MODE_MODE14_Msk);
	SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);	
	
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
}

void GW_power_down_interface(void) {
		GPO_PD03_AMP_EN = NO;
		//TIMER_DisableInt(TIMER0);

		UART_DisableInt(UART0, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		UART_DisableInt(UART1, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		UART_DisableInt(UART2, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
		
		UART_Close(UART0);
		UART_Close(UART1);
		UART_Close(UART2);
		
		
		GW_AW5808_Tx_On_Off(0);
		
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
		

    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA6MFP_Msk);
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA7MFP_Msk);

    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA8MFP_Msk);
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA9MFP_Msk);
	
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk);		
		
		GW_free_gpio();
}

uint8_t usci_need_re_init = NO;

void check_usci_need_re_init(void)
{
		if(usci_need_re_init == NO)
			return;

		UI2C_Close(UI2C0);
		UI2C_DISABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk | UI2C_PROTIEN_ARBLOIEN_Msk |UI2C_PROTIEN_ERRIEN_Msk));
    UI2C_DisableInt(UI2C0, UI2C_TO_INT_MASK | UI2C_STAR_INT_MASK | UI2C_STOR_INT_MASK | UI2C_NACK_INT_MASK | UI2C_ARBLO_INT_MASK | UI2C_ERR_INT_MASK | UI2C_ACK_INT_MASK);		
		NVIC_DisableIRQ(USCI0_IRQn);

		GW_AW5808_Init();
		usci_need_re_init = NO;
}

volatile uint8_t DFP1_connect = 0x00;
volatile uint8_t DFP2_connect = 0x00;

const uint8_t SPK_DFP_cable[] = {0x17, 0xEF, 0xA0, 0x6B};
const uint8_t MIC_DFP_cable[] = {0x17, 0xEF, 0xA0, 0x75};
const uint8_t SPK_DFP_PD[] 		= {0x17, 0xEF, 0xA0, 0x51};
const uint8_t G2_DFP_PD[] 		= {0x2E, 0x99, 0x03, 0x11};
const uint8_t MIC_UFP_PD[] 		= {0x17, 0xEF, 0xA0, 0x70};

void GW_check_PD_data(uint8_t *read_data)
{
		uint8_t new_connect;
		//Send_VID_PID (0x01)
		if(((read_data[0]&0x7F) == 0x01) && (read_data[1] == 0x05)) {
				new_connect = read_data[0]&0x80 ? DFP2_connect : DFP1_connect;
				new_connect |= DFP_CONNECT;

				if(read_data[2] == 0x00) {                //PD
						new_connect &= 0xE3;

						if(memcmp(SPK_DFP_PD, read_data + 3, 4) == 0)
								new_connect |= DFP_PD_IS_SPK;

						if(memcmp(G2_DFP_PD, read_data + 3, 4) == 0)
								new_connect |= DFP_PD_IS_G2;

						if(memcmp(MIC_UFP_PD, read_data + 3, 4) == 0)
								new_connect |= DFP_PD_IS_MIC;
				}
				else if(read_data[2] == 0x01) {           //cable
						if(memcmp(SPK_DFP_cable, read_data + 3, 4) == 0)
								new_connect |= DFP_CABLE_PASS;
						else if(memcmp(MIC_DFP_cable, read_data + 3, 4) == 0)
								new_connect |= DFP_CABLE_PASS;
						else
								new_connect &= 0xFD;
				}
				else {
						return;
				}

				if(read_data[0]&0x80)
						DFP2_connect = new_connect;
				else {
						if(new_connect != DFP1_connect) {
								if(new_connect == (DFP_CONNECT | DFP_CABLE_PASS | DFP_PD_IS_SPK) || new_connect == (DFP_CONNECT | DFP_CABLE_PASS | DFP_PD_IS_G2)) {
										m_module.who = new_connect&DFP_PD_IS_SPK? WHO_IS_SPK: WHO_IS_G2;
										system_state_update(STATE_UP_PORT_CONNECTED);
								}
								else {
										m_module.who = WHO_IS_NO;
										system_state_update(STATE_UP_PORT_DISCONNECT);
								}
						}
						DFP1_connect = new_connect;
				}
		}

		//Send_PD_Status
		if(((read_data[0]&0x7F) == 0x04) && (read_data[1] == 0x02)) {
				if((read_data[2]&DFP_DISCONNECT) == DFP_DISCONNECT) {
						if(read_data[0]&0x80)
								DFP2_connect = 0x00;
						else
								DFP1_connect = 0x00;
				}
				else {
						if(read_data[0]&0x80)
								DFP2_connect |= DFP_CONNECT;
						else
								DFP1_connect |= DFP_CONNECT;
				}
		}
}

uint8_t GW_check_DFP_level(uint8_t connect)
{
		if((connect & DFP_CONNECT) != DFP_CONNECT)
				return WHO_IS_NO;

		if((connect & DFP_CABLE_PASS) != DFP_CABLE_PASS)
				return WHO_IS_NO;

		if((connect & DFP_PD_IS_SPK) == DFP_PD_IS_SPK)
				return WHO_IS_SPK;

		if((connect & DFP_PD_IS_G2) == DFP_PD_IS_G2)
				return WHO_IS_G2;

		if((connect & DFP_PD_IS_MIC) == DFP_PD_IS_MIC)
				return WHO_IS_MIC;

		return WHO_IS_NO;
}

void GW_check_DSP_mode(void)
{
		uint8_t now = DSP_MODE_SINGLE;

		//wire
		if(hw_current_version == 1) {
				if(m_module.vdm_ready == NO)
						return;

				DFP1_level = GW_check_DFP_level(DFP1_connect);
				now = DFP1_level == WHO_IS_G2? DSP_MODE_WIRE_M_TO_G2: DSP_MODE_WIRE_M_TO_SPK;
		}
		//wireless
		else {
				if(m_module.first_pair_check == NO)
						return;

				wireless_level = m_module.who;
				if(wireless_level == WHO_IS_NO) {
						//do nothing, DSP always mode 31
				}
				else if(wireless_level == WHO_IS_SPK) {
						now = DSP_MODE_WIRELESS_M_TO_SPK;
				}
				else if(wireless_level == WHO_IS_G2) {
						now = DSP_MODE_WIRELESS_M_TO_G2;
				}
		}

		if(m_module.dsp_mode == now)
				return;

		if(m_module.dsp_update_time_ms == 0)
				m_module.dsp_update_time_ms = m_module.current_time_ms;

		if(m_module.dsp_update_time_ms != m_module.current_time_ms) {
				if(m_module.current_time_ms - m_module.dsp_update_time_ms >= 200)
						m_module.dsp_update_time_ms = 0;
				return;
		}

		//for audio mode 3.4 version
		const uint8_t DSP_MODE_01_single_cmd[]   = {0xa5, 0x01, 0x04, DSP_MODE_SINGLE_CMD, 0x00, 0xdb};
		const uint8_t DSP_MODE_31_wire_cmd[]     = {0xa5, 0x01, 0x04, DSP_MODE_WIRED_CMD, 0x00, 0xdd};    //wire to spk-pro
		const uint8_t DSP_MODE_33_wire_cmd[]     = {0xa5, 0x01, 0x04, DSP_MODE_WIRED_CMD, 0x01, 0xde};    //wire to g2
		const uint8_t DSP_MODE_32_wireless_cmd[] = {0xa5, 0x01, 0x04, DSP_MODE_WIRELESS_CMD, 0x00, 0xdf}; //wireless to spk-pro
		const uint8_t DSP_MODE_34_wireless_cmd[] = {0xa5, 0x01, 0x04, DSP_MODE_WIRELESS_CMD, 0x01, 0xe0}; //wireless to g2

		switch(now) {
				case DSP_MODE_SINGLE:
						satellite_mode = WIRE_MODE;
						GW_send_cmd_to_3607D((uint8_t*)DSP_MODE_01_single_cmd, sizeof(DSP_MODE_01_single_cmd));
						break;
				case DSP_MODE_WIRE_M_TO_SPK:
						satellite_mode = WIRE_MODE;
						GW_send_cmd_to_3607D((uint8_t*)DSP_MODE_31_wire_cmd, sizeof(DSP_MODE_31_wire_cmd));
						break;
				case DSP_MODE_WIRE_M_TO_G2:
						satellite_mode = WIRE_MODE;
						GW_send_cmd_to_3607D((uint8_t*)DSP_MODE_33_wire_cmd, sizeof(DSP_MODE_33_wire_cmd));
						break;
				case DSP_MODE_WIRELESS_M_TO_SPK:
						satellite_mode = WIRELESS_MODE;
						GW_send_cmd_to_3607D((uint8_t*)DSP_MODE_32_wireless_cmd, sizeof(DSP_MODE_32_wireless_cmd));
						break;
				case DSP_MODE_WIRELESS_M_TO_G2:
						satellite_mode = WIRELESS_MODE;
						GW_send_cmd_to_3607D((uint8_t*)DSP_MODE_34_wireless_cmd, sizeof(DSP_MODE_34_wireless_cmd));
						break;
		}
}

void GW_USB_send_to_DSP_direct(void)
{
		GW_send_cmd_to_3607D(&USB_raw_request[3], USB_raw_req_len - 3);
}

void GW_USB_send_to_DSP_ISP_request(void)
{	
		const uint8_t DSP_ISP_cmd[] = { DSP_MAGIC, DSP_CMD_FLAG, 0x03, DSP_CMD_ISP, 0xea};
		GW_send_cmd_to_3607D((uint8_t*)DSP_ISP_cmd, sizeof(DSP_ISP_cmd));
}

void GW_check_USB_raw_request(void) {
	
		uint8_t data[4] = { 0x00, 0x00, 0x00, 0x00 };
	
		if(USB_raw_req_len == 0)
			return;

		switch(USB_raw_request[2]) {
			case USB_DSP_RAW_CMD:
				GW_led_lock(NO);
				GW_USB_send_to_DSP_direct();
				break;
			case USB_DSP_ISP_CMD:
				GW_USB_send_to_DSP_ISP_request();
				break;
			case USB_REQ_RECOVERY:
				//GW_AW5808_TX_Write_XX_data(REG_ADDR_OPERATED_ID_0, data ,4);
				//GW_AW5808_RX_Write_XX_data(REG_ADDR_OPERATED_ID_0, data ,4);
				//check_aw5808_tx_update_flag = YES;
				check_aw5808_rx_update_flag = YES;
				GW_AW5808_RX_Unpair();
				RESET_TMRINT_count = 0;
				while(RESET_TMRINT_count > 1000);
				GW_reboot_for_power_on();
				break;
			case USB_REQ_SET_TX_ID:
				GW_AW5808_TX_Write_XX_data(REG_ADDR_OPERATED_ID_0,&USB_raw_request[3],4);
				check_aw5808_tx_update_flag = YES;
				break;
			case USB_REQ_SET_RX_ID:
				GW_AW5808_RX_Write_XX_data(REG_ADDR_OPERATED_ID_0,&USB_raw_request[3],4);
				aw5808_rx_data.online=0xff;
				check_aw5808_rx_update_flag = YES;
				break;
			case USB_REQ_RF_POWER_SET:
				data[0] = 0x7D;
				data[1] = (USB_raw_request[5] & 0x0F) << 4;
				data[1] |= USB_raw_request[5] & 0x0F;
				if(USB_raw_request[3] == 0x01) {
					RF_module_set[0] = YES;
					RF_module_set[1] = USB_raw_request[4];
					RF_module_set[2] = data[1];
					GW_AW5808_TX_Write_XX_data(REG_ADDR_SELECT_FIX_RF_POWER, &data[0], 1);
					GW_AW5808_TX_Write_XX_data(REG_ADDR_RF_POWER, &data[1], 1);
				} else if(USB_raw_request[3] == 0x00){
					RF_module_set[3] = YES;
					RF_module_set[4] = USB_raw_request[4];
					RF_module_set[5] = data[1];					
					GW_AW5808_RX_Write_XX_data(REG_ADDR_SELECT_FIX_RF_POWER, &data[0], 1);
					GW_AW5808_RX_Write_XX_data(REG_ADDR_RF_POWER, &data[1], 1);
				}
				break;
			default:
				break;
		}
		
		USB_raw_req_len = 0;
}

void GW_response_USB_direct(void) {
	
		uint8_t USB_key_status = 0x00;
		int i;
		switch(USB_raw_request[2]) {
			case USB_DSP_READ_LAST_DSP:
				GW_hid_app_cs_response(USB_raw_response, USB_raw_res_len);
				USB_raw_req_len = 0;
				GW_led_unlock();
				break;
			case USB_REQ_LED_FULL_WHITE:
				DOA_RGB[IDLE_R_F] = 0xFF;
				DOA_RGB[IDLE_G_F] = 0xFF;
				DOA_RGB[IDLE_B_F] = 0xFF;
				for(i=0; i<8; i++)
					factory_led_set[i] = 0xFF;
			
				GW_hid_app_cs_response(USB_raw_request, USB_raw_req_len);
				USB_raw_req_len = 0;
				break;
			case USB_REQ_LED_COLOR_SET:
				{
						uint8_t cross_1[8] = {0xFF, 0xFF, 0xFF, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}, cross_2[8] = {0xFF, 0xFF, 0xFF, 0x55, 0x55, 0x55, 0x55, 0x55};
						factory_led_cross = 0;
						if(memcmp(cross_1, USB_raw_request + 3, 8) == 0) {
								factory_led_cross = 1;
						}
						else if(memcmp(cross_2, USB_raw_request + 3, 8) == 0) {
								factory_led_cross = 2;
						}
						else
								memcpy(factory_led_set, &USB_raw_request[3], 8);
						GW_hid_app_cs_response(USB_raw_request, USB_raw_req_len);
						USB_raw_req_len = 0;
				}				
				break;
			case USB_REQ_GET_TX_ID:
				memcpy(USB_raw_request, aw5808_tx_data.id, 4);
				GW_hid_app_cs_response(USB_raw_request, 4);
				USB_raw_req_len = 0;
				break;
			case USB_REQ_GET_RX_ID:
				memcpy(USB_raw_request, aw5808_rx_data.id, 4);
				GW_hid_app_cs_response(USB_raw_request, 4);
				USB_raw_req_len = 0;
				break;
			case USB_REQ_INTO_FACTORY:
				memcpy(factory_led_set, DOA_RGB + IDLE_R_F, 8);
				factory_mode = YES;
				GW_hid_app_cs_response(USB_raw_request, 4);
			  //close auto pairing and standby mode
				m_module.power_down = POWER_DOWN_IDLE;
				enable_standby_mode=0;
				break;
			case USB_REQ_EXIT_FACTORY:
				factory_mode = NO;
				GW_hid_app_cs_response(USB_raw_request, 4);
				m_module.power_down = POWER_DOWN_READY;
				enable_standby_mode=1;
				break;
			case USB_REQ_TOUCH_GET:
				if(GPI_PB11_PWR_SW == LOW)
					USB_key_status |= 0x01;
				
				if(GPI_PB10_Wireless_SW == LOW)
					USB_key_status |= 0x02;
				
				if(GPO_PB07_5GRX_WWL_SW == HIGH)
					USB_key_status |= 0x20;
				
				GW_hid_app_cs_response(&USB_key_status, 1);
				break;

			case USB_REQ_BATTERY_INFO:
					//wireless		
					if(hw_current_version==0)
					{
				GW_hid_app_cs_response(&gw_battery.battery_value, 4);
					}
				break;

			case USB_REQ_RX_LEVEL_INFO:
				GW_hid_app_cs_response(&aw5808_rx_data.rf_rssi_level, 1);
				break;

			default:
				GW_hid_app_cs_response(USB_raw_request, USB_raw_req_len);
				break;
		}
	
}

void GW_monitor_LED_lock(void)
{
		if(m_module.led_lock == YES) {
				if(m_module.current_time_ms > (m_module.led_lock_time_ms + m_module.led_lock_timeout_ms)) {
						GW_led_unlock();
				}
		}
}

void GW_LED_loop(void)
{
		//wireless
		if(hw_current_version == 0) {
				//GW_system_state_machine();
				GW_monitor_POWER_btn();
				GW_monitor_MUTE_btn();
				GW_monitor_PAIR_btn();
				GW_monitor_LED_lock();

				//GW_Check_AutoConnection();
				GW_standby_mode_watcher();

				//Check_Charging_Bay_Status();
				check_first_time_boot_up_pair_mode();
				exec_power_off_mode();

				if(ats3607_update_running())
						return;

				if(GPO_PD02_MCU_PWREN == NO)
						return;

				GW_check_3607D_in_buf();
				GW_check_USB_raw_request();
				GW_monitor_5G_I2S_switch();
				GW_5G_monitor_pair();
				GW_5G_monitor_CCH();
				GW_check_DSP_mode();
				GW_AW5808_Task_Handler();
				//GW_menu_loop();
				GW_message_update();
		}
		//wire
		else {
				check_usci_need_re_init();
				Show_VDM_Slave_Data();

				GW_monitor_MUTE_btn();
				GW_monitor_LED_lock();

				if(ats3607_update_running())
						return;

				if(GPO_PD02_MCU_PWREN == NO)
						return;

				GW_check_3607D_in_buf();
				GW_check_USB_raw_request();
				GW_wire_check_need_ack();
				GW_wire_check_need_req();
				GW_monitor_VDM();
				GW_check_DSP_mode();

				//GW_menu_loop();
				GW_message_update();
		}
}

void timer0_init(void)
{
		CLK_EnableModuleClock(TMR0_MODULE);
		CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
		if (TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, TIMER0_FREQ) != 1) {
		}
		TIMER_EnableInt(TIMER0);
		NVIC_EnableIRQ(TMR0_IRQn);
		TIMER_Start(TIMER0);
}

void check_first_time_boot_up_pair_mode(void)
{
		if(m_module.first_boot == YES) {
				m_module.first_boot = NO;
				m_module.first_pair_check = YES;
				if(m_cch_rx.pair.steps == CCH_PAIR_SUCCESS) {
						GW_send_DSP_tone(DSP_TONE_PAIR_OK_CN);
						system_state_update(STATE_PAIR_SUCCESS_S);
				}
				else
						GW_send_DSP_tone(DSP_TONE_PAIR_NG_EN);
		}

		return;
}

void exec_power_off_mode(void)
{
		if(factory_mode == YES)
				return;
		
		if(m_module.power_down == POWER_DOWN_IDLE)
				return;

#ifdef STATE_MACHINE_TEST
		if(m_module.power_down == POWER_DOWN_READY)
		{
				if(m_module.ultralow_power == NO && m_module.low_power == YES) {
						//m_module.power_down = POWER_DOWN_IDLE;
						//return;
				}
				system_state_update(STATE_POWER_OFF_S);
		}
#endif

		m_module.power_on_timeout_ms = 0;
		if(m_module.power_down != POWER_DOWN_NOW || m_module.power_on_timeout_ms >= (TIME_S_TO_MS(POWER_DOWN_PRESS_HOLD_TIME_S) + POWER_DOWN_PRESS_HOLD_RESERVE_TIME_MS))
				return;

		PrintMsg__("Power-down!\r\n");

		GW_led_color_update();

		m_module.power_on_timeout_ms = 0;
		do {
				if(GPI_PB11_PWR_SW == LOW)
						m_module.power_on_timeout_ms = 0;
		} while(m_module.power_on_timeout_ms <= POWER_DOWN_RELEASE_HOLD_TIME_MS);

		m_module.power_down = POWER_DOWN_IDLE;

		uint8_t sleep_now = YES, keep_check = YES;
		uint8_t usb_connection_last = GPI_PB02_VBUS_DET, button_int_source;
		uint32_t adc_accum, adc_count;
		do {
				if(sleep_now) {
						PowerDown();
						m_module.power_on_timeout_ms = 0;
						timer0_init();

						button_int_source = GPI_PB11_PWR_SW == LOW? YES: NO;
						if(button_int_source == NO && usb_connection_last == LOW) {
								adc_accum = 0;
								adc_count = 0;
								GW_usb_adc_init();
						}
				}

				if(button_int_source) {
						if(GPI_PB11_PWR_SW != HIGH) {
								keep_check = m_module.power_on_timeout_ms < TIME_S_TO_MS(POWER_DOWN_PRESS_HOLD_TIME_S)? YES: NO;
								sleep_now = NO;
						}
						else
								sleep_now = YES;
				}
				else {
						if(usb_connection_last == LOW) {
								sleep_now = NO;
								if(m_module.power_on_timeout_ms >= USB_PLUG_IN_ADC_TIME_MS) {
										if((adc_accum/adc_count) >= USB_PLUG_IN_ADC_LEVEL)
												keep_check = NO;
										else
												sleep_now = YES;
										GW_usb_adc_uninit();
								}
								else {
										if(g_u32EadcInt2Flag == 1) {
												g_u32EadcInt2Flag = 0;
												uint32_t adc = EADC_GET_CONV_DATA(EADC, 2);
												adc_accum += adc;
												adc_count++;
												EADC_START_CONV(EADC, BIT2);
										}
								}
						}
						else {
								if(GPI_PB02_VBUS_DET == LOW)
										keep_check = NO;
						}
				}
		} while(keep_check);
		GW_reboot_for_power_on();
}

void reset_VDM_REQ_ACK_DATA(void)
{
		uint8_t GW_vdm_req_org[19] = {0x13, 0x11, 0x00, 0x17, 0xEF, 0xA0, VDM_HEAD_REQ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		uint8_t GW_vdm_ack_org[19] = {0x13, 0x11, 0x00, 0x17, 0xEF, 0xA0, VDM_HEAD_ACK, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

		memcpy(GW_vdm_req, GW_vdm_req_org, sizeof(GW_vdm_req_org));
		memcpy(GW_vdm_ack, GW_vdm_ack_org, sizeof(GW_vdm_ack_org));	
}

void reset_VDM_ACK_DATA(void)
{
		uint8_t GW_vdm_ack_org[19] = {0x13, 0x11, 0x00, 0x17, 0xEF, 0xA0, VDM_HEAD_ACK, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		memcpy(GW_vdm_ack, GW_vdm_ack_org, sizeof(GW_vdm_ack_org));
}

void reset_VDM_REQ_DATA(void)
{
		uint8_t GW_vdm_req_org[19] = {0x13, 0x11, 0x00, 0x17, 0xEF, 0xA0, VDM_HEAD_REQ, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		memcpy(GW_vdm_req, GW_vdm_req_org, sizeof(GW_vdm_req_org));
		//uint8_t GW_vdm_response_org[19]={0x03,0x04,0x00,0x00,0x00,0x00};		
		//memcpy(GW_vdm_response,GW_vdm_response_org,sizeof(GW_vdm_response_org));		
}

void send_atten_req(void)
{
		wire_check_need_ack_done = 0;
		GW_vdm_ack[6] = 0x06;		//VDM Header bit[4:0] 00110b attention
		need_wire_DFP1_ack = YES;
}

uint8_t enable_check_wire_mic_sendout_time = 0;
void check_wire_mic_sendout_time(void)
{
		if(enable_check_wire_mic_sendout_time == 0) {
				wire_mic_sendout_count = 0;
				return;
		}

		if(wire_mic_sendout_count > 50) {
				GPO_PC04_PD_DFP_INT = 0;
				enable_check_wire_mic_sendout_time = 0;
				pd_send_out_status = 0;
		}
}

void GW_power_off_led_done_cb(void)
{
PrintMsg__("Power-off \r\n");
}

void GW_pair_init_cb(uint8_t unpair_timeout_clear)
{
		GW_menu_init();
		system_state_update(STATE_PAIR_RX_INIT);

		GW_AW5808_RX_Set_Random_Pair_Timeout();
		GW_AW5808_RX_Enter_Binding();
		system_state_update(STATE_PAIR_RX_START);

		m_cch_rx.pair.mode_check_timeout_ticks_ms = 0;
		m_cch_rx.pair.steps = CCH_PAIR_MODE_CHECK;

		if(unpair_timeout_clear == YES)
				system_unpair_restart();
	PrintMsg__("Pair init: \r\n");
}

void GW_pair_start_cb(void)
{
		GW_AW5808_RX_Set_Random_Pair_Timeout();
		GW_AW5808_RX_Enter_Binding();
		system_state_update(STATE_PAIR_RX_START);
PrintMsg__("Pair rx start\r\n");
}

void GW_pair_mode_check_cb(void)
{
		m_cch_rx.pair.steps = CCH_PAIR_STATUS_CHECK;
	PrintMsg__("Pair check: \r\n");
}

void GW_pair_success_cb(uint8_t in_range)
{
		rx_pair_cch_count = aw5808_rx_data.cch_read_data[9];

		GW_AW5808_RX_Set_rf_power(0x22);
		GW_AW5808_RX_Set_rf_select(0x01);
		GW_AW5808_RX_Read_rf_select(&aw5808_rx_data);
		GW_AW5808_RX_Read_rf_power(&aw5808_rx_data);
		GW_AW5808_RX_Read_Module_Type(&aw5808_rx_data);
		GW_AW5808_RX_Read_Version(&aw5808_rx_data);
		GW_AW5808_RX_Set_Audio_Detection(0x00);
		GW_AW5808_RX_Set_Lose_Sync_Period(0x00);

		if(in_range == YES)
				m_module.off_range = NO;

		m_cch_rx.pair.steps = CCH_PAIR_SUCCESS;
		m_module.sync_retry_timeout_ms = 0;
		if(m_module.first_pair_check == YES) {
				GW_send_DSP_tone(DSP_TONE_PAIR_OK_CN);
				system_state_update(STATE_PAIR_SUCCESS_S);
		}
PrintMsg__("Pair OK\r\n");
}

void GW_unpair_timeout_cb(void)
{
		GW_send_DSP_tone(DSP_TONE_PAIR_NG_EN);
		system_state_update(STATE_PAIR_FAIL_S);
PrintMsg__("Timeout unpair\r\n");
}

void GW_button_power_off_cb(void)
{
		if(m_module.power_down == POWER_DOWN_IDLE) {
		//if(m_module.low_power == NO && m_module.power_down == POWER_DOWN_IDLE) {
				if(m_module.fw_upgrade_ats3607 == NO && m_module.fw_upgrade_aw5808 == NO) {
						m_module.power_down = POWER_DOWN_READY;
PrintMsg__("Button power-off\r\n");
				}
		}
}

void GW_button_unpair_cb(void)
{
		wait_PAIR_btn_release = YES;
		pre_battery_mode = 0xff;

		aw5808_rx_data.online = 0xFF;
		m_cch_rx.pair.steps = CCH_PAIR_INIT;
PrintMsg__("Button unpair\r\n");
}

uint32_t button_mute_count = 0;
void GW_button_mute_cb(void)
{
PrintMsg__("Button mute\r\n");
		//mute key only work when mic is in-call mode 
		if(m_module.system_state == SYSTEM_STATE_IN_CALL) {
				if(m_module.dsp_mode == DSP_MODE_WIRELESS_M_TO_SPK || m_module.dsp_mode == DSP_MODE_WIRELESS_M_TO_G2) {
						GW_5G_CCH_push(CCH_SYNC_BUTTON_EVENT_MUTE_TYPE);
				}
				else if(m_module.dsp_mode ==  DSP_MODE_WIRE_M_TO_SPK || m_module.dsp_mode ==  DSP_MODE_WIRE_M_TO_G2) {
						//printf("\x1b[%d;110H--- got button---\n",debug_row++);
						//GW_wire_button_key_event(VDM_BUTTON_MUTE);

						if(m_vdm.attention == NO) {
								m_vdm.attention = YES;
								m_vdm.send_timeout_ms = VDM_RESEND_PERIOD_MS;
								m_vdm.send_retry_count = VDM_SENDER_RETRY_COUNT;
								m_vdm.vdm_send_count++;
						}
				}
				else {
						//uint8_t dsp_vol_mic[] = {DSP_MAGIC, DSP_CMD_FLAG, 0x04, DSP_CMD_MUTE_REQ, m_module.mute? 0x00: 0x01, m_module.mute? 0xCB: 0xCC};
						//GW_send_cmd_to_3607D(dsp_vol_mic, 6);
				}
		}
}

void GW_usb_connect_cb(uint8_t usb)
{
		system_state_update(usb == YES? STATE_USB_CONNECTED: STATE_USB_DISCONNECT);
//PrintMsg__("USB %s!\r\n", usb == YES? "connected": "disconnect");
}

void GW_battery_charging_cb(void)
{
		system_state_update(STATE_BATTERY_CHARGING);
//PrintMsg__("Battery charge!\r\n");
}

void GW_battery_discharge_cb(void)
{
		system_state_update(STATE_BATTERY_DISCHARGE);
//PrintMsg__("Battery discharge!\r\n");
}

void GW_battery_done_cb(void)
{
		system_state_update(STATE_BATTERY_FULL_CHARGE);
//PrintMsg__("Battery done!\r\n");
}

void GW_battery_ultralow_cb(void)
{
		system_state_update(STATE_BATTERY_ULTRALOW_POWER);
//PrintMsg__("Battery ultralow!\r\n");
}

void GW_battery_low_cb(void)
{
		system_state_update(STATE_BATTERY_LOW_POWER);
//PrintMsg__("Battery low! \r\n");
}

void GW_battery_medium_cb(void)
{
		system_state_update(STATE_BATTERY_MEDIUM_POWER);
//PrintMsg__("Battery medium! \r\n");
}

void GW_battery_high_cb(void)
{
		system_state_update(STATE_BATTERY_HIGH_POWER);
//PrintMsg__("Battery high! \r\n");
}

void GW_sync_range_cb(uint8_t evt)
{
		system_state_update(evt == YES? STATE_CONNECTED_OUT_RANGE: STATE_CONNECTED_IN_RANGE);
}

void GW_firware_upgrade_aw5808_cb(uint8_t evt)
{
		system_state_update(evt == YES? STATE_FIRMWARE_UPGRADE_AW5808_START: STATE_FIRMWARE_UPGRADE_AW5808_STOP);
}

void GW_firware_upgrade_ats3607_cb(uint8_t evt)
{
		system_state_update(evt == YES? STATE_FIRMWARE_UPGRADE_ATS3607_START: STATE_FIRMWARE_UPGRADE_ATS367_STOP);
}

void GW_fw_upgrade_mcu(uint8_t evt)
{
		system_state_update(evt? STATE_FIRMWARE_UPGRADE_MCU_START: STATE_FIRMWARE_UPGRADE_MCU_STOP);
}

void GW_sync_device_type_cb(uint8_t evt)
{
		system_state_update(STATE_CONNECTED_DEVICE_TYPE);
PrintMsg__("Sync who: %02X! \r\n", evt);
}

void GW_sync_system_state_cb(uint8_t evt)
{
		system_state_update(STATE_CONNECTED_SYSTEM_NORMAL + evt);
}

void GW_sync_mute_volume_cb(uint8_t mute, uint8_t volume)
{
		system_state_update(mute == YES? STATE_CONNECTED_MUTE: STATE_CONNECTED_UNMUTE);
}

void GW_sync_unknown_cb(uint8_t evt, uint8_t params)
{
//PrintMsg__("DSP unknown evt: %02X %02X \r\n", evt, params);
}

void GW_paired_awake(uint8_t evt)
{
		system_state_update(STATE_PAIR_AWAKE_S);
PrintMsg__("Evt: Standby awake! source: %02X \r\n", evt);
}

void GW_led_second_timeout_cb(uint8_t led_mode)
{
		m_led.state_cycle[0]++;
		GW_led_state_machine();
}

void GW_system_event_init(void)
{
		memset(&m_led, 0, sizeof(m_led));
		memset(&m_cch_rx, 0, sizeof(m_cch_rx));
		memset(&m_module, 0, sizeof(m_module));
		memset(&m_debug, 0, sizeof(m_debug));
		memset(&sys_evt, NULL, sizeof(sys_evt));

		m_module.wired_module = hw_current_version? YES: NO;
    m_module.dsp_mode = DSP_MODE_INIT;

		m_led.led_mode = LED_STATUS_CONSTANT_MODE;
		m_led.color_state = LED_STATE_IDLE;

		sys_evt.pair_init_cb = GW_pair_init_cb;
		sys_evt.pair_start_cb = GW_pair_start_cb;
		sys_evt.pair_mode_check_cb = GW_pair_mode_check_cb;
		sys_evt.pair_success_cb = GW_pair_success_cb;
		sys_evt.unpair_timeout_cb = GW_unpair_timeout_cb;
		sys_evt.button_power_off_cb = GW_button_power_off_cb;
		sys_evt.button_unpair_cb = GW_button_unpair_cb;
		sys_evt.button_mute_cb = GW_button_mute_cb;
		sys_evt.usb_connect_cb = GW_usb_connect_cb;
		sys_evt.battery_charging_cb = GW_battery_charging_cb;
		sys_evt.battery_discharge_cb = GW_battery_discharge_cb;
		sys_evt.battery_ultralow_cb = GW_battery_ultralow_cb;
		sys_evt.battery_low_cb = GW_battery_low_cb;
		sys_evt.battery_medium_cb = GW_battery_medium_cb;
		sys_evt.battery_high_cb = GW_battery_high_cb;
		sys_evt.battery_done_cb = GW_battery_done_cb;
		sys_evt.sync_range_cb = GW_sync_range_cb;
		sys_evt.sync_device_type_cb = GW_sync_device_type_cb;
		sys_evt.sync_system_state_cb = GW_sync_system_state_cb;
		sys_evt.sync_mute_volume_cb = GW_sync_mute_volume_cb;
		sys_evt.sync_unknown_cb = GW_sync_unknown_cb;
		sys_evt.paired_awake = GW_paired_awake;
		sys_evt.fw_upgrade_aw5808 = GW_firware_upgrade_aw5808_cb;
		sys_evt.fw_upgrade_ats3607 = GW_firware_upgrade_ats3607_cb;
		sys_evt.fw_upgrade_mcu = GW_fw_upgrade_mcu;
		sys_evt.led_second_timeout_cb = GW_led_second_timeout_cb;

		system_state_update(STATE_POWER_ON_S);
}
