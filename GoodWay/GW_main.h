#ifndef __GW_main_H__
#define __GW_main_H__

#include <stdio.h>
#include "NuMicro.h"

#define FUNCTION_NOT_SUPPORT
#define FUNCTION_SIMULATOR_ENABLE

//#define for_console_menu
#define for_UI2_7_DOA
#define use_middle_pair

#define VERSION_MAJOR_ID              0x00
#define VERSION_MINOR_ID              0x79

// USB definition
#define WIRE_USBD_PID                 0xA064
#define WIRELESS_USBD_PID             0xA074

#define USBD_VID                      0x17EF
#define USBD_PID                      0xA074

#define GPI_PF15_SYS_FW_UPDATE				PF15
#define GPO_PA04_DOA_LED_SEL					PA4
#define	GPI_PA05_WM8804B_I2C_INT			PA5

#define GPO_PD15_TOUCH_LED_SEL				PD15
#define GPI_PC06_DSP_INT1							PC6

#define GPO_PC07_Daisy_Chain1_EN			PC7
#define GPO_PF02_Daisy_Chain2_EN			PF2

#define GPO_PF03_WWL_SW								PF3
#define GPO_PA10_TYPEC1_SW						PA10
#define GPO_PA11_DSP_ONOFF						PA11
#define GPI_PB00_GL3525_C2_VBUS_DET		PB0
#define GPI_PB01_GL3525_C1_VBUS_DET		PB1
#define GPI_PB02_VBUS_DET							PB2
#define GPI_PB03_USBC1_DFP_DET				PB3
#define GPO_PB06_TYPEC2_SW						PB6
//hi:wire	lo:wireless
#define GPO_PB07_5GRX_WWL_SW					PB7
#define GPO_PB08_5GTX_WWL_SW					PB8

#define GPI_PB09_5G_RX_MS_SEL					PB9
#define GPI_PB10_Wireless_SW					PB10
#define GPI_PB11_PWR_SW								PB11
#define GPO_PB12_USBC_UDP_SEL1				PB12
#define GPO_PB13_USBC_UDP_SEL0				PB13
#define GPO_PB14_USB2SW								PB14
#define GPO_PB15_5GTX_MS_SEL					PB15

#define GPI_PC14_USBC1_UFP_ID					PC14
#define GPO_PD02_MCU_PWREN						PD2
#define GPO_PD03_AMP_EN								PD3
#define GPI_PC02_5GTX_INT1						PC2
#define GPI_PC03_5GRX_INT1						PC3
#define GPO_PC04_PD_DFP_INT							PC4

#define GPI_PC05_MCU_I2C0_INT					PC5
#define GPO_PF04_TYPEC_UART1_EN				PF4
#define GPO_PF05_TYPEC_UART2_EN				PF5

#define EN_UART0

#define YES 1
#define NO  0

#define HIGH 1
#define LOW  0

#define ON 1
#define OFF 0

#define UFP_PC		0
#define UFP_TV		1
#define UFP_NONE 	2

#define SLAVE_5G  0
#define MASTER_5G 1

#define SLAVE_WIRE 0
#define MASTER_WIRE 1

#define WIRELESS_MODE 0
#define WIRE_MODE 1

#define HOST_AUTO 0
#define HOST_FIX	1

#define MAX_DSP_BUF	32

#define CENTRAL_START_X      1
#define CENTRAL_START_Y      16
#define SELECT_X             CENTRAL_START_X
#define SELECT_Y             CENTRAL_START_Y
#define SELECT_Y_COUNT       8
#define GPIO_START_X         CENTRAL_START_X
#define GPIO_START_Y         (CENTRAL_START_Y + SELECT_Y_COUNT)
#define GPIO_Y_COUNT         14
#define CCH_START_X          CENTRAL_START_X
#define CCH_START_Y          (GPIO_START_Y + GPIO_Y_COUNT + 1)
#define CCH_START_Y_COUNT    5
#define DSP_START_X          CENTRAL_START_X
#define DSP_START_Y          (CCH_START_Y + CCH_START_Y_COUNT + 1)
#define DSP_START_Y_COUNT    5

// Delay in: 10~N ms OK
#define POWER_ON_DELAY_MS		100

typedef struct MENU_KEY {
	uint8_t buf[16];
	uint8_t start;
	uint8_t end;
} MENU_KEY;

#define DFP_CONNECT			0x01
#define DFP_CABLE_PASS	0x02
#define DFP_PD_IS_SPK		0x04
#define DFP_PD_IS_G2		0x08
#define DFP_PD_IS_MIC		0x10
#define DFP_PD_IS_M			0x20

#define DFP_NAK					0x01
#define DFP_TIMEOUT			0x02
#define DFP_HS					0x04
#define DFP_DISCONNECT	0x08
#define DFP_C_MISMATCH	0x10
#define DFP_P_MISMATCH	0x20

#define CCH_VER									0x40	//BIT 7..6 : 01
#define CCH_REQ									0x00  //BIT 5..4
#define CCH_ACK									0x10	//BIT 5..4
#define CCH_NAK									0x20	//BIT 5..4

#if 1
#define DSP_MODE_INIT               0
#define DSP_MODE_SINGLE             1
#define DSP_MODE_WIRE_M_TO_SPK      31
#define DSP_MODE_WIRE_M_TO_G2       33
#define DSP_MODE_WIRELESS_M_TO_SPK  32
#define DSP_MODE_WIRELESS_M_TO_G2   34


#define DSP_MODE_SINGLE_CMD         0x31
#define DSP_MODE_WIRED_CMD          0x33
#define DSP_MODE_WIRELESS_CMD       0x35

#else
#define DSP_MODE_31_M_ONE				0x31
#define DSP_MODE_36_M_WIRELESS 	0x36
#define DSP_MODE_39_S_WIRELESS	0x39
#define DSP_MODE_36_M_WIRE_DFP1	0xB6
#define DSP_MODE_37_S_WIRE_DFP1	0x37
#define DSP_MODE_38_M_WIRE_DFP2 0x38
#define DSP_MODE_37_S_WIRE_DFP2 0xB7
#define DSP_MODE_33_S_WIRE_DFP1 0x33
#define DSP_MODE_35_S_WIRELESS 	0x35
#endif

#define DSP_MAGIC								0xA5
#define DSP_CMD_FLAG						0x01
#define DSP_EVENT_FLAG					0x04

#define DSP_CMD_SLAVE_VOL				0x04
#define DSP_CMD_SN							0x05
#define DSP_CMD_POWER_OFF_POP		0x06
#define DSP_CMD_VERSION					0x07
#define DSP_CMD_SLAVE_MIC_MUTE	0x08
#define DSP_CMD_MUTE_REQ				0x21	//DSP need sned request to OS and return mute event
#define DSP_CMD_VOL_UP_KEY			0x25
#define DSP_CMD_VOL_DN_KEY			0x26
#define DSP_CMD_START_PLAY			0x2D  //send cmd to DSP when MCU enable AMP power
#define DSP_CMD_ISP							0x41
#define DSP_CMD_TONE						0x44

#define DSP_TONE_MUTE						0x00  //mute/unmute
#define DSP_TONE_VOL						0x01  //vol +/-
#define DSP_TONE_MAX_MIN_VOL		0x02	//max/min volume
#define DSP_TONE_PAIR_OK_CN			0x03	//pair success - chinese
#define DSP_TONE_PAIR_OK_EN			0x04  //pair success - english
#define DSP_TONE_PAIR_NG_CN			0x05  //pair fail - chinese
#define DSP_TONE_PAIR_NG_EN			0x06  //pair fail - english
#define DSP_TONE_DISCON_CN			0x07	//wireless disconnect - chinese
#define DSP_TONE_DISCON_EN			0x08  //wireless disconnect - english
#define DSP_TONE_CON_CN					0x09	//wireless connect - chinese
#define DSP_TONE_CON_EN					0x0A	//wireless connect - english
#define DSP_TONE_SW_HOST				0x0B  //host switch

#define DSP_EVENT_UAC_ST				0x01  //DSP status
#define DSP_EVENT_RX_VOL				0x02
#define DSP_EVENT_MUTE					0x04
#define DSP_EVENT_SN						0x05
#define DSP_EVENT_VERSION				0x07
#define DSP_EVENT_ISP_FAIL			0x06
#define DSP_EVENT_ISP_SUCCESS		0x08
#define DSP_EVENT_READY					0x0A
#define DSP_EVENT_VOICE_DETECT	0x20
#define DSP_EVENT_DOA						0x21
#define DSP_EVENT_CMD_COMPLETE	0x61

#define VDM_HEAD_REQ									0x10
#define VDM_HEAD_ACK									0x50

#define VDM_BUTTON_VOL_UP							0x40
#define VDM_BUTTON_VOL_DN							0x60
#define VDM_BUTTON_MUTE								0x20

#define WHO_IS_NO								0x00
#define WHO_IS_G2								0x01
#define WHO_IS_SPK							0x02
#define WHO_IS_MIC							0x03

#define USB_DSP_RAW_CMD 					0x01
#define USB_DSP_ISP_CMD 					0x02
#define USB_REQ_INTO_FACTORY			0x42
#define USB_REQ_EXIT_FACTORY			0x41
#define USB_REQ_LED_FULL_WHITE		0x1A
#define USB_REQ_LED_COLOR_SET			0x1B
#define USB_REQ_BATTERY_INFO			0x2A
#define USB_REQ_RX_LEVEL_INFO			0x2B
#define USB_REQ_GET_TX_ID					0x08
#define USB_REQ_GET_RX_ID					0x09
#define USB_REQ_RECOVERY					0x0E
#define USB_REQ_SET_TX_ID					0x0A
#define USB_REQ_SET_RX_ID					0x0B
#define USB_REQ_TOUCH_GET					0x0C
#define USB_REQ_RF_POWER_SET			0x0D
#define USB_DSP_READ_LAST_DSP 		0xFF

extern volatile uint8_t DFP1_connect;
extern volatile uint8_t DFP2_connect;
void GW_led_lock(uint8_t long_lock);
void GW_led_unlock(void);

void GW_check_PD_data(uint8_t *read_data);

void clear_screen(void);
void GW_menu_init(void);
void GW_menu_loop(void);
void SYS_LED_Init(void);
void GW_LED_init(void);
void GW_LED_loop(void);
void show_GPIO_menu_status(void);
void GW_gpio_control(uint8_t key);
extern MENU_KEY menu_key;
void GW_host_switch(void);
void GW_mcu_switch(void);
void led_mic_cal(void);
void GW_write_LDROM(void);
void SYS_ADC_Init(void);
void GW_battery_adc_init(void);
uint8_t check_5G_sync_status(void);
void PowerDown();
void GW_power_down_interface(void);
void GW_boot_to_LDROM(uint8_t boot);
void GW_send_cmd_to_3607D(uint8_t *buf, uint8_t len);
void GW_AW5808_TX_Send_CCH_Data(uint8_t *data, uint8_t data_len);
void GW_USB_send_to_DSP_ISP_request(void);
void GW_wire_process_master_req(uint8_t* data, uint8_t _data_len);
void GW_wire_button_key_event(uint8_t button);
void GW_send_DSP_tone(uint8_t tone);
void GW_response_USB_direct(void);
void GW_reboot_for_power_on(void);

extern volatile uint8_t ats3607d_in_buf[];
extern volatile uint8_t ats3607d_in_start, ats3607d_in_end ;
extern uint32_t timer_ms_delay_count;

extern volatile uint8_t wire_vdm_button;
extern uint8_t USB_raw_request[], USB_raw_response[];
extern uint8_t USB_raw_req_len, USB_raw_res_len;
extern volatile uint8_t factory_mode;

extern uint8_t GW_vdm_req[], GW_vdm_ack[];

extern uint8_t check_aw5808_tx_update_flag, check_aw5808_rx_update_flag;

void Show_VDM_Slave_Data(void);
void GW_Power_Down_Mode(void);

extern uint8_t i2c_need_re_init;
extern void check_i2c_need_re_init(void);
extern volatile uint32_t ats3607_update_used_ms;
void check_first_time_boot_up_pair_mode(void);

extern uint8_t exec_power_off_enable;
void exec_power_off_mode(void);
void GW_Check_AutoConnection(void);

extern uint8_t hw_current_version;
extern uint16_t usb_current_vid;
extern volatile uint8_t satellite_mode;
extern volatile uint32_t I2C_READY_TMRINT_count;
extern volatile uint8_t get_usb_cmd;
extern volatile uint8_t i2c_init;
extern volatile uint32_t BATTERY_TMRINT_count;
extern volatile uint32_t BATTERY_TMRINT_status_count;
extern volatile uint32_t dfu_scan_time_count_2;
extern volatile uint32_t UI2C_READY_TMRINT_count;

extern volatile uint8_t standby_mode_start_count;
extern uint8_t pre_battery_mode;
extern volatile uint8_t send_attention;

extern void reset_VDM_REQ_DATA(void);
extern void reset_VDM_ACK_DATA(void);
extern void send_atten_req(void);
extern void GW_wire_check_need_req(void); 
extern void GW_USCI_I2C_On_Off(uint8_t _value);
extern uint8_t debug_row;
extern volatile uint8_t check_autoconnection_start;
extern volatile uint32_t check_auto_connection_wait_count;

extern volatile uint32_t TEMPERATURE_TMRINT_count;
extern int32_t GW_Battery_Temperature_check(void);

extern volatile uint32_t g_u32EadcInt0Flag;
extern volatile uint32_t g_u32EadcInt1Flag;
extern uint32_t adc_count;
extern uint32_t adc_temp_count;

extern uint8_t power_level_check_count;
extern void disable_low_power_check_function(void);
extern void enable_low_power_check_function(void);

extern uint32_t standby_mode_detect_time;
extern volatile uint32_t RESET_TMRINT_count;
extern uint8_t pre_wireless_mode;
extern volatile uint32_t wire_mic_sendout_count;
extern uint8_t enable_check_wire_mic_sendout_time;
extern void check_wire_mic_sendout_time(void);
extern uint8_t pd_send_out_status;
extern uint8_t wire_check_need_ack_done;

//extern volatile uint8_t check_wire_button_timeout_enable;
//extern volatile uint8_t check_wireless_button_timeout_enable;
extern uint8_t wire_seq_num;
extern uint32_t unpair_wait_count;
extern uint8_t exec_unpair_action;

#define TIMER0_FREQ                                     1000

#define DOA_LED_total                                   8
#define DOA_size                                        (DOA_LED_total*9)
#define DOA_FPS                                         50
#define DOA_TOTAL_STEP                                  2

#define	LED_MANAGER_TEST
#define STATE_MACHINE_TEST
//#define TEST_ENABLE

//#define IGNORE_PRINTF

#define LOG_GPIO_INDEX                                  1
#define TIME_S_TO_MS(x)                                 (x*1000)
#ifndef TEST_ENABLE
#define UNPAIR_TIMEOUT_S                                120
#define FIRST_BOOT_UP_PAIR_MODE_CHECK_TIME_S            10
#else
#define UNPAIR_TIMEOUT_S                                30
#define FIRST_BOOT_UP_PAIR_MODE_CHECK_TIME_S            5
#endif
#define BUTTON_UNPAIR_TIMEOUT_S                         3
#define BUTTON_UNPAIR_RELEASE_TIMEOUT_MS                200
#define POWER_UP_RELEASE_TIME_MS                        50
#define POWER_DOWN_RELEASE_HOLD_TIME_MS                 500
#define POWER_DOWN_PRESS_HOLD_TIME_S                    3
#define POWER_DOWN_PRESS_HOLD_RESERVE_TIME_MS           500
#define POWER_DOWN_USB_HOLD_TIME_S                      1
#define POWER_DOWN_USB_HOLD_TIME_MS                     10
#define POWER_DOWN_WAIT_TIME_S                          5
#define FACTORY_PRESS_HOLD_TIME_S                       5
#define AW5808_FIRST_SYNC_TIME_MS                       500
#define AW5808_SYNC_TIME_MS                             2000
#define AW5808_NOSYNC_MAX_TIME_MS                       3000
#define AW5808_NOSYNC_MIN_TIME_MS                       0
#define AW5808_LOSS_SYNC_TIME_S                         1
#define STANDBY_WAKEUP_TIME_S                           1

#define POWER_ULTRALOW_CHECK_COUNT                      5
#define POWER_LOW_CHECK_COUNT                           10

#define LED_HIGH_LOW_SAMPLE_RATE                        (DOA_FPS*2)
#define LED_HIGH_LOW_SAMPLE_PERIOD_MS                   (TIMER0_FREQ/LED_HIGH_LOW_SAMPLE_RATE)
#define LED_COLOR_UPDATE_RATE_MASK                      0x80
#define LED_STATUS_CONSTANT_MODE                        1
#define LED_STATUS_CONSTANT_RETAIN_MODE                 2
#define LED_STATUS_BLINKY_MODE                          3
#define LED_STATUS_BREEZE_MODE                          4

#define POWER_ON_BLINKY_TIME_S                          3
#define POWER_OFF_BLINKY_TIME_S                         2
#define POWER_CONNECTED_SUCCESS_TIME_S                  3
#define POWER_PAIR_FAIL_TIME_S                          3
#define POWER_UNPAIR_TIME_S                             5
#define POWER_STANDBY_AWAKE_TIME_S                      2
#define POWER_ULTRALOW_PERIOD_S                         1
#define POWER_LOW_PERIOD_1_S                            10
#define POWER_LOW_PERIOD_2_S                            5
#define LED_BLINKY_ON_DUTY                              3
#define LED_BLINKY_ONOFF_DUTY                           5

#define CCH_PAIR_ID                                     0x00
#define CCH_PAIR_ID_OLD                                 0xFF
#define	CCH_PAIR_CHECK_TIMEOUT_MS                       50
#define	CCH_PAIR_CHECK_ID_0_TIMEOUT_S                   1
#define	CCH_RX_SYNC_RETRY_COUNT                         100
#define	CCH_RX_RESEND_PERIOD_MS                         100
#define	CCH_RX_SENDER_RETRY_COUNT                       5
#define	CCH_RX_SEND_PERIOD_MIN_MS                       40
#define	CCH_RX_SEQUENCE_NUMBER_MAX                      255
#define	CCH_RX_ACCESS_MAX_PERIOD_MS                     10
#define	CCH_RX_ACCESS_MIN_PERIOD_MS                     0
#define	CCH_RX_ACCESS_TIMEOUT_MS                        100

#define	VDM_RESEND_PERIOD_MS                            300
#define	VDM_SENDER_RETRY_COUNT                          3

#ifndef TEST_ENABLE
#define STANBY_MODE1_PAIRED_WITHOUT_CONNECTION_S                        300
#define STANBY_MODE2_PAIRED_AND_CHARGING_WITHOUT_CONNECTION_S           1800
#define STANBY_MODE3_PAIRED_AND_CONNECTED_WITHOUT_CALL_S                1800
#define STANBY_MODE_LED_BREEZE_TIMEOUT_S                                300
#define STANBY_MODE_LED_OFF_TIMEOUT_S                                   300
#else
#define STANBY_MODE1_PAIRED_WITHOUT_CONNECTION_S                        20
#define STANBY_MODE2_PAIRED_AND_CHARGING_WITHOUT_CONNECTION_S           STANBY_MODE1_PAIRED_WITHOUT_CONNECTION_S
#define STANBY_MODE3_PAIRED_AND_CONNECTED_WITHOUT_CALL_S                STANBY_MODE1_PAIRED_WITHOUT_CONNECTION_S
#define STANBY_MODE_LED_BREEZE_TIMEOUT_S                                10
#define STANBY_MODE_LED_OFF_TIMEOUT_S                                   5
#endif

#define	USB_PLUG_IN_ADC_TIME_MS                         500
#define	USB_PLUG_IN_ADC_LEVEL                           2400

typedef enum {
		BUTTON_EVENT_IDLE,
		BUTTON_EVENT_CLEAR_LOW,
		BUTTON_EVENT_POWER_OFF,
		BUTTON_EVENT_WAKE_UP,
		BUTTON_EVENT_REBOOT_RESET,
		BUTTON_EVENT_MUTE,
} BUTTON_EVENT_E;

typedef enum {
		BATTERY_EVENT_IDLE,
		BATTERY_EVENT_CHARGED,
		BATTERY_EVENT_DISCHARGE,
		BATTERY_EVENT_FULL_CHARGED
} BATTERY_EVENT_E;

typedef enum {
		SYSTEM_STATE_IDLE,
		SYSTEM_STATE_NORMAL,
		SYSTEM_STATE_STANDBY,
		SYSTEM_STATE_INACTIVE,
		SYSTEM_STATE_IN_CALL,
} SYSTEM_STATE_TYPE_E;

typedef enum {
		POWER_DOWN_IDLE,
		POWER_DOWN_READY,
		POWER_DOWN_READY_LED,
		POWER_DOWN_NOW,
} POWER_DOWN_STSTE_MACHINE_E;

typedef enum {
		LED_IDLE,
		LED_POWER_ON_PEND,
		LED_POWER_OFF_PEND,
		LED_POWER_ULTRALOW_BLINKY,
		LED_POWER_LOW_BLINKY,
		LED_CONNECTED_SUCCESS_PEND,
		LED_PAIR_FAIL_BLINKY,
		LED_STANDBY_AWAKE,
		LED_STATE_COUNT,
} LED_STATE_MACHINE_E;

typedef enum {
		LED_STATE_IDLE,
		LED_STATE_POWER_DOWN,
		LED_STATE_ULTRALOW_POWER_BLINKY,
		LED_STATE_LOW_POWER_BLINKY,
		LED_STATE_LOW_POWER_CONSTANT,
		LED_STATE_POWER_ON,
		LED_STATE_POWER_ON_FIRST_BOOT,
		LED_STATE_STANDBY_POWER_DOWN,
		LED_STATE_STANDBY_POWER_ON,
		LED_STATE_STANDBY_BREEZE,
		LED_STATE_STANDBY_OFF,
		LED_STATE_FIRMWARE_UPGRADE,
		LED_STATE_PAIR_FAIL_POWER_DOWN,
		LED_STATE_PAIR_FAIL,
		LED_STATE_CONNECTION_FAIL,
		LED_STATE_CONNECTION_INACTIVE,
		LED_STATE_CONNECTION_MUTE,
		LED_STATE_CONNECTION_STANDBY,
		LED_STATE_CONNECTION_CALL,
		LED_STATE_CONNECTION_IDLE,
		LED_STATE_PAIR_SUCCESS,
		LED_STATE_PAIR_RETRY,
		LED_STATE_NUMBER,
} LED_STATE_E;

typedef enum {
		POWER_LOW_BLINKY,
		POWER_LOW_CONSTANT_OFF,
} POWER_LOW_STSTE_MACHINE_E;

typedef enum {
		PAIR_STATE_RETRY,
		PAIR_STATE_TRYING,
		PAIR_STATE_SUCCESS,
		PAIR_STATE_FAIL,
} PAIR_STSTE_MACHINE_E;

typedef enum {
		WIRELESS_PAIR_STATE_IDLE,
		WIRELESS_PAIR_STATE_PAIRING,
		WIRELESS_PAIR_STATE_FINISHED,
} WIRELESS_PAIR_STSTE_MACHINE_E;

typedef enum {
		STANDBY_IDLE,
		STANDBY_PAIRED_WITHOUT_CONNECTION,
		STANBY_PAIRED_AND_CHARGING_WITHOUT_CONNECTION,
		STANBY_PAIRED_AND_CONNECTING_WITHOUT_CALL,
} STANDBY_STATUS_E;

typedef enum {
		STATE_STANDBY_IDLE,
		STATE_STANDBY_LED_BREEZE,
		STATE_STANDBY_LED_OFF,
		STATE_STANDBY_LED_AWAKE,
} STANDBY_STATE_MACHINE_E;

typedef enum {
		AWAKE_IDLE,
		AWAKE_BUTTON,
		AWAKE_CHARGING,
		AWAKE_CALL,
} PAIR_AWAKE_SOURCE_E;

typedef enum {
		FIRMWARE_UPGRADE_IDLE,
		FIRMWARE_UPGRADE_AW5808,
		FIRMWARE_UPGRADE_ATS3607,
} FIRMWARE_UPGRADE_SOURCE_E;

typedef enum {
    STATE_INIT,
		STATE_EVENT_UPDATED,
		STATE_ULTRA_LOW_POWER_OFF,
		STATE_POWER_ON,
    STATE_POWER_ON_S = STATE_POWER_ON,
    STATE_POWER_ON_W,
    STATE_POWER_ON_E,
		STATE_POWER_OFF,
    STATE_POWER_OFF_S = STATE_POWER_OFF,
    STATE_POWER_OFF_W,
		STATE_POWER_OFF_E,
		STATE_PAIR_RX_INIT,
		STATE_PAIR_RX_START,
		STATE_PAIR_RX_WAIT,
		STATE_PAIR_SUCCESS,
    STATE_PAIR_SUCCESS_S = STATE_PAIR_SUCCESS,
    STATE_PAIR_SUCCESS_W,
		STATE_PAIR_SUCCESS_E,
		STATE_PAIR_FAIL,
    STATE_PAIR_FAIL_S = STATE_PAIR_FAIL,
    STATE_PAIR_FAIL_W,                              //0x10
		STATE_PAIR_FAIL_E,
		STATE_UNPAIRED,
		STATE_PAIR_STANDBY_LED_BREEZE,
		STATE_PAIR_STANDBY_LED_OFF,
		STATE_PAIR_STANDBY_LED_POWER_OFF,
		STATE_PAIR_AWAKE,
		STATE_PAIR_AWAKE_S = STATE_PAIR_AWAKE,
		STATE_PAIR_AWAKE_W,
		STATE_PAIR_AWAKE_E,
		STATE_BATTERY_ULTRALOW_POWER,
		STATE_BATTERY_LOW_POWER,
		STATE_BATTERY_MEDIUM_POWER,
		STATE_BATTERY_HIGH_POWER,
		STATE_USB_CONNECTED,
		STATE_USB_DISCONNECT,
		STATE_BATTERY_CHARGING,
		STATE_BATTERY_DISCHARGE,                         //0x20
		STATE_BATTERY_FULL_CHARGE,
		STATE_CONNECTED_DEVICE_TYPE,
		STATE_CONNECTED_SYSTEM_NORMAL,
		STATE_CONNECTED_SYSTEM_STANDBY,
		STATE_CONNECTED_SYSTEM_INACTIVE,
		STATE_CONNECTED_SYSTEM_IN_CALL,
		STATE_CONNECTED_UNMUTE,
		STATE_CONNECTED_MUTE,
		STATE_CONNECTED_IN_RANGE,
		STATE_CONNECTED_OUT_RANGE,
		STATE_UP_PORT_CONNECTED,
		STATE_UP_PORT_DISCONNECT,
		STATE_FIRMWARE_UPGRADE_AW5808_START,
		STATE_FIRMWARE_UPGRADE_AW5808_STOP,
		STATE_FIRMWARE_UPGRADE_ATS3607_START,
		STATE_FIRMWARE_UPGRADE_ATS367_STOP,              //0x30
		STATE_FIRMWARE_UPGRADE_MCU_START,
		STATE_FIRMWARE_UPGRADE_MCU_STOP,
} SYSTEM_STSTE_MACHINE_E;

typedef enum {
		CCH_PAIR_INIT_CHECK,
		CCH_PAIR_INIT,
		CCH_PAIR_MODE_CHECK,
		CCH_PAIR_STATUS_CHECK,
		CCH_PAIR_SUCCESS,
} CCH_PAIR_E;

typedef enum {
		CCH_SYNC_DEVICE_TYPE_SYNC_TYPE,
		CCH_SYNC_SYSTEM_COMMAND_TYPE,
		CCH_SYNC_AUDIO_VOLUME_SYNC_TYPE,
		CCH_SYNC_BUTTON_EVENT_MUTE_TYPE,
		CCH_SYNC_STATUS_COUNT,
} CCH_TYPE_E;

#define CCH_SYNC_DEVICE_TYPE_SYNC_TYPE_BIT_FIELD      (1 << CCH_SYNC_DEVICE_TYPE_SYNC_TYPE)
#define CCH_SYNC_SYSTEM_COMMAND_TYPE_BIT_FIELD        (1 << CCH_SYNC_SYSTEM_COMMAND_TYPE)
#define CCH_SYNC_AUDIO_VOLUME_SYNC_TYPE_BIT_FIELD     (1 << CCH_SYNC_AUDIO_VOLUME_SYNC_TYPE)
#define CCH_SYNC_STATE_INIT_BIT_FIELD                 (CCH_SYNC_DEVICE_TYPE_SYNC_TYPE_BIT_FIELD | CCH_SYNC_SYSTEM_COMMAND_TYPE_BIT_FIELD | CCH_SYNC_AUDIO_VOLUME_SYNC_TYPE_BIT_FIELD)

typedef void (* system_evt_cb)(void);

typedef void (* system_evt_no_cb)(uint8_t evt);
typedef void (* system_evt_2no_cb)(uint8_t evt1, uint8_t evt2);
typedef void (* system_evt_params_cb)(uint8_t evt, uint8_t params);

typedef struct {
		uint8_t   init:           1;
		uint8_t   update:         1;
		uint8_t   rsv:            6;
    uint8_t   hz;
    uint8_t   cycle_index;
		uint8_t   cycle_step;
		uint8_t   mode;
		uint8_t   update_count;
		uint8_t   side_color_now;
		uint8_t   center_color_now;
		uint8_t   side_colors[3];
		uint8_t   center_colors[3];
		uint16_t  second_index;
		uint8_t   led_mode;
		uint8_t   color_state;
		uint8_t   rsv2;
		uint8_t   state_cycle[LED_STATE_COUNT];
		uint8_t   state_buffer[LED_STATE_COUNT];
		uint32_t  step_ms;
		uint32_t  init_step_ms;
		uint32_t  update_timeout_ms;
} led_status_s;

typedef struct
{
    system_evt_cb          first_boot_up_pair_check_cb;
    system_evt_no_cb       pair_init_cb;
    system_evt_cb          pair_start_cb;
    system_evt_cb          pair_mode_check_cb;
    system_evt_no_cb       pair_success_cb;
    system_evt_cb          pair_fail_cb;
    system_evt_cb          unpair_timeout_cb;
    system_evt_cb          button_power_off_cb;
    system_evt_cb          button_unpair_cb;
    system_evt_cb          button_mute_cb;
		system_evt_no_cb       usb_connect_cb;
    system_evt_cb          battery_charging_cb;
    system_evt_cb          battery_discharge_cb;
    system_evt_cb          battery_ultralow_cb;
    system_evt_cb          battery_low_cb;
    system_evt_cb          battery_medium_cb;
    system_evt_cb          battery_high_cb;
    system_evt_cb          battery_done_cb;
		system_evt_no_cb       fw_upgrade_aw5808;
		system_evt_no_cb       fw_upgrade_ats3607;
		system_evt_no_cb       fw_upgrade_mcu;
    system_evt_no_cb       sync_range_cb;
    system_evt_no_cb       sync_device_type_cb;
    system_evt_no_cb       sync_system_state_cb;
    system_evt_2no_cb      sync_mute_volume_cb;
    system_evt_params_cb   sync_unknown_cb;
    system_evt_no_cb       paired_awake;
    system_evt_no_cb       led_second_timeout_cb;
} system_evt_cb_s;

typedef struct {
		uint8_t     mute:                     1;
		uint8_t     power_off:                1;
		uint8_t     pair:                     1;
		uint8_t     rsv:                      5;
} bt_lock_s;

typedef struct {
		uint8_t     lengh;
		uint8_t     buffer[63];
} buffer_s;

typedef struct {
    uint8_t   power_init:                 1;
    uint8_t   battery_init:               1;
    uint8_t   first_boot:                 1;
    uint8_t   first_pair_check:           1;
		uint8_t   button_init:                1;
    uint8_t   wired_module:               1;
    uint8_t   wireless_pair:              2;
    uint8_t   wired_pair:                 1;
		uint8_t   connected:                  1;
    uint8_t   mute:                       1;
    uint8_t   system_state:               3;
    uint8_t   charging:                   1;
    uint8_t   usb_connected:              1;
		uint8_t   ultralow_power:             1;
		uint8_t   power_check:                1;
		uint8_t   low_power:                  1;
		uint8_t   low_power_led:              1;
		uint8_t   off_range:                  1;
		uint8_t   standby:                    2;
		uint8_t   standby_work:               1;
		uint8_t   standby_evt:                2;
		uint8_t   pair_fail_power_down:       1;
		uint8_t   pair_wake:                  1;
		uint8_t   power_down:                 2;
		uint8_t   fw_upgrade_aw5808:          1;
		uint8_t   fw_upgrade_ats3607:         1;
		uint8_t   fw_upgrade_mcu:             1;
		uint8_t   attention:                  1;
		uint8_t   charge_status:              2;
		uint8_t   vdm_ready:                  1;
		uint8_t   led_lock:                   1;
		uint8_t   led_lock_long:              1;
		uint8_t   rsv:                        1;     // 40-bits
		uint8_t   who;
		buffer_s  rx;
		buffer_s  tx;
		uint8_t   rsv1[2];
		bt_lock_s button_lock;
		uint8_t   dsp_mode;
		uint8_t   button_evt;
		uint8_t   state;
		uint8_t   state_last;
		uint16_t  battery_level;
		uint8_t   battery_level_count;
		uint32_t	current_time_ms;
		uint32_t	sync_retry_timeout_ms;
		uint32_t	button_power_timeout_ms;
		uint32_t	button_mute_timeout_ms;
		uint32_t	button_unpair_timeout_ms;
		uint32_t	power_on_timeout_ms;
		uint32_t	standby_timeout_ms;
		uint32_t	standby_timeout_s;
		uint32_t	wakeup_timeout_ms;
		uint32_t	dsp_update_time_ms;
		uint32_t	led_lock_time_ms;
		uint32_t	led_lock_timeout_ms;
		uint32_t	i2c_timeout_ms;
} module_status_s;

typedef struct {
    uint8_t     steps;
		uint8_t     rsv[3];
    uint8_t     id[4];
    uint32_t    steps_timeout_ticks_ms;
    uint32_t    mode_check_timeout_ticks_ms;
    uint32_t    pairing_timeout_ticks_ms;
} cch_pair_s;

typedef struct {
    uint8_t     retry_count;
    uint8_t     seq_num;
    uint8_t     seq_num_last;
    uint8_t     size;
		uint8_t     connected:                     1;
		uint8_t     state_init:                    3;
		uint8_t     long_nosync:                   1;
		uint8_t     long_sync:                     1;
		uint8_t     id_confirm:                    1;
		uint8_t     rsv:                           1;
    uint8_t     status_count;
    uint8_t     buffer[CCH_SYNC_STATUS_COUNT];
		uint8_t     req[CCH_SYNC_STATUS_COUNT];
    uint32_t    rx_access_period_ms;
    uint32_t    rx_access_timeout_ms;
    uint32_t    time_ticks_ms;
    uint32_t    send_period_time_ticks_ms;
		cch_pair_s  pair;
		uint8_t     status;
    uint16_t    cch_int_count;
    uint16_t    cch_lose_sync_count;
		uint8_t     req_seq_num;
		uint8_t     long_nosync_count;
		uint8_t     rsv1[3];
} cch_module_s;

typedef struct {
		uint8_t   attention:                  1;
		uint8_t   rsv:                        7;
		uint8_t   send_retry_count;
		uint8_t   rsv1[2];
    uint16_t  vdm_send_count;
    uint16_t  vdm_timeout_count;
    uint16_t  vdm_resend_count;
    uint16_t  vdm_spend_time_ms;
		uint32_t  send_timeout_ms;
} vdm_module_s;

typedef struct {
    uint8_t     pair_status;
    uint8_t     pair_steps;
		uint8_t     rsv[2];
    uint32_t    pair_id;
} tips_s;

typedef struct {
    uint8_t     cch_rx_log_index;
    uint8_t     cch_tx_log_index;
    uint8_t     dsp_rx_log_index;
    uint8_t     dsp_tx_log_index;
    uint16_t    cch_int_count;
    uint16_t    cch_read_count;
    uint16_t    cch_recv_count;
    uint16_t    range_in_count;
    uint16_t    range_out_count;
    uint16_t    reconnect_count;
    uint16_t    inactive_count_test;
    uint16_t    rsv2;
		uint16_t    led_count[LED_STATE_NUMBER];
    uint16_t    normal_count;
    uint16_t    standby_count;
    uint16_t    inactive_count;
    uint16_t    call_count;
    uint16_t    err_seq_num_count;
    uint32_t    cch_int_time_ms;
    uint32_t    cch_read_time_ms;
    uint32_t    cch_read_last_time_ms;
    uint32_t    led_init_time_ms;
    uint32_t    led_begin_time_ms;
} debug_s;

extern system_evt_cb_s	sys_evt;
uint8_t time_to_work(uint32_t * time_ms, uint32_t time_period_ms);

void GW_system_event_init(void);

extern volatile uint8_t  rx_pair_cch_count;
extern module_status_s   m_module;
extern cch_module_s      m_cch_rx;
extern debug_s           m_debug;
#endif  /* __NUMICRO_H__ */
