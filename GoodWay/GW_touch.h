#ifndef __GW_touch_H__
#define __GW_touch_H__

#include <stdio.h>
#include "NuMicro.h"

#define TOUCH_ADDR 0x37
#define REG_ADDR_SENSOR_EN 0x00
#define REG_ADDR_FSS_EN 	 0x02
#define REG_ADDR_TOGGLE_EN 	 0x04
#define REG_ADDR_LED_ON_EN 	 0x06
#define REG_ADDR_SENSITIVITY0 0x08
#define REG_ADDR_SENSITIVITY1 0x09
#define REG_ADDR_SENSITIVITY2 0x0a
#define REG_ADDR_SENSITIVITY3 0x0b
#define REG_ADDR_BASE_THRESHOLD0 0x0c
#define REG_ADDR_BASE_THRESHOLD1 0x0d
#define REG_ADDR_FINGER_THRESHOLD2 0x0e
#define REG_ADDR_FINGER_THRESHOLD3 0x0f
#define REG_ADDR_FINGER_THRESHOLD4 0x10
#define REG_ADDR_FINGER_THRESHOLD5 0x11
#define REG_ADDR_FINGER_THRESHOLD6 0x12
#define REG_ADDR_FINGER_THRESHOLD7 0x13
#define REG_ADDR_FINGER_THRESHOLD8 0x14
#define REG_ADDR_FINGER_THRESHOLD9 0x15
#define REG_ADDR_FINGER_THRESHOLD10 0x16
#define REG_ADDR_FINGER_THRESHOLD11 0x17
#define REG_ADDR_FINGER_THRESHOLD12 0x18
#define REG_ADDR_FINGER_THRESHOLD13 0x19
#define REG_ADDR_FINGER_THRESHOLD14 0x1a
#define REG_ADDR_FINGER_THRESHOLD15 0x1b
#define REG_ADDR_SENSOR_DEBOUNCE 0x1c
#define REG_ADDR_BUTTON_HYS 0x1d
#define REG_ADDR_BUTTON_LBR 0x1f
#define REG_ADDR_BUTTON_NNT 0x20
#define REG_ADDR_BUTTON_NT 0x21
#define REG_ADDR_PROX_EN 0x26
#define REG_ADDR_PROX_CFG 0x27
#define REG_ADDR_PROX_CFG2 0x28
#define REG_ADDR_PROX_TOUCH_TH0 0x2a
#define REG_ADDR_PROX_TOUCH_TH1 0x2c
#define REG_ADDR_PROX_RESOLUTION0 0x2e
#define REG_ADDR_PROX_RESOLUTION1 0x2f
#define REG_ADDR_PROX_HYS 0x30
#define REG_ADDR_PROX_LBR 0x32
#define REG_ADDR_PROX_NNT 0x33
#define REG_ADDR_PROX_NT 0x34
#define REG_ADDR_PROX_POSITIVE_TH0 0x35
#define REG_ADDR_PROX_POSITIVE_TH1 0x36
#define REG_ADDR_PROX_NEGATIVE_TH0 0x39
#define REG_ADDR_PROX_NEGATIVE_TH1 0x3a
#define REG_ADDR_LED_ON_TIME 0x3d
#define REG_ADDR_BUZZER_CFG 0x3e
#define REG_ADDR_BUZZER_ON_TIME 0x3f
#define REG_ADDR_GPO_CFG 0x40
#define REG_ADDR_PWM_DUTYCYCLE_CFG0 0x41
#define REG_ADDR_PWM_DUTYCYCLE_CFG1 0x42
#define REG_ADDR_PWM_DUTYCYCLE_CFG2 0x43
#define REG_ADDR_PWM_DUTYCYCLE_CFG3 0x44
#define REG_ADDR_PWM_DUTYCYCLE_CFG4 0x45
#define REG_ADDR_PWM_DUTYCYCLE_CFG5 0x46
#define REG_ADDR_PWM_DUTYCYCLE_CFG6 0x47
#define REG_ADDR_PWM_DUTYCYCLE_CFG7 0x48
#define REG_ADDR_SPO_CFG 0x4c
#define REG_ADDR_DEVICE_CFG0 0x4d
#define REG_ADDR_DEVICE_CFG1 0x4e
#define REG_ADDR_DEVICE_CFG2 0x4f
#define REG_ADDR_DEVICE_CFG3 0x50
#define REG_ADDR_I2C_ADDR 0x51
#define REG_ADDR_REFRESH_CTRL 0x52
#define REG_ADDR_STATE_TIMEOUT 0x55
#define REG_ADDR_SLIDER_CFG 0x5d
#define REG_ADDR_SLIDER1_CFG 0x61
#define REG_ADDR_SLIDER1_RESOLUTION 0x62
#define REG_ADDR_SLIDER1_THRESHOLD 0x63
#define REG_ADDR_SLIDER2_CFG 0x67
#define REG_ADDR_SLIDER2_RESOLUTION 0x68
#define REG_ADDR_SLIDER2_THRESHOLD 0x69
#define REG_ADDR_SLIDER_LBR 0x71
#define REG_ADDR_SLIDER_NNT 0x72
#define REG_ADDR_SLIDER_NT 0x73
#define REG_ADDR_SCRATCHPAD0 0x7a
#define REG_ADDR_SCRATCHPAD1 0x7b
#define REG_ADDR_CONFIG_CRC 0x7e
#define REG_ADDR_GPO_OUTPUT_STATE 0x80
#define REG_ADDR_SENSOR_ID 0x82
#define REG_ADDR_CTRL_CMD 0x86
#define REG_ADDR_CTRL_CMD_STATUS 0x88
#define REG_ADDR_CTRL_CMD_ERR 0x89
#define REG_ADDR_SYSTEM_STATUS 0x8a
#define REG_ADDR_PREV_CTRL_CMD_CODE 0x8c
#define REG_ADDR_FAMILY_ID 0x8f
#define REG_ADDR_DEVICE_ID 0x90
#define REG_ADDR_DEVICE_REV 0x92
#define REG_ADDR_CALC_CRC 0x94
#define REG_ADDR_TOTAL_WORKING_SNS 0x97
#define REG_ADDR_SNS_CP_HIGH 0x98
#define REG_ADDR_SNS_VDD_SHORT 0x9a
#define REG_ADDR_SNS_GND_SHORT 0x9c
#define REG_ADDR_SNS_SNS_SHORT 0x9e
#define REG_ADDR_CMOD_SHIELD_TEST 0xa0
#define REG_ADDR_BUTTON_STAT 0xaa
#define REG_ADDR_LATCHED_BUTTON_STAT 0xac
#define REG_ADDR_PROX_STAT 0xae
#define REG_ADDR_LATCHED_PROX_STAT 0xaf
#define REG_ADDR_SLIDER1_POSITION 0xb0
#define REG_ADDR_LIFTOFF_SLIDER1_POSITION 0xb1
#define REG_ADDR_SLIDER2_POSITION 0xb2
#define REG_ADDR_LIFTOFF_SLIDER2_POSITION 0xb3
#define REG_ADDR_SYNC_COUNTER0 0xb9
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR0 0xba
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR1 0xbc
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR2 0xbe
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR3 0xc0
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR4 0xc2
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR5 0xc4
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR6 0xc6
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR7 0xc8
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR8 0xca
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR9 0xcc
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR10 0xce
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR11 0xd0
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR12 0xd2
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR13 0xd4
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR14 0xd6
#define REG_ADDR_DIFFERENCE_COUNT_SENSOR15 0xd8
#define REG_ADDR_GPO_DATA 0xda
#define REG_ADDR_SYNC_COUNTER1 0xdb
#define REG_ADDR_DEBUG_SENSOR_ID 0xdc
#define REG_ADDR_DEBUG_CP 0xdd
#define REG_ADDR_DEBUG_DIFFERENCE_COUNT0 0xde
#define REG_ADDR_DEBUG_BASELINE0 0xe0
#define REG_ADDR_DEBUG_RAW_COUNT0 0xe2
#define REG_ADDR_DEBUG_AVG_RAW_COUNT0 0xe4
#define REG_ADDR_SYNC_COUNTER2 0xe7


//REG_ADDR_CTRL_CMD  0x86
#define OP_CODE_CALC_CRC_SAVE_NVM	0x02
#define OP_CODE_RESET_DEVICE	0xff


extern uint8_t random_value_array[100];


//-----battery --------------------------
#define BAT_ADDR  0x53
#define BAT_NOT_CONNECT	0
#define BAT_DISCHARGE		1
#define BAT_CHARGING		2
#define BAT_CHARG_DONE	3

#define BAT_ULTRALOW_THRESHOLD_ADC    1870    //1710(almost shut-down)/1880(ready to shut-down)
#define BAT_LOW_THRESHOLD_ADC         2016		//3.3V 
#define BAT_HIGH_THRESHOLD_ADC        2483		//4.0V

#define BAT_LOW_VOLTAGE 		0
#define BAT_MEDIUM_VOLTAGE 	1
#define BAT_HIGH_VOLTAGE 		2

int8_t GW_Battery_Read_XX_data(uint8_t reg_address, uint8_t* _get_data, uint8_t data_len);
int8_t GW_Battery_Write_XX_data(int8_t reg_address, uint8_t* _write_data, uint8_t data_len);
void GW_Battery_Charger_Init_Command(void);
int32_t GW_Battery_Charger_Voltage_check(void);

typedef struct _battery
{
	uint8_t status;					//0: not connect (charge fail) 1: discharge 2: charge 3:charge done 
	uint8_t status_last;		//0: not connect (charge fail) 1: discharge 2: charge 3:charge done 
	uint8_t voltage_level;	//0:low 1:medium 2:high
	uint8_t rsv;
	int32_t battery_value;	//adc
	int32_t temperature_value; //adc
}GW_Battery;

extern GW_Battery gw_battery;
extern uint8_t enable_standby_mode;
extern volatile uint32_t dfu_scan_time_count_2;
#endif