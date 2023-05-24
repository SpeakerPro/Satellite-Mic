#ifndef __GW_aw5808_H__
#define __GW_aw5808_H__
#include <stdio.h>
#include "NuMicro.h"
#include "GW_main.h"

#define AW5808_USE_FW_TOOL 0
#define AW5808_DFU_DEBUG 0


#define AW5808_ADDR 0x34	//0x68
#define AW5808_ADDR_2 0x35		//0x6a

#define CH_UI2C0 0
#define CH_I2C0 1

#define AW5808_RX_ADDR AW5808_ADDR
#define AW5808_TX_ADDR AW5808_ADDR_2


#define DFU_SCAN_TIME 3
#define DFU_FLASH_WRITE_TIME 220

#define REG_ADDR_FW_V_0 0X00		//RW 
#define REG_ADDR_FW_V_1 0X01		//RW 
#define REG_ADDR_SYS_CONTROL 0X02		//RW
#define REG_ADDR_VOLUME_AUDIO 0X03		//RW 
#define REG_ADDR_PAIR_TONE 0X04	//R
#define REG_ADDR_SYS_STATUS 0X06	//R
#define REG_ADDR_MS_MODE_SETTING    0X07	//R
#define REG_ADDR_SAMPLE_RATE    	  0X08	//R
#define REG_ADDR_RF_POWER    	  0X09	//RW
#define REG_ADDR_PAIR_TIMEOUT_TIME    	  0X0A	//RW
#define REG_ADDR_RF_SELECT			0x0C	//RW
#define REG_ADDR_VOLUME_VOICE    	  0X0D	//RW
#define REG_ADDR_PC_AUDIO_VOLUME    	  0X0E	//R
#define REG_ADDR_CCH_DATA_1    	  0X10	//W,TX,RX
#define REG_ADDR_CCH_DATA_2    	  0X11	//W,TX,RX
#define REG_ADDR_CCH_DATA_3    	  0X12	//W,TX,RX
#define REG_ADDR_CCH_DATA_4    	  0X13	//W,TX,RX
#define REG_ADDR_CCH_DATA_5    	  0X14	//W,RX
#define REG_ADDR_CCH_DATA_6    	  0X15	//W,RX
#define REG_ADDR_MEDIA_CONTROL    	  0X21	//RW
#define REG_ADDR_LOSE_SYNC_PERIOD_TO_SLEEP    	  0X2E	//RW
#define REG_ADDR_SLEEP_WAKE_UP_PERIOD    	  0X2F	//RW
#define REG_ADDR_SELECT_FIX_RF_POWER				0x30
#define REG_ADDR_TX_RX_INDICATION    	  0X40	//R, 0x01:Tx  0x02:Rx
#define REG_ADDR_BRAND_INDICATION_0    	  0X44	//R
#define REG_ADDR_BRAND_INDICATION_1    	  0X45	//R
#define REG_ADDR_RANDOM_ID    	  				0X46	//R
#define REG_ADDR_OPERATED_ID_0    	  				0X47	//RW
#define REG_ADDR_OPERATED_ID_1    	  				0X48	//RW
#define REG_ADDR_OPERATED_ID_2    	  				0X49	//RW
#define REG_ADDR_OPERATED_ID_3    	  				0X4A	//RW
#define REG_ADDR_UNIQUE_ID_0    	  				0X5C	//RW
#define REG_ADDR_UNIQUE_ID_1    	  				0X5D	//RW
#define REG_ADDR_UNIQUE_ID_2    	  				0X5E	//RW
#define REG_ADDR_UNIQUE_ID_3    	  				0X5F	//RW
#define REG_ADDR_CCH_REC_1    	  0X66	//R,TX,RX
#define REG_ADDR_CCH_REC_2    	  0X67	//R,TX,RX
#define REG_ADDR_CCH_REC_3    	  0X68	//R,TX,RX
#define REG_ADDR_CCH_REC_4    	  0X69	//R,TX,RX
#define REG_ADDR_CCH_REC_5    	  0X6A	//R,RX
#define REG_ADDR_CCH_REC_6    	  0X6B	//R,RX
#define REG_ADDR_RF_POWER_DURNING_PAIRING    	  0X88	//R,RX

#define AW5808_FREQ	300
#define ROUTINE_TASK_READ_STATUS_BIT_MASK 0x01
#define ROUTINE_TASK_READ_ID_BIT_MASK 0x02
#define ROUTINE_TASK_PAIR_BIT_MASK 0x04
#define ROUTINE_TASK_WIRTE_ID_BIT_MASK 0x08
#define ROUTINE_TASK_READ_ROLE_BIT_MASK 0x10

#define START_BYTE 0xBB
#define TARGET_CHANNEL_HSCI_I2C 	1
#define TARGET_CHANNEL_I2C 		 		2
#define TARGET_CHANNEL_I2C_TOUCH 	3

#define ACTION_READ								1
#define ACTION_WRITE							2
#define ACTION_RESULT_PASS				1
#define ACTION_RESULT_FAIL				2
#define ACTION_RESULT_FAIL_TIMEOUT	3
#define ACTION_RESULT_FAIL_MNAK			4
#define ACTION_RESULT_FAIL_AL				5
#define ACTION_RESULT_FAIL_SNAK			6
#define ACTION_RESULT_FAIL_BUSERR		7

#define AW5808_AUTO_PAIR_STATUS_STOP 	0
#define AW5808_AUTO_PAIR_STATUS_RUNNING 1
#define AW5808_AUTO_PAIR_STATUS_IDLE 2
#define AW5808_AUTO_PAIR_RESULT_FAIL 	0
#define AW5808_AUTO_PAIR_RESULT_PASS  1
#define AW5808_AUTO_PAIR_RESULT_IDLE  2

extern volatile uint32_t g_au32AW5808INTCount[1];


extern volatile uint8_t g_u8MstRxData[20];
extern volatile uint8_t g_u8MstTxData[200];


extern void GW_AW5808_Tx_On_Off(uint8_t _value);
extern void GW_AW5808_Rx_On_Off(uint8_t _value);

extern uint8_t scan_time;
extern uint8_t i2c_stop_count_enable;
extern uint32_t aw5808_auto_test_count;

extern void Delay_Test_Function(void);
extern uint8_t enable_touch_update_process;

extern uint8_t enable_tx_rx_random_test;

extern uint8_t test_times;
extern uint8_t show_time_field_count;
extern uint8_t width_test;


extern uint8_t exec_rx_read;
extern uint8_t exec_tx_read;

extern volatile uint8_t aw5808_dfu_enable;
extern volatile uint8_t aw5808_dfu_channel;


typedef struct AW5808_DATA
{	
	uint8_t online;
	uint8_t id[4];
	uint8_t version[2];
	uint8_t status;
	uint8_t dfu_process_valule;
	uint8_t cch_read_data[10];
	uint8_t cch_read_data_length;	
	int8_t  cch_read_data_status; //0: pass  -1:fail
	
	uint8_t old_online;
	uint8_t old_id[4];
	uint8_t old_version[2];	
	uint8_t old_status;
	uint8_t dfu_process_old_valule;
	uint8_t connect;
	
	uint8_t module_type;//1:Tx 2:Rx
	
	uint8_t rf_select;
	uint8_t rf_power;
	uint8_t rf_rssi_level;	//0x00:Lowest RF strength <50%
													//0x01:third RF strength 50%~70%
													//0x02:second RF strength 70%~90%
													//0x03:Highest RF strength >90%		
	uint8_t odd_rf_rssi_level;
}AW5808_DATA;

extern AW5808_DATA aw5808_rx_data;
extern AW5808_DATA aw5808_tx_data;

extern int8_t USCI_I2C_WriteMultiByteOneReg(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t wdata[], uint32_t u32rLen);

extern uint32_t usci_check_value;
extern uint8_t usci_check_start;
extern volatile uint8_t usci_g_u8MstEndFlag;
extern void stop_usci_i2c(void);
extern volatile uint8_t g_u8MstEndFlag;
extern uint32_t aw5808TX_TMRINTCount_old;

extern uint8_t aw5808_dfu_wait_time_ms;
extern uint8_t aw5808_dfu_wait_count;
extern uint8_t aw5808_dfu_update_flag;

extern uint32_t aw5808TX_TMRINTCount;
extern uint32_t aw5808RX_TMRINTCount;

extern uint8_t check_aw5808_autopair_flag;
extern uint8_t check_aw5808_dfu_flag;

extern int8_t GW_AW5808_RX_Write_XX_data(uint8_t reg_address, uint8_t* _write_data, uint8_t data_len);
extern int8_t GW_AW5808_RX_Read_XX_data(uint8_t reg_address, uint8_t* _get_data, uint8_t data_len);
extern uint16_t check_aw5808_dfu_scan_time;

extern uint8_t aw5808_auto_pair_status;
extern uint8_t aw5808_auto_pair_result;
extern uint8_t check_mcu_power_enable;
extern uint8_t mcu_power_on_delay_tx;
extern uint8_t mcu_power_on_delay_rx;
extern uint8_t mcu_power_on_delay_touch;

typedef int8_t (*AW5808_FUNC_P)(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t wdata[], uint32_t u32rLen);


extern uint8_t check_aw5808_hdi_update_flag;
extern uint8_t enable_aw5808_hdi_check;
extern void aw5808_simulator_query(void);
extern uint8_t aw5805_dfu_step;
extern uint32_t file_index, file_size;

typedef struct AW5808_HID_DATA_TEMP
{
	uint32_t* file_idx;
	uint8_t  data[20];
	uint8_t  len;
}AW5808_HID_DATA_TEMP;


extern void save_hid_aw5808_data(uint32_t* _file_idx, uint8_t* _data, uint8_t _len);
extern volatile uint8_t get_aw5808_hid_event_response;
extern AW5808_HID_DATA_TEMP aw5808_temp_data;

extern uint8_t AW5808_TX_Addr;
extern uint8_t dfu_error;


#define	ERR_LOST_COMPANY_NAME 1
#define	ERR_WAIF_FOR_POLL 2
#define	ERR_PKT_SEQUENCE 3
#define	ERR_READ_0X7F 	4
#define	ERR_CK_ERROR 	5
#define	ERR_QUERY_TIMEOUT 	6

extern uint8_t touch_online;
extern uint8_t aw5808_tx_connect;
extern uint8_t aw5808_rx_connect;

extern int8_t I2C0_ReadMultiBytes(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t rdata[], uint8_t u8rLen);
extern int8_t I2C0_WriteMultiByteOneReg(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t wdata[], uint32_t u32rLen);

extern uint8_t touch_connect;

extern uint32_t i2c0_check_start;
extern uint32_t i2c0_check_value;

extern void stop_i2c0(void);

extern int8_t GW_AW5808_TX_Write_XX_data_new(int8_t slave_address,int8_t reg_address, uint8_t* _write_data, uint8_t data_len);
extern int8_t GW_AW5808_TX_Read_XX_data_new(int8_t slave_address,uint8_t reg_address, uint8_t* _get_data, uint8_t data_len);


#define SEND_VID_PID 			0x01
#define SEND_PD_ROLE 			0x02
#define SEND_VDM_RESPONSE 0x03
#define SEND_PD_STATUS 		0x04

#define PORT_NUM_0 0x00
#define PORT_NUM_1 0x01
#define PR_SINK 	0x00
#define PR_SOURCE 0x01
#define DR_UFP 0x00
#define DR_DFP 0x01

#define PR_SINK_PORT0 (PR_SINK<<0)			//0x00
#define PR_SINK_PORT1 (PR_SINK<<1)			//0x00
#define PR_SOURCE_PORT0 (PR_SOURCE<<0)	//0x01
#define PR_SOURCE_PORT1 (PR_SOURCE<<1)	//0x02
#define DR_UFP_PORT0 (DR_UFP<<0)				//0x00
#define DR_UFP_PORT1 (DR_UFP<<1)				//0x00
#define DR_DFP_PORT0 (DR_DFP<<0)				//0x01
#define DR_DFP_PORT1 (DR_DFP<<1)				//0x02

//---VDM-----
extern uint8_t VDM_DATA_REQ[19];
extern uint8_t VDM_DATA_ACK[19];
void VDM_Response_Check_Process(uint8_t* data, uint8_t _data_len);
void Request_To_SendOut_Data(uint8_t* data, uint8_t _data_len);

//header
#define SVID 0x17EF  					//bit31-bit16
#define VDM_Type (0x01<<7)		//bit15
#define VDM_Version (0x01<<5)	//BIT14-bit13

#define VDM_HEADER_CMD_TYPE_BIT_MASK (0x03<<6)		//0xC0
#define VDM_HEADER_CMD_TYPE_REQ 		 (0x00<<6)		//0x00
#define VDM_HEADER_CMD_TYPE_ACK 		 (0x01<<6)		//0x40
#define VDM_HEADER_CMD_TYPE_NAK 		 (0x02<<6)		//0x80
#define VDM_HEADER_CMD_GET_STATUS    0x10 
#define VDM_HEADER_ATTENTION				 0x06
#define VDM_HEADER_GET_STATUS				 0x10
typedef struct _VDM_DATA
{
	uint8_t VDM_Header[4];
	uint8_t VDO_0[4];
	uint8_t VDO_1[4];
	uint8_t VDO_2[4];
}VDM_DATA;

extern VDM_DATA local_vdm_data;

extern uint8_t check_battery_status_update_flag;

void GW_Battery_Process(void);
extern uint8_t DOA_RGB[];
extern uint8_t check_battery_voltage_flag;

extern uint8_t check_vdm_update_flag;

extern int8_t GW_AW5808_RX_Read_RSSI_Level(AW5808_DATA* aw5808_item);
#endif