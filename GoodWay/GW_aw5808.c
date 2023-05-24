#include <stdio.h>
#include "NuMicro.h"
#include "GW_aw5808.h"
#include "usci_i2c.h"
#include <string.h>
#include "GW_main.h"
#include "GW_wm8804_cs8442.h"

//#define IGNORE_PRINTF
#ifdef IGNORE_PRINTF
	#define printf(fmt, ...) (0)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t enable_tx_rx_random_test=0;
uint8_t current_test_mode=0; //0:unpair both 1:Rx 2:Tx
uint8_t current_test_result=0; //0:none 1:under pair 2: pair-ok 3:pair-fail
uint8_t run_test=0; //0:none 1:keep running 2:leave test
uint8_t rx_test_status=0; //0:send 1:receive
uint8_t tx_test_status=0; //0:send 1:receive

uint8_t random_value_array[100]=
{
	103,123, 74, 55,148, 56,138, 80,118, 63,
	101, 61, 84,105, 65,125, 52,104, 66, 54,
	 85,108, 50,132,113,126, 79, 87, 96,114,
	134,139,106, 92, 95,137,129, 73, 81,143,
	 88, 58, 72,124, 89, 97, 51,128,120,127,
	131, 82,144,141, 53, 68, 94,140, 91, 78,
	145, 77, 69,146, 48, 64, 70,110,117, 86,
	102, 62, 59, 71,130,111,100,119, 60, 67,
	 99, 75,121, 90,133,136, 93, 83, 49,116,
	147,142,135,107,115, 76,122,112, 98, 57	
};

volatile uint32_t g_au32AW5808INTCount[1]={0};

//rx
AW5808_DATA aw5808_rx_data=
{
	.online=0xff,
	.id="\x00\x00\x00\x00",
	.version="\x00\x00",
	.status=0,
	.dfu_process_valule=0,	
	.old_online=0xff,
	.old_id={CCH_PAIR_ID_OLD,CCH_PAIR_ID_OLD,CCH_PAIR_ID_OLD,CCH_PAIR_ID_OLD},
	.old_version="\x00\x00",
	.old_status=0xff,
	.dfu_process_old_valule=0,
	.cch_read_data_length=9,
	.cch_read_data_status=-1,
	.rf_select=0xff,
	.rf_power=0xff,
	.rf_rssi_level=0,
};

//tx
AW5808_DATA aw5808_tx_data=
{
	.online=0xff,
	.id="\x00\x00\x00\x00",
	.version="\x00\x00",
	.status=0,
	.dfu_process_valule=0,
	.old_online=0xff,
	.old_id="\xff\xff\xff\xff",
	.old_version="\x00\x00",
	.old_status=0xff,
	.dfu_process_old_valule=0,
	.cch_read_data_length=8,
	.cch_read_data_status=-1,
	.rf_select=0xff,
	.rf_power=0xff	
};


volatile AW5808_FUNC_P s_pfn_AT5808RX_Handler_R = NULL;
volatile AW5808_FUNC_P s_pfn_AT5808RX_Handler_W = NULL;

//-----------------------------------------------------------------------------------------------------------

void GPC_IRQHandler(void)
{
    /* To check if PC.3 interrupt occurred */
		if(GPIO_GET_INT_FLAG(PC, BIT3))
		{
        GPIO_CLR_INT_FLAG(PC, BIT3);
        //printf("PC.3 INT occurred.\n");
				exec_rx_read = 1;
				m_debug.cch_int_time_ms = m_module.current_time_ms;
		}
    else
    {
        uint32_t u32Status;
        u32Status = PC->INTSRC;
        /* Un-expected interrupt. Just clear all PC interrupts */
        PC->INTSRC = u32Status;
        //printf("Un-expected interrupts.\n");
    }
}

//=====================USCI==================================================================================
volatile enum UI2C_MASTER_EVENT g_eMEvent;
typedef void (*UI2C_FUNC)(uint32_t u32Status);
volatile static UI2C_FUNC s_UI2C0HandlerFn = NULL;
volatile uint8_t usci_g_u8MstDataLen;
volatile uint8_t usci_g_u8MstEndFlag = 0;
volatile uint8_t usci_g_u8SlaveAddr=0;
volatile uint8_t usci_g_u8DataRegAddr=0;
volatile uint8_t usci_g_u8MstTxData[20];
volatile uint8_t usci_g_u8MstRxData[20];
uint16_t usci_g_u8MstDataLen_expected;
uint8_t usci_rx_counter=0;


#define USCI_I2C_SLAVE_DATA_LEN 20
volatile uint8_t g_au8SlvData[256];

volatile uint8_t g_au8SlvRxData[USCI_I2C_SLAVE_DATA_LEN];
volatile uint16_t g_u16RecvAddr;
volatile uint32_t g_u32SlaveBuffAddr;
volatile uint32_t g_u32SlaveBuffAddr_len=0;
volatile uint16_t g_u16SlvRcvAddr;
volatile uint8_t g_u8SlvDataLen;
volatile enum UI2C_SLAVE_EVENT g_eSEvent;


volatile uint8_t g_au8SlvRxData_fifo[16][30]={0};
volatile uint8_t g_au8SlvRxData_w_idx=0;
volatile uint8_t g_au8SlvRxData_r_idx=0;


//----------------------------------------------------------------------------------------------------------
uint8_t write_circle=0;
uint8_t slave_print_out_loc_2=0;
int8_t Write_Slave_Data_To_fifo(uint8_t* w_data, uint8_t w_data_len)
{
	int8_t result=-1;	
  if(write_circle==0)
	{
		if(g_au8SlvRxData_w_idx>=g_au8SlvRxData_r_idx)
		{
			g_au8SlvRxData_fifo[g_au8SlvRxData_w_idx][0]=w_data_len;	//packet len 
			memcpy((void*)&(g_au8SlvRxData_fifo[g_au8SlvRxData_w_idx][1]),w_data,w_data_len);			
			g_au8SlvRxData_w_idx++;
			result=0;				
		}
	}
	else
	{
		if(g_au8SlvRxData_w_idx<g_au8SlvRxData_r_idx)
		{
			g_au8SlvRxData_fifo[g_au8SlvRxData_w_idx][0]=w_data_len;	//packet len 
			memcpy((void*)&(g_au8SlvRxData_fifo[g_au8SlvRxData_w_idx][1]),w_data,w_data_len);			
			g_au8SlvRxData_w_idx++;
			result=0;		
		}			
	}

	if(g_au8SlvRxData_w_idx>=16)
	{
		g_au8SlvRxData_w_idx=0;
		write_circle=1;
	}
	return result;
}
//----------------------------------------------------------------------------------------------------------
int8_t Read_Slave_Data_From_fifo(uint8_t* r_data, uint8_t* r_data_len)
{
	int8_t result=-1;

	if(write_circle==0)
	{
		if(g_au8SlvRxData_r_idx<g_au8SlvRxData_w_idx)
		{
			*r_data_len=g_au8SlvRxData_fifo[g_au8SlvRxData_r_idx][0];
			memcpy(r_data,(void*)&(g_au8SlvRxData_fifo[g_au8SlvRxData_r_idx][1]),g_au8SlvRxData_fifo[g_au8SlvRxData_r_idx][0]);			
			g_au8SlvRxData_r_idx++;
			result=0;		
		}			
	}
	else
	{
		if(g_au8SlvRxData_r_idx>=g_au8SlvRxData_w_idx)
		{
			*r_data_len=g_au8SlvRxData_fifo[g_au8SlvRxData_r_idx][0];
			memcpy(r_data,(void*)&(g_au8SlvRxData_fifo[g_au8SlvRxData_r_idx][1]),g_au8SlvRxData_fifo[g_au8SlvRxData_r_idx][0]);			
			g_au8SlvRxData_r_idx++;
			result=0;
		}
	}
	
	
	if(g_au8SlvRxData_r_idx>=16)
	{
		g_au8SlvRxData_r_idx=0;
		write_circle=0;		
	}
	return result;
}
//----------------------------------------------------------------------------------------------------------
void Show_VDM_Slave_Data(void)
{
		uint8_t read_data[20], read_len = 0, i = 0;

		if(Read_Slave_Data_From_fifo(read_data, &read_len) == 0) {
				GW_check_PD_data(read_data);
				if((slave_print_out_loc_2%10) == 0)
						clear_screen();
				m_module.vdm_ready = YES;

				VDM_Response_Check_Process(read_data, read_len);
				//PrintMsg__("C\r\n");
		}
		else {
			//printf("\x1b[%d;100H[r=%d w=%d]  fifo read error\n",slave_print_out_loc_2++,g_au8SlvRxData_r_idx,g_au8SlvRxData_w_idx);
		}	
}
		
//----------------------------------------------------------------------------------------------------------
//from which port number 0x00
uint8_t VDM_DATA_REQ[19]={0x13,0x11,0x00,0x17,0xef,0xa0,0x12,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c};
//from which port number 0x01
//uint8_t VDM_DATA_REQ[19]={0x13,0x11,0x01,0x17,0xef,0xa0,0x12,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c};

//from which port number 0x00	
uint8_t VDM_DATA_ACK[19]={0x13,0x11,0x00,0x17,0xef,0xa0,0x52,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25};	
//from which port number 0x01
//uint8_t VDM_DATA_ACK[19]={0x13,0x11,0x01,0x17,0xef,0xa0,0x52,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25};
	
VDM_DATA local_vdm_data;

void VDM_Response_Check_Process(uint8_t * data, uint8_t _data_len)
{
		GW_wire_process_master_req(data, _data_len);
}

void Request_To_SendOut_Data(uint8_t * data, uint8_t _data_len)
{
		//wireless
		if(hw_current_version == 0) {
				if(GPO_PC04_PD_DFP_INT == 0)
						return;
				GPO_PC04_PD_DFP_INT = 0;
		}
		//wire
		else {
				GPO_PC04_PD_DFP_INT = 1;
		}

		//memset((void*)g_au8SlvData,0x00,sizeof(0x00));
		memset((void*)g_au8SlvData, 0x00, sizeof(g_au8SlvData));

		g_u32SlaveBuffAddr = 0;
		memcpy((void*)g_au8SlvData, data, _data_len);
		g_u32SlaveBuffAddr_len = g_au8SlvData[1] + 2;	
}
//----------------------------------------------------------------------------------------------------------
void UI2C0_Init(void)
{
    /* Open USCI_I2C0 and set clock to 100k */

    UI2C_Open(UI2C0, 400000);

    /* Get UI2C0 Bus Clock */
    //printf("UI2C clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set UI2C1 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x28, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x50 */

	  //UI2C_PROTIEN_STORIEN_Msk    //this setting will let isr keep working when booting up from aprom
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk | UI2C_PROTIEN_ARBLOIEN_Msk |UI2C_PROTIEN_ERRIEN_Msk));	
		


    NVIC_EnableIRQ(USCI0_IRQn);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = (UI2C0->PROTSTS);

    if (s_UI2C0HandlerFn != NULL)
    {
        s_UI2C0HandlerFn(u32Status);
    }
		else
		{
			
			UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk | 
																		UI2C_PROTSTS_ACKIF_Msk |
																		UI2C_PROTSTS_NACKIF_Msk |
																		UI2C_PROTSTS_STORIF_Msk |
																		UI2C_PROTSTS_ERRIF_Msk |
																		UI2C_PROTSTS_ARBLOIF_Msk |
																		UI2C_PROTSTS_TOIF_Msk);
		}	
}


/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 TRx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t slave_print_out_loc=1;
uint8_t slave_check_data_len=0;
uint8_t pkt_data_len=0;
uint8_t byte_count_value=0;
uint8_t do_write_fifo_action=0;
uint8_t pd_send_out_status=0;  //0:none 1:get the first read command  2: get the second read command

extern uint8_t usci_need_re_init;

void UI2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8Data;
    uint32_t u32TempAddr;
    uint8_t i=0;

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)                   /* Re-Start been received */
    {
        //printf("\x1b[%d;100H  UI2C_PROTSTS_STARIF_Msk\n",slave_print_out_loc++);

        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        g_eSEvent = SLAVE_ADDRESS_ACK;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)                /* USCI I2C Bus have been received ACK */
    {
        //printf("\x1b[%d;100H  UI2C_PROTSTS_ACKIF_Msk\n",slave_print_out_loc++);
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (g_eSEvent == SLAVE_ADDRESS_ACK)                                                   /* Address Data has been received */
        {
            //printf("\x1b[%d;100H  SLAVE_ADDRESS_ACK\n",slave_print_out_loc++);
            /* Check address if match address 0 or address 1*/
            if ((UI2C0->ADMAT & UI2C_ADMAT_ADMAT0_Msk) == UI2C_ADMAT_ADMAT0_Msk)
            {
                //printf("\x1b[%d;100H  Address 0 match\n",slave_print_out_loc++);
                /* Address 0 match */
                UI2C0->ADMAT = UI2C_ADMAT_ADMAT0_Msk;
            }
            else
            {
                return;
            }

            /* USCI I2C receives Slave command type */
            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
								pd_send_out_status++;
                g_eSEvent = SLAVE_SEND_DATA;/* Slave address read has been received */
                UI2C_SET_DATA(UI2C0, g_au8SlvData[g_u32SlaveBuffAddr]);
                g_u32SlaveBuffAddr++;
            }
            else
            {
							   //write
							
                //printf("\x1b[%d;100H  UI2C_PROTSTS_SLAREAD_Msk else\n",slave_print_out_loc++);
                g_u8SlvDataLen = 0;/* Slave address write has been received */
                g_eSEvent = SLAVE_GET_DATA;
            }

            /* Read address from USCI I2C RXDAT*/
            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
            //printf("\x1b[%d;100H  g_u16RecvAddr=%x\n",slave_print_out_loc++,g_u16RecvAddr);

            slave_check_data_len=0;
            memset((uint8_t*)g_au8SlvRxData,0x00,sizeof(g_au8SlvRxData));
            pkt_data_len=0;
						byte_count_value=0;
						do_write_fifo_action=0;						
//PrintMsg__("A\r\n");
        }
        else if (g_eSEvent == SLAVE_GET_DATA)
        {
            /* Read data from USCI I2C RXDAT*/
            u8Data = (uint8_t)UI2C_GET_DATA(UI2C0);
				
            if(pkt_data_len==0)
            {
							  //check cmd
							  if( (u8Data&0x7f)>=SEND_VID_PID && (u8Data&0x7f)<=SEND_PD_STATUS)
								{
                g_au8SlvRxData[pkt_data_len++] = u8Data; //cmd
									do_write_fifo_action=1;								
            }
							}
            else if(pkt_data_len==1)
							{
								byte_count_value = u8Data;
                g_u8SlvDataLen=u8Data+2;	//pkt_len = cmd + data_len +data
								g_au8SlvRxData[pkt_data_len++] = u8Data;
							}
            else
            {
                if(g_u8SlvDataLen<= USCI_I2C_SLAVE_DATA_LEN)
                {
                    if (pkt_data_len < g_u8SlvDataLen)
                    {
                        g_au8SlvRxData[pkt_data_len++] = u8Data;
                    }
                }
            }
					
        }
        else if (g_eSEvent == SLAVE_SEND_DATA)
        {
            /* Write transmit data to USCI I2C TXDAT*/
					  if(g_u32SlaveBuffAddr<g_u32SlaveBuffAddr_len)
            UI2C0->TXDAT = g_au8SlvData[g_u32SlaveBuffAddr++];
//PrintMsg__("D\r\n");
        }
        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
				//printf("\x1b[%d;%dHS_i2c: NACK", SELECT_Y+20, SELECT_X);
        //printf("\x1b[%d;100H  UI2C_PROTSTS_NACKIF_Msk\n",slave_print_out_loc++);
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);

        /* Event process */
        g_u8SlvDataLen = 0;
        g_eSEvent = SLAVE_ADDRESS_ACK;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
			
			  if(pd_send_out_status>1)
				{
					pd_send_out_status=0;
					GPO_PC04_PD_DFP_INT=0;
					enable_check_wire_mic_sendout_time=0;
					}
//PrintMsg__("NA\r\n");
				}
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
		
			  if(do_write_fifo_action)
				{
					if(Write_Slave_Data_To_fifo((void*)g_au8SlvRxData, g_u8SlvDataLen)!=0)
					printf("\x1b[%d;100H  fifo write error\n",slave_print_out_loc++);
					
					do_write_fifo_action=0;
//PrintMsg__("W\r\n");
			  }	
				
        //printf("\x1b[%d;100H  UI2C_PROTSTS_STORIF_Msk\n",slave_print_out_loc++);
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);

        g_u8SlvDataLen = 0;
        g_eSEvent = SLAVE_ADDRESS_ACK;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    } else {
			printf("\x1b[%d;%dHS_i2c:0x%04x", SELECT_Y+20, SELECT_X,  u32Status);
			
			if ((u32Status & UI2C_PROTSTS_ERRIF_Msk) == UI2C_PROTSTS_ERRIF_Msk) {
				UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ERRIF_Msk);
				usci_need_re_init = YES;
				//GW_USCI_I2C_On_Off(0);
				//GW_USCI_I2C_On_Off(1);
			}
			
		}
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C Rx Callback Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void stop_usci_i2c(void)
{
	usci_g_u8MstEndFlag = 1;
	g_eMEvent = MASTER_STOP;
	UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */	
}
void USCI_I2C_MasterRx(uint32_t u32Status)
{
	  uint8_t i=0;
	
    if (UI2C_GET_TIMEOUT_FLAG(UI2C0))
{
			  //printf("USCI_I2C_MasterRx-> UI2C_GET_TIMEOUT_FLAG\n");			
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_ClearTimeoutFlag(UI2C0);
				UI2C_DisableTimeout(UI2C0);
			  usci_g_u8MstEndFlag = 1;
    }
    else if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
			  //0x100,8
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk); /* Clear START INT Flag */

        if (g_eMEvent == MASTER_SEND_START)
        {
			      //printf("USCI_I2C_MasterRx-> MASTER_SEND_START\n");
            UI2C_SET_DATA(UI2C0, (usci_g_u8SlaveAddr << 1) | 0x00);     /* Write SLA+W to Register TXDAT */						
            g_eMEvent = MASTER_SEND_ADDRESS;
        }
        else if (g_eMEvent == MASTER_SEND_REPEAT_START)
        {
			      //printf("USCI_I2C_MasterRx-> MASTER_SEND_REPEAT_START\n");	
            UI2C_SET_DATA(UI2C0, (usci_g_u8SlaveAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */					
            g_eMEvent = MASTER_SEND_H_RD_ADDRESS;
        }
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);

    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
			   //0x2000,13
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */

        if (g_eMEvent == MASTER_SEND_ADDRESS)
        {
			      //printf("USCI_I2C_MasterRx-> MASTER_SEND_ADDRESS\n");					
					  UI2C_SET_DATA(UI2C0, usci_g_u8DataRegAddr);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            g_eMEvent = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (g_eMEvent == MASTER_SEND_DATA)
        {
			      //printf("USCI_I2C_MasterRx-> MASTER_SEND_DATA\n");						
            g_eMEvent = MASTER_SEND_REPEAT_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send repeat START signal */
        }
        else if (g_eMEvent == MASTER_SEND_H_RD_ADDRESS)
        {
            g_eMEvent = MASTER_READ_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }			
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
			  //0x400,10
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk); /* Clear NACK INT Flag */

        if (g_eMEvent == MASTER_SEND_ADDRESS)
        {
					  //printf("USCI_I2C_MasterRx-> MASTER_SEND_ADDRESS\n");		

            g_eMEvent = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send START signal */
					
						//---------------------		
						if(usci_rx_counter>5)
						{
							g_eMEvent = MASTER_STOP;
							UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */				
						}
						else
						usci_rx_counter++;
						//----------------------
        }
        else if (g_eMEvent == MASTER_READ_DATA)
        {
					  //printf("USCI_I2C_MasterRx-> MASTER_READ_DATA \n");						
				    usci_g_u8MstRxData[usci_g_u8MstDataLen++] = (unsigned char) UI2C_GET_DATA(UI2C0) & 0xFF;			
            g_eMEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */
        }
        else
				{
            /* TO DO */
            //printf("UI2C_PROTSTS_NACKIF_Msk-> Status 0x%x is NOT processed\n", u32Status);
					  g_eMEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
				}
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
			  //0x200,9
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);  /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        usci_g_u8MstEndFlag = 1;
    }
		else if((u32Status & UI2C_PROTSTS_ERRARBLO_Msk) == UI2C_PROTSTS_ERRARBLO_Msk)
		{
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG| UI2C_CTL_STO);

				GW_AW5808_Rx_On_Off(0);
				GW_AW5808_Rx_On_Off(1);

		}
		else if((u32Status & UI2C_PROTSTS_BUSHANG_Msk) == UI2C_PROTSTS_BUSHANG_Msk)
		{
			  g_eMEvent = MASTER_STOP;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG| UI2C_CTL_STO);

				GW_AW5808_Rx_On_Off(0);
				GW_AW5808_Rx_On_Off(1);

		}
		else if((u32Status & UI2C_PROTIEN_ARBLOIEN_Msk) == UI2C_PROTIEN_ARBLOIEN_Msk)
		{
			  //0x10,4
        g_eMEvent = MASTER_STOP;			
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTIEN_ARBLOIEN_Msk);  /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG| UI2C_CTL_STA);
			

				GW_AW5808_Rx_On_Off(0);
				GW_AW5808_Rx_On_Off(1);

		}	
		else if((u32Status & UI2C_PROTSTS_ERRIF_Msk) == UI2C_PROTSTS_ERRIF_Msk)
		{
			  //0x1000,12
			  g_eMEvent = MASTER_STOP;
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ERRIF_Msk);  /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG| UI2C_CTL_STO);

				GW_AW5808_Rx_On_Off(0);
				GW_AW5808_Rx_On_Off(1);
		
		}		
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C Tx Callback Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_I2C_MasterTx(uint32_t u32Status)
{
    if (UI2C_GET_TIMEOUT_FLAG(UI2C0))
    {
			  //printf("USCI_I2C_MasterTx-> UI2C_GET_TIMEOUT_FLAG\n");
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_ClearTimeoutFlag(UI2C0);
				usci_g_u8MstEndFlag = 1;
    }
    else if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
			  //printf("USCI_I2C_MasterTx-> UI2C_PROTSTS_STARIF_Msk\n");		
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk); /* Clear START INT Flag */
        UI2C_SET_DATA(UI2C0, (usci_g_u8SlaveAddr << 1) | 0x00);     /* Write SLA+W to Register TXDAT */			
        g_eMEvent = MASTER_SEND_ADDRESS;

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
			  //printf("USCI_I2C_MasterTx-> UI2C_PROTSTS_ACKIF_Msk\n");					
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */

        if (g_eMEvent == MASTER_SEND_ADDRESS)
        {
					  //printf("USCI_I2C_MasterRx-> MASTER_SEND_ADDRESS\n");
            UI2C_SET_DATA(UI2C0, usci_g_u8DataRegAddr);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */					
            g_eMEvent = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (g_eMEvent == MASTER_SEND_DATA)
        {
					  //printf("USCI_I2C_MasterRx-> MASTER_SEND_DATA\n");	
            if (usci_g_u8MstDataLen < usci_g_u8MstDataLen_expected)							
            {
                UI2C_SET_DATA(UI2C0, usci_g_u8MstTxData[usci_g_u8MstDataLen++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                g_eMEvent = MASTER_STOP;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));        /* Send STOP signal */
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
			  //printf("USCI_I2C_MasterTx-> UI2C_PROTSTS_NACKIF_Msk\n");		
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk); /* Clear NACK INT Flag */

        usci_g_u8MstEndFlag = 0;

        if (g_eMEvent == MASTER_SEND_ADDRESS)
        {
			      //printf("USCI_I2C_MasterTx-> MASTER_SEND_ADDRESS\n");						
            /* SLA+W has been transmitted and NACK has been received */
						g_eMEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        }
        else if (g_eMEvent == MASTER_SEND_DATA)
        {
					  //printf("USCI_I2C_MasterTx-> MASTER_SEND_DATA\n");		
            /* ADDRESS has been transmitted and NACK has been received */
            g_eMEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));            /* Send STOP signal */
        }
        else
				{
            //printf("Get Wrong NACK Event\n");				
				}
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
			  //printf("USCI_I2C_MasterTx-> UI2C_PROTSTS_STORIF_Msk\n");				
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);  /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        usci_g_u8MstEndFlag = 1;
    }
}
//-----------------------------------------------------------------------------------------------------------
int32_t USCI_I2C_ReadMultiByteOneReg(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t rdata[], uint32_t u32rLen)
{
	usci_g_u8MstDataLen = 0;
	usci_g_u8MstEndFlag = 0;	
	usci_g_u8SlaveAddr = u8SlaveAddr;
	usci_g_u8DataRegAddr=u8DataRegAddr;
	usci_rx_counter=0;
	
	if(check_mcu_power_enable==0)
	return usci_g_u8MstDataLen;
	
	/* USCI_I2C function to read data to slave */
	g_eMEvent = MASTER_STOP;	
	s_UI2C0HandlerFn = (UI2C_FUNC)USCI_I2C_MasterRx;
	
	/* USCI_I2C as master sends START signal */
	g_eMEvent = MASTER_SEND_START;
	UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);			 

	if(aw5808_dfu_enable==0)
	{
		usci_check_value=0;
		usci_check_start=1;	
	}

  while (usci_g_u8MstEndFlag == 0);
			
  UI2C_ClearTimeoutFlag(UI2C0);		
	UI2C_DisableTimeout(UI2C0);
	uint8_t i=0;
	for(i=0;i<usci_g_u8MstDataLen;i++)
	rdata[i]=usci_g_u8MstRxData[i];
	
	return usci_g_u8MstDataLen;		 
}
//----------------------------------------------------------------------------------------------------------
int8_t USCI_I2C_ReadMultiBytes(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t rdata[], uint8_t u8rLen)
{
	uint8_t i=0,len=0,temp_data[200]={0},r_data=0;
	int8_t r_value=-1;
	
	for(i=0;i<u8rLen;i++)
	{
		r_data=0;
		if(USCI_I2C_ReadMultiByteOneReg(u8SlaveAddr,u8DataRegAddr+i,&r_data,1))
		{
			temp_data[i]=r_data;
			len++;			
		}
	}
			 
	if(len==u8rLen)
	{
		memcpy(rdata,temp_data,len);
		r_value=0;
	}
	
	return r_value;
}

//----------------------------------------------------------------------------------------------------------
int8_t USCI_I2C_WriteMultiByteOneReg(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t wdata[], uint32_t u32rLen)
{		 
	usci_g_u8MstDataLen = 0;	
	usci_g_u8MstEndFlag = 0;
	usci_g_u8SlaveAddr = u8SlaveAddr;
	usci_g_u8DataRegAddr=u8DataRegAddr;	
	usci_g_u8MstDataLen_expected=u32rLen;

	if(check_mcu_power_enable==0)
	return 0;
		
	uint16_t i=0;
	for(i=0;i<u32rLen;i++)	
	usci_g_u8MstTxData[i]=wdata[i];
	
	/* USCI_I2C function to write data to slave */
	s_UI2C0HandlerFn = (UI2C_FUNC)USCI_I2C_MasterTx;
	
	/* USCI_I2C as master sends START signal */
	g_eMEvent = MASTER_SEND_START;
	UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);
			 
	if(aw5808_dfu_enable==0)
	UI2C_EnableTimeout(UI2C0, 200);
  /* Wait I2C Tx Finish */
  while (usci_g_u8MstEndFlag == 0);


	UI2C_ClearTimeoutFlag(UI2C0);		
	UI2C_DisableTimeout(UI2C0);
	return 0;
}
//----------------------------------------------------------------------------------
int8_t GW_AW5808_RX_Read_XX_data(uint8_t reg_address, uint8_t* _get_data, uint8_t data_len)
{
	int8_t r_value=-1;
	uint8_t get_data[20]={0},i=0;	

	if(I2C0_ReadMultiBytes(AW5808_RX_ADDR, reg_address, get_data,data_len)==0)
	{
		for(i=0;i<data_len;i++)
		*(_get_data+i)=get_data[i];
		
		r_value=0;
	}	
	return r_value;	
}
//----------------------------------------------------------------------------------
int8_t GW_AW5808_RX_Write_XX_data(uint8_t reg_address, uint8_t* _write_data, uint8_t data_len)
{
	I2C0_WriteMultiByteOneReg(AW5808_RX_ADDR,reg_address,_write_data,data_len);	
  return 0;	
}
//----------------------------------------------------------------------------------
void GW_AW5808_RX_Enter_Binding(void)
{
	uint8_t w_data=0x02;	
	GW_AW5808_RX_Write_XX_data(REG_ADDR_SYS_CONTROL,&w_data,1);	
}
//----------------------------------------------------------------------------------
void GW_AW5808_RX_Unpair(void)	
{
	uint8_t w_data[4]={0x00,0x00,0x00,0x00};	
	GW_AW5808_RX_Write_XX_data(REG_ADDR_OPERATED_ID_0,w_data,4);	
}
//----------------------------------------------------------------------------------
int8_t GW_AW5808_RX_Get_ID(AW5808_DATA* aw5808_item)
{	
	int8_t r_value=-1;
	if(GW_AW5808_RX_Read_XX_data(REG_ADDR_OPERATED_ID_0,aw5808_item->id,4)==0)
	r_value=0;
	
  return r_value;
}
//----------------------------------------------------------------------------------
int8_t GW_AW5808_RX_Read_Status(AW5808_DATA* aw5808_item)
{
	int8_t r_value=-1;
	if(GW_AW5808_RX_Read_XX_data(REG_ADDR_SYS_STATUS,&(aw5808_item->status),1)==0)		
	r_value=0;
	return r_value;	
}
//----------------------------------------------------------------------------------
int8_t GW_AW5808_RX_Read_RSSI_Level(AW5808_DATA* aw5808_item)
{
	int8_t r_value=-1;
	if(GW_AW5808_RX_Read_XX_data(0x41,&(aw5808_item->rf_rssi_level),1)==0)		
	r_value=0;
	return r_value;	
}
//----------------------------------------------------------------------------------
uint8_t width_test=0;
void GW_AW5808_RX_Set_Random_Pair_Timeout(void)
{
	uint8_t w_data=0;
  w_data=random_value_array[g_au32AW5808INTCount[0]%100];

	//printf("\x1b[%d;%dHRx->%02d.%d\n", SELECT_Y+30, SELECT_X, w_data/10,w_data%10);	
	GW_AW5808_RX_Write_XX_data(REG_ADDR_PAIR_TIMEOUT_TIME,&w_data,1);	
}

//-------------------------------------------------------------------------
uint8_t old_rx_read_counter=0;

int8_t GW_AW5808_RX_Read_CCH_Data(void)
{
		int8_t r_value = -1;
		uint8_t get_data[10] = {0};
		uint8_t get_counter = 0;

		if(GW_AW5808_RX_Read_XX_data(REG_ADDR_CCH_REC_1, get_data, 10) == 0) {
				m_debug.cch_read_count++;

				r_value = 0;
				if(old_rx_read_counter != get_data[9]) {
						old_rx_read_counter = get_data[9];
						memcpy(aw5808_rx_data.cch_read_data, get_data, 10);
				}
		}
		aw5808_rx_data.cch_read_data_status = r_value;
		return r_value;
}
//-------------------------------------------------------------------------
//data_len: max value is 8
void GW_AW5808_RX_Send_CCH_Data(uint8_t *data, uint8_t data_len)
{
	uint8_t temp_len=0;
	if(data_len>8)
	temp_len=8;
	else
	temp_len=data_len;
	
	GW_AW5808_RX_Write_XX_data(REG_ADDR_CCH_DATA_1,data,temp_len);	
}
//-------------------------------------------------------------------------
uint8_t rx_s_idx=0;
void GW_AW5808_RX_Send_CCH_Data_Test(void)
{
	uint8_t w_data[][8]={
		{0xa2,0xa4,0xa6,0xa8,0xa0,0xab,0xad,0xaf},
		{0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8},
		{0xca,0xcb,0xcc,0xcd,0xce,0xcf,0xc0,0xc1},
		{0xd7,0xd8,0xd9,0xd0,0xda,0xdb,0xdc,0xdd},
		{0xec,0xed,0xee,0xef,0xe0,0xe1,0xe2,0xe4}		
	};
	
	//clean rx send data field
	printf("\x1b[34;50H                      \n");
	
	//clean tx receive data field
	printf("\x1b[28;10H                      \n");	
	printf("\x1b[29;10H                      \n");
	printf("\x1b[34;10H                      \n");
	
	if(rx_s_idx>4)
	rx_s_idx=0;
	
	printf("\x1b[34;50H%02x%02x%02x%02x%02x%02x%02x%02x\n",w_data[rx_s_idx][0],w_data[rx_s_idx][1],w_data[rx_s_idx][2],w_data[rx_s_idx][3],w_data[rx_s_idx][4],w_data[rx_s_idx][5],w_data[rx_s_idx][6],w_data[rx_s_idx][7]);	
  GW_AW5808_RX_Send_CCH_Data(&(w_data[rx_s_idx][0]),8);			
	rx_s_idx++;
}
//-------------------------------------------------------------------------
int8_t GW_AW5808_RX_Read_Version(AW5808_DATA* aw5808_item)
{
	int8_t r_value=-1;
	if(GW_AW5808_RX_Read_XX_data(REG_ADDR_FW_V_0,aw5808_item->version,2)==0)
		r_value=0;

	return r_value;	
}	
//-------------------------------------------------------------------------
int8_t GW_AW5808_RX_Read_2E_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data=0;	
	if(GW_AW5808_RX_Read_XX_data(REG_ADDR_LOSE_SYNC_PERIOD_TO_SLEEP,&get_data,1)==0)		
	{
		r_value=0;
		printf("\x1b[32;80H2E=%02x\n",get_data);
	}	

	return r_value;	
}
int8_t GW_AW5808_RX_Read_2F_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data=0;	
	if(GW_AW5808_RX_Read_XX_data(REG_ADDR_SLEEP_WAKE_UP_PERIOD,&get_data,1)==0)		
	{
		r_value=0;
		printf("\x1b[30;80H2F=%02x\n",get_data);
	}	

	return r_value;	
}
int8_t GW_AW5808_RX_Read_Module_Type(AW5808_DATA* aw5808_item)
{
	int8_t r_value=-1;
	uint8_t get_data=0;	
	if(GW_AW5808_RX_Read_XX_data(REG_ADDR_TX_RX_INDICATION,&(aw5808_item->module_type),1)==0)		
	{
		r_value=0;
		//printf("\x1b[50;100HRX(0x40)=%02x\n",aw5808_item->module_type);
	}	

	return r_value;	
}

int8_t GW_AW5808_RX_Read_rf_select(AW5808_DATA* aw5808_item)
{
	int8_t r_value=-1;
	uint8_t get_data=0;	
	if(GW_AW5808_RX_Read_XX_data(REG_ADDR_RF_SELECT,&(aw5808_item->rf_select),1)==0)		
	{
		r_value=0;
		//printf("\x1b[50;100HRX(0x40)=%02x\n",aw5808_item->module_type);
	}	

	return r_value;	
}

int8_t GW_AW5808_RX_Read_rf_power(AW5808_DATA* aw5808_item)
{
	int8_t r_value=-1;
	uint8_t get_data=0;	
	if(GW_AW5808_RX_Read_XX_data(REG_ADDR_RF_POWER,&(aw5808_item->rf_power),1)==0)		
	{
		r_value=0;
		//printf("\x1b[50;100HRX(0x40)=%02x\n",aw5808_item->module_type);
	}	

	return r_value;	
}

int8_t GW_AW5808_RX_Read_90_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data=0;	
	if(GW_AW5808_RX_Read_XX_data(0x90,&get_data,1)==0)		
	{
		r_value=0;
		printf("\x1b[33;80H90=%02x\n",get_data);
	}	

	return r_value;	
}
int8_t GW_AW5808_RX_Read_7F_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data=0;	
	if(GW_AW5808_RX_Read_XX_data(0x7f,&get_data,1)==0)		
	{
		r_value=0;
		printf("\x1b[28;80H7F=%02x\n",get_data);
	}	

	return r_value;	
}

void GW_AW5808_RX_Set_Audio_Detection(uint8_t _set_value)
{
	uint8_t set_value =_set_value;
	GW_AW5808_RX_Write_XX_data(0x90,&set_value,1);
}

void GW_AW5808_RX_Set_Lose_Sync_Period(uint8_t _set_value)
{
	uint8_t set_value =_set_value;
	GW_AW5808_RX_Write_XX_data(REG_ADDR_LOSE_SYNC_PERIOD_TO_SLEEP,&set_value,1);
}

void GW_AW5808_RX_Set_rf_select(uint8_t _set_value)
{
	uint8_t set_value =_set_value;
	GW_AW5808_RX_Write_XX_data(REG_ADDR_RF_SELECT,&set_value,1);		
}

void GW_AW5808_RX_Set_rf_power(uint8_t _set_value)
{
	uint8_t set_value =_set_value;
	GW_AW5808_RX_Write_XX_data(REG_ADDR_RF_POWER,&set_value,1);		
}
//--------------------------UPDATE UI--------------------------------------
uint8_t got_dfu_enable=0;
void update_aw5808_TX_item(void)
{
	if(aw5808_dfu_enable==0)
	{
		got_dfu_enable=0;
	if(aw5808_tx_data.online==0)
	{
		 if(aw5808_tx_data.old_online!=aw5808_tx_data.online)
			{
			aw5808_tx_data.old_online=aw5808_tx_data.online;
			printf("\x1b[9;72Hfail\n");
			printf("\x1b[10;64H\x1b[0;K");	
			printf("\x1b[11;68H\x1b[0;K");
			printf("\x1b[12;69H\x1b[0;K");			
			}
	}
	else
	{
		if(aw5808_tx_data.old_online!=aw5808_tx_data.online)
		{
			aw5808_tx_data.old_online=aw5808_tx_data.online;
				printf("\x1b[9;72H  ok\n");				
		}
		//id	
		if(memcmp(aw5808_tx_data.id,aw5808_tx_data.old_id,4)!=0)					
		{
			memcpy(aw5808_tx_data.old_id,aw5808_tx_data.id,4);					
			printf("\x1b[10;64H%02x%02x%02x%02x\n",aw5808_tx_data.id[0],aw5808_tx_data.id[1],aw5808_tx_data.id[2],aw5808_tx_data.id[3]);	
		}
			//status
		if(aw5808_tx_data.old_status!=aw5808_tx_data.status)				
		{
			aw5808_tx_data.old_status=aw5808_tx_data.status;					
			printf("\x1b[11;68H%02x",aw5808_tx_data.status);
			printf("\x1b[0;K");
			
			//bit 0
			if(aw5808_tx_data.status&0x01)
				printf("|sync");
			else
				printf("|no sync");
	
			//bit5,4
			switch( (aw5808_tx_data.status&0x30)>>4)
			{
				case 0x03:
					printf("|under-pairing");		
				break;
				case 0x01:
					printf("|pairing-ok");		
				break;							
				case 0x02:
					printf("|pairing-fail");		
				break;
				case 0x00:
					printf("|exit-pairing");			
				break;						
			}
			printf("\n");			
		}
		//version
		if(memcmp(aw5808_tx_data.version,aw5808_tx_data.old_version,2)!=0)					
		{
			memcpy(aw5808_tx_data.old_version,aw5808_tx_data.version,2);					
			printf("\x1b[12;69H%02x.%02x\n",aw5808_tx_data.version[0],aw5808_tx_data.version[1]);	
			
			//show module	
			printf("\x1b[12;76HType=%02x/%02x/%02x\n",aw5808_tx_data.module_type, aw5808_tx_data.rf_select, aw5808_tx_data.rf_power);	
		}		
	}
}
	else
	{
		//first enter the dfu update process
		if(got_dfu_enable==0)
		{
			got_dfu_enable=1;
			aw5808_tx_data.online=0;
			aw5808_tx_data.old_online=0xff;
			aw5808_tx_data.dfu_process_valule=0;
			aw5808_tx_data.dfu_process_old_valule=0xff;

			memset(aw5808_tx_data.old_id,0xff,4);
			memset(aw5808_tx_data.old_version,0,2);
			aw5808_tx_data.old_status=0xff;			
			
			printf("\x1b[9;72H  0%%\n");
			printf("\x1b[10;64H\x1b[0;K");	
			printf("\x1b[11;68H\x1b[0;K");
			printf("\x1b[12;69H\x1b[0;K");
		}
		
		if(aw5808_tx_data.online==0)
		{
			
			if(dfu_error)
				{
				  if(dfu_error==ERR_WAIF_FOR_POLL)
					printf("\x1b[9;72HWait Response   \n");
          else if(dfu_error==ERR_LOST_COMPANY_NAME)
					printf("\x1b[9;72HErr company name\n");
          else if(dfu_error==ERR_PKT_SEQUENCE)
					printf("\x1b[9;72HErr Pkt Sequence\n");
          else if(dfu_error==ERR_READ_0X7F)
					printf("\x1b[9;72HErr read 0x7f   \n");						
          else if(dfu_error==ERR_QUERY_TIMEOUT)
					printf("\x1b[9;72HErr Querytimeout\n");					
					else
					printf("\x1b[9;72HErr=%d           \n",dfu_error);					
				}
				else
				{
				if(aw5808_tx_data.dfu_process_old_valule!=aw5808_tx_data.dfu_process_valule)
		{
					aw5808_tx_data.dfu_process_old_valule=aw5808_tx_data.dfu_process_valule;
					printf("\x1b[9;72H%03d%%            \n",aw5808_tx_data.dfu_process_valule);			
		}
				}					
			
	
			}
	}	
}

uint8_t rssi_old_value=0xff, sync_retry_count = 0;
void update_aw5808_RX_item(void)
{
		if(aw5808_dfu_enable==0) {
				got_dfu_enable=0;
				if(aw5808_rx_data.online==0) {
						if(aw5808_rx_data.old_online != aw5808_rx_data.online) {
								aw5808_rx_data.old_online=aw5808_rx_data.online;
								printf("\x1b[5;72Hfail\n");
								printf("\x1b[6;64H\x1b[0;K");
								printf("\x1b[7;68H\x1b[0;K");
								printf("\x1b[8;69H\x1b[0;K");
								printf("\x1b[9;66H\x1b[0;K");
						}
				}
				else {
						if(aw5808_rx_data.old_online!=aw5808_rx_data.online) {
								aw5808_rx_data.old_online=aw5808_rx_data.online;
								printf("\x1b[5;72H  ok\n");
						}
						//id
						if(memcmp(aw5808_rx_data.id,aw5808_rx_data.old_id,4) != 0) {
								memcpy(aw5808_rx_data.old_id,aw5808_rx_data.id,4);
								printf("\x1b[6;64H%02x%02x%02x%02x\n",aw5808_rx_data.id[0],aw5808_rx_data.id[1],aw5808_rx_data.id[2],aw5808_rx_data.id[3]);	
						}
						//status
						if(aw5808_rx_data.old_status != aw5808_rx_data.status) {
								aw5808_rx_data.old_status=aw5808_rx_data.status;
								printf("\x1b[7;68H%02x",aw5808_rx_data.status);
								printf("\x1b[0;K");
								//bit 0
								if(aw5808_rx_data.status&0x01)
										printf("|sync");
								else
										printf("|no sync");
								//bit5,4
								switch( (aw5808_rx_data.status&0x30)>>4)
								{
										case 0x03:
												printf("|under-pairing");
												break;
										case 0x01:
												printf("|pairing-ok");
												break;
										case 0x02:
												printf("|pairing-fail");
												break;
										case 0x00:
												printf("|exit-pairing");
												break;
								}
								printf("\n");
						}
						//else
						//		m_module.sync_retry_timeout_ms = 0;
						//version
						if(memcmp(aw5808_rx_data.version,aw5808_rx_data.old_version,2)!=0) {
								memcpy(aw5808_rx_data.old_version,aw5808_rx_data.version,2);
								printf("\x1b[8;69H%02x.%02x\n",aw5808_rx_data.version[0],aw5808_rx_data.version[1]);
								//show module
								printf("\x1b[8;76HType=%02x/%02x/%02x\n",aw5808_rx_data.module_type, aw5808_rx_data.rf_select, aw5808_rx_data.rf_power);			
						}
						//RSSI
						if(aw5808_rx_data.odd_rf_rssi_level!=aw5808_rx_data.rf_rssi_level) {
								aw5808_rx_data.odd_rf_rssi_level=aw5808_rx_data.rf_rssi_level;
								printf("\x1b[9;66H%02x\n",aw5808_rx_data.rf_rssi_level);
						}
				}
		}
		else {
				//first enter the dfu update process
				if(got_dfu_enable==0) {
						got_dfu_enable=1;
						aw5808_rx_data.online=0;
						aw5808_rx_data.old_online=0xff;
						aw5808_rx_data.dfu_process_valule=0;
						aw5808_rx_data.dfu_process_old_valule=0xff;
						memset(aw5808_rx_data.old_id,0x11,4);
						memset(aw5808_rx_data.old_version,0,2);
						aw5808_rx_data.old_status=0xff;
						printf("\x1b[5;72H  0%%\n");
						printf("\x1b[6;64H\x1b[0;K");
						printf("\x1b[7;68H\x1b[0;K");
						printf("\x1b[8;69H\x1b[0;K");
				}

				if(aw5808_rx_data.online==0) {
						if(dfu_error) {
								if(dfu_error==ERR_WAIF_FOR_POLL)
										printf("\x1b[5;72HWait Response   \n");
								else if(dfu_error==ERR_LOST_COMPANY_NAME)
										printf("\x1b[5;72HErr company name\n");
								else if(dfu_error==ERR_PKT_SEQUENCE)
										printf("\x1b[5;72HErr Pkt Sequence\n");
								else if(dfu_error==ERR_READ_0X7F)
										printf("\x1b[5;72HErr read 0x7f   \n");
								else if(dfu_error==ERR_QUERY_TIMEOUT)
										printf("\x1b[5;72HErr Querytimeout\n");
								else
										printf("\x1b[5;72HErr=%d           \n",dfu_error);
						}
						if(aw5808_rx_data.dfu_process_old_valule!=aw5808_rx_data.dfu_process_valule) {
								aw5808_rx_data.dfu_process_old_valule=aw5808_rx_data.dfu_process_valule;
								printf("\x1b[5;72H%03d%%            \n",aw5808_rx_data.dfu_process_valule);
						}
				}
		}	
}
//-----------------------------------------------------------------------------------------------------------
uint8_t test_times=0;
uint8_t start_count=0;

uint8_t unpair_done=0;

uint8_t do_rx=1;
uint8_t check_under_pair=0;
uint8_t show_time_field_count=0;

uint8_t wait_for_next_step=0;
uint32_t total_time=0;

uint8_t rx_pair_done=0;
uint8_t tx_pair_done=0;
uint8_t show_idx=0;

uint8_t aw5808_auto_pair_status=AW5808_AUTO_PAIR_STATUS_IDLE;
uint8_t aw5808_auto_pair_result=AW5808_AUTO_PAIR_RESULT_IDLE;
uint8_t retry_times=10;

uint8_t old_aw5808_auto_pair_status=0xff;
uint8_t old_aw5808_auto_pair_result=0xff;
//-----------------------------------------------------------------
void AW5808_Test_Pair_Unpair_process_init_parameters(void)
{
	uint8_t i=0;
	for(i=0;i<30;i++)
	printf("\x1b[%d;0H\x1b[0;K",27+i);			 

	tx_test_status=0;
	rx_test_status=0;
	wait_for_next_step=2;
	current_test_mode=0;
	show_idx=0;
	start_count=0;	
	test_times=0;
	rx_pair_done=0;
	tx_pair_done=0;
	total_time=0;
	width_test=0;
	show_time_field_count=0;
			
	do_rx=1;
	unpair_done=0;		 

  aw5808_auto_pair_status=AW5808_AUTO_PAIR_STATUS_IDLE;
	aw5808_auto_pair_result=AW5808_AUTO_PAIR_RESULT_IDLE;
	retry_times=10;	
}

#ifndef FUNCTION_NOT_SUPPORT
//-----------------------------------------------------------------
uint8_t tx_temp_id[4]={0};
void AW5808_Test_Pair_Unpair_process_unpair(void)
{

	
	if(unpair_done==0)
	{
		if(wait_for_next_step>0)
		{
			wait_for_next_step--;
		}
		else
		{
			//store tx id before unpairing
			memcpy(tx_temp_id,aw5808_tx_data.id,4);
			
			GW_AW5808_RX_Unpair();
			GW_AW5808_TX_Unpair();

			rx_pair_done=0;
			tx_pair_done=0;			
			unpair_done=1;						
		}
	}
	else
	{
		if( memcmp("\x11\x22\x33\x44",aw5808_rx_data.id,4)==0 && memcmp(tx_temp_id,aw5808_tx_data.id,4)!=0)				
		{
			unpair_done=0;							
			if(do_rx==1)
			{
				//printf("\x1b[46;45H1 \n");
				current_test_mode=1;//RX
				rx_test_status=0;
				wait_for_next_step=0;									
			}
			else
			{
				//printf("\x1b[46;45H2 \n");
				current_test_mode=2;//TX
				tx_test_status=0;
				wait_for_next_step=0;									
			}
		}						
	}
}
#endif
//-----------------------------------------------------------------

#ifndef FUNCTION_NOT_SUPPORT
void AW5808_Test_Pair_Unpair_process_Rx(void)
{
	uint8_t temp_value=0,temp_value_2=0;
	//set random timeout value of rx pairing
	if(rx_test_status==0)
	{
		//printf("\x1b[46;45H3 \n");
		if(wait_for_next_step>0)
		{
			wait_for_next_step--;
		}
		else
		{
			if(memcmp("\x11\x22\x33\x44",aw5808_rx_data.id,4)==0)
			{
				GW_AW5808_RX_Set_Random_Pair_Timeout();
				rx_test_status=1;
				wait_for_next_step=2;
			}
		}
	}
	//set rx pairing
	else if(rx_test_status==1)
	{
		//printf("\x1b[46;45H3b\n");
		if(wait_for_next_step>0)
		{
			wait_for_next_step--;
		}
		else 
		{
			GW_AW5808_RX_Enter_Binding();
			if(start_count==0)
			{
				start_count=1;
				aw5808_auto_test_count=0;							
			}			
			wait_for_next_step=2;
			rx_test_status=2;								
		}
	}
	//check rx status
	else if(rx_test_status==2)
	{	
		if(wait_for_next_step>0)
		{
			wait_for_next_step--;					
		}
		else
		{			
			temp_value=((aw5808_rx_data.status&0x30)>>4);
			temp_value_2=aw5808_rx_data.status&0x01;
					
			//under-pair
			if(temp_value==0x03)
			{
				//printf("\x1b[46;45H4 \n");		
				//check_under_pair=1;
			}		
			//pair-ok, leave process
			else if(temp_value==0x01)						
			{
					//printf("\x1b[%d;0H%d-R %10d\n",28+show_idx,test_times,aw5808_auto_test_count);
				show_idx++;
				total_time+=aw5808_auto_test_count;
				rx_pair_done=1;

				if(tx_pair_done==1)
				{	
					run_test=0;
					//printf("\x1b[%d;15H->%10d\n",27+show_idx,total_time);
					aw5808_auto_pair_status=AW5808_AUTO_PAIR_STATUS_STOP;
					aw5808_auto_pair_result=AW5808_AUTO_PAIR_RESULT_PASS;
					rx_test_status=0;								
				}
				else
				{
					start_count=0;
					aw5808_auto_test_count=0;
		
					current_test_mode=2;//change to tx
					rx_test_status=0;
					tx_test_status=0;
					wait_for_next_step=2;
					memcpy(tx_temp_id,aw5808_tx_data.id,4);
					GW_AW5808_TX_Unpair();								
				}			
			}
			//pair-fail, change mode
			else if(temp_value==0x02 && temp_value_2==0x00)				
			{
					if(retry_times>0)
					{
						retry_times--;
						GW_AW5808_RX_Unpair();
						if(tx_pair_done==0)
						{
							current_test_mode=2;//change to tx
							rx_test_status=0;
							tx_test_status=0;
							wait_for_next_step=2;
							memcpy(tx_temp_id,aw5808_tx_data.id,4);
							GW_AW5808_TX_Unpair();								
						}
						else
						{
							current_test_mode=1;//re-start rx	
							rx_test_status=0;
							tx_test_status=0;
							wait_for_next_step=2;
						}						
					}					
					else
					{
							run_test=0;
							aw5808_auto_pair_status=AW5808_AUTO_PAIR_STATUS_STOP;
						  aw5808_auto_pair_result=AW5808_AUTO_PAIR_RESULT_FAIL;							
					}
			}					
		}
	}
}
#endif

#ifndef FUNCTION_NOT_SUPPORT
//-----------------------------------------------------------------
void AW5808_Test_Pair_Unpair_process_Tx(void)
{
	uint8_t temp_value=0,temp_value_2=0;	
	//set random timeout value of tx pairing
	if(tx_test_status==0)
	{				
		//printf("\x1b[46;45H7 \n");
		if(wait_for_next_step>0)
		{
			wait_for_next_step--;
		}
		else
		{
			if(memcmp(tx_temp_id,aw5808_tx_data.id,4)!=0)
			{
				GW_AW5808_TX_Set_Random_Pair_Timeout();
				tx_test_status=1;
				wait_for_next_step=2;						
			}
		}
	}
	//set tx pairing	
	else if(tx_test_status==1)
	{
		if(wait_for_next_step>0)
		{
			wait_for_next_step--;
		}
		else
		{
			GW_AW5808_TX_Enter_Binding();
			if(start_count==0)
			{
				start_count=1;
				aw5808_auto_test_count=0;						
			}				
			wait_for_next_step=2;
			tx_test_status=2;						
		}
	}
	//check tx status
	else if(tx_test_status==2)
	{
		if(wait_for_next_step>0)
		{
			wait_for_next_step--;					
		}
		else
		{
			temp_value=((aw5808_tx_data.status&0x30)>>4);
			temp_value_2=aw5808_tx_data.status&0x01;
					
			if(temp_value==0x03)
			{
				//printf("\x1b[46;45H8 \n");
				//check_under_pair=1;
			}
			//pair-ok, leave process
			else if(temp_value==0x01)						
			{
				//printf("\x1b[46;45H9 \n");
				//check_under_pair=0;
				//printf("\x1b[%d;0H%d-T %10d\n",28+show_idx,test_times,aw5808_auto_test_count);
				show_idx++;
				total_time+=aw5808_auto_test_count;
				tx_pair_done=1;

				if(rx_pair_done==1)
				{	
					run_test=0;
					//printf("\x1b[%d;15H->%10d\n",27+show_idx,total_time);	
					aw5808_auto_pair_status=AW5808_AUTO_PAIR_STATUS_STOP;
					aw5808_auto_pair_result=AW5808_AUTO_PAIR_RESULT_PASS;
					tx_test_status=0;								
				}
				else
				{
					start_count=0;
					aw5808_auto_test_count=0;
						
					current_test_mode=1;//change to rx
					rx_test_status=0;
					tx_test_status=0;
					wait_for_next_step=2;
					GW_AW5808_RX_Unpair();								
				}								
			}
			//pair-fail, change mode
			else if(temp_value==0x02 && temp_value_2==0x00)
			{
				if(retry_times>0)
				{
					retry_times--;
					GW_AW5808_TX_Unpair();
					if(rx_pair_done==0)
					{
						current_test_mode=1;//change to rx
						rx_test_status=0;
						tx_test_status=0;
						wait_for_next_step=2;
						GW_AW5808_RX_Unpair();									
					}
					else
					{
						current_test_mode=2;//re-start tx	
						rx_test_status=0;
						tx_test_status=0;
						wait_for_next_step=2;							
					}
				}			
				else
				{
					aw5808_auto_pair_status=AW5808_AUTO_PAIR_STATUS_STOP;
					aw5808_auto_pair_result=AW5808_AUTO_PAIR_RESULT_FAIL;				
				}
			}			
		}				
	}		
}
#endif

#ifndef FUNCTION_NOT_SUPPORT
//--------------------------------------------------------------------------------------------
//test process:
//0: initialize parameters
//1: unpair process, will unpair tx and rx then verify by the id
//2: rx pairing
//3: tx pairing
void AW5808_Test_Pair_Unpair_process(void)
{	
	if(check_aw5808_autopair_flag)
	{
		check_aw5808_autopair_flag=0;
		if(check_mcu_power_enable==0)
		return;
		
	 //initialize parameters
	 if(enable_tx_rx_random_test)
	 {
		 enable_tx_rx_random_test=0;		 		 
		 AW5808_Test_Pair_Unpair_process_init_parameters();
		 run_test=1;				 
		 aw5808_auto_pair_status=AW5808_AUTO_PAIR_STATUS_RUNNING;				
	 }		
		
	 if(run_test==1)
	 {
		  //unpair
			if(current_test_mode==0)
			{
				AW5808_Test_Pair_Unpair_process_unpair();
			}
			//RX
		  else if(current_test_mode==1)
			{
				AW5808_Test_Pair_Unpair_process_Rx();
			}
			//TX
			else if(current_test_mode==2)
			{
				AW5808_Test_Pair_Unpair_process_Tx();
			}
		}			
	}
}
#endif

//--------------------------------------------------------------------------------------------
uint8_t exec_rx_read=0;
uint8_t temp_lock=0;
void AW5808_RX_CCH_GetData_process(void)
{
		if(exec_rx_read == 1) {
				if(temp_lock == 0) {
						temp_lock = 1;
						m_debug.cch_int_count++;

						if(GW_AW5808_RX_Read_Status(&aw5808_rx_data) == 0) {
//PrintMsg__("Status 3: %02X \r\n", aw5808_rx_data.status);
								if((m_cch_rx.status_count++)%20 == 0)
										GW_AW5808_RX_Read_RSSI_Level(&aw5808_rx_data);
								update_aw5808_RX_item();

								if(GW_AW5808_RX_Read_CCH_Data() == 0) {
										m_debug.cch_read_time_ms = m_module.current_time_ms;
								}
						}
						temp_lock = 0;
						exec_rx_read = 0;
				}
		}
}
//---------------------------------------------------------------------------------------------
uint8_t exec_tx_read=0;
uint8_t temp_lock_2=0;
void AW5808_TX_CCH_GetData_process(void)
{
		if(exec_tx_read==1)
		{
			if(temp_lock_2==0)
			{
				temp_lock_2=1;

			if(GW_AW5808_TX_Read_CCH_Data()==0)
				{
					exec_tx_read=0;		
				}
				temp_lock_2=0;
			}
		}
}
//---------------------------------------------------------------------------------------
uint8_t check_aw5808_rx_update_flag=0;
uint8_t check_aw5808_tx_update_flag=0;
uint8_t check_aw5808_tx_cch_update_flag=0;
uint8_t check_touch_update_flag=0;
uint8_t check_touch_fwupdate_flag=0;
uint8_t check_aw5808_autopair_flag=0;
uint8_t check_aw5808_dfu_flag=0;
uint16_t check_aw5808_dfu_scan_time=DFU_SCAN_TIME;
uint8_t check_aw5808_hdi_update_flag=0;
uint8_t enable_aw5808_hdi_check=0;
uint8_t aw5808_hdi_start_count=0;
uint32_t aw5808_freq_count=0;
uint32_t aw5808_scan_time_count=0;
uint32_t aw5808_tx_cch_count=0;
uint32_t dfu_scan_time_count=0;
uint32_t bat_detect_count=0;
uint32_t bat_detect_update_count=0;
uint8_t check_battery_status_update_flag=0;
uint8_t check_battery_voltage_flag=0;
uint32_t check_vdm_count=0;
uint8_t check_vdm_update_flag=0;

void aw5808_check_action(void)
{
	if(check_mcu_power_enable==0)
	return;
	
	if(aw5808_dfu_enable)
	{
		dfu_scan_time_count = (dfu_scan_time_count==0)?g_au32AW5808INTCount[0]:dfu_scan_time_count;
		if( (g_au32AW5808INTCount[0]-dfu_scan_time_count)>=check_aw5808_dfu_scan_time)
		{
			dfu_scan_time_count=0;
			check_aw5808_dfu_flag=1;
		}	
	}
	else
	{
		aw5808_freq_count = (aw5808_freq_count==0)?g_au32AW5808INTCount[0]:aw5808_freq_count;
		aw5808_scan_time_count = (aw5808_scan_time_count==0)?g_au32AW5808INTCount[0]:aw5808_scan_time_count;
		aw5808_tx_cch_count = (aw5808_tx_cch_count==0)?g_au32AW5808INTCount[0]:aw5808_tx_cch_count;	
		bat_detect_count = (bat_detect_count==0)?g_au32AW5808INTCount[0]:bat_detect_count;			
		bat_detect_update_count = (bat_detect_update_count==0)?g_au32AW5808INTCount[0]:bat_detect_update_count;
		check_vdm_count = (check_vdm_count==0)?g_au32AW5808INTCount[0]:check_vdm_count;		
		
		if((g_au32AW5808INTCount[0]-aw5808_freq_count)>=AW5808_FREQ) 
		{
			aw5808_freq_count=0;
			check_aw5808_rx_update_flag=1;
			check_aw5808_tx_update_flag=1;

			if(mcu_power_on_delay_tx==0)			
			check_touch_fwupdate_flag=1;

			if(mcu_power_on_delay_tx==0 && mcu_power_on_delay_rx==0)				
			check_aw5808_autopair_flag=1;
		}

		if((g_au32AW5808INTCount[0]-aw5808_scan_time_count)>=scan_time) 
		{
			aw5808_scan_time_count=0;
			check_touch_update_flag=1;
		}
	
		if((g_au32AW5808INTCount[0]-aw5808_tx_cch_count)>=100) 
		{
			aw5808_tx_cch_count=0;
			check_aw5808_tx_cch_update_flag=1;
		}
		
		if((g_au32AW5808INTCount[0]-bat_detect_count)>=3000) 
		{
			bat_detect_count=0;
			check_battery_voltage_flag=1;
		}
		
		if((g_au32AW5808INTCount[0]-bat_detect_update_count)>=1000) 
		{
			bat_detect_update_count=0;
			check_battery_status_update_flag=1;
		}
		if((g_au32AW5808INTCount[0]-check_vdm_count)>=50) 
		{
			check_vdm_count=0;
			check_vdm_update_flag=1;
		}		
	}
}
//---------------------------------------------------------------------------------------------
volatile uint8_t aw5808_dfu_enable=0;
volatile uint8_t aw5808_dfu_channel=0; //0:tx 1:rx

uint8_t aw5808_dfu_wait_time_ms = 10;
uint8_t aw5808_dfu_wait_count = 0;
uint8_t aw5808_dfu_update_flag=0;
uint8_t aw5808_dfu_show_updating=0;

//mcu power enable
uint8_t check_mcu_power_enable=0;
//uint8_t old_check_mcu_power_enable=0xff;
uint8_t old_check_mcu_power_enable=0;

uint8_t mcu_power_on_delay_rx=0;
uint8_t mcu_power_on_delay_tx=0;
uint8_t mcu_power_on_delay_touch=0;

uint8_t aw5808_tx_connect=0;
uint8_t aw5808_rx_connect=0;
uint8_t touch_connect=0;
//-----------------------------------------------------------------------
void mcu_power_check_function()
{
	check_mcu_power_enable = GPO_PD02_MCU_PWREN;
	if(old_check_mcu_power_enable!=check_mcu_power_enable)
	{
		 old_check_mcu_power_enable=check_mcu_power_enable;
		 if(check_mcu_power_enable)
		 {
				GW_AW5808_Rx_On_Off(1);
			  memset(&aw5808_rx_data,0xff,sizeof(aw5808_rx_data));
		 }
		 else
		 {
				GW_AW5808_Rx_On_Off(0);
			  aw5808_rx_data.online=0;
			  update_aw5808_RX_item();			 
		 }
	}
}
//-----------------------------------------------------------------------
uint8_t usci_check_start=0;

void aw5808_RX_work_function(void)
{
		int8_t r_code;

		if(GPO_PD02_MCU_PWREN == NO)
				return;

		if(aw5808_rx_data.online == 0xFF) {
				r_code = GW_AW5808_RX_Get_ID(&aw5808_rx_data);
				if(r_code == 0) {
						GW_AW5808_RX_Set_rf_power(0x22);
						GW_AW5808_RX_Set_rf_select(0x01);
						GW_AW5808_RX_Read_rf_select(&aw5808_rx_data);
						GW_AW5808_RX_Read_rf_power(&aw5808_rx_data);
						GW_AW5808_RX_Read_Module_Type(&aw5808_rx_data);
						GW_AW5808_RX_Read_Version(&aw5808_rx_data);
						//write register 0x90 to value 0x00 for disabling no audo detection
						GW_AW5808_RX_Set_Audio_Detection(0x00);
						GW_AW5808_RX_Set_Lose_Sync_Period(0x00);
						aw5808_rx_data.online = 1;
				}
				else {
				}
		}

		AW5808_RX_CCH_GetData_process();
}

//-----------------------------------------------------------------------
void aw5808_TX_work_function(void)
{
	if(check_aw5808_tx_update_flag)
	{
		check_aw5808_tx_update_flag=0;		
		if(check_mcu_power_enable==0)
		return;
			
    if(mcu_power_on_delay_tx>0)
		{
			mcu_power_on_delay_tx--;
			return;
		}
		
		if(aw5808_tx_data.online==0xff || aw5808_tx_data.online==1)
		{			
			printf("\x1b[13;61HMIC HW=%d\n",get_hardware_version());			
			
			if(GW_AW5808_TX_Get_ID(&aw5808_tx_data)<0)				
			{
				aw5808_tx_data.online=0;	
				aw5808_tx_connect=0;
			}			
			else
			{
				if(aw5808_tx_data.online==0xff)
				{
					uint8_t set_data[4]={0,0,0,0};
					set_data[0] = 0x22; //3.6dBm
					GW_AW5808_TX_Set_rf_power(set_data,1);
					set_data[0] = 0x01;
					GW_AW5808_TX_Set_rf_select(set_data,1);
					
					GW_AW5808_TX_Read_module_type(&aw5808_tx_data);					
					GW_AW5808_TX_Read_rf_select(&aw5808_tx_data);
					GW_AW5808_TX_Read_rf_power(&aw5808_tx_data);
					GW_AW5808_TX_Read_Version(&aw5808_tx_data);	
					//write register from 0x90 to 0x93 to value 0x00 for disabling no audio detection 
					set_data[0] = 0x00;
					set_data[1] = 0x07;
					set_data[2] = 0x53;
					set_data[3] = 0x00;
					GW_AW5808_TX_Set_Audio_Detection(set_data, 4);
				}					
				aw5808_tx_data.online=1;
				aw5808_tx_connect=1;
				GW_AW5808_TX_Read_Status(&aw5808_tx_data);					
			}
		}
		update_aw5808_TX_item();
		//Show_VDM_Slave_Data();			
	}
	
	if(check_aw5808_tx_cch_update_flag)
	{
		check_aw5808_tx_cch_update_flag=0;
		if(check_mcu_power_enable==0)
		return;		

    if(aw5808_tx_data.online==0)
		return;		
	  AW5808_TX_CCH_GetData_process();		
	}
}
//-----------------------------------------------------------------------
uint8_t touch_online=0xff;

//-----------------------------------------------------------------------
volatile uint8_t get_aw5808_hid_event_response=0;
//-----------------------------------------------------------------------
void GW_AW5808_Task_Handler(void)
{
		if((I2C_READY_TMRINT_count > 1500) || (get_usb_cmd == DESC_STRING)) {
				mcu_power_check_function();
		}

		if((i2c_init == YES) && (I2C_READY_TMRINT_count > 100)) {
				if(aw5808_dfu_enable == NO) {
						aw5808_RX_work_function();
						GW_Battery_Process();
						dfu_scan_time_count_2 = 0;
				}
				else {
						AW5808_DFU_process();
				}
		}
}

/*---------------------------------------------------------------------------------------------------------*/
/*  GW_AW5808_Init                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t AW5808_TX_Addr=0;

void GW_AW5808_Init(void)
{
    /* Init USCI_I2C0 */
    UI2C0_Init();	

    g_eSEvent = SLAVE_ADDRESS_ACK;
    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    /* I2C function to Slave receive/transmit data */
    s_UI2C0HandlerFn = UI2C_SlaveTRx;


		GPO_PC04_PD_DFP_INT=1;
}

volatile uint8_t i2c_init = NO;
/*---------------------------------------------------------------------------------------------------------*/
void GW_AW5808_Rx_On_Off(uint8_t _value)
{
	//on
	if(_value)
	{
		//printf("\x1b[35;92H--- on---\n");	
		if(i2c_init == NO) 
		{
		NVIC_EnableIRQ(I2C0_IRQn);
    I2C_EnableInt(I2C0);
    I2C_Open(I2C0, 100000);		
			i2c_init = YES;
			I2C_READY_TMRINT_count = 0;	
		}
	}
	//off
	else
	{
		//printf("\x1b[35;92H---off---\n");					
    NVIC_DisableIRQ(I2C0_IRQn);
    I2C_DisableInt(I2C0);
    I2C_Close(I2C0);			
		i2c_init=NO;		
	}
}
//------------------------------------------------
void GW_USCI_I2C_On_Off(uint8_t _value)
{
	//on
	if(_value)
	{
		NVIC_EnableIRQ(USCI0_IRQn);	
    UI2C_Open(UI2C0, 400000);
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk | UI2C_PROTIEN_ARBLOIEN_Msk |UI2C_PROTIEN_ERRIEN_Msk));	
    UI2C_EnableInt(UI2C0, UI2C_TO_INT_MASK | UI2C_STAR_INT_MASK | UI2C_STOR_INT_MASK | UI2C_NACK_INT_MASK | UI2C_ARBLO_INT_MASK | UI2C_ERR_INT_MASK | UI2C_ACK_INT_MASK);	

	}
	//off
	else
	{	
		UI2C_Close(UI2C0);
		UI2C_DISABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk | UI2C_PROTIEN_ARBLOIEN_Msk |UI2C_PROTIEN_ERRIEN_Msk));
    UI2C_DisableInt(UI2C0, UI2C_TO_INT_MASK | UI2C_STAR_INT_MASK | UI2C_STOR_INT_MASK | UI2C_NACK_INT_MASK | UI2C_ARBLO_INT_MASK | UI2C_ERR_INT_MASK | UI2C_ACK_INT_MASK);		
		NVIC_DisableIRQ(USCI0_IRQn);
	}
}