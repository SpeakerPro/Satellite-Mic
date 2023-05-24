#include <stdio.h>
#include "NuMicro.h"
#include "GW_main.h"
#include "GW_aw5808.h"
#include "GW_touch.h"
#include <string.h>

//#define IGNORE_PRINTF
#ifdef IGNORE_PRINTF
	#define printf(fmt, ...) (0)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr;
uint8_t g_u8SlaveAddr=0;
uint8_t g_u8DataRegAddr=0;
//volatile uint8_t g_u8MstTxData[20];
volatile uint8_t g_u8MstTxData[200];//touch config update
volatile uint8_t g_u8MstRxData[20];

uint8_t g_u8MstDataLen;
uint8_t g_u8MstDataLen_expected;
uint8_t g_u8MstDataLen_expected_r;
volatile uint8_t g_u8MstEndFlag = 0;
typedef void (*I2C_FUNC)(uint32_t u32Status);
static volatile I2C_FUNC s_pfnI2C0Handler = NULL;

uint8_t pre_touch_button_status=0;

uint8_t CY8CMBR3108_LQXI_configuration[128]= 
{
    0xE4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x80, 0x7F,
    0x7F, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
    0x05, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01,
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x14, 0x03, 0x01, 0x58,
    0x00, 0x37, 0x06, 0x00, 0x00, 0x0A, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD9, 0xE6
};

uint8_t scan_time=20;
uint8_t i2c_stop_count_enable=0;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
				GPO_PD02_MCU_PWREN = LOW;
				GPO_PA11_DSP_ONOFF = HIGH;
				RESET_TMRINT_count = 0;
				while(RESET_TMRINT_count > 1000);
				outpw(&SYS->RSTSTS, SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk); //clear bit
				outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
				NVIC_SystemReset();
				while (1);

      /* Clear I2C0 Timeout Flag */
      I2C_ClearTimeoutFlag(I2C0);
			I2C_DisableTimeout(I2C0);
			I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
			g_u8MstEndFlag=1;
    }
    else
    {
       if (s_pfnI2C0Handler != NULL)
       s_pfnI2C0Handler(u32Status);
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
uint16_t c_t=0;
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
    {
			  //printf("I2C_MasterRx->0x08\n");			
        I2C_SET_DATA(I2C0, (g_u8SlaveAddr << 1));    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
			  //printf("I2C_MasterRx->0x18\n");
        I2C_SET_DATA(I2C0, g_u8DataRegAddr);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
		 	 if(aw5808_dfu_enable==0)
				c_t=0;
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
			  //printf("I2C_MasterRx->0x20\n");				
        I2C_STOP(I2C0);
			
			if(aw5808TX_TMRINTCount>100)
			{
				I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);        /* Clear SI and send STOP */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
				g_u8MstEndFlag=1;	
				aw5808TX_TMRINTCount=0;				
			}
			else
			{
        I2C_START(I2C0);				
			}
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
			  //printf("I2C_MasterRx->0x28\n");
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);//repeat start		
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
			  //printf("I2C_MasterRx->0x10\n");				
        I2C_SET_DATA(I2C0, ((g_u8SlaveAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
    {
			  //printf("I2C_MasterRx->0x40\n");					
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
    {
			  //printf("I2C_MasterRx->0x58\n");
        g_u8MstRxData[g_u8MstDataLen++] = (unsigned char) I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);	
        g_u8MstEndFlag = 1;
    }
		else if(u32Status == 0x30) /* Master transmit data NACK */
		{
			  //printf("I2C_MasterRx->0x30\n");				
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
				g_u8MstEndFlag=1;			
		}			
		else if(u32Status == 0x38) /* Arbitration Lost */
		{
			  //printf("I2C_MasterRx->0x38\n");			
			  I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);        /* Clear SI and send STOP */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
				g_u8MstEndFlag=1;			

				GW_AW5808_Tx_On_Off(0);
				GW_AW5808_Tx_On_Off(1);	
				//i2c_need_re_init=YES;			
		}
		else if(u32Status == 0x48) /* Slave Address NACK */
		{
			  //printf("I2C_MasterRx->0x48\n");			
			  I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
				g_u8MstEndFlag=1;			
		}			
		//else if(u32Status == 0x00 || u32Status==0x10)  //bus error
		else if(u32Status == 0x00)			
		{	
			  I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);        /* Clear SI and send STOP */
			  I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
				GW_AW5808_Tx_On_Off(0);
				GW_AW5808_Tx_On_Off(1);
				//i2c_need_re_init=YES;			
		}
    else
    {
        /* TO DO */
        printf("I2C_MasterRx-> Status 0x%x is NOT processed\n", u32Status);
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
		//uint8_t i=0;
	  
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
			  //printf("I2C_MasterTx->0x08\n");
        I2C_SET_DATA(I2C0, g_u8SlaveAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
			  //printf("I2C_MasterTx->0x18\n");
        I2C_SET_DATA(I2C0, g_u8DataRegAddr);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
			  //printf("I2C_MasterTx->0x20\n");			
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
			  //printf("I2C_MasterTx->0x28 len=%d ex_len=%d\n",g_u8MstDataLen,g_u8MstDataLen_expected);
        if (g_u8MstDataLen < g_u8MstDataLen_expected)
        {
            I2C_SET_DATA(I2C0, g_u8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
						//printf("set end flag\n");
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
		else if (u32Status == 0x30 || u32Status == 0xf8)
		{
				I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
				I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
				g_u8MstEndFlag = 1;	
				GW_AW5808_Tx_On_Off(0);
				GW_AW5808_Tx_On_Off(1);			
		}		 
    else if (u32Status == 0x00)
		{
				I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);        /* Clear SI and send STOP */
				I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
				g_u8MstEndFlag = 1;
		}
    else
    {
        /* TO DO */
        printf("I2C_MasterTx-> Status 0x%x is NOT processed\n", u32Status);
				I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);        /* Clear SI and send STOP */
				g_u8MstEndFlag = 1;	
    }
}
//----------------------------------------------------------------------------------
void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    //I2C_Open(I2C0, 100000);
    I2C_Open(I2C0, 100000);//Due to the limitation of touch ic, downgrade the speed of i2c. 
    /* Get I2C0 Bus Clock */
    //printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}
//----------------------------------------------------------------------------------
void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

	  s_pfnI2C0Handler = NULL;
}
//----------------------------------------------------------------------------------
void stop_i2c0(void)
{
	g_u8MstEndFlag=1;
	I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
}
//----------------------------------------------------------------------------------
int32_t I2C0_ReadMultiByteOneReg(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t rdata[], uint32_t u32rLen)
{	
	   c_t=0;
	
     g_u8MstDataLen = 0;
     g_u8MstEndFlag = 0;	
     g_u8SlaveAddr = u8SlaveAddr;
		 g_u8DataRegAddr=u8DataRegAddr;
		 g_u8MstDataLen_expected_r=u32rLen;
				
		 if(check_mcu_power_enable==0)
		 return g_u8MstDataLen;
		
		 /* I2C function to read data from slave */
     s_pfnI2C0Handler = (I2C_FUNC)I2C_MasterRx;
     I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

		aw5808TX_TMRINTCount=0;
		
		if(aw5808_dfu_enable==0)
    {
        i2c0_check_value=0;
        i2c0_check_start=1;
    }
		
		m_module.i2c_timeout_ms = 0;
     /* Wait I2C Rx Finish */
     while (g_u8MstEndFlag == 0) {
				if(m_module.i2c_timeout_ms >= 500) {
						GPO_PD02_MCU_PWREN = LOW;
						GPO_PA11_DSP_ONOFF = HIGH;
						RESET_TMRINT_count = 0;
						while(RESET_TMRINT_count > 1000);
						outpw(&SYS->RSTSTS, SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk); //clear bit
						outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
						NVIC_SystemReset();
						while (1);
				}
		 }

	   I2C_ClearTimeoutFlag(I2C0);
		 I2C_DisableTimeout(I2C0);	
	   uint8_t i=0;
	   for(i=0;i<g_u8MstDataLen;i++)
		 rdata[i]=g_u8MstRxData[i];
	
	   return g_u8MstDataLen;
}
//----------------------------------------------------------------------------------
int8_t I2C0_ReadMultiBytes(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t rdata[], uint8_t u8rLen)
{
	uint8_t i=0,len=0,temp_data[200]={0},r_data=0;
	int8_t r_value=-1;
	
	for(i=0;i<u8rLen;i++)
	{
		r_data=0;
		if(I2C0_ReadMultiByteOneReg(u8SlaveAddr,u8DataRegAddr+i,&r_data,1))
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
//----------------------------------------------------------------------------------
int8_t I2C0_WriteMultiByteOneReg(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t wdata[], uint32_t u32rLen)
{
	   g_u8MstDataLen = 0;	
     g_u8MstEndFlag = 0;
     g_u8SlaveAddr = u8SlaveAddr;
		 g_u8DataRegAddr=u8DataRegAddr;	
		 g_u8MstDataLen_expected=u32rLen;

		 if(check_mcu_power_enable==0)
		 return 0;
			
		 uint16_t i=0;
		 for(i=0;i<u32rLen;i++)	
		 g_u8MstTxData[i]=wdata[i];
	
		 /* I2C function to write data to slave */
     s_pfnI2C0Handler = (I2C_FUNC)I2C_MasterTx;
	
     I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

	aw5808TX_TMRINTCount=0;
	
		 if(aw5808_dfu_enable==0)
    {
        i2c0_check_value=0;
        i2c0_check_start=1;
    }
		
		m_module.i2c_timeout_ms = 0;
     /* Wait I2C Tx Finish */
     while (g_u8MstEndFlag == 0) {
				if(m_module.i2c_timeout_ms >= 500) {
						GPO_PD02_MCU_PWREN = LOW;
						GPO_PA11_DSP_ONOFF = HIGH;
						RESET_TMRINT_count = 0;
						while(RESET_TMRINT_count > 1000);
						outpw(&SYS->RSTSTS, SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk); //clear bit
						outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
						NVIC_SystemReset();
						while (1);
				}
		 }
	   
	   return 0;
}
//-------------------------------------------------------------------------------
int8_t GW_AW5808_TX_Read_XX_data_new(int8_t slave_address,uint8_t reg_address, uint8_t* _get_data, uint8_t data_len)
{
	int8_t r_value=-1;
	uint8_t get_data[20]={0},i=0;	
		
	if(I2C0_ReadMultiBytes(slave_address, reg_address, get_data, data_len)==0)
	{
		for(i=0;i<data_len;i++)
		*(_get_data+i)=get_data[i];
		
		r_value=0;
	}	

	return r_value;	
}

int8_t GW_AW5808_TX_Read_XX_data(uint8_t reg_address, uint8_t* _get_data, uint8_t data_len)
{
	int8_t r_value=-1;
	uint8_t get_data[20]={0},i=0;	
		
	if(I2C0_ReadMultiBytes(AW5808_TX_ADDR, reg_address, get_data, data_len)==0)
	{
		for(i=0;i<data_len;i++)
		*(_get_data+i)=get_data[i];
		
		r_value=0;
	}	

	return r_value;	
}
//---------------------------------------------------------------------------------
int8_t GW_AW5808_TX_Write_XX_data_new(int8_t slave_address,int8_t reg_address, uint8_t* _write_data, uint8_t data_len)
{
	I2C0_WriteMultiByteOneReg(slave_address,reg_address,_write_data,data_len);	
	return 0;	
}
int8_t GW_AW5808_TX_Write_XX_data(uint8_t reg_address, uint8_t* _write_data, uint8_t data_len)
{
	I2C0_WriteMultiByteOneReg(AW5808_TX_ADDR,reg_address,_write_data,data_len);	
	return 0;	
}
/*---------------------------------------------------------------------------------------------------------*/
//return value: 0:pass -1:error
int8_t GW_AW5808_TX_Get_ID(AW5808_DATA* aw5808_item)	
{
	int8_t r_value=-1;
	if(GW_AW5808_TX_Read_XX_data(REG_ADDR_OPERATED_ID_0,aw5808_item->id,4)==0)		
	r_value=0;

 return r_value;
}
//----------------------------------------------------------------------------------
int8_t GW_AW5808_TX_Read_Status(AW5808_DATA* aw5808_item)
{
	int8_t r_value=-1;
	if(GW_AW5808_TX_Read_XX_data(REG_ADDR_SYS_STATUS,&(aw5808_item->status),1)==0)		
		r_value=0;
	return r_value;	
}
//----------------------------------------------------------------------------------
void GW_AW5808_TX_Enter_Binding(void)
{
	uint8_t w_data=0x02;
	GW_AW5808_TX_Write_XX_data(REG_ADDR_SYS_CONTROL,&w_data,1);	
}
/*---------------------------------------------------------------------------------------------------------*/
void GW_AW5808_TX_Unpair(void)
{
  //random generate id
	uint8_t w_data[4]={0},i=0;
	for(i=0;i<4;i++)
		w_data[i]=(g_au32AW5808INTCount[0]+17*i)%256;
		
	GW_AW5808_TX_Write_XX_data(REG_ADDR_OPERATED_ID_0,w_data,4);		
}
/*---------------------------------------------------------------------------------------------------------*/
void GW_AW5808_TX_Set_Random_Pair_Timeout(void)
{
	uint8_t w_data=0;
  w_data=random_value_array[g_au32AW5808INTCount[0]%100];

	printf("\x1b[%d;%dHTx->%02d.%d\n", SELECT_Y+29, SELECT_X, w_data/10,w_data%10);
	GW_AW5808_TX_Write_XX_data(REG_ADDR_PAIR_TIMEOUT_TIME,&w_data,1);	
}
//-------------------------------------------------------------------------
//data_len: max value is 8
void GW_AW5808_TX_Send_CCH_Data(uint8_t *data, uint8_t data_len)
{
	uint8_t temp_len=0;
	if(data_len>9)
	temp_len=9;
	else
	temp_len=data_len;
	
	GW_AW5808_TX_Write_XX_data(REG_ADDR_CCH_DATA_1,data,data_len);	
}
//-----------------------------------------------------------------------------------------------------------
uint8_t tx_s_idx=0;
void GW_AW5808_TX_Send_CCH_Data_Test(void)
{
	uint8_t w_data[][9]={
		{0x12,0x34,0x56,0x78,0x90,0xab,0xcd,0xef,0x01},
		{0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99},
		{0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x00,0x11,0x22},
		{0x77,0x88,0x99,0x00,0xaa,0xbb,0xcc,0xdd,0xee},
		{0xcc,0xdd,0xee,0xff,0x00,0x11,0x12,0x34,0x56}		
	};
	
	//clean tx send data field
	printf("\x1b[32;10H                      \n");
	
	//clean rx receive data field
	printf("\x1b[28;50H                      \n");	
	printf("\x1b[29;50H                      \n");
	printf("\x1b[32;50H                      \n");
	
	if(tx_s_idx>4)
	tx_s_idx=0;
	
	printf("\x1b[32;10H%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",w_data[tx_s_idx][0],w_data[tx_s_idx][1],w_data[tx_s_idx][2],w_data[tx_s_idx][3],w_data[tx_s_idx][4],w_data[tx_s_idx][5],w_data[tx_s_idx][6],w_data[tx_s_idx][7],w_data[tx_s_idx][8]);	
	GW_AW5808_TX_Send_CCH_Data(&(w_data[tx_s_idx][0]),9);	
	
	tx_s_idx++;
}
//-------------------------------------------------------------------------
uint8_t old_tx_read_count=0;

int8_t GW_AW5808_TX_Read_CCH_Data(void)
{
	int8_t r_value=-1;
	uint8_t get_data[9]={0};
	uint8_t get_counter=0;

	if(GW_AW5808_TX_Read_XX_data(REG_ADDR_CCH_REC_1,get_data,9)==0)		
	{
		r_value=0;
		if(old_tx_read_count!=get_data[8])
		{
			old_tx_read_count=get_data[8];
			memcpy(aw5808_tx_data.cch_read_data,get_data,9);			
		}			
	}
	aw5808_tx_data.cch_read_data_status=r_value;
	return r_value;
}
//-------------------------------------------------------------------------------
void GW_AW5808_TX_Wakeup(void)
{
	uint8_t w_data=0x00;
	GW_AW5808_TX_Write_XX_data(REG_ADDR_SYS_CONTROL,&w_data,1);	
}
//-------------------------------------------------------------------------------
void GW_AW5808_TX_Sleep(void)
{
	uint8_t w_data=0x04;
	GW_AW5808_TX_Write_XX_data(REG_ADDR_SYS_CONTROL,&w_data,1);	
}
//-------------------------------------------------------------------------------
int8_t GW_AW5808_TX_Read_Version(AW5808_DATA* aw5808_item)
{
	int8_t r_value=-1;
	if(GW_AW5808_TX_Read_XX_data(REG_ADDR_FW_V_0,aw5808_item->version,2)==0)	
		r_value=0;

	return r_value;	
}
//--------------------------------------------------------------------------------
int8_t GW_AW5808_TX_Read_2E_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data=0;	
	if(GW_AW5808_TX_Read_XX_data(REG_ADDR_LOSE_SYNC_PERIOD_TO_SLEEP,&get_data,1)==0)		
	{
		r_value=0;
		printf("\x1b[29;100H2E=%02x\n",get_data);
	}	

	return r_value;	
}
int8_t GW_AW5808_TX_Read_2F_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data=0;	
	if(GW_AW5808_TX_Read_XX_data(REG_ADDR_SLEEP_WAKE_UP_PERIOD,&get_data,1)==0)			
	{
		r_value=0;
		printf("\x1b[30;100H2F=%02x\n",get_data);
	}	

	return r_value;	
}

int8_t GW_AW5808_TX_Read_module_type(	AW5808_DATA* aw5808_item)
{

	int8_t r_value=-1;
	if(GW_AW5808_TX_Read_XX_data(REG_ADDR_TX_RX_INDICATION,&(aw5808_item->module_type),1)==0)			
	{
		r_value=0;
		//printf("\x1b[31;100HTX(0x40)=%02x\n",aw5808_item->module_type);
	}	

	return r_value;	
}

int8_t GW_AW5808_TX_Read_rf_select(	AW5808_DATA* aw5808_item)
{

	int8_t r_value=-1;
	if(GW_AW5808_TX_Read_XX_data(REG_ADDR_RF_SELECT,&(aw5808_item->rf_select),1)==0)			
	{
		r_value=0;
		//printf("\x1b[31;100HTX(0x40)=%02x\n",aw5808_item->module_type);
	}	

	return r_value;	
}

int8_t GW_AW5808_TX_Read_rf_power(	AW5808_DATA* aw5808_item)
{

	int8_t r_value=-1;
	if(GW_AW5808_TX_Read_XX_data(REG_ADDR_RF_POWER,&(aw5808_item->rf_power),1)==0)			
	{
		r_value=0;
		//printf("\x1b[31;100HTX(0x40)=%02x\n",aw5808_item->module_type);
	}	

	return r_value;	
}

int8_t GW_AW5808_TX_Read_90_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data[4]={0};	
	if(GW_AW5808_TX_Read_XX_data(0x90,get_data,4)==0)			
	{
		r_value=0;
		printf("\x1b[32;100H90=%02x%02x%02x%02x\n",get_data[3],get_data[2],get_data[1],get_data[0]);
	}	

	return r_value;	
}
int8_t GW_AW5808_TX_Read_95_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data[4]={0};	
	if(GW_AW5808_TX_Read_XX_data(0x95,get_data,4)==0)				
	{
		r_value=0;
		printf("\x1b[33;100H95=%02x%02x%02x%02x\n",get_data[3],get_data[2],get_data[1],get_data[0]);
	}	

	return r_value;	
}
int8_t GW_AW5808_TX_Read_99_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data[4]={0};	
	if(GW_AW5808_TX_Read_XX_data(0x99,get_data,4)==0)				
	{
		r_value=0;
		printf("\x1b[34;100H99=%02x%02x%02x%02x\n",get_data[3],get_data[2],get_data[1],get_data[0]);
	}	

	return r_value;	
}
int8_t GW_AW5808_TX_Read_9D_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data[4]={0};	
	if(GW_AW5808_TX_Read_XX_data(0x9d,get_data,4)==0)			
	{
		r_value=0;
		printf("\x1b[35;100H9D=%02x%02x%02x%02x\n",get_data[3],get_data[2],get_data[1],get_data[0]);
	}	

	return r_value;	
}
int8_t GW_AW5808_TX_Read_A1_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data[4]={0};	
	if(GW_AW5808_TX_Read_XX_data(0xa1,get_data,4)==0)					
	{
		r_value=0;
		printf("\x1b[36;100HA1=%02x%02x%02x%02x\n",get_data[3],get_data[2],get_data[1],get_data[0]);
	}	

	return r_value;	
}
int8_t GW_AW5808_TX_Read_A5_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data[4]={0};	
	if(GW_AW5808_TX_Read_XX_data(0xa5,get_data,4)==0)				
	{
		r_value=0;
		printf("\x1b[37;100HA5=%02x%02x%02x%02x\n",get_data[3],get_data[2],get_data[1],get_data[0]);
	}	

	return r_value;	
}
int8_t GW_AW5808_TX_Read_A9_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data[4]={0};	
	if(GW_AW5808_TX_Read_XX_data(0xa9,get_data,4)==0)		
	{
		r_value=0;
		printf("\x1b[38;100HA9=%02x%02x%02x%02x\n",get_data[3],get_data[2],get_data[1],get_data[0]);
	}	

	return r_value;	
}

int8_t GW_AW5808_TX_Read_7F_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data[4]={0};	
	if(GW_AW5808_TX_Read_XX_data(0x7f,get_data,1)==0)				
	{
		r_value=0;
		printf("\x1b[28;100H7F=%02x\n",get_data[0]);
	}	

	return r_value;	
}

int8_t GW_AW5808_TX_Read_46_data(void)
{
	int8_t r_value=-1;
	uint8_t get_data[4]={0};	
	if(GW_AW5808_TX_Read_XX_data(0x46,get_data,1)==0)				
	{
		r_value=0;
		printf("\x1b[39;100H46=%02x\n",get_data[0]);
	}	

	return r_value;	
}
	
int8_t GW_AW5808_TX_Read_7D_data(void)
	{
	int8_t r_value=-1;
	uint8_t get_data[4]={0};
	if(GW_AW5808_TX_Read_XX_data(0x7d,get_data,1)==0)				
	{
		r_value=0;
		printf("\x1b[28;100H7D=%02x\n",get_data[0]);
	}

	return r_value;	
}
void GW_AW5808_TX_Set_Audio_Detection(uint8_t* data, uint8_t data_len)
{
	GW_AW5808_TX_Write_XX_data(0x90,data,data_len);	
}

void GW_AW5808_TX_Set_rf_power(uint8_t* data, uint8_t data_len)
{
	GW_AW5808_TX_Write_XX_data(REG_ADDR_RF_POWER,data,data_len);	
}

void GW_AW5808_TX_Set_rf_select(uint8_t* data, uint8_t data_len)
{
	GW_AW5808_TX_Write_XX_data(REG_ADDR_RF_SELECT,data,data_len);	
}
//-----------------------------------------------------------------------------------------------------------
uint8_t AW5808_DFU_Calculate_CheckSum(uint8_t* _data)
{
	uint16_t checksum_value=0,i=0;
	for(i=0;i<18;i++)
	{
		checksum_value+=_data[i];
	}
	
	return checksum_value%256;	
}
//-----------------------------------------------------------------------------------------------------------
AW5808_HID_DATA_TEMP aw5808_temp_data={0};

void save_hid_aw5808_data(uint32_t* _file_idx, uint8_t* _data, uint8_t _len)
{
	aw5808_temp_data.file_idx = _file_idx;
	aw5808_temp_data.len=_len;
	memcpy(aw5808_temp_data.data,_data,_len);
}
//------------------------------------------------------------------------------------------------------------
uint8_t aw5805_dfu_step=100;
uint16_t aw5805_dfu_current_packet_number=0;
uint16_t aw5805_dfu_read_packet_address=0;

typedef int8_t (*AW5808_DFU_FUNC_P)(uint8_t u8DataRegAddr, uint8_t wdata[], uint32_t u32rLen);
AW5808_DFU_FUNC_P s_pfnDFUHandler_R = NULL;	
AW5808_DFU_FUNC_P s_pfnDFUHandler_W = NULL;
uint8_t w_data[20]={0};	
uint8_t setup_flash_mode=0;
uint8_t dfu_update_tx_ui=0;
uint8_t dfu_update_rx_ui=0;
uint8_t dfu_error=0;
uint8_t retry_time=0;
uint8_t start_to_wait_for_hid_response=0;
//-----------------------------------------------------------------------------------------------------------
//_ch: 0:Rx  1:RX
//update_item: 0:update ui flag    1: update ui flag when getting process upgated  2:update the final process value
void update_dfu_ui_flag(uint8_t _ch, uint8_t update_item, uint8_t update_value)
{
	if(_ch==0)
	{	
		if(update_item==0)
		{
			aw5808_tx_data.dfu_process_old_valule=0xff;
			dfu_update_tx_ui=1;
		}			
		else if(update_item==1)
		{
			aw5808_tx_data.dfu_process_valule=update_value;
			if(aw5808_tx_data.dfu_process_valule%5==0 && aw5808_tx_data.dfu_process_valule!=0)
			dfu_update_tx_ui=1;	
		}
		else if(update_item==2)
		{
			aw5808_tx_data.dfu_process_valule=100;
			dfu_update_tx_ui=1;			
		}
	}
	else
	{
		if(update_item==0)
		{
			aw5808_rx_data.dfu_process_old_valule=0xff;
			dfu_update_rx_ui=1;
		}
		else if(update_item==1)
		{
			aw5808_rx_data.dfu_process_valule=update_value;
			if(aw5808_rx_data.dfu_process_valule%5==0 && aw5808_rx_data.dfu_process_valule!=0)
			dfu_update_rx_ui=1;		
		}
		else if(update_item==2)
		{
			aw5808_rx_data.dfu_process_valule=100;
			dfu_update_rx_ui=1;			
		}		
	}
}
//-----------------------------------------------------------------------------------------------------------

void AW5808_DFU_process(void)
{
	uint8_t read_data=0,i=0;
	uint8_t read_data_array[10]={0};
	
	if(aw5808_dfu_enable)
	{
		if(check_mcu_power_enable==0)
		return;
		
		//exec_power_off_enable=0;
		m_module.power_down = NO;
		enable_standby_mode=0;
		
		if(dfu_scan_time_count_2>check_aw5808_dfu_scan_time)
		{
			dfu_scan_time_count_2=0;
		//set up callback function
		if(aw5805_dfu_step==0xff)
		{
			if(start_to_wait_for_hid_response==0)
			{
			//Tx
			if(aw5808_dfu_channel==0)
			{
				s_pfnDFUHandler_R = (AW5808_DFU_FUNC_P)GW_AW5808_TX_Read_XX_data;
				s_pfnDFUHandler_W = (AW5808_DFU_FUNC_P)GW_AW5808_TX_Write_XX_data;
				update_aw5808_TX_item();
				dfu_update_tx_ui=0;		
			}
			//Rx
			else
			{
				s_pfnDFUHandler_R = (AW5808_DFU_FUNC_P)GW_AW5808_RX_Read_XX_data;
				s_pfnDFUHandler_W = (AW5808_DFU_FUNC_P)GW_AW5808_RX_Write_XX_data;
				update_aw5808_RX_item();
				dfu_update_rx_ui=0;
			}
			aw5805_dfu_step=0;			
			setup_flash_mode=0;
			aw5808_simulator_query();			
			//check hid's response time					
			start_to_wait_for_hid_response=1;
			aw5808_auto_test_count=0;
			
			//set up working time
			check_aw5808_dfu_scan_time=DFU_SCAN_TIME;	
			g_au32AW5808INTCount[0]=0;			
      return;			
		}
		}		
		
		if(start_to_wait_for_hid_response && aw5808_auto_test_count>5000)
		{
			aw5805_dfu_step=8;
			dfu_error=ERR_QUERY_TIMEOUT;
			update_dfu_ui_flag(aw5808_dfu_channel,0,0);			
		}
		
		//get 16 byte data from usb
	  if(aw5805_dfu_step==0)
		{
			//set up working time
			if(check_aw5808_dfu_scan_time!=DFU_SCAN_TIME)
			{
				check_aw5808_dfu_scan_time=DFU_SCAN_TIME;				
				g_au32AW5808INTCount[0]=0;				
			}

			//check if getting dat from usb
			if(get_aw5808_hid_event_response==2)
			{
				get_aw5808_hid_event_response=0;
				start_to_wait_for_hid_response=0;

				//check if getting company name at the first 16-bytes packet
				if( *(aw5808_temp_data.file_idx)==16)	
				{
					 if(memcmp("everestek co ltd",aw5808_temp_data.data,16)==0)						 
					 {
							aw5805_dfu_step=0;
							aw5808_simulator_query();						 
							//check hid's response time					
							start_to_wait_for_hid_response=1;
							aw5808_auto_test_count=0;						 
					 }
					 //error leave process
					 else
					 {
							aw5805_dfu_step=8;
						 	dfu_error=ERR_LOST_COMPANY_NAME;
							update_dfu_ui_flag(aw5808_dfu_channel,0,0);
					 }
				}
				else
				{
					aw5805_dfu_current_packet_number=*(aw5808_temp_data.file_idx)/16-2;	
					
					if(setup_flash_mode==0)
					aw5805_dfu_step=1;	
					else
					{
						//copy data from usb
						memset(w_data,0x00,sizeof(w_data));
						w_data[0]=(uint8_t)((aw5805_dfu_current_packet_number & 0xff00)>>8);	//packet no. high byte
						w_data[1]=(uint8_t)(aw5805_dfu_current_packet_number & 0x00ff);   		//packet no. low byte
						memcpy(&w_data[2],&(aw5808_temp_data.data[0]),8);	//w_data[2] ~ w_data[9]  //data[0]~data[7]
						s_pfnDFUHandler_W(0x6a, w_data, 10);		//write	from w_data[0] ~ w_data[9]						
						aw5805_dfu_step=4;
					}						
				}
			}
		}
		//wirte reg 0x6c		
		else if(aw5805_dfu_step==1)
		{
			memcpy(w_data,"\x11\x22\x33\x44\x55\x66\x77\x88",8);			
			s_pfnDFUHandler_W(0x6c, w_data, 8);	
			aw5805_dfu_step=2;
		}
		//write reg 0x7e
		else if(aw5805_dfu_step==2)
		{
			w_data[0]=0xa5;
			s_pfnDFUHandler_W(0x7e, w_data, 1);	
			aw5805_dfu_step=3;		
			setup_flash_mode=1;			
		}
		//write packet part1 data[0]~data[7]
		else if(aw5805_dfu_step==3)
		{
		//copy data from usb
			memset(w_data,0x00,sizeof(w_data));
			w_data[0]=(uint8_t)((aw5805_dfu_current_packet_number & 0xff00)>>8);	//packet no. high byte
			w_data[1]=(uint8_t)(aw5805_dfu_current_packet_number & 0x00ff);   		//packet no. low byte
			memcpy(&w_data[2],&(aw5808_temp_data.data[0]),8);	//w_data[2] ~ w_data[9]  //data[0]~data[7]
			s_pfnDFUHandler_W(0x6a, w_data, 10);		//write	from w_data[0] ~ w_data[9]
			aw5805_dfu_step=4;						
		}
		//write packet part2 data[8]~data[15]		
		else if(aw5805_dfu_step==4)
		{
				memcpy(&w_data[10],&(aw5808_temp_data.data[8]),8);		 //w_data[10] ~ w_data[17]  	//data[8]~data[15]
			w_data[18] = AW5808_DFU_Calculate_CheckSum(w_data);   //calculate checksum from w_data[0] ~ w_data[17]
			w_data[19] = 0x01;
				s_pfnDFUHandler_W(0x74, &w_data[10], 10);		//write	from w_data[10] ~ w_data[19]
			aw5805_dfu_step=5;
		}
		//read reg ox7f to check if write is ok
		else if(aw5805_dfu_step==5)
		{
			if(s_pfnDFUHandler_R(0x7f,&read_data,1)==0)				
			{
				//packet write ok, go on next packet
				if(read_data==0x01)
				{
					if(dfu_error!=0)
					{
						dfu_error=0;
						update_dfu_ui_flag(aw5808_dfu_channel,0,0);
					}

					//delay 200ms for flash erase
					if(*(aw5808_temp_data.file_idx)==96)						
					{					
						check_aw5808_dfu_scan_time=DFU_FLASH_WRITE_TIME;						
						g_au32AW5808INTCount[0]=0;						
					}
					
					
					//calculate the dfu process
					update_dfu_ui_flag(aw5808_dfu_channel,1,aw5805_dfu_current_packet_number/36);
					
					if(file_index < file_size)
					{
						aw5805_dfu_step=0;
						aw5808_simulator_query();
						//check hid's response time
						start_to_wait_for_hid_response=1;
						aw5808_auto_test_count=0;
					}
					else				
					{
						aw5805_dfu_step=6;
						aw5808_simulator_finish();
					}
				}
				//ET51K is busy, should read reg 0x7f again
				else if(read_data==0x02)
				{
					aw5805_dfu_step=5;
					dfu_error=ERR_WAIF_FOR_POLL;
					update_dfu_ui_flag(aw5808_dfu_channel,0,0);				
				}
				//checksum is error,send the same packet again
				else if(read_data==0x04)
				{
					setback_file_index();
					aw5805_dfu_step=0;
					aw5808_simulator_query();
					//check hid's response time					
					start_to_wait_for_hid_response=1;
					aw5808_auto_test_count=0;
				}
				//packet sequence is error,leave the process
				else if(read_data==0x08)
				{
					aw5805_dfu_step=8;
					dfu_error=ERR_PKT_SEQUENCE;
					update_dfu_ui_flag(aw5808_dfu_channel,0,0);					
				}				
			}
		}
		//write reg 0x7d
		else if(aw5805_dfu_step==6)
		{
			w_data[0]=0x81;
			s_pfnDFUHandler_W(0x7d, w_data, 1);
				
			check_aw5808_dfu_scan_time=5000;
			g_au32AW5808INTCount[0]=0;					
			aw5805_dfu_step=7;
			retry_time=0;
		}
		//read reg 0x7d
		else if(aw5805_dfu_step==7)
		{
			if(s_pfnDFUHandler_R(0x7d,&read_data,1)==0)
			{				
				if(read_data==0)
				{
					aw5805_dfu_step=8;
					retry_time=0;
					update_dfu_ui_flag(aw5808_dfu_channel,2,100);
					
					check_aw5808_dfu_scan_time=1000;					
					g_au32AW5808INTCount[0]=0;						
					}
					else
					{
					retry_time++;
					if(retry_time>5)
				{
						aw5805_dfu_step=8;						
						dfu_error=ERR_READ_0X7F;
					  update_dfu_ui_flag(aw5808_dfu_channel,0,0);		
					}
				}				
			}
		}	
		//leave dfu process
		else if(aw5805_dfu_step==8)
		{
			aw5808_dfu_enable=0;
			aw5805_dfu_step=0xff;//idle
			s_pfnDFUHandler_R = NULL;
			s_pfnDFUHandler_W = NULL; 
			
			aw5808_rx_data.online=0xff;
			aw5808_tx_data.online=0xff;
			start_to_wait_for_hid_response=0;
			aw5808_auto_test_count=0;
		
			enable_standby_mode=1;	
				sys_evt.fw_upgrade_aw5808(NO);
		}
		}
	}
	
	//update ui
	if(dfu_update_tx_ui)
	{
		dfu_update_tx_ui=0;
		update_aw5808_TX_item();
	}
	if(dfu_update_rx_ui)
	{
		dfu_update_rx_ui=0;
		update_aw5808_RX_item();
	}	
}

/*---------------------------------------------------------------------------------------------------------*/
/*  SYS_Touch_Init                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Touch_Init(void)
{
    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPC_MODULE);
    /* Enable peripheral clock */	
    CLK_EnableModuleClock(USCI0_MODULE);
	
    /* Update core clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set I2C0 multi-function pins */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk)) |
                    (SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL);
    /* Set UI2C0 multi-function pins */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk)) |
                    (SYS_GPD_MFPL_PD0MFP_USCI0_CLK | SYS_GPD_MFPL_PD1MFP_USCI0_DAT0);	
    /* I2C pins enable schmitt trigger */
    PC->SMTEN |= GPIO_SMTEN_SMTEN0_Msk | GPIO_SMTEN_SMTEN1_Msk;		
    PD->SMTEN |= GPIO_SMTEN_SMTEN0_Msk | GPIO_SMTEN_SMTEN1_Msk;		
}
	
/*---------------------------------------------------------------------------------------------------------*/
/*  GW_Touch_Init                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void GW_Touch_Init(void)
{
		//enable gpio interrupt for cch 
    GPIO_EnableInt(PC, 3, GPIO_INT_FALLING);		
    NVIC_EnableIRQ(GPC_IRQn);
	
    /* Init I2C0 */
    I2C0_Init();
}
//-----------------------------------------------------------------------------------------
void GW_AW5808_Tx_On_Off(uint8_t _value)
{
	//on
	if(_value)
	{
		//printf("\x1b[35;92H--- on---\n");
		NVIC_EnableIRQ(I2C0_IRQn);
    I2C_EnableInt(I2C0);
    I2C_Open(I2C0, 100000);
	}
	//off
	else
	{
		//printf("\x1b[35;92H---off---\n");	
    NVIC_DisableIRQ(I2C0_IRQn);
    I2C_DisableInt(I2C0);
    I2C_Close(I2C0);
	}
}

//----------------------------------------------------------------------------
int8_t GW_Touch_Read_Button_Status(uint8_t *get_button)	
{
	int8_t r_value=-1;
	uint8_t r_data=0;

	if(I2C0_ReadMultiBytes(TOUCH_ADDR,REG_ADDR_BUTTON_STAT,&r_data,1)==0)
	{
		*get_button=r_data;
		r_value=0;
	}	
	return r_value;	
}
//----------------------------------------------------------------------------------
void GW_Touch_Configuration_Data_Update(void)
{
	I2C0_WriteMultiByteOneReg(TOUCH_ADDR, 0, CY8CMBR3108_LQXI_configuration, 128);	
}
//----------------------------------------------------------------------------------
void GW_Touch_Calc_CRC_Save_NVM(void)
{
	uint8_t w_data=OP_CODE_CALC_CRC_SAVE_NVM;
	I2C0_WriteMultiByteOneReg(TOUCH_ADDR, REG_ADDR_CTRL_CMD, &w_data, 1);		
}
//----------------------------------------------------------------------------------
void GW_Touch_Reset_Device(void)
{
	uint8_t w_data=OP_CODE_RESET_DEVICE;
	I2C0_WriteMultiByteOneReg(TOUCH_ADDR, REG_ADDR_CTRL_CMD, &w_data, 1);		
}
//----------------------------------------------------------------------------------
int8_t GW_Touch_Check_Data(void)
{
	int8_t r_value=-1;
	uint8_t temp_data[128]={0};
								
	if(I2C0_ReadMultiBytes(TOUCH_ADDR,0,temp_data,128)==0)
	{
		if((temp_data[126] == CY8CMBR3108_LQXI_configuration[126]) && (temp_data[127] == CY8CMBR3108_LQXI_configuration[127]))
		r_value=0;
	}								
	return r_value;
}
//----------------------------------------------------------------------------------
uint8_t touch_update_step=1;
uint8_t enable_touch_update_process=1;

//----------------------------------------------------------------------------------
void Delay_Test_Function(void)
{
	uint16_t i=0;
  for (i=0;i<1000;i++);	
}


//return get_version:
//0: wireless MIC version
//1: wire MIC version
uint8_t get_hardware_version(void)
{
	uint8_t get_version;
	get_version=PA10;
	return get_version;
}


//--------------battery charge -------------------------------------------------------------
int8_t GW_Battery_Read_XX_data(uint8_t reg_address, uint8_t* _get_data, uint8_t data_len)
{
	int8_t r_value=-1;
	uint8_t get_data[20]={0},i=0;	
		
	if(I2C0_ReadMultiBytes(BAT_ADDR, reg_address, get_data, data_len)==0)
	{
		for(i=0;i<data_len;i++)
		*(_get_data+i)=get_data[i];
		
		r_value=0;
	}	

	return r_value;	
}
//--------------------------------------------------------------------------------------------
int8_t GW_Battery_Write_XX_data(int8_t reg_address, uint8_t* _write_data, uint8_t data_len)
{
	I2C0_WriteMultiByteOneReg(BAT_ADDR,reg_address,_write_data,data_len);	
	return 0;	
}
//--------------------------------------------------------------------------------------------
int8_t GW_Battery_Get_Charge_Status(uint8_t* _get_data)
{
	int8_t r_value=-1;
	uint8_t get_data=0;	
		
	if(I2C0_ReadMultiBytes(BAT_ADDR, 0x0f, _get_data, 1)==0)		
		{
			r_value=0;
		}

	return r_value;	
}
//---------------------------------------------------------------------------------------------
void GW_Battery_Charger_Init_Command(void)
{
	uint8_t write_data=0;
	write_data=0xC0;
	GW_Battery_Write_XX_data(0x01,&write_data,1);
	write_data=0x12;
	GW_Battery_Write_XX_data(0x03,&write_data,1);
	write_data=0x12;
	GW_Battery_Write_XX_data(0x08,&write_data,1);
	write_data=0x32;
	GW_Battery_Write_XX_data(0x0A,&write_data,1);
	//write_data=0x04;
	write_data=0xBC;
	GW_Battery_Write_XX_data(0x0C,&write_data,1);
	//write_data=0x22;
	// Recharge if the voltage is lower than 4.0V
	write_data=0xA0;
	GW_Battery_Write_XX_data(0x07,&write_data,1);
}
//----------------------------------------------------------------------------------------------
#define GW_BAT_FIFO_SIZE 10
int32_t gw_battery_fifo[GW_BAT_FIFO_SIZE]={0};
uint8_t gw_battery_fifo_idx=0;
uint8_t gw_battery_enable_calculate_avg=0;
int32_t current_battery_voltage=0;

int32_t gw_temperature_fifo[GW_BAT_FIFO_SIZE]={0};
uint8_t gw_temperature_fifo_idx=0;
uint8_t gw_temperature_enable_calculate_avg=0;
int32_t current_temperature_voltage=0;

GW_Battery gw_battery=
{
	.status=BAT_NOT_CONNECT,
	.battery_value=0,
	.voltage_level=BAT_LOW_VOLTAGE,
	.temperature_value=0
};
//----------------------------------------------------------------------------------------------
int32_t GW_Battery_Voltage_Average(int32_t _data_in)
{
	int32_t r_value=0,avg_value=0;
	uint8_t i=0;
	
	gw_battery_fifo[gw_battery_fifo_idx++]=_data_in;
	
	if(gw_battery_fifo_idx>(GW_BAT_FIFO_SIZE-1))
	{
		gw_battery_fifo_idx=0;
		gw_battery_enable_calculate_avg=1;
	}
	
	
	if(gw_battery_enable_calculate_avg)
	{
		for(i=0;i<GW_BAT_FIFO_SIZE;i++)
		{
			avg_value+=gw_battery_fifo[i];
		}
		r_value=avg_value/GW_BAT_FIFO_SIZE;		
	}
	else
	{
		for(i=0;i<gw_battery_fifo_idx;i++)
		{
			avg_value+=gw_battery_fifo[i];
		}
		r_value=avg_value/gw_battery_fifo_idx;			
	}
	
	return r_value;
}
//----------------------------------------------------------------------------------------------
int32_t GW_Temperature_Value_Average(int32_t _data_in)
{
	int32_t r_value=0,avg_value=0;
	uint8_t i=0;
	
	gw_temperature_fifo[gw_temperature_fifo_idx++]=_data_in;
	
	if(gw_temperature_fifo_idx>(GW_BAT_FIFO_SIZE-1))
	{
		gw_temperature_fifo_idx=0;
		gw_temperature_enable_calculate_avg=1;
	}

	if(gw_temperature_enable_calculate_avg)
	{
		for(i=0;i<GW_BAT_FIFO_SIZE;i++)
		{
			avg_value+=gw_temperature_fifo[i];
		}
		r_value=avg_value/GW_BAT_FIFO_SIZE;		
	}
	else
	{
		for(i=0;i<gw_temperature_fifo_idx;i++)
		{
			avg_value+=gw_temperature_fifo[i];
		}
		r_value=avg_value/gw_temperature_fifo_idx;	
	
	}
	
	return r_value;
}
//----------------------------------------------------------------------------------------------
uint8_t battery_init_done=0;
uint8_t pre_battery_status=0xff;
uint8_t bat_read_enable=0;
uint8_t temperature_read_enable=0;
uint8_t power_level_check_count=0, power_done_check_count=0;
void GW_Battery_Process(void)
{
#define LED_IDLE_MODE				4
#define LED_BAT_LOW_POWER		13
#define LED_BAT_CHARGING		14
	
		uint8_t  battery_status = 0, temp_status = 0;
		int32_t  get_value = 0;
		int8_t   r_code;

		if(GPO_PD02_MCU_PWREN == NO)
				return;

		if(m_module.battery_init == NO) {
				m_module.battery_init = YES;
				m_module.usb_connected = !GPI_PB02_VBUS_DET;
		}

		if(m_module.usb_connected != GPI_PB02_VBUS_DET) {
				sys_evt.usb_connect_cb(GPI_PB02_VBUS_DET? YES: NO);
		}

		if(bat_read_enable) {
				if(adc_count < 100) {
						if(g_u32EadcInt0Flag == 1) {
								gw_battery.battery_value = GW_Battery_Voltage_Average(EADC_GET_CONV_DATA(EADC, 0));
								bat_read_enable = 0;
								g_u32EadcInt0Flag = 0;
						}
				}
				else {
						bat_read_enable = 0;
				}
		}	

		if(temperature_read_enable) {
				if(adc_temp_count < 100) {
						if(g_u32EadcInt1Flag == 1) {
								gw_battery.temperature_value = GW_Temperature_Value_Average(EADC_GET_CONV_DATA(EADC, 1));
								temperature_read_enable = 0;
								g_u32EadcInt1Flag = 0;
						}
				}
				else {
						temperature_read_enable = 0;
				}
		}

		if(battery_init_done == 0) {
				battery_init_done = 1;
				GW_Battery_Charger_Init_Command();
				BATTERY_TMRINT_count = 0;
		}
		else {
				if(BATTERY_TMRINT_status_count > 1000) {
						r_code=GW_Battery_Get_Charge_Status(&battery_status);
						if(r_code == 0) {
								temp_status = battery_status&0x0F;
								if(temp_status != pre_battery_status) {
										//printf("\x1b[3;108H                    \n");
										pre_battery_status = temp_status;
										//in discharge mode
										if(temp_status == 0) {
												gw_battery.status = BAT_DISCHARGE; //1
										}
										//in charge mode
										else if (temp_status >= 2 && temp_status <= 6) {
												gw_battery.status = BAT_CHARGING; //2
										}
										//charge done
										else if(temp_status == 7) {
												gw_battery.status = BAT_CHARG_DONE; //3
										}
										//charge fail (not connect the battery)
										else if(temp_status == 8) {
												gw_battery.status = BAT_NOT_CONNECT; //charge fail	  //0
										}
								}

								if(gw_battery.status == BAT_CHARG_DONE) {
										power_done_check_count = 0;
										sys_evt.battery_done_cb();
								}
								//usb connect
								else if(gw_battery.status == BAT_CHARGING) {
										sys_evt.battery_charging_cb();
								}
								//usb not connect
								else if(gw_battery.status == BAT_DISCHARGE) {
										//usb connect
										sys_evt.battery_discharge_cb();
								}
						}
						BATTERY_TMRINT_status_count = 0;
				}

				if(BATTERY_TMRINT_count > 3000) {
						GW_Battery_Charger_Voltage_check();
						bat_read_enable = 1;
//PrintMsg__("Battery: %4d last %d %3d\r\n", gw_battery.battery_value, gw_battery.voltage_level, power_level_check_count);
						m_module.battery_level = gw_battery.battery_value;
						if(gw_battery.battery_value < BAT_LOW_THRESHOLD_ADC) {
								if(gw_battery.voltage_level != BAT_LOW_VOLTAGE)
										power_level_check_count = 0;

								if(gw_battery.battery_value <= BAT_ULTRALOW_THRESHOLD_ADC && m_module.power_check == YES)
										sys_evt.battery_ultralow_cb();
								else
										sys_evt.battery_low_cb();
								//enable_low_power_check_function();
						}
						else if(gw_battery.battery_value >= BAT_LOW_THRESHOLD_ADC && gw_battery.battery_value < BAT_HIGH_THRESHOLD_ADC) {
								gw_battery.voltage_level = BAT_MEDIUM_VOLTAGE;
								sys_evt.battery_medium_cb();
						}
						else if(gw_battery.battery_value >= BAT_HIGH_THRESHOLD_ADC) {
								gw_battery.voltage_level = BAT_HIGH_VOLTAGE;
								sys_evt.battery_high_cb();
						}
						BATTERY_TMRINT_count = 0;
				}
		}

	//temperature
#if 0
	if(TEMPERATURE_TMRINT_count>3000)
	{
		GW_Battery_Temperature_check();
		temperature_read_enable=1;
		
		TEMPERATURE_TMRINT_count=0;
	}
#endif 	
}

//======================================================================================
#define STANDBY_DETECT_TIME_20_SEC (20)*(1000)
#define STANDBY_DETECT_TIME_MIN_UNIT (60)*(1000)
#define STANDBY_DETECT_TIME_MIN_NO_CHARGE	(5)*(STANDBY_DETECT_TIME_MIN_UNIT)
#define STANDBY_DETECT_TIME_MIN_CHARGE	(30)*(STANDBY_DETECT_TIME_MIN_UNIT)

extern volatile uint8_t wireless_who;

volatile uint8_t standby_mode_start_count=0;

uint32_t standby_mode_detect_time=STANDBY_DETECT_TIME_MIN_NO_CHARGE;
uint8_t pre_battery_mode=0xff;
uint8_t dg_pos=1;

uint8_t enable_standby_mode=1;
uint8_t pre_wireless_mode=0xff;

//-------------------------------------------
volatile uint8_t check_autoconnection_start=0;
uint8_t exec_unpair_action=0, re_get_id=0;
void GW_Check_AutoConnection(void)
{
		if(factory_mode == YES)
				return;

		if(check_auto_connection_wait_count < 10000)
				return;

		if((aw5808_rx_data.status&0x01) == 0) {
				if(re_get_id == 0) {
						re_get_id = 1;
						GW_AW5808_RX_Get_ID(&aw5808_rx_data);
				}

				if(memcmp(aw5808_rx_data.id, "\x11\x22\x33\x44", 4) != 0 && memcmp(aw5808_rx_data.id, "\x00\x00\x00\x00", 4) !=0 && memcmp(aw5808_rx_data.id, "\xff\xff\xff\xff", 4) != 0) {
						if(check_autoconnection_start == 0) {
								check_autoconnection_start = 1;
								unpair_wait_count = 0;
								//printf("\x1b[%d;90H--- count unpair---\n",debug_row++);
						}
				}
		}
		else {
				re_get_id = 0;
				check_autoconnection_start = 0;
		}

		if(check_autoconnection_start) {
				if(wireless_who == WHO_IS_NO) {
						if(unpair_wait_count > (5*STANDBY_DETECT_TIME_MIN_UNIT)) {
								if(exec_unpair_action == 0) {
										exec_unpair_action = 1;
										GW_AW5808_RX_Unpair();
										//printf("\x1b[%d;90H--- do unpair---\n",debug_row++);
										check_autoconnection_start = 0;
										re_get_id = 0;
								}
						}
				}
				//connected
				else if(wireless_who == WHO_IS_SPK || wireless_who == WHO_IS_G2) {
						check_autoconnection_start = 0;
				}
		}
}


//--------------------------------------------
uint8_t charging_bay_status=0xff;
uint8_t old_charging_bay_status=0xff;
void Check_Charging_Bay_Status(void)
{
	if(gw_battery.status==BAT_NOT_CONNECT)
	return;
	
	charging_bay_status=PB2;
	if(old_charging_bay_status!=charging_bay_status)
	{
		//printf("\x1b[3;100HPB2-> %d %d \n",old_charging_bay_status,charging_bay_status);	
		
		if(old_charging_bay_status==0 && charging_bay_status==1)
		{
			GW_reboot_for_power_on();
		}
		
		old_charging_bay_status=charging_bay_status;
	}
}

//----------------------------------------------------
extern volatile uint32_t PWOER_OFF_TMRINT_count;
void GW_Power_Down_Mode(void)
{
		GPO_PA11_DSP_ONOFF = HIGH;
		GPO_PD02_MCU_PWREN = LOW;
		PWOER_OFF_TMRINT_count = 0;
		while((PWOER_OFF_TMRINT_count < 1000));
		PowerDown();
}

//-----------------------------------------------------
uint8_t i2c_need_re_init = NO;

void check_i2c_need_re_init(void) {

		if(i2c_need_re_init == NO)
		return;

		GW_AW5808_Tx_On_Off(0);
		GW_AW5808_Tx_On_Off(1);
		i2c_need_re_init=NO;
		
}