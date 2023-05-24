#include <stdio.h>
#include "NuMicro.h"
#include "GW_aw5808.h"
#include "usci_i2c.h"
#include <string.h>
#include "GW_main.h"
#include "GW_wm8804_cs8442.h"

#define IGNORE_PRINTF
#ifdef IGNORE_PRINTF
	#define printf(fmt, ...) (0)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

extern int8_t I2C0_ReadMultiBytes(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t rdata[], uint8_t u8rLen);
extern int8_t I2C0_WriteMultiByteOneReg(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t wdata[], uint32_t u32rLen);

//=================================================================================================
//GW_WM8804_TX
//
//=================================================================================================

int8_t GW_WM8804_TX_Write_data(uint8_t reg_address, uint8_t* _write_data, uint8_t data_len)
{
    I2C0_WriteMultiByteOneReg(WM8804_ADDR,reg_address,_write_data,data_len);
    return 0;
}
//-------------------------------------------------------------------------------
int8_t GW_WM8804_TX_Read_data(uint8_t reg_address, uint8_t* _get_data, uint8_t data_len)
{
    int8_t r_value=-1;
    uint8_t get_data[20]= {0},i=0;
    if(I2C0_ReadMultiBytes(WM8804_ADDR, reg_address, get_data,data_len)==0)			
    {
        for(i=0; i<data_len; i++)
            *(_get_data+i)=get_data[i];

        r_value=0;
    }

    return r_value;
}
//-------------------------------------------------------------------------------
int8_t GW_WM8804_TX_Read_ID1(uint8_t* get_id1)
{
    int8_t r_value=-1;
    if(GW_WM8804_TX_Read_data(0x00,get_id1,1)==0)
    {
        r_value=0;
        printf("\x1b[31;60HID1=%02x\n",*get_id1);
    }

    return r_value;
}
//-------------------------------------------------------------------------------
int8_t GW_WM8804_TX_Read_ID2(uint8_t* get_id2)
{
    int8_t r_value=-1;
    if(GW_WM8804_TX_Read_data(0x01,get_id2,1)==0)
    {
        r_value=0;
        printf("\x1b[32;60HID2=%02x\n",*get_id2);
    }

    return r_value;
}
//-------------------------------------------------------------------------------
int8_t GW_WM8804_TX_Read_version(uint8_t* get_version)
{
    int8_t r_value=-1;
		printf("\x1b[29;60HWM8804Tx:\n");	
    if(GW_WM8804_TX_Read_data(0x02,get_version,1)==0)
    {
        r_value=0;
				printf("\x1b[29;69H  ok\n");				
        printf("\x1b[30;60HVersion=%02x\n",*get_version);
    }
		else
		{
				printf("\x1b[29;69Hfail\n");	
    }

    return r_value;
}

//-------------------------------------------------------------------------------
int8_t GW_WM8804_TX_Read_Reg(uint8_t reg, uint8_t loc)
{
    int8_t r_value=-1;
    uint8_t data=0;
    if(GW_WM8804_TX_Read_data(reg,&data,1)==0)
    {
        r_value=0;
        //printf("\x1b[%d;80H[%02x]=%02x\n",loc,reg,data);
    }

    return r_value;
}

void GW_WM8804_TX_Write_InitTable(void)
{
	uint8_t w_data=0x00;	
	GW_WM8804_TX_Write_data(0x00,&w_data,1);

  w_data=0x08;	
	GW_WM8804_TX_Write_data(0x08,&w_data,1);

  w_data=0x0b;	
	GW_WM8804_TX_Write_data(0x1e,&w_data,1);	

  w_data=0x0a;	
	GW_WM8804_TX_Write_data(0x1c,&w_data,1);	

}

//=================================================================================================
//GW_WM8804_RX
//
//=================================================================================================

int8_t GW_WM8804_TXRX_Write_data(uint8_t reg_address, uint8_t* _write_data, uint8_t data_len)
{
    I2C0_WriteMultiByteOneReg(WM8804_ADDR_2,reg_address,_write_data,data_len);
    return 0;
}
//-------------------------------------------------------------------------------
int8_t GW_WM8804_TXRX_Read_data(uint8_t reg_address, uint8_t* _get_data, uint8_t data_len)
{
    int8_t r_value=-1;
    uint8_t get_data[20]= {0},i=0;
    if(I2C0_ReadMultiBytes(WM8804_ADDR_2, reg_address, get_data,data_len)==0)
    {
        for(i=0; i<data_len; i++)
            *(_get_data+i)=get_data[i];

        r_value=0;
    }

    return r_value;
}
//-------------------------------------------------------------------------------
int8_t GW_WM8804_TXRX_Read_ID1(uint8_t* get_id1)
{
    int8_t r_value=-1;
    if(GW_WM8804_TXRX_Read_data(0x00,get_id1,1)==0)
    {
        r_value=0;

        printf("\x1b[35;60HID1=%02x\n",*get_id1);
    }

    return r_value;
}
//-------------------------------------------------------------------------------
int8_t GW_WM8804_TXRX_Read_ID2(uint8_t* get_id2)
{
    int8_t r_value=-1;
    if(GW_WM8804_TXRX_Read_data(0x01,get_id2,1)==0)
    {
        r_value=0;
        printf("\x1b[36;60HID2=%02x\n",*get_id2);
    }

    return r_value;
}
//-------------------------------------------------------------------------------
int8_t GW_WM8804_TXRX_Read_version(uint8_t* get_version)
{
    int8_t r_value=-1;
		printf("\x1b[33;60HWM8804TxRx:\n");		
    if(GW_WM8804_TXRX_Read_data(0x02,get_version,1)==0)
    {
        r_value=0;
				printf("\x1b[33;71H  ok\n");				
        printf("\x1b[34;60HVersion=%02x\n",*get_version);
    }
		else
		{
				printf("\x1b[33;71Hfail\n");			
    }

    return r_value;
}
//-------------------------------------------------------------------------------
int8_t GW_WM8804_TXRX_Read_Reg(uint8_t reg, uint8_t loc)
{
    int8_t r_value=-1;
    uint8_t data=0;
    if(GW_WM8804_TXRX_Read_data(reg,&data,1)==0)
    {
        r_value=0;
        //printf("\x1b[%d;100H[%02x]=%02x\n",loc,reg,data);
    }

    return r_value;
}
//-------------------------------------------------------------------------------
void GW_WM8804_TXRX_Write_InitTable(void)
{
	uint8_t w_data=0x00;	
	GW_WM8804_TXRX_Write_data(0x00,&w_data,1);

  w_data=0x08;	
	GW_WM8804_TXRX_Write_data(0x08,&w_data,1);

  w_data=0x00;	
	GW_WM8804_TXRX_Write_data(0x1e,&w_data,1);	

  w_data=0x0a;	
	GW_WM8804_TXRX_Write_data(0x1b,&w_data,1);		
	
  w_data=0x4a;	
	GW_WM8804_TXRX_Write_data(0x1c,&w_data,1);		
}
//=================================================================================================
//GW_CS8422
//
//=================================================================================================

//-------------------------------------------------------------------------------
int8_t GW_CS8422_Write_data(uint8_t reg_address, uint8_t* _write_data, uint8_t data_len)
{
    I2C0_WriteMultiByteOneReg(CS8422_ADDR,reg_address,_write_data,data_len);
    return 0;
}
//-------------------------------------------------------------------------------
int8_t GW_CS8422_Read_data(uint8_t reg_address, uint8_t* _get_data, uint8_t data_len)
{
    int8_t r_value=-1;
    uint8_t get_data[20]= {0},i=0;
    if(I2C0_ReadMultiBytes(CS8422_ADDR, reg_address, get_data,data_len)==0)
    {
        for(i=0; i<data_len; i++)
            *(_get_data+i)=get_data[i];

        r_value=0;
    }

    return r_value;
}
//-------------------------------------------------------------------------------
int8_t GW_CS8422_Read_version(uint8_t* get_version)
{
    int8_t r_value=-1;
		printf("\x1b[37;60HCS8422:\n");	
    if(GW_CS8422_Read_data(0x01,get_version,1)==0)
    {
        r_value=0;
				printf("\x1b[37;67H  ok\n");	
        printf("\x1b[38;60HVersion=%01x\n",*get_version);
    }
		else
		{
				printf("\x1b[37;67Hfail\n");			
    }

    return r_value;
}
//-------------------------------------------------------------------------------
void GW_CS8422_Write_InitTable(void)
{
	uint8_t w_data=0x50;	
	GW_CS8422_Write_data(0x02,&w_data,1);	
	
	w_data=0x90;
	GW_CS8422_Write_data(0x03,&w_data,1);

	w_data=0x40;
	GW_CS8422_Write_data(0x08,&w_data,1);	
	
	w_data=0x40;
	GW_CS8422_Write_data(0x09,&w_data,1);		

	w_data=0x16;
	GW_CS8422_Write_data(0x0a,&w_data,1);	

	w_data=0x04;
	GW_CS8422_Write_data(0x0c,&w_data,1);
	
	w_data=0x04;
	GW_CS8422_Write_data(0x0d,&w_data,1);	
}
//-------------------------------------------------------------------------------
void GW_CS8422_Read_InitTable(void)
{
    uint8_t data=0,i=0;
	  uint8_t r_reg[]={0x02,0x03,0x08,0x09,0x0a,0x0c,0x0d};
		
		printf("\x1b[29;120H --CS8422--\n");
		
		for(i=0;i<sizeof(r_reg);i++)
		{
			data=0;
			if(GW_CS8422_Read_data(r_reg[i],&data,1)==0)
			{
        printf("\x1b[%d;120H[%02x]=%01x\n",30+i,r_reg[i],data);
			}		
		}
}
//-------------------------------------------------------------------------------
uint8_t wm8402_cs8422_init=0;
void GW_WM8402_CS8422_Init(void)
{
	uint8_t get_id=0,get_version=0;
	if(wm8402_cs8422_init==0)
	{
		wm8402_cs8422_init=1;
		GPO_PF03_WWL_SW=1;
		GPO_PB14_USB2SW=1;
		
		if(GW_WM8804_TX_Read_version(&get_version)==0)
		{
			GW_WM8804_TX_Read_ID1(&get_id);	
			GW_WM8804_TX_Read_ID2(&get_id);
		GW_WM8804_TX_Write_InitTable();
		}


		if(GW_WM8804_TXRX_Read_version(&get_version)==0)
		{
			GW_WM8804_TXRX_Read_ID1(&get_id);	
			GW_WM8804_TXRX_Read_ID2(&get_id);
		GW_WM8804_TXRX_Write_InitTable();
		}
		
		if(GW_CS8422_Read_version(&get_version)==0)
		GW_CS8422_Write_InitTable();	
	}
}