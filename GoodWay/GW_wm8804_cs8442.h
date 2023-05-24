#ifndef __GW_wm8804_cs8442_H__
#define __GW_wm8804_cs8442_H__
#include <stdio.h>
#include "NuMicro.h"
#include "GW_main.h"

#define WM8804_ADDR 	0x3A		//0x74
#define WM8804_ADDR_2 0x3B		//0x76
#define CS8422_ADDR 	0x10		//0x20

extern uint8_t wm8402_cs8422_init;

extern int8_t I2C0_WriteMultiByteOneReg(uint8_t u8SlaveAddr, uint8_t u8DataRegAddr, uint8_t wdata[], uint32_t u32rLen);
void GW_WM8804_TX_Write_InitTable(void);
void GW_WM8804_TXRX_Write_InitTable(void);
void GW_CS8422_Write_InitTable(void);
void GW_WM8402_CS8422_Init(void);
#endif