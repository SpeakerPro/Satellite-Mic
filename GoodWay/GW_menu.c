#include <stdio.h>
#include "NuMicro.h"
#include "GW_main.h"
#include "GW_aw5808.h"

//#define IGNORE_PRINTF
#ifdef IGNORE_PRINTF
	#define printf(fmt, ...) (0)
#endif

MENU_KEY menu_key;

void GW_read_config0(void) {
	
	uint32_t config0;
	int32_t r_code;
	
	SYS_UnlockReg();

	FMC_Open();	
	r_code = FMC_ReadConfig(&config0, 1);
	FMC_Close();
	SYS_LockReg();
	
	printf("\x1b[3;1H 1.config0(0x%04X)(%d)\n", config0, r_code);
}

void GW_write_config0(void) {
	
	uint32_t config0;
	int32_t r_code;
	
	SYS_UnlockReg();

	FMC_Open();	
	r_code = FMC_ReadConfig(&config0, 1);
	
	if(config0 & 0x80)
		config0 &= 0xFFFFFF7F;
	else
		config0 |= 0x80;
	
	FMC_ENABLE_CFG_UPDATE();
	FMC_Erase(FMC_CONFIG_BASE);
	r_code = FMC_WriteConfig(&config0, 1);
	FMC_DISABLE_CFG_UPDATE();
	
	FMC_Close();
	SYS_LockReg();
	
	GW_read_config0();
}

void clear_screen(void) {
	
	printf("\x1b[2;J");
}

void show_menu(void) {
	printf("\x1b[1;1H ZZX9103Z1[DV1] debug menu v%02x.%02x / %s %s\n", VERSION_MAJOR_ID, VERSION_MINOR_ID, __DATE__, __TIME__);
	
	GW_read_config0();
	printf("\x1b[4;1H 2.boot from LDROM\n");
	
	printf("\x1b[5;60H AW5808(Rx):\n");
	printf("\x1b[6;60H ID:\n");
	printf("\x1b[7;60H Status:\n");
	printf("\x1b[8;60H Version:\n");	
	printf("\x1b[9;60H RSSI:\n");		
	//printf("\x1b[9;60H AW5808(Tx):\n");
	//printf("\x1b[10;60H ID:\n");
  //printf("\x1b[11;60H Status:\n");
  //printf("\x1b[12;60H Version:\n");

	printf("\x1b[5;50HSmart:\n");
	printf("\x1b[6;50HVol+:\n");
	printf("\x1b[7;50HVol-:\n");
	printf("\x1b[8;50HMute:\n");
	printf("\x1b[9;50HCFUD:\n");//touch config update
	
}

void GW_menu_init(void)
{	
		menu_key.start = 0;
		menu_key.end = 0;

		clear_screen();
		show_menu();
		//show_GPIO_menu_status();
}

void GW_boot_to_LDROM(uint8_t boot) {
	
		SYS_UnlockReg();

		FMC_Open();
		
		FMC_SetBootSource(boot);					// Boot from LDROM

		NVIC->ICPR[0] = 0xFFFFFFFF;		// Clear Pending Interrupt

		SYS_ResetCPU();	
	
}

void GW_menu_loop(void) {
	
	uint8_t new_start;
	
	if(menu_key.start != menu_key.end) {
		
		printf("\x1b[%d;%dH(%3d)%c\n", SELECT_Y, SELECT_X+8, menu_key.buf[menu_key.start], menu_key.buf[menu_key.start]);
		
		switch(menu_key.buf[menu_key.start])
		{
#if 0			
			case '1':
				GW_AW5808_RX_Enter_Binding();			
				break;
			case '3':
				GW_AW5808_RX_Unpair();			
				break;			
			case '2':
				GW_AW5808_TX_Enter_Binding();						
				break;
			case '4':
				GW_AW5808_TX_Unpair();
				break;		
			case '5':
				GW_write_LDROM();
				break;
			case '6':
				enable_touch_update_process=1;			
				break;	
#endif			
			case '1':
				GW_write_config0();
				break;
			case '2':
				GW_boot_to_LDROM(YES);
				break;
			default:
				GW_gpio_control(menu_key.buf[menu_key.start]);
				break;
		}
		
		new_start = menu_key.start;
		
		new_start ++;
		
		if(new_start == sizeof(menu_key.buf))
			new_start = 0;
		
		menu_key.start = new_start;
	}
}
