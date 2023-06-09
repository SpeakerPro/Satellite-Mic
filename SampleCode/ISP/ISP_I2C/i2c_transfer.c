/***************************************************************************//**
 * @file     i2c_transfer.c
 * @brief    ISP support function source file
 * @version  0x32
 * @date     14, June, 2017
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t i2c_rcvbuf[64];
volatile uint8_t bI2cDataReady;

volatile uint8_t g_u8SlvDataLen;

__STATIC_INLINE void I2C_SlaveTRx(I2C_T *i2c, uint32_t u32Status);

extern uint32_t Pclk0;

void I2C_Init(void)
{
    /* Reset I2C0 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;
    /* Open I2C0 and set clock to 100k */
    I2C0->CLKDIV = (uint32_t)(((Pclk0 * 10U) / (100000 * 4U) + 5U) / 10U - 1U); /* Compute proper divider for I2C clock */;
    I2C0->CTL0 |= I2C_CTL0_I2CEN_Msk;
    /* Set I2C0 ADDR0 Slave Addresses */
    I2C0->ADDR0  = (I2C_ADDR << 1U) | I2C_GCMODE_DISABLE;
    I2C0->CTL0 |= I2C_CTL0_INTEN_Msk;
    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    NVIC_EnableIRQ(I2C0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C0->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        I2C_SlaveTRx(I2C0, u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(I2C_T *i2c, uint32_t u32Status)
{
    uint8_t u8data;

    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        bI2cDataReady = 0;
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        i2c_rcvbuf[g_u8SlvDataLen] = I2C_GET_DATA(i2c);
        g_u8SlvDataLen++;
        g_u8SlvDataLen &= 0x3F;
        bI2cDataReady = (g_u8SlvDataLen == 0);

        if (g_u8SlvDataLen == 0x3F)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
        }
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        u8data = response_buff[g_u8SlvDataLen];
        I2C_SET_DATA(i2c, u8data);
        g_u8SlvDataLen++;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xB8)
    {
        u8data = response_buff[g_u8SlvDataLen];
        I2C_SET_DATA(i2c, u8data);
        g_u8SlvDataLen++;
        g_u8SlvDataLen &= 0x3F;

        if (g_u8SlvDataLen == 0x00)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
        }
    }
    else if (u32Status == 0xC8)
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        i2c_rcvbuf[g_u8SlvDataLen] = I2C_GET_DATA(i2c);
        g_u8SlvDataLen++;
        bI2cDataReady = (g_u8SlvDataLen == 64);
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        // printf("Status 0x%x is NOT processed\n", u32Status);
    }
}


