/*
 * (C) Copyright 2013
 * Kentaro Sekimoto
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>

#define DEF_BAUDRATE    115200

DECLARE_GLOBAL_DATA_PTR;

//FM3_USART *usart = (FM3_USART *)FM3_MFS0_UART_BASE;
FM3_MFS03_UART_TypeDef *usart = (FM3_MFS03_UART_TypeDef *)FM3_MFS0_UART_BASE;

void serial_setbrg(void)
{
    usart->BGR = (uint16_t)(((uint32_t)SysFrePCLK2 / (uint32_t)gd->baudrate) - 1);
}

s32 serial_init(void)
{
    //bFM3_GPIO_ADE_ANF = 0;       // Disable AN15
    FM3_GPIO->ADE &= ~0x8000;       // Disable AN15
    bFM3_GPIO_PFR2_P1 = 1;
    bFM3_GPIO_EPFR07_SIN0S0 = 1;
    bFM3_GPIO_EPFR07_SIN0S1 = 0;
    bFM3_GPIO_PFR2_P2 = 1;
    bFM3_GPIO_EPFR07_SOT0B0 = 1;
    bFM3_GPIO_EPFR07_SOT0B1 = 0;

    usart->SMR = (SMR_MD_UART | SMR_SOE);
    usart->SCR = 0x00;
    //serial_setbrg();
    usart->BGR = (uint16_t)(((uint32_t)SysFrePCLK2 / (uint32_t)DEF_BAUDRATE) - 1);
    usart->SCR &= ~SCR_UPCL;
    usart->SSR = 0;
    usart->ESCR = ESCR_DATABITS_8;
    usart->SCR |= (SCR_RXE | SCR_TXE);

    return 0;
}

s32 serial_getc(void)
{
    uint16_t c;
    while ((usart->SSR & SSR_RDRF) == 0) ;
    c = (uint16_t)(usart->RDR);
    return (s32)c;
}

void serial_raw_putc(const char c)
{
    while ((usart->SSR & SSR_TDRE) == 0) ;
    usart->TDR = (uint16_t)c;
}

void serial_putc(const char c)
{
    if (c == '\n')
        serial_raw_putc('\r');
    serial_raw_putc(c);
}

void serial_puts(const char *s)
{
    char c;
    while ((c = *s++) != 0)
        serial_putc(c);
}

s32 serial_tstc(void)
{
    return (usart->SSR & SSR_RDRF) ? 1 : 0;
}
