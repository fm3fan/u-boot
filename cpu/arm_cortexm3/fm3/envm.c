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
#include <errno.h>
#include "envm.h"

#define FLASH_DPOLL         0x80        // data pollbit
#define FLASH_DQ6           0x00000040  // Data toggle flag bit (TOGG) position
#define FLASH_DQ5           0x00000020  // Time limit exceeding flag bit (TLOV) position

#if 0
static inline void __attribute__((used)) __enable_irq(void)
{
    asm volatile ("cpsie i");
}

static inline void __attribute__((used)) __disable_irq(void)
{
    asm volatile ("cpsid i");
}
#endif

#define FLASH_MIN_SEC       0x4000

uint32_t
__attribute__ ((section(".ramcode")))
__attribute__ ((long_call))
sector_top(uint32_t addr)
{
    if (addr < 0x00004000)
        return 0x0000;
    else if (addr < 0x00008000)
        return 0x4000;
    else if (addr < 0x00020000)
        return 0x8000;
    else if (addr < 0x00040000)
        return 0x20000;
    else if (addr < 0x00060000)
        return 0x40000;
    else if (addr < 0x00080000)
        return 0x60000;
#ifdef MB9BF618T
    else if (addr < 0x000A0000)
        return 0x80000;
    else if (addr < 0x000C0000)
        return 0xA0000;
    else if (addr < 0x000E0000)
        return 0xC0000;
    else if (addr < 0x00100000)
        return 0xE0000;
    else {
        printf("sector_top err\r\n");
        return 0xE0000;
    }
#endif
#ifdef MB9BF506N
    else {
        printf("sector_top err\r\n");
        return 0x60000;
    }
#endif
}

uint32_t
__attribute__ ((section(".ramcode")))
__attribute__ ((long_call))
fm3_flash_iserased(uint32_t addr, uint32_t size)
{
    uint32_t *start = (uint32_t *)addr;
    uint32_t *end = (uint32_t *)(addr + size);
    while (start < end)
        if (*start++ != (uint32_t)0xFFFFFFFF)
            return FALSE;
    return TRUE;
}

uint32_t
__attribute__ ((section(".ramcode")))
__attribute__ ((long_call))
fm3_flash_erase_block(uint32_t addr)
{
    FM3_FLASH_IF_TypeDef *flash_if = FM3_FLASH_IF;
    volatile uint32_t dummy;
    flash_if->FASZR = (uint32_t)0x0001;
    dummy = flash_if->FASZR;
    uint32_t off = sector_top((uint32_t)addr);
    uint32_t i;
    uint32_t ret = TRUE;
    __disable_irq();
    for (i = 0; i < 2 ; i++) {
        if (i)
            off += 4;
        *(volatile uint16_t *)0x1550 = (uint16_t)0x00AA;
        *(volatile uint16_t *)0x0AA8 = (uint16_t)0x0055;
        *(volatile uint16_t *)0x1550 = (uint16_t)0x0080;
        *(volatile uint16_t *)0x1550 = (uint16_t)0x00AA;
        *(volatile uint16_t *)0x0AA8 = (uint16_t)0x0055;
        *(volatile uint16_t *)off    = (uint16_t)0x0030;
        volatile uint16_t dummy1;
        volatile uint16_t dummy2;
        dummy1 = *(uint16_t *)off;
        while (TRUE) {
            dummy1 = *(uint16_t *)off;
            dummy2 = *(uint16_t *)off;
            if ((dummy1 & FLASH_DQ6) == (dummy2 & FLASH_DQ6))
                break;
            if (dummy1 & FLASH_DQ5) {
                break;
            }
        }
        dummy1 = *(uint16_t *)off;
        dummy2 = *(uint16_t *)off;
        if ((dummy1 & FLASH_DQ6) != (dummy2 & FLASH_DQ6)) {
            ret = FALSE;
            break;
        }
    }
    flash_if->FASZR = (uint32_t)0x0002;
    dummy = flash_if->FASZR;
    __enable_irq();
    return ret;
}

uint32_t
__attribute__ ((section(".ramcode")))
__attribute__ ((long_call))
fm3_flash_erase(uint32_t offset, uint32_t size)
{
    uint32_t unit;
    while (size > 0) {
        if (size > FLASH_MIN_SEC)
            unit = FLASH_MIN_SEC;
        else
            unit = size;
        if (!fm3_flash_iserased(offset, unit)) {
            if (!fm3_flash_erase_block(offset)) {
                printf("flash erase NG around 0x%08x\n", offset);
                return FALSE;
            }
        }
        size -= unit;
        offset += unit;
    }
    return TRUE;
}

uint32_t
__attribute__ ((section(".ramcode")))
__attribute__ ((long_call))
fm3_flash_write(uint32_t addr, void *buf, uint32_t size)
{
    uint32_t ret = TRUE;
    //if (!fm3_flash_iserased(addr, size)) {
    //    return FALSE;
    //}
    FM3_FLASH_IF_TypeDef *flash_if = FM3_FLASH_IF;
    volatile uint32_t dummy;
    volatile uint16_t src_dpoll;
    volatile uint32_t timeout;
    volatile uint16_t *start = (uint16_t *)((uint32_t)addr);
    volatile uint16_t *end = (uint16_t *)((uint32_t)addr + size);
    volatile uint16_t *src = (uint16_t *)buf;
    __disable_irq();
    flash_if->FASZR = (uint32_t)0x0001;
    dummy = flash_if->FASZR;
    while (start < end) {
        *(volatile uint16_t *)0x1550 = (uint16_t)0x00AA;
        *(volatile uint16_t *)0x0AA8 = (uint16_t)0x0055;
        *(volatile uint16_t *)0x1550 = (uint16_t)0x00A0;
        *start = *src;
        dummy = *start;    // dummy read
        src_dpoll = *src & FLASH_DPOLL;
        timeout = 10000;
        while (TRUE) {
            if ((*start & FLASH_DPOLL) == src_dpoll)
                break;
            if (timeout-- == 0)
                break;
        }
        if ((timeout == 0) && ((*start & FLASH_DPOLL) != src_dpoll)) {
            ret = FALSE;
            break;
        }
        start++;
        src++;
    }
    flash_if->FASZR = (uint32_t)0x0002;
    dummy = flash_if->FASZR;
    __enable_irq();
    if (memcmp((void *)buf, (void *)addr, size) != 0)
        ret = FALSE;
    return ret;
}

void
__attribute__ ((section(".ramcode")))
__attribute__ ((long_call))
envm_init(void)
{
}

u32
__attribute__ ((section(".ramcode")))
__attribute__ ((long_call))
envm_write(uint32_t offset, void * buf, uint32_t size)
{
    uint32_t ret = 0;
    if (!fm3_flash_erase(offset, size))
        return ret;
    if (fm3_flash_write(offset, buf, size))
        ret = size;
    return ret;
}
