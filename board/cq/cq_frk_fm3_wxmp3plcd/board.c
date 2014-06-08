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
#include <netdev.h>

#include <asm/arch-fm3/fm3.h>
#include <st7781.h>

//#define WXMP3PLCDFL	// For WX-MP3PLCD-F 2MB version

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_LCD_ST7781
static void lcd_reset(int n)
{
	if (n)
		FM3_GPIO->PDOR1 |= 0x0008;
	else
		FM3_GPIO->PDOR1 &= 0xFFF7;
}
#endif

static void gpio_init(void)
{
#ifdef CONFIG_LCD_ST7781
	// LCD init
	FM3_GPIO->ADE &= 0xFFF7;	// P13 NOT Analog
	FM3_GPIO->PFR1 &= 0xFFF7;	// P13 GPIO
	FM3_GPIO->DDR1 |= 0x0008;	// P13 output
    lcd_reset(0);
    udelay(30000);        // Delay 30ms // This delay time is necessary
    lcd_reset(1);
    udelay(30000);        // Delay 30ms // This delay time is necessary
    gd->fb_base = CONFIG_FB_ADDR;
#endif
}

DECLARE_GLOBAL_DATA_PTR;

void sdram_init(void)
{

}

int board_init(void)
{
#ifdef WXMP3PLCDFL
    // For WX-MP3PLCD-F 2MB version
    FM3_EXBUS->DCLKR    = (0 << 4)      //MCLKON
                        | (1 << 0);     //MDIV
    FM3_EXBUS->DCLKR = 0x00000011;
    FM3_EXBUS->MODE0    = (1 << 0)      //WDTH 16bit
                        | (1 << 2);     //RBMON
    FM3_EXBUS->TIM0     = (0 << 28)     //WIDLC
                        | (3 << 24)     //WWEC
                        | (0 << 20)     //WADC
                        | (4 << 16)     //WACC
                        | (0 << 12)     //RIDLC
                        | (0 << 8)      //FRADC
                        | (0 << 4)      //RADC
                        | (8 << 0);     //RACC
    FM3_EXBUS->AREA0    = (0x0f << 16)  //MASK
                        | (0x00 << 0);  //ADDR 0x60000000
    FM3_EXBUS->MODE1    = (0 << 0);     //WDTH 8bit
    FM3_EXBUS->TIM1     = (0 << 28)     //WIDLC
                        | (5 << 24)     //WWEC
                        | (5 << 20)     //WADC
                        | (15 << 16)    //WACC
                        | (15 << 12)    //RIDLC
                        | (0 << 8)      //FRADC
                        | (0 << 4)      //RADC
                        | (15 << 0);    //RACC
    FM3_EXBUS->AREA1    = (0x0f << 16)  //MASK
                        | (0x10 << 0);  //ADDR 0x61000000
    FM3_EXBUS->MODE2    = 0x00000809;
    FM3_EXBUS->TIM2     = 0x01030013;
    FM3_EXBUS->AREA2    = 0x000f0020;	//ADDR 0x62000000-0x623FFFFF
    FM3_GPIO->PFR1      = 0xfff0;       //P14...P1F/A0...A11
    FM3_GPIO->PFR2      = 0x03f1;       //P20,P24...P29/A18,A12...A17
    FM3_GPIO->PFR3      = 0x0040;       //P36/CS2
    FM3_GPIO->PFR4      = 0x7c00;       //P4A...P4E/D0...D4
    FM3_GPIO->PFR5      = 0x0c0f;       //P50...P53,P5A,P5B/MOEX,MWEX,MDQM0,MDQM1,CS0,CS1
    FM3_GPIO->PFR7      = 0x07ff;       //P70...P7A/D5...D15
    FM3_GPIO->PFR9      = 0x0003;       //P90,P91
    //FM3_GPIO->ADE       &= ~0x3f00fff0; //AN04...AN15,AN24...AN29
    FM3_GPIO->ADE       = 0; //AN04...AN15,AN24...AN29
    FM3_GPIO->EPFR10    = 0x0fff80bb;
    FM3_GPIO->EPFR11    = 0x01fffffe;
#endif
    gpio_init();
    return 0;
}

int checkboard(void)
{
#ifdef WXMP3PLCDFL
    printf("Board: CQ_FRK_FM3 + WXMP3 Rev %s, Kentaro Sekimoto\n", CONFIG_SYS_BOARD_REV_STR);
#endif
    return 0;
}

int dram_init(void)
{
    gd->bd->bi_dram[0].start = CONFIG_SYS_RAM_BASE;
    gd->bd->bi_dram[0].size  = CONFIG_SYS_RAM_SIZE;
    return 0;
}

#ifdef CONFIG_FM3_ETH
int board_eth_init(bd_t *bis)
{
#define PFRC_ETH    0xF1FF
#define PFRD_ETH    0x000F

#define B_E_TD0E    18
#define B_E_TD1E    19
#define B_E_TE0E    20
#define B_E_TE1E    21
#define B_E_MC0E    22
#define B_E_MC1B    23
#define B_E_MD0B    24
#define B_E_MD1B    25
#define B_E_CKE     26
#define B_E_PSE     27
#define B_E_SPLC    28

    FM3_GPIO->PFR6 &= ~0x0004;
    FM3_GPIO->DDR6 &= ~0x0004;
    FM3_GPIO->PFRC = PFRC_ETH;      // GPIO configuration for CH0 and CH1
    FM3_GPIO->PFRD = PFRD_ETH;      // GPIO configuration for CH0 and0 CH1
    FM3_GPIO->EPFR14 = (3 << B_E_SPLC) |
            //                        (1 << B_E_PSE) |
            (1 << B_E_CKE) |
            (1 << B_E_MD1B) |
            (1 << B_E_MD0B) |
            (1 << B_E_MC1B) |
            (1 << B_E_MC0E) |
            (1 << B_E_TE1E) |
            (1 << B_E_TE0E) |
            (1 << B_E_TD1E) |
            (1 << B_E_TD0E);
    return fm3_eth_driver_init(bis);
}
#endif

#ifdef CONFIG_LCD_INFO
#include <version.h>

void lcd_show_board_info(void)
{
	ulong dram_size;
	int i;

	lcd_printf ("%s\n", U_BOOT_VERSION);
	lcd_printf (" (C) 2013 Kentaro Sekimoto\n");
	dram_size = 0;
	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++)
		dram_size += gd->bd->bi_dram[i].size;
	lcd_printf (" SDRAM %ld MB\n", dram_size >> 20);
}
#endif /* CONFIG_LCD_INFO */
