/*
 * board.c
 *
 *  Created on: 2013/06/08
 *      Author: ksekimoto
 */

#include <common.h>
#include <netdev.h>

#include <asm/arch-fm3/fm3.h>


static void gpio_init(void)
{

}

DECLARE_GLOBAL_DATA_PTR;

int board_init(void)
{
#ifdef MB9BF506N
    FM3_GPIO->ADE       = 0x00000033;   //AN00,AN01,,,AN04,AN05
    FM3_GPIO->PFR0      = 0xfc1f;       //P0A...P0F/A7...A2
    FM3_GPIO->PFR1      = 0xffcc;       //P12...P1F/A9,A8,,,OE,WE,D8..D15
    FM3_GPIO->PFR3      = 0x00cf;       //D6,D7,LB,UB,,,CS3,CS2
    FM3_GPIO->PFR4      = 0x7f3c;       //A20,A19,A18,A17,,,A16..A10
    FM3_GPIO->PFR5      = 0x003f;       //D0..D5
    FM3_GPIO->PFR6      = 0x0008;       //A1
    FM3_GPIO->EPFR10    = 0x0fff833b;
    // CS3 - SRAM 2MB
    FM3_EXBUS->MODE3    = (1 << 0)      //WDTH 16bit
                        | (1 << 2);     //RBMON
    FM3_EXBUS->TIM3     = (0 << 28)     //WIDLC
                        | (2 << 24)     //WWEC
                        | (0 << 20)     //WADC
                        | (2 << 16)     //WACC
                        | (1 << 12)     //RIDLC
                        | (0 << 8)      //FRADC
                        | (0 << 4)      //RADC
                        | (6 << 0);     //RACC
    FM3_EXBUS->AREA3    = (0x0f << 16)  //MASK 2MB
                        | (0x30 << 0);  //ADDR 0x63000000
    // CS2 - LCD
    FM3_EXBUS->MODE2    = (0 << 0)      //WDTH 8bit
                        | (1 << 2);     //RBMON
    FM3_EXBUS->TIM2     = (0 << 28)     //WIDLC
                        | (2 << 24)     //WWEC
                        | (0 << 20)     //WADC
                        | (2 << 16)     //WACC
                        | (1 << 12)     //RIDLC
                        | (0 << 8)      //FRADC
                        | (0 << 4)      //RADC
                        | (6 << 0);     //RACC
    FM3_EXBUS->AREA2    = (0x0f << 16)  //MASK 2MB
                        | (0x20 << 0);  //ADDR 0x62000000
#endif
#ifdef MB9BF618T
    // For WX-MP3PLCD-F 2MB version
    FM3_GPIO->ADE = (uint32_t)0;    // A/D off
    // P10              P20 MAD18        P40       P50 MOEX nRD  P70 D5    P90 MAD19 RST
    // P11              P21              P41       P51 MWEX nWR  P71 D6
    // P12              P22              P42       P52 MDQM0 LB  P72 D7
    // P13              P23              P43       P53 MDQM1 UB  P73 D8
    // P14 MAD00(AN04)  P24 MAD17(AN29)  P44       P54           P74 D9
    // P15 MAD01(AN05)  P25 MAD16(AN28)  P45       P55           P75 D10
    // P16 MAD02(AN06)  P26 MAD15(AN27)  P46       P56           P76 D11
    // P17 MAD03(AN07)  P27 MAD14(AN26)  P47       P57           P77 D12
    // P18 MAD04(AN08)  P28 MAD13(AN25)  P48 :     P58           P78 D13
    // P19 MAD05(AN09)  P29 MAD12(AN24)  P49 :     P59           P79 D14
    // P1A MAD06(AN10)  P2A              P4A D0    P5A nCS0      P7A D15
    // P1B MAD07(AN11)  P2B              P4B D1    P5B nCS1      P7B
    // P1C MAD08(AN12)  P2C              P4C D2    P5C           P7C
    // P1D MAD09(AN13)  P2D              P4D D3    P5D           P7D
    // P1E MAD10(AN14)  P2E              P4E D4    P5E           P7E
    // P1F MAD11(AN15)  P2F              P4F :     P5F           P7F
#ifndef FAST_PARAM
    // CS0
    FM3_EXBUS->DCLKR = 0x00000011;  /* MCLK=ベースクロックの1/2分周(36MHz)出力 */
    //FM3_EXBUS->MODE0 = 0x0000088D;// SRAM WDTH:01-16bit
    FM3_EXBUS->MODE0 = 0x00000005;  // SRAM WDTH:01-16bit
    FM3_EXBUS->TIM0 =           // Timing Register for SRAM (default 0x055FF00F)
            ((2 - 1) << 28) +   // 31-28: WIDLC (Write Idle Cycle)
            ((2 - 1) << 24) +   // 27-24: WWEC (Write Enable Cycle) - WR-Lパルス幅
            ((1 - 1) << 20) +   // 23-20: WADC (Write Address Setup cycle) - CS-L to WR-L
            ((4 - 1) << 16) +   // 19-16: WACC (Write Access Cycle) > WWEC + WADC
            ((2 - 1) << 12) +   // 15-12: RIDLC (Read Idle Cycle)
            ((2 - 1) << 8) +    // 11-08: FRADC (First Read Address Cycle
            ((1 - 1) << 4) +    // 07-04: RADC (Read Address Setup cycle)
            ((4 - 1) << 0);     // 03-00: RACC (Read Access Cycle)
#else
    // CS0
    FM3_EXBUS->DCLKR = 0x00000011;  /* MCLK=ベースクロックの1/2分周(36MHz)出力 */
    //FM3_EXBUS->MODE0 = 0x0000088D;// SRAM WDTH:01-16bit
    FM3_EXBUS->MODE0 = 0x0000000D;  // SRAM WDTH:01-16bit
    FM3_EXBUS->TIM0 =           // Timing Register for SRAM (default 0x055FF00F)
            ((5 - 1) << 28) +   // 31-28: WIDLC (Write Idle Cycle)
            ((2 - 1) << 24) +   // 27-24: WWEC (Write Enable Cycle) - WR-Lパルス幅
            ((2 - 1) << 20) +   // 23-20: WADC (Write Address Setup cycle) - CS-L to WR-L
            ((5 - 1) << 16) +   // 19-16: WACC (Write Access Cycle) > WWEC + WADC
            ((5 - 1) << 12) +   // 15-12: RIDLC (Read Idle Cycle)
            ((2 - 1) << 8) +    // 11-08: FRADC (First Read Address Cycle
            ((3 - 1) << 4) +    // 07-04: RADC (Read Address Setup cycle)
            ((6 - 1) << 0);     // 03-00: RACC (Read Access Cycle)
#endif
    FM3_EXBUS->AREA0 = 0x000F0000;
    //FM3_EXBUS->ATIM0 = 0x0000045F;
    // CS1
    FM3_EXBUS->MODE1 = 0x00000001;  // LCD  WDTH:01-16bit
    FM3_EXBUS->TIM1 =           // Timing Register for LCD (default 0x055FF00F)
            ((2 - 1) << 28) +   // 31-28: WIDLC (Write Idle Cycle)
            ((6 - 1) << 24) +   // 27-24: WWEC (Write Enable Cycle) - WR-Lパルス幅
            ((8 - 1) << 20) +   // 23-20: WADC (Write Address Setup cycle) - CS-L to WR-L
            ((16- 1) << 16) +   // 19-16: WACC (Write Access Cycle) > WWEC + WADC
            ((16- 1) << 12) +   // 15-12: RIDLC (Read Idle Cycle)
            ((1 - 1) << 8) +    // 11-08: FRADC (First Read Address Cycle
            ((1 - 1) << 4) +    // 07-04: RADC (Read Address Setup cycle)
            ((16- 1) << 0);     // 03-00: RACC (Read Access Cycle)
    FM3_EXBUS->AREA1 = 0x000F0010;
    //FM3_EXBUS->ATIM1 = 0x0000045F;
    //FM3_GPIO->EPFR03 |= 0x2000000;    // IC23_0 -> IC23_1
    //FM3_GPIO->EPFR08 |= 0x0002000;    // SIN6_0 -> SIN6_1
    FM3_GPIO->EPFR10 = 0x07FFC0BF;  // A08-A19, A0,
    FM3_GPIO->EPFR11 = 0x01FFFFFE;
    //FM3_GPIO->PZR5 |= 0x0000000F; //
    FM3_GPIO->ADE &= ~0x3F00FFF0;   //
    FM3_GPIO->PFR1 |= 0x0000FFF0;   //
    FM3_GPIO->PFR2 |= 0x000003F1;   //
    FM3_GPIO->PFR4 |= 0x00007C00;   //
    FM3_GPIO->PFR5 |= 0x00000C0F;   //
    FM3_GPIO->PFR7 |= 0x000007FF;   //
    FM3_GPIO->PFR9 |= 0x00000001;   // P90 A19
    FM3_GPIO->PCR1 |= 0x0000FFF0;   //
    FM3_GPIO->PCR2 |= 0x000003F1;   //
    //FM3_GPIO->PCR4 |= 0x00007C00; //
    FM3_GPIO->PCR5 |= 0x00000C0F;   //
    FM3_GPIO->PCR9 |= 0x00000001;   // P90 A19
#endif
    gpio_init();
    return 0;
}

int checkboard(void)
{
    printf("Board: WX-MP3PLCD-F Rev %s, Kentaro Sekimoto\n", CONFIG_SYS_BOARD_REV_STR);
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
