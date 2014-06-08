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

#ifndef FM3_H_
#define FM3_H_

#include "fm3_eth.h"

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE   0
#endif

/*
 * Clocks enumeration
 */
enum clock {
    CLOCK_SYSCLK,   /* SYSCLK clock frequency expressed in Hz     */
    CLOCK_HCLK,     /* HCLK clock frequency expressed in Hz       */
    CLOCK_PCLK1,    /* PCLK1 clock frequency expressed in Hz      */
    CLOCK_PCLK2,    /* PCLK2 clock frequency expressed in Hz      */
    CLOCK_SYSTICK,  /* Systimer clock frequency expressed in Hz   */
    CLOCK_END       /* for internal usage                 */
};

#if 1
#ifdef MB9BF506N
#include "mb9bf506r.h"

// Fujitsu Default
//#define APBC0_PSR_Val         0x00000002    // <<< Define APBC0_PSR here
//#define APBC1_PSR_Val         0x00000082    // <<< Define APBC1_PSR here
//#define APBC2_PSR_Val         0x00000082    // <<< Define APBC2_PSR here

// Default
//#define APBC0_PSR_Val         0x00000001    // <<< Define APBC0_PSR here
//#define APBC1_PSR_Val         0x00000081    // <<< Define APBC1_PSR here
//#define APBC2_PSR_Val         0x00000081    // <<< Define APBC2_PSR here

#define SYSTEM_CYCLE_CLOCK_HZ   80000000
#define SysFreHCLK      (SYSTEM_CYCLE_CLOCK_HZ)
#define SysFrePCLK1     (SYSTEM_CYCLE_CLOCK_HZ/2)
#define SysFrePCLK2     (SYSTEM_CYCLE_CLOCK_HZ/2)
#define SysFreSYSTICK   (SYSTEM_CYCLE_CLOCK_HZ/8)
#endif

#ifdef MB9BF618T
#include "mb9bf618t.h"

// Fujitsu Default
//#define APBC0_PSR_Val         0x00000002    // <<< Define APBC0_PSR here
//#define APBC1_PSR_Val         0x00000082    // <<< Define APBC1_PSR here
//#define APBC2_PSR_Val         0x00000082    // <<< Define APBC2_PSR here

// Default
//#define APBC0_PSR_Val         0x00000001    // <<< Define APBC0_PSR here
//#define APBC1_PSR_Val         0x00000081    // <<< Define APBC1_PSR here
//#define APBC2_PSR_Val         0x00000081    // <<< Define APBC2_PSR here

#define SYSTEM_CYCLE_CLOCK_HZ   144000000
#define SysFreHCLK      (SYSTEM_CYCLE_CLOCK_HZ)
#define SysFrePCLK1     (SYSTEM_CYCLE_CLOCK_HZ/2)
#define SysFrePCLK2     (SYSTEM_CYCLE_CLOCK_HZ/2)
#define SysFreSYSTICK   (SYSTEM_CYCLE_CLOCK_HZ/8)
#endif

#define FM3_SPI_0_0 0
#define FM3_SPI_0_1 1
#define FM3_SPI_0_2 2
#define FM3_SPI_1_0 3
#define FM3_SPI_1_1 4
#define FM3_SPI_1_2 5
#define FM3_SPI_2_0 6
#define FM3_SPI_2_1 7
#define FM3_SPI_2_2 8
#define FM3_SPI_3_0 9
#define FM3_SPI_3_1 10
#define FM3_SPI_3_2 11
#define FM3_SPI_4_0 12
#define FM3_SPI_4_1 13
#define FM3_SPI_4_2 14
#define FM3_SPI_5_0 15
#define FM3_SPI_5_1 16
#define FM3_SPI_5_2 17
#define FM3_SPI_6_0 18
#define FM3_SPI_6_1 19
#define FM3_SPI_6_2 20
#define FM3_SPI_7_0 21
#define FM3_SPI_7_1 22
#define FM3_SPI_7_2 23

#define SMR_SOE          0x01U
#define SMR_SCKE         0x02U
#define SMR_BDS          0x04U
#define SMR_SCINV        0x08U
#define SMR_WUCR         0x10U
#define SMR_MD_UART      0x00U
#define SMR_MD_UART_MP   0x20U
#define SMR_MD_SIO       0x40U
#define SMR_MD_LIN       0x60U
#define SMR_MD_I2C       0x80U

#define SCR_TXE          0x01U
#define SCR_RXE          0x02U
#define SCR_TBIE         0x04U
#define SCR_TIE          0x08U
#define SCR_RIE          0x10U
#define SCR_SPI          0x20U
#define SCR_MS           0x40U
#define SCR_UPCL         0x80U

#define SSR_TBI          0x01U
#define SSR_TDRE         0x02U
#define SSR_RDRF         0x04U
#define SSR_ORE          0x08U
#define SSR_FRE          0x10U
#define SSR_PE           0x20U
#define SSR_REC          0x80U

#define ESCR_P           0x08U
#define ESCR_PEN         0x10U
#define ESCR_INV         0x20U
#define ESCR_ESBL        0x40U
#define ESCR_FLWEN       0x80U
#define ESCR_DATABITS_8  0x00U
#define ESCR_DATABITS_5  0x01U
#define ESCR_DATABITS_6  0x02U
#define ESCR_DATABITS_7  0x03U
#define ESCR_DATABITS_9  0x04U

#define BGR_EXT          0x8000U

#define FCR1_FSEL        0x01U
#define FCR1_FTIE        0x02U
#define FCR1_FDRQ        0x04U
#define FCR1_FRIIE       0x08U
#define FCR1_FLSTE       0x10U

#define FCR0_FE1         0x01U
#define FCR0_FE2         0x02U
#define FCR0_FCL1        0x04U
#define FCR0_FCL2        0x08U
#define FCR0_FSET        0x10U
#define FCR0_FLD         0x20U
#define FCR0_FLST        0x40U

typedef struct
{
    volatile uint8_t SMR;
    volatile uint8_t SCR;
    uint8_t RESERVED0[2];
    volatile uint8_t ESCR;
    volatile uint8_t SSR;
    uint8_t RESERVED1[2];
    union {
        volatile uint16_t RDR;
        volatile uint16_t TDR;
    };
    uint8_t RESERVED2[2];
    volatile uint16_t BGR;
} FM3_MFS_CSIO_TypeDef;

#else

//#define SYSTEM_CYCLE_CLOCK_HZ   __MASTERCLK
#define SYSTEM_CYCLE_CLOCK_HZ   80000000
#define SysFreHCLK      (SYSTEM_CYCLE_CLOCK_HZ)
#define SysFrePCLK1     (SYSTEM_CYCLE_CLOCK_HZ/2)
#define SysFrePCLK2     (SYSTEM_CYCLE_CLOCK_HZ/2)
#define SysFreSYSTICK   (SYSTEM_CYCLE_CLOCK_HZ)

typedef struct _FM3_CRG_TD
{
    volatile uint8_t SCM_CTL;
    uint8_t RESERVED0[3];
    volatile uint8_t SCM_STR;
    uint8_t RESERVED1[3];
    volatile uint32_t STB_CTL;
    volatile uint16_t RST_STR;
    uint8_t RESERVED2[2];
    volatile uint8_t BSC_PSR;
    uint8_t RESERVED3[3];
    volatile uint8_t APBC0_PSR;
    uint8_t RESERVED4[3];
    volatile uint8_t APBC1_PSR;
    uint8_t RESERVED5[3];
    volatile uint8_t APBC2_PSR;
    uint8_t RESERVED6[3];
    volatile uint8_t SWC_PSR;
    uint8_t RESERVED7[7];
    volatile uint8_t TTC_PSR;
    uint8_t RESERVED8[7];
    volatile uint8_t CSW_TMR;
    uint8_t RESERVED9[3];
    volatile uint8_t PSW_TMR;
    uint8_t RESERVED10[3];
    volatile uint8_t PLL_CTL1;
    uint8_t RESERVED11[3];
    volatile uint8_t PLL_CTL2;
    uint8_t RESERVED12[3];
    volatile uint16_t CSV_CTL;
    uint8_t RESERVED13[2];
    volatile uint8_t CSV_STR;
    uint8_t RESERVED14[3];
    volatile uint16_t FCSWH_CTL;
    uint8_t RESERVED15[2];
    volatile uint16_t FCSWL_CTL;
    uint8_t RESERVED16[2];
    volatile uint16_t FCSWD_CTL;
    uint8_t RESERVED17[2];
    volatile uint8_t DBWDT_CTL;
    uint8_t RESERVED18[11];
    volatile uint8_t INT_ENR;
    uint8_t RESERVED19[3];
    volatile uint8_t INT_STR;
    uint8_t RESERVED20[3];
    volatile uint8_t INT_CLR;
} FM3_CRG_TD;

typedef struct _FM3_CRTRIM_TD
{
    volatile uint8_t MCR_PSR;
    uint8_t RESERVED0[3];
    volatile uint16_t MCR_FTRM;
    uint8_t RESERVED1[6];
    volatile uint32_t MCR_RLR;
} FM3_CRTRIM_TD;

typedef struct _FM3_FLASH_IF_TD
{
    volatile uint32_t FASZR;
    volatile uint32_t FRWTR;
    volatile uint32_t FSTR;
    uint8_t RESERVED0[4];
    volatile uint32_t FSYNDN;
    uint8_t RESERVED1[236];
    volatile uint32_t CRTRMM;
} FM3_FLASH_IF_TD;


typedef struct _FM3_HWWDT_TD
{
    volatile uint32_t WDG_LDR;
    volatile uint32_t WDG_VLR;
    volatile uint8_t WDG_CTL;
    volatile uint8_t RESERVED0[3];
    volatile uint8_t WDG_ICL;
    volatile uint8_t RESERVED1[3];
    volatile uint8_t WDG_RIS;
    volatile uint8_t RESERVED2[3055];
    volatile uint32_t WDG_LCK;
} FM3_HWWDT_TD;

typedef struct _FM3_GPIO_TD
{
    volatile uint32_t PFR0;
    volatile uint32_t PFR1;
    volatile uint32_t PFR2;
    volatile uint32_t PFR3;
    volatile uint32_t PFR4;
    volatile uint32_t PFR5;
    volatile uint32_t PFR6;
    volatile uint32_t PFR7;
    volatile uint32_t PFR8;
    uint8_t RESERVED0[220];
    volatile uint32_t PCR0;
    volatile uint32_t PCR1;
    volatile uint32_t PCR2;
    volatile uint32_t PCR3;
    volatile uint32_t PCR4;
    volatile uint32_t PCR5;
    volatile uint32_t PCR6;
    volatile uint32_t PCR7;
    uint8_t RESERVED1[224];
    volatile uint32_t DDR0;
    volatile uint32_t DDR1;
    volatile uint32_t DDR2;
    volatile uint32_t DDR3;
    volatile uint32_t DDR4;
    volatile uint32_t DDR5;
    volatile uint32_t DDR6;
    volatile uint32_t DDR7;
    volatile uint32_t DDR8;
    uint8_t RESERVED2[220];
    volatile uint32_t PDIR0;
    volatile uint32_t PDIR1;
    volatile uint32_t PDIR2;
    volatile uint32_t PDIR3;
    volatile uint32_t PDIR4;
    volatile uint32_t PDIR5;
    volatile uint32_t PDIR6;
    volatile uint32_t PDIR7;
    volatile uint32_t PDIR8;
    uint8_t RESERVED3[220];
    volatile uint32_t PDOR0;
    volatile uint32_t PDOR1;
    volatile uint32_t PDOR2;
    volatile uint32_t PDOR3;
    volatile uint32_t PDOR4;
    volatile uint32_t PDOR5;
    volatile uint32_t PDOR6;
    volatile uint32_t PDOR7;
    volatile uint32_t PDOR8;
    uint8_t RESERVED4[220];
    volatile uint32_t ADE;
    uint8_t RESERVED5[124];
    volatile uint32_t SPSR;
    uint8_t RESERVED6[124];
    volatile uint32_t EPFR00;
    volatile uint32_t EPFR01;
    volatile uint32_t EPFR02;
    uint8_t RESERVED7[4];
    volatile uint32_t EPFR04;
    volatile uint32_t EPFR05;
    volatile uint32_t EPFR06;
    volatile uint32_t EPFR07;
    volatile uint32_t EPFR08;
    volatile uint32_t EPFR09;
    volatile uint32_t EPFR10;
} FM3_GPIO_TD;

typedef struct _FM3_EXBUS_TD
{
    volatile uint32_t MODE0;
    volatile uint32_t MODE1;
    volatile uint32_t MODE2;
    volatile uint32_t MODE3;
    volatile uint32_t MODE4;
    volatile uint32_t MODE5;
    volatile uint32_t MODE6;
    volatile uint32_t MODE7;
    volatile uint32_t TIM0;
    volatile uint32_t TIM1;
    volatile uint32_t TIM2;
    volatile uint32_t TIM3;
    volatile uint32_t TIM4;
    volatile uint32_t TIM5;
    volatile uint32_t TIM6;
    volatile uint32_t TIM7;
    volatile uint32_t AREA0;
    volatile uint32_t AREA1;
    volatile uint32_t AREA2;
    volatile uint32_t AREA3;
    volatile uint32_t AREA4;
    volatile uint32_t AREA5;
    volatile uint32_t AREA6;
    volatile uint32_t AREA7;
} FM3_EXBUS_TD;

#define FM3_FLASH_BASE      (0x00000000UL)                 /* Flash Base                             */
#define FM3_PERIPH_BASE     (0x40000000UL)                 /* Peripheral  Base                       */
#define FM3_CM3_BASE        (0xE0100000UL)                 /* CM3 Private                            */

#define FM3_FLASH_IF_BASE   (FM3_PERIPH_BASE + 0x00000UL)  /* Flash interface registers              */
#define FM3_CRG_BASE        (FM3_PERIPH_BASE + 0x10000UL)  /* Clock and reset registers              */
#define FM3_HWWDT_BASE      (FM3_PERIPH_BASE + 0x11000UL)  /* Hardware watchdog registers            */
#define FM3_CRTRIM_BASE     (FM3_PERIPH_BASE + 0x2E000UL)  /* CR trimming registers                  */
#define FM3_GPIO_BASE       (FM3_PERIPH_BASE + 0x33000UL)  /* General purpose I/O registers          */
#define FM3_MFS0_UART_BASE  (FM3_PERIPH_BASE + 0x38000UL)
#define FM3_EXBUS_BASE      (FM3_PERIPH_BASE + 0x3F000UL)  /* External bus interface registers       */

#define FM3_FLASH_IF        ((FM3_FLASH_IF_TD *)FM3_FLASH_IF_BASE)
#define FM3_CRG             ((FM3_CRG_TD *)FM3_CRG_BASE)
#define FM3_HWWDT           ((FM3_HWWDT_TD *)FM3_HWWDT_BASE)
#define FM3_CRTRIM          ((FM3_CRTRIM_TD *)FM3_CRTRIM_BASE)
#define FM3_GPIO            ((FM3_GPIO_TD *)FM3_GPIO_BASE)
#define FM3_EXBUS           ((FM3_EXBUS_TD *)FM3_EXBUS_BASE)

#define bFM3_GPIO_ADE_ANF                      *((volatile unsigned int*)(0x4266A03CUL))
#define bFM3_GPIO_PFR2_P1                      *((volatile unsigned int*)(0x42660104UL))
#define bFM3_GPIO_PFR2_P2                      *((volatile unsigned int*)(0x42660108UL))
#define bFM3_GPIO_EPFR07_SIN0S0                *((volatile unsigned int*)(0x4266C390UL))
#define bFM3_GPIO_EPFR07_SIN0S1                *((volatile unsigned int*)(0x4266C394UL))
#define bFM3_GPIO_EPFR07_SOT0B0                *((volatile unsigned int*)(0x4266C398UL))
#define bFM3_GPIO_EPFR07_SOT0B1                *((volatile unsigned int*)(0x4266C39CUL))

#define SMR_SOE          0x01U
#define SMR_SCKE         0x02U
#define SMR_BDS          0x04U
#define SMR_SCINV        0x08U
#define SMR_WUCR         0x10U
#define SMR_MD_UART      0x00U
#define SMR_MD_UART_MP   0x20U
#define SMR_MD_SIO       0x40U
#define SMR_MD_LIN       0x60U
#define SMR_MD_I2C       0x80U

#define SCR_TXE          0x01U
#define SCR_RXE          0x02U
#define SCR_TBIE         0x04U
#define SCR_TIE          0x08U
#define SCR_RIE          0x10U
#define SCR_SPI          0x20U
#define SCR_MS           0x40U
#define SCR_UPCL         0x80U

#define SSR_TBI          0x01U
#define SSR_TDRE         0x02U
#define SSR_RDRF         0x04U
#define SSR_ORE          0x08U
#define SSR_FRE          0x10U
#define SSR_PE           0x20U
#define SSR_REC          0x80U

#define ESCR_P           0x08U
#define ESCR_PEN         0x10U
#define ESCR_INV         0x20U
#define ESCR_ESBL        0x40U
#define ESCR_FLWEN       0x80U
#define ESCR_DATABITS_8  0x00U
#define ESCR_DATABITS_5  0x01U
#define ESCR_DATABITS_6  0x02U
#define ESCR_DATABITS_7  0x03U
#define ESCR_DATABITS_9  0x04U

#define BGR_EXT          0x8000U

#define FCR1_FSEL        0x01U
#define FCR1_FTIE        0x02U
#define FCR1_FDRQ        0x04U
#define FCR1_FRIIE       0x08U
#define FCR1_FLSTE       0x10U

#define FCR0_FE1         0x01U
#define FCR0_FE2         0x02U
#define FCR0_FCL1        0x04U
#define FCR0_FCL2        0x08U
#define FCR0_FSET        0x10U
#define FCR0_FLD         0x20U
#define FCR0_FLST        0x40U

typedef struct _FM3_USART
{
    volatile uint8_t SMR;
    volatile uint8_t SCR;
    volatile uint8_t RESERVED0[2];
    volatile uint8_t ESCR;
    volatile uint8_t SSR;
    volatile uint8_t RESERVED1[2];
    volatile uint16_t RDR;
    volatile uint16_t TDR;
    volatile uint8_t RESERVED2[2];
    volatile uint16_t BGR;
} FM3_USART;
#endif

#endif /* FM3_H_ */
