/*
 * Copyright (C) 2013
 * Kentaro Sekimoto
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

#include <config.h>
#include <common.h>
#include <net.h>
#include <netdev.h>
#include <asm/errno.h>

/* LAN8187C PHY Registers */
#define PHY_REG_BMCR    0x00    // Basic Mode Control Register
#define PHY_REG_BMSR    0x01    // Basic Mode Status Register
#define PHY_REG_IDR1    0x02    // PHY Identifier 1
#define PHY_REG_IDR2    0x03    // PHY Identifier 2
#define PHY_REG_STS     0x10    // Status Register
#define PHY_REG_19      19      // Extended Control Register 19
#define PHY_REG_LEDC    20
#define PHY_REG_INTC    22

#define BMCR_RESET      0x8000
#define BMSR_AUTO_DONE  0x0020

//#define PHY_AUTO_NEG  0x1000    // Select Auto Negotiation
#define PHY_AUTO_NEG    0x1200      // Select Auto Negotiation

#define STS_LINK_ON     0x1
#define STS_10_MBIT     0x2
#define STS_FULL_DUP    0x4

#define MII_WR_TOUT     0x010000    // MII Write timeout count
#define MII_RD_TOUT     0x010000    // MII Read timeout count

#define ICS1894_ADDR    0x10        // PHY device address for Wakamatsu FM3 LAN board
#define LAN8187_ADDR    0x06        // PHY device address for Wakamatsu ARM LAN board
#define DP83848_ADDR    0x01        // PHY device address for DP83848
#define LAN8700_ADDR    0x1F        // PHY device address for Will Electronics WX-PHY
#define ICS1894_ID      0x0015f450  // PHY Identifier of ICS1894
#define LAN8187_ID      0x0006C0C4  // PHY Identifier of LAN8187
#define DP83848_ID      0x20005C90  // PHY Identifier of DP83848
#define LAN8700_ID      0x0007C0C0  // PHY Identifier of LAN8700i
#define PHY_ADDR        ICS1894_ADDR
#define DEF_PHY_RESET_RETRY     3
#define DEF_PHY_AUTONEG_RETRY   1
#define DEF_PHY_RESET_STATUS_RETRY      10000
#define DEF_PHY_AUTONEG_STATUS_RETRY    100000

#define ETH_BUF_MIN
// DMA Register 0 (BMR)
#define DEF_DMA_PBL     16      // Programmable Burst Length
#define DEF_DMA_DSL     0       // Ring Mode
#define DEF_DMA_PR      1       // 1 -> Rx:Tx=2:1
// TX and RX descriptors info
#ifdef ETH_BUF_MIN
#define DEF_TXDESC_NUM  3       // Num of TX Descripter
#define DEF_RXDESC_NUM  4       // Num of RX Descripter
#define DEF_TX_BUF_NUM  DEF_TXDESC_NUM      // Num of TX buffer
#define DEF_TX_BUF_SIZE 1536                // Size of TX buffer
#define DEF_RX_BUF_NUM  DEF_RXDESC_NUM      // Num of RX buffer
#define DEF_RX_BUF_SIZE 1536                // Size of RX buffer
#else
#define DEF_TXDESC_NUM  4       // Num of TX Descripter
#define DEF_RXDESC_NUM  4       // Num of RX Descripter
#define DEF_TX_BUF_NUM  (DEF_TXDESC_NUM*2)  // Num of TX buffer
#define DEF_TX_BUF_SIZE 1536                // Size of TX buffer
#define DEF_RX_BUF_NUM  (DEF_RXDESC_NUM*2)  // Num of RX buffer
#define DEF_RX_BUF_SIZE 1536                // Size of RX buffer
#endif
#define MFFR_PROMISCUOUS                    // PROMISCUOUS mode

FM3_USBETHERNETCLK_TypeDef *ethclk = FM3_USBETHERNETCLK;
FM3_ETHERNET_MAC_TypeDef *ethmac = FM3_ETHERNET_MAC0;
FM3_ETHERNET_CONTROL_TypeDef *ethctrl = FM3_ETHERNET_CONTROL;

char TxBuf[DEF_TX_BUF_NUM][DEF_TX_BUF_SIZE] __attribute__((aligned(4)));
char RxBuf[DEF_RX_BUF_NUM][DEF_RX_BUF_SIZE] __attribute__((aligned(4)));
EMAC_DMA_TXDESC txdesc[DEF_TXDESC_NUM] __attribute__((aligned(4)));
EMAC_DMA_RXDESC rxdesc[DEF_RXDESC_NUM] __attribute__((aligned(4)));
//char tmpbuf[DEF_TX_BUF_SIZE] __attribute__((aligned(4)));
uint32_t txdesc_id = 0;    // Current index of TX Descriptor

// PHY Address
char PhyAddr[] = {
    ICS1894_ADDR,
    LAN8187_ADDR,
    DP83848_ADDR,
    LAN8700_ADDR
};
uint32_t PhyAddrIdx = -1;
uint32_t phy_addr = PHY_ADDR;
#define PHY_MAX (sizeof(PhyAddr)/sizeof(char))

// MAC Address
char MACAddr[6] = {0x00, 0x22, 0x33, 0x44, 0x55, 0x66};
char Broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static void wait(volatile uint32_t count)
{
    while (count-- > 0) ;
}

int32_t PHY_Write(uint32_t phy_addr, uint32_t phy_register, uint32_t value, uint32_t timeout)
{
    while ((ethmac->GAR & GMAC_GAR_GB) != 0) {
        if (timeout-- == 0)
            return FALSE;
    }
    ethmac->GDR = (uint16_t)value;
    ethmac->GAR &= ~((0x1f << GMAC_GAR_B_PA) | (0x1f << GMAC_GAR_B_GR));
    ethmac->GAR |= ((phy_addr << GMAC_GAR_B_PA) | (phy_register << GMAC_GAR_B_GR));
    ethmac->GAR |=  ((1 << GMAC_GAR_B_GW) |(1 << GMAC_GAR_B_GB));
    return TRUE;
}

int32_t PHY_Read(uint32_t phy_addr, uint32_t phy_register, uint32_t *value, uint32_t timeout)
{
    uint32_t count = timeout;
    while ((ethmac->GAR & GMAC_GAR_GB) != 0) {
        if (count-- == 0)
            return FALSE;
    }
    ethmac->GAR &= ~((0x1f << GMAC_GAR_B_PA) | (0x1f << GMAC_GAR_B_GR));
    ethmac->GAR |= ((phy_addr << GMAC_GAR_B_PA) | (phy_register << GMAC_GAR_B_GR));
    ethmac->GAR &=  ~(1 << GMAC_GAR_B_GW);
    ethmac->GAR |= (1 << GMAC_GAR_B_GB);
    count = timeout;
    while ((ethmac->GAR & GMAC_GAR_GB) != 0) {
        if (count-- == 0)
            return FALSE;
    }
    *value = (uint32_t)ethmac->GDR;
    return TRUE;
}

int32_t PHY_Reset(uint32_t phy_addr)
{
    uint32_t value;
    int32_t flag = FALSE;
    uint32_t reset_retry = DEF_PHY_RESET_RETRY;
    while (reset_retry-- > 0) {
        if (PHY_Write(phy_addr, PHY_REG_BMCR, BMCR_RESET, MII_WR_TOUT) == TRUE) {
            uint32_t status_retry = DEF_PHY_RESET_STATUS_RETRY;
            while (status_retry-- > 0) {
                if (PHY_Read(phy_addr, PHY_REG_BMCR, &value, MII_RD_TOUT) == TRUE) {
                    if ((value & BMCR_RESET) == 0) {
                        flag = TRUE;
                        break;
                    }
                }
            }
            if (flag == TRUE)
                break;
        }
    }
    return flag;
}

int32_t PHY_Find(void)
{
    int32_t i;
    for (i = 0; i < PHY_MAX; i++) {
        if (PHY_Reset((uint32_t)PhyAddr[i]) == TRUE) {
            PhyAddrIdx = i;
            phy_addr = (uint32_t)PhyAddr[PhyAddrIdx];
            return TRUE;
        }
    }
    return FALSE;
}

int32_t PHY_AutoNegotiate(void)
{
    uint32_t value;
    int32_t flag = FALSE;
    uint32_t reset_retry = DEF_PHY_RESET_RETRY;
    while (reset_retry-- > 0) {
        if (PHY_Write(phy_addr, PHY_REG_BMCR, PHY_AUTO_NEG, MII_WR_TOUT) == TRUE) {
            //PHY_Write(phy_addr, 4, 0x1E1, MII_WR_TOUT);
            //PHY_Write(phy_addr, PHY_REG_BMCR, 0x1200, MII_WR_TOUT);
            uint32_t status_retry = DEF_PHY_AUTONEG_STATUS_RETRY;
            while (status_retry-- > 0) {
                if (PHY_Read(phy_addr, PHY_REG_BMSR, &value, MII_RD_TOUT) == TRUE) {
                    if (value & BMSR_AUTO_DONE) {
                        flag = TRUE;
                        break;
                    }
                }
            }
            if (flag == TRUE)
                break;
        }
    }
    if (flag)
        printf("PHY: AutoNegotiate OK\r\n");
    else
        printf("PHY: AutoNegotiate NG\r\n");
    return flag;
}

int32_t PHY_Init(void)
{
    uint32_t id1, id2;
    int32_t flag = FALSE;

    if (PHY_Find() == TRUE) {
        printf("PHY: Find OK id=%02x\r\n", phy_addr);
    } else {
        printf("PHY: Find NG\r\n");
        return flag;
    }
    if (PHY_Read(phy_addr, PHY_REG_IDR1, &id1, MII_RD_TOUT) == TRUE) {
        printf("PHY: ReadID OK %04x\r\n", id1);
    } else {
        printf("PHY: ReadID NG\r\n");
    }
    if (PHY_Read(phy_addr, PHY_REG_IDR2, &id2, MII_RD_TOUT) == TRUE) {
        printf("PHY: ReadID OK %04x\r\n", id2);
    } else {
        printf("PHY: ReadID NG\r\n");
    }
    if (id2 == (ICS1894_ID & 0xffff)) {
#ifndef DEBUG_EMAC_PHY
        PHY_Write(phy_addr, PHY_REG_19, 0x0201, MII_WR_TOUT);
        /* LED0/LED1 (LED0:Link up/LED1:Active) */
        PHY_Write(phy_addr, PHY_REG_LEDC, 0x0008, MII_WR_TOUT);
#else
        if (PHY_Write(phy_addr, PHY_REG_19, 0x0201, MII_WR_TOUT) == TRUE) {
            printf("PHY: MDI_MODE OK\r\n");
        } else {
            printf("PHY: MDI_MODE NG\r\n");
        }
        /* LED0/LED1 (LED0:Link up/LED1:Actinve) */
        if (PHY_Write(phy_addr, PHY_REG_LEDC, 0x0008, MII_WR_TOUT) == TRUE) {
            printf("PHY: LED initialize OK\r\n");
        } else {
            printf("PHY: LED initialize NG\r\n");
        }
#endif
#ifdef DEBUG_EMAC_REG_DUMP
        PHY_Register_Dump();
#endif
    }
    flag = TRUE;
    return flag;
}

int32_t PHY_LinkSpeed(void)
{
    uint32_t value;
    //int32_t full_duplex, mbit_100;

    printf("Waiting for auto negotiation...\r\n");
    if (PHY_AutoNegotiate() != TRUE) {
        //mbit_100 = TRUE;
        //full_duplex = TRUE;
        PHY_Write(phy_addr, PHY_REG_BMCR, 0x3100, MII_WR_TOUT);
        ethmac->MCR |= (GMAC_MCR_FES | GMAC_MCR_DM);
        printf("LinkSpeed: Full Duplex\r\n");
        printf("LinkSpeed: 100Mbs\r\n");
    } else {
        if (PHY_Read(phy_addr, PHY_REG_BMSR, &value, MII_RD_TOUT) == FALSE){
            return FALSE;
        }
        if (value & STS_FULL_DUP) {
            printf("LinkSpeed: Full Duplex\r\n");
            //full_duplex = TRUE;
        } else {
            printf("LinkSpeed: Half Duplex\r\n");
            //full_duplex = FALSE;
        }
        if (value & STS_10_MBIT) {
            printf("LinkSpeed: 10Mbs\r\n");
            //mbit_100 = FALSE;
        } else {
            printf("LinkSpeed: 100Mbs\r\n");
            //mbit_100 = TRUE;
            PHY_Write(phy_addr, PHY_REG_BMCR, 0x2100, MII_WR_TOUT);
            ethmac->MCR |= (GMAC_MCR_FES | GMAC_MCR_DM);
        }
    }
    return TRUE;
}

// Initialize TX Descriptor
void TXDESC_Init(EMAC_DMA_TXDESC *txdesc, uint32_t b1ap, uint32_t b2ap)
{
    txdesc->TDES0 = 0;
    txdesc->TDES1_f.TBS1 = 0;
    txdesc->TDES1_f.TBS2 = 0;
    txdesc->TDES2 = b1ap;
    txdesc->TDES3 = b2ap;
    txdesc_id = 0;
}

// Initialize RX Desctiptor
void RXDESC_Init(EMAC_DMA_RXDESC *rxdesc, uint32_t b1ap, uint32_t b2ap)
{
    rxdesc->RDES0 = 0;
    rxdesc->RDES1_f.RBS1 = DEF_RX_BUF_SIZE;
#ifdef ETH_BUF_MIN
    rxdesc->RDES1_f.RBS2 = 0;
#else
    rxdesc->RDES1_f.RBS2 = DEF_RX_BUF_SIZE;
#endif
    rxdesc->RDES2 = b1ap;
    rxdesc->RDES3 = b2ap;
}

// Initialize TX and RX Descriptors
void DESC_Init(void)
{
    uint32_t i;
#ifdef ETH_BUF_MIN
    for (i=0; i<DEF_TXDESC_NUM; i++) {
        TXDESC_Init(&txdesc[i], (uint32_t)&TxBuf[i], (uint32_t)0);
        if (i == (DEF_TXDESC_NUM-1))
            txdesc[i].TDES0_f.TER = 1;
    }
    for (i=0; i<DEF_RXDESC_NUM; i++) {
        RXDESC_Init(&rxdesc[i], (uint32_t)&RxBuf[i], (uint32_t)0);
        rxdesc[i].RDES0_f.OWN = 1;
        if (i == (DEF_RXDESC_NUM-1))
            rxdesc[i].RDES1_f.RER = 1;
    }
#else
    for (i=0; i<DEF_TXDESC_NUM; i++) {
        TXDESC_Init(&txdesc[i], (uint32_t)&TxBuf[i*2], (uint32_t)&TxBuf[i*2+1]);
        if (i == (DEF_TXDESC_NUM-1))
            txdesc[i].TDES0_f.TER = 1;
    }
    for (i=0; i<DEF_RXDESC_NUM; i++) {
        RXDESC_Init(&rxdesc[i], (uint32_t)&RxBuf[i*2], (uint32_t)&RxBuf[i*2+1]);
        rxdesc[i].RDES0_f.OWN = 1;
        if (i == (DEF_RXDESC_NUM-1))
            rxdesc[i].RDES1_f.RER = 1;
    }
#endif
    ethmac->RDLAR = (uint32_t)&rxdesc[0];
    ethmac->TDLAR = (uint32_t)&txdesc[0];
}

#define DMA_TIMEOUT 1000

// Initialize Ethernet DMA
// Refer to Programming Guide P.170
int32_t DMA_Init(void)
{
    uint32_t dma_timeout = DMA_TIMEOUT;
    ethmac->BMR = DMA_BMR_SWR;                      // soft reset
    while ((ethmac->BMR & DMA_BMR_SWR) == 1) {      // wait for reset completion (== 0)
        if (dma_timeout-- == 0)
            return FALSE;
    }
    while ((ethmac->AHBSR & DMA_AHBSR_AHBS) == 1);  // wait for AHB master transaction completion
    ethmac->MCR |= GMAC_MCR_PS;                     // select MII/RMII interface (MUST)
    // DMA Bus Mode Register
    // Recommend: Mixed Burst=1 , AAL=0, FB = 0, Fixed burst or undefined burst.
#if 0
    ethmac->BMR &= ~(DMA_BMR_FB | DMA_BMR_AAL);
    ethmac->BMR |= (DMA_BMR_MB |                    // Mixed Burst = 1
            (DEF_DMA_PBL << DMA_BMR_B_PBL) |        // Programmable Burst Length
            (DEF_DMA_DSL << DMA_BMR_B_DSL) |        // Descriptor Skip Length
            DMA_BMR_DA |                            // DMA arbitration scheme
            (DEF_DMA_PR << DMA_BMR_B_PR));          // Priority Ratio
#else
    ethmac->BMR = 0x04025002;
#endif
    DESC_Init();
    // DMA Operation Mode Register
    // RSF (Receive Store and Forward)
    // TSF (Transmit Store Forward)
    // FUF (Forward Undersized Good Frames) Transfer less than 64 bytes
    ethmac->OMR = (DMA_OMR_RSF | DMA_OMR_TSF | DMA_OMR_FUF);
    ethmac->SR |= DMA_SR_NIS;                       // clear normal interrupt status
    // NIE (Normal Interrupt Summary Enable)
    // ERE (Early Receive Interrupt Enable)
    // RIE (Receive Interrupt Enable)
    // TIE (Transmit Interrupt)
#ifdef FM3_ETH_INT
    ethmac->IER = (DMA_IER_NIE |DMA_IER_TIE | DMA_IER_RIE | DMA_IER_ERE);
#endif
    while ((ethmac->AHBSR & DMA_AHBSR_AHBS) == 1);  // wait for AHB master transaction completion
    // SR DMA RX Running
    // ST DMA TX Running
    ethmac->OMR |= (DMA_OMR_SR | DMA_OMR_ST);
    //ethmac->OMR |= DMA_OMR_SR;
    return TRUE;
}

// Initialize GMAC
// GMAC : Ethernet Media Access Controller
int32_t GMAC_Init(uint8_t *pmac)
{
    ethmac->GAR |= GMAC_GAR_CR;             // CR:0001 -> SYS_CLK 100-150MHz SYS_CLK/62
    if (PHY_Init() == FALSE)                // PHY Initialize
        return FALSE;
    PHY_LinkSpeed();                        // Set LinkSpeed
    memcpy((void *)MACAddr, (void *)pmac, 6);
    ethmac->MAR0H = (uint32_t)(*((uint16_t *)pmac));
    ethmac->MAR0L = (uint32_t)(*((uint32_t *)(pmac+2)));
#ifdef MFFR_PROMISCUOUS
    ethmac->MFFR = (GMAC_MFFR_PR | GMAC_MFFR_RA);
#else
    ethmac->MAR1H = (uint32_t)(*((uint16_t *)pmac)) | 0x80000000;
    ethmac->MAR1L = (uint32_t)(*((uint32_t *)(pmac+2)));
    ethmac->MFFR &= ~(GMAC_MFFR_DB);
    ethmac->MFFR |= (GMAC_MFFR_HUC | GMAC_MFFR_RA);
#endif
    ethmac->MCR |= GMAC_MCR_TE | GMAC_MCR_RE;   // enable transmit and receive
    return TRUE;
}

// Initialize Ethernet Control registers
void ETHCTRL_Init(void)
{
    ethctrl->ETH_CLKG = (3 << ETH_CLKG_MACEN);      // start EMAC clock
#ifdef CONFIG_FM3_ETH_CH0
    ethctrl->ETH_MODE = (1 << ETH_MODE_IFMODE) |    // set RMII mode and reset EMAC
            (1 << ETH_MODE_RST0);
    wait(500000);
    ethctrl->ETH_MODE = (1 << ETH_MODE_IFMODE) |    // set RMII mode and start EMAC
            (0 << ETH_MODE_RST0);
#endif
#ifdef CONFIG_FM3_ETH_CH1
    ethctrl->ETH_MODE = (1 << ETH_MODE_IFMODE) |    // set RMII mode and reset EMAC
            (1 << ETH_MODE_RST1);
    wait(500000);
    ethctrl->ETH_MODE = (1 << ETH_MODE_IFMODE) |    // set RMII mode and start EMAC
            (0 << ETH_MODE_RST1);
#endif
}

// Get an available TX descriptor
EMAC_DMA_TXDESC *TXDESC_Available(void)
{
    int32_t i = DEF_TXDESC_NUM;
    EMAC_DMA_TXDESC *ptxdesc;
    while (i-- > 0) {
        ptxdesc = (EMAC_DMA_TXDESC *)&txdesc[txdesc_id];
        txdesc_id++;
        if (txdesc_id == DEF_TXDESC_NUM)
            txdesc_id = 0;
        if (ptxdesc->TDES0_f.OWN == 0)
            return ptxdesc;
    }
    return (EMAC_DMA_TXDESC *)NULL;
}

// Get a received RX descriptor
EMAC_DMA_RXDESC *RXDESC_Received(void)
{
    int32_t i;
    for (i=0; i<DEF_RXDESC_NUM; i++) {
        if (rxdesc[i].RDES0_f.OWN == 0) {
            return (EMAC_DMA_RXDESC *)&rxdesc[i];
        }
    }
    return (EMAC_DMA_RXDESC *)NULL;
}

void Packet_Send(uint8_t *p, uint32_t size)
{
    //lcd_printf("PSend: %04X\r\n", size);
    EMAC_DMA_TXDESC *ptxdesc;
    while ((ptxdesc = TXDESC_Available()) == NULL);
    uint8_t *dst = (uint8_t *)ptxdesc->TDES2;
    memcpy(dst, p, size);
    //ptxdesc->TDES0_f.CIC = 3;         // Automatically adding checksum
    ptxdesc->TDES0_f.CIC = 0;           // Not automatically adding checksum
    //ptxdesc->TDES0_f.IC = 1;          //@TX interrupt invoked after sending current frame
    ptxdesc->TDES0_f.FS = 1;            // Buffer includes the first frame
    ptxdesc->TDES0_f.LS = 1;            // Buffer includes the last frame
    ptxdesc->TDES1_f.TBS1 = size;       // Size of TX
    ptxdesc->TDES0_f.OWN = 1;           // Set descriptor to DMA own to start TX operation
    ethmac->OMR_f.ST = 1;               // Set DMA to run state
    ethmac->TPDR = (uint32_t)0;         // Request polling descriptors to restart TX operation
}

#ifdef FM3_ETH_INT
// Ethernet interrupt handler
void ETH_IRQHandler(FM3_ETHERNET_MAC_TypeDef *pethmac)
{
    int32_t i;
    EMAC_DMA_TXDESC *ptxdesc = 0;
    EMAC_DMA_RXDESC *prxdesc = 0;
    uint32_t gmac_isr = pethmac->ISR;
    uint32_t dma_sr = pethmac->SR;
    if ((dma_sr & DMA_SR_RI) != 0) {
        if ((prxdesc = RXDESC_Received()) != NULL) {
            struct eth_packet *ethp = (struct eth_packet *)prxdesc->RDES2;
#ifdef ONLY_DIRECT_AND_BROADCAST
            if ((hal_strncmp_s((const char*)ethp, (const char*)&MACAddr[0], 6) == 0) ||
                (hal_strncmp_s((const char*)ethp, (const char*)&Broadcast[0], 6) == 0))
            {
                FM3_EMAC_LWIP_recv(&g_FM3_EMAC_NetIF);
            } else {
                prxdesc->RDES0_f.OWN = 1;
            }
#else
            FM3_EMAC_LWIP_recv(&g_FM3_EMAC_NetIF);
#endif
            pethmac->SR |= DMA_SR_RI;
        }
    } else
        if ((dma_sr & DMA_SR_TU) != 0) {
            // TU (Transmit Buffer Unavailable)
            // For continuously sending
            // TODO
            pethmac->SR |= (DMA_SR_TU);
            printf("TU\n");
        } else if ((dma_sr & DMA_SR_TI) != 0) {
            // TI TI (Transmit Interrupt)
            // For continuously sending
            // TODO
            //ethmac->SR |= (DMA_SR_TI);
            printf("TI\n");
        } else if ((dma_sr & DMA_SR_ERI) != 0) {
            // ERI (Early Receive Interrupt))
            //  For DMA, indicate DMA receives the first packet
            // ToDo
            //printf("ISR:%08x DSR:%08x\r\n", gmac_isr, dma_sr);
        } else {
            // For debugging
            printf("ISR:%08x DSR:%08x\r\n", gmac_isr, dma_sr);
        }
}

#ifdef CONFIG_FM3_ETH_CH0
void ETH0_IRQHandler(void* param) __attribute__((interrupt));
void ETH0_IRQHandler(void* param)
{
    ETH_IRQHandler(FM3_ETHERNET_MAC0);
}
#endif

#ifdef CONFIG_FM3_ETH_CH1
void ETH1_IRQHandler(void* param) __attribute__((interrupt));
void ETH1_IRQHandler(void* param)
{
    ETH_IRQHandler(FM3_ETHERNET_MAC1);
}
#endif
#endif

// Initialize GPIO pins for Ethernet and set interrupt configuration
void ETHHW_Init(void)
{
    FM3_GPIO->PFR6 &= ~0x0004;
    FM3_GPIO->DDR6 &= ~0x0004;
    FM3_GPIO->PFRC = PFRC_ETH;      // GPIO configuration for CH0 and CH1
    FM3_GPIO->PFRD = PFRD_ETH;      // GPIO configuration for CH0 and CH1
    FM3_GPIO->EPFR14 = (3 << B_E_SPLC) |
                    // (1 << B_E_PSE) |
            (1 << B_E_CKE) |
            (1 << B_E_MD1B) |
            (1 << B_E_MD0B) |
            (1 << B_E_MC1B) |
            (1 << B_E_MC0E) |
            (1 << B_E_TE1E) |
            (1 << B_E_TE0E) |
            (1 << B_E_TD1E) |
            (1 << B_E_TD0E);
}

#ifdef DEBUG_EMAC_REG_DUMP
uint32_t phy_register_mask = 0b00000011111111110000000111111111;

// Dump PHY registers
void PHY_Register_Dump(void)
{
    uint32_t value;
    uint32_t bit_mask = phy_register_mask;
    int32_t i;
    for (i = 0; i < 31; i++) {
        PHY_Read(phy_addr, (uint32_t)i, &value, MII_RD_TOUT);
        if (bit_mask & 0x1)
            printf("Reg %02d: %04x\r\n", i, value);
        bit_mask >>= 1;
    }
}
#endif

static int fm3_eth_init(struct eth_device *dev, bd_t *bd);
static int fm3_eth_send(struct eth_device *dev, volatile void *pkt, int len);
static int fm3_eth_recv(struct eth_device *dev);
static void fm3_eth_halt(struct eth_device *dev);

int fm3_eth_init(struct eth_device *dev, bd_t *bd)
{
    ETHHW_Init();
    DESC_Init();
    ETHCTRL_Init();
    if (DMA_Init() == FALSE)
        return 1;
    if ((uint32_t)GMAC_Init((uint8_t *)&MACAddr[0]) == FALSE)
        return 1;
    return 0;
}

int fm3_eth_send(struct eth_device *dev, volatile void *pkt, int len)
{
    if (len > DEF_TX_BUF_SIZE) {
        printf("Eth Frame truncated\r\n");
        return -EINVAL;
    }
    Packet_Send((uint8_t *)pkt, len);
    //printf("TX:%d\n", len);
    return 0;
}

int fm3_eth_recv(struct eth_device *dev)
{
    EMAC_DMA_RXDESC *prxdesc;
    int32_t recvd = 0;
    while ((prxdesc = RXDESC_Received()) != NULL) {
        recvd = prxdesc->RDES0_f.FL;
        if (recvd > 0)
            NetReceive((void *)prxdesc->RDES2, recvd);
        prxdesc->RDES0_f.OWN = 1;
    }
    return recvd;
}

void fm3_eth_halt(struct eth_device *dev)
{
    // ToDo
}

struct eth_device fm3_netdev;

/*
 * Initialize driver
 */
int fm3_eth_driver_init(bd_t *bd)
{
    int ret = 0;

    sprintf(fm3_netdev.name, "fm3_eth");

    fm3_netdev.init = fm3_eth_init;
    fm3_netdev.halt = fm3_eth_halt;
    fm3_netdev.send = fm3_eth_send;
    fm3_netdev.recv = fm3_eth_recv;

    ret = eth_register(&fm3_netdev);
    return ret;
}
