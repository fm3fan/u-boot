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

#include <config.h>
#include <common.h>
#include <command.h>
#include <w5500_mac.h>
#include <net.h>
#include <netdev.h>
#include <asm/errno.h>
#include <spi.h>

#ifdef CONFIG_DRIVER_W5500

#define RX_BUF_SIZE	0x4000
#define RX_OFF_MASK (RX_BUF_SIZE - 1)
#define TX_BUF_SIZE 0x4000
#define TX_OFF_MASK (TX_BUF_SIZE - 1)

//#define DEBUG_W5500
//#define DEF_BAUDRATE    8000000	// 8MHz, unit=Hz
#define DEF_BAUDRATE    16000000	// 16MHz, unit=Hz
//#define DEF_BAUDRATE    24000000	// 24MHz, unit=Hz -> NG

#if defined(MB9BF506N)
#define W5500_BUS   14      // CH4_2 -> 3*4+2
#define W5500_CS    0x39    // P39
#elif defined(MB9BF568R)
#define W5500_BUS   21      // CH7_0 -> 3*7+0
#define W5500_CS    0x74    // P74
#else
#endif

#ifndef TOWORD
#define TOWORD(h,l) ((((unsigned short)h) << 8) + (((unsigned short)l) & 0xff))
#endif

#if defined(CONFIG_CMD_NET)

//#undef DEBUG

#ifdef DEBUG_W5500
    #define DPRINTF(args...) printf(args)
#else
    #define DPRINTF(args...) { }
#endif /* DEBUG_W5500 */

#if 0
static void wait(volatile int count)
{
	while (count-- > 0);
}
#endif

void Memory_Dump(unsigned long addr, int len)
{
    int i, j;
    char *p = (char *)addr;
    char *q = (char *)addr;
    for (i = 0; i < len; i++) {
        if ((i & 0xf) == 0) {
            DPRINTF("%08x ", (unsigned long)p);
        }
        DPRINTF("%02x ", *p & 0xff);
        if (((i & 0xf) == 0xf) || (i == (len - 1))) {
            j = 16- (i & 0xf);
            while (j-- > 0)
                DPRINTF("   ");
            for (j = 0; j <= (i & 0xf); j++) {
                if (*q >= 0x20 && *q <= 0x7e) {
                    DPRINTF("%c", *q);
                } else
                    DPRINTF(" ");
                q++;
            }
            DPRINTF("\r\n");
        }
        p++;
    }
    DPRINTF("\r\n");
}

struct spi_slave *spi;

unsigned char w5500_spi_read_creg(unsigned short addr)
{
    unsigned char cmd[4];
    unsigned char data[4];
    cmd[0] = (unsigned char)(addr >> 8);
    cmd[1] = (unsigned char)addr;
    cmd[2] = (0x00 << 3) + (0x00 << 2) + 0x00;
    cmd[3] = 0xff;
    spi_cs_activate(spi);
    spi_xfer(spi, 8*3, (const void *)&cmd, NULL, 0);
    spi_xfer(spi, 8*1, (const void *)NULL, &data, 0);
    spi_cs_deactivate(spi);
    return data[0];
}

unsigned char w5500_spi_read_sreg(unsigned char regno, unsigned short addr)
{
    unsigned char cmd[4];
    unsigned char data[4];
    cmd[0] = (unsigned char)(addr >> 8);
    cmd[1] = (unsigned char)addr;
    cmd[2] = ((regno*4 + 0x01) << 3) + (0x00 << 2) + 0x00;
    cmd[3] = 0xff;
    spi_cs_activate(spi);
    spi_xfer(spi, 8*3, (const void *)&cmd, NULL, 0);
    spi_xfer(spi, 8*1, (const void *)NULL, &data, 0);
    spi_cs_deactivate(spi);
    return data[0];
}

void w5500_spi_readbuf(unsigned short addr, unsigned char *rbuf, unsigned short len)
{
    unsigned char cmd[3];
    cmd[0] = (unsigned char)(addr >> 8);
    cmd[1] = (unsigned char)addr;
    cmd[2] = (0x03 << 3) + (0x00 << 2) + 0x00;
    spi_cs_activate(spi);
    spi_xfer(spi, 8*3, (const void *)&cmd, NULL, 0);
    spi_xfer(spi, 8*len, (const void *)NULL, (void *)rbuf, 0);
    spi_cs_deactivate(spi);
}


void w5500_spi_write_creg(unsigned short addr, unsigned char data)
{
	unsigned char cmd[4];
    cmd[0] = (unsigned char)(addr >> 8);
    cmd[1] = (unsigned char)addr;
	cmd[2] = (0x00 << 3) + (0x01 << 2) + 0x00;
    cmd[3] = data;
    spi_cs_activate(spi);
    spi_xfer(spi, 8*4, (const void *)&cmd, NULL, 0);
    spi_cs_deactivate(spi);
}

void w5500_spi_write_sreg(unsigned char regno, unsigned short addr, unsigned char data)
{
    unsigned char cmd[4];
    cmd[0] = (unsigned char)(addr >> 8);
    cmd[1] = (unsigned char)addr;
    cmd[2] = ((regno*4 + 0x01) << 3) + (0x01 << 2) + 0x00;
    cmd[3] = data;
    spi_cs_activate(spi);
    spi_xfer(spi, 8*4, (const void *)&cmd, NULL, 0);
    spi_cs_deactivate(spi);
}

void w5500_spi_writebuf(unsigned short addr, unsigned char *wbuf, unsigned short len)
{
    unsigned char cmd[3];
    cmd[0] = (unsigned char)(addr >> 8);
    cmd[1] = (unsigned char)addr;
    cmd[2] = (0x02 << 3) + (0x01 << 2) + 0x00;
    spi_cs_activate(spi);
    spi_xfer(spi, 8*3, (const void *)&cmd, NULL, 0);
    spi_xfer(spi, 8*len, (const void *)wbuf, (void *)NULL, 0);
    spi_cs_deactivate(spi);
}

static void eth_reset (void)
{
	int i;
	unsigned char v;

	i = 0;
CHIP_RST_LP:
	if (i++ > 10) {
		DPRINTF("CHIP RESET failed...\n");
		return;
	}
	w5500_spi_write_creg(MR, MR_RST);
	udelay (200000);	/* wait for 200ms */
	if ((w5500_spi_read_creg(MR) & MR_RST) !=  0x00) {
		goto CHIP_RST_LP;
	}
	v = w5500_spi_read_creg(PHYCFGR) | PHYCFGR_OPMDC_ALLA;
	w5500_spi_write_creg(PHYCFGR, v);
	v = w5500_spi_read_creg(PHYCFGR) & PHYCFGR_RST;
	w5500_spi_write_creg(PHYCFGR, v);
	v = w5500_spi_read_creg(PHYCFGR) | ~PHYCFGR_RST;
	w5500_spi_write_creg(PHYCFGR, v);
	i = 0;
PHY_NEG_LP:
    if (i++ > 100) {
    	DPRINTF("PHY NEG failed...\n");
    	return;
    }
	udelay (500000);	/* wait for 500ms */
	if ((w5500_spi_read_creg(PHYCFGR) & 0x07) !=  0x07) {
		goto PHY_NEG_LP;
	}
}

static void eth_reginit (void)
{
    int i;

	w5500_spi_write_creg(IMR, 0x0);	// no interrupt
    w5500_spi_write_sreg(SR0, Sn_RXBUF_SIZE, RX_BUF_SIZE >> 10);
    w5500_spi_write_sreg(SR0, Sn_TXBUF_SIZE, TX_BUF_SIZE >> 10);
    for (i = 1; i < SR_NUM; i++) {
        w5500_spi_write_sreg(i, Sn_RXBUF_SIZE, 0);
        w5500_spi_write_sreg(i, Sn_TXBUF_SIZE, 0);
    }
    // channel 0 : MACRAW mode
    i = 0;
MACRAW:
    if (i++ > 10) {
        DPRINTF("MACRAW open failed...\n");
        return;
    }
    w5500_spi_write_sreg(SR0, Sn_MR, Sn_MR_MACRAW);
    w5500_spi_write_sreg(SR0, Sn_CR, Sn_CR_OPEN);
    udelay(10000);      // 10ms
    //DPRINTF("Sn_SSR = 0x%04x\n", IINCHIP_READ(Sn_SSR(0)));
    if((w5500_spi_read_sreg(SR0, Sn_SR) & Sn_SR_MACRAW) != Sn_SR_MACRAW) {
        w5500_spi_write_sreg(SR0, Sn_CR, Sn_CR_CLOSE);
        goto MACRAW;
    }
}

static void _eth_halt (void)
{
	w5500_spi_write_creg(MR, MR_RST);
}

char MACAddr[6] = {0x00, 0x22, 0x33, 0x44, 0x55, 0x44};

static int _eth_init (bd_t * bd)
{
	IPaddr_t ip;

	eth_reset();
	/* set the ethernet address */
    w5500_spi_write_creg(SHAR0, MACAddr[0]);
    w5500_spi_write_creg(SHAR1, MACAddr[1]);
    w5500_spi_write_creg(SHAR2, MACAddr[2]);
    w5500_spi_write_creg(SHAR3, MACAddr[3]);
    w5500_spi_write_creg(SHAR4, MACAddr[4]);
    w5500_spi_write_creg(SHAR5, MACAddr[5]);
	/* set the IP,Gateway,Subnet */
	ip = bd->bi_ip_addr;
	ip = ntohl(ip);
	w5500_spi_write_creg(SIPR0, (unsigned char)ip>>24);
    w5500_spi_write_creg(SIPR1, (unsigned char)ip>>16);
    w5500_spi_write_creg(SIPR2, (unsigned char)ip>>8);
    w5500_spi_write_creg(SIPR3, (unsigned char)ip&0xff);
	ip = getenv_IPaddr ("netmask");
	ip = ntohl(ip);
    w5500_spi_write_creg(SUBR0, (unsigned char)ip>>24);
    w5500_spi_write_creg(SUBR1, (unsigned char)ip>>16);
    w5500_spi_write_creg(SUBR2, (unsigned char)ip>>8);
    w5500_spi_write_creg(SUBR3, (unsigned char)ip&0xff);
	ip = getenv_IPaddr ("gatewayip");
	ip = ntohl(ip);
    w5500_spi_write_creg(GAR0, (unsigned char)ip>>24);
    w5500_spi_write_creg(GAR1, (unsigned char)ip>>16);
    w5500_spi_write_creg(GAR2, (unsigned char)ip>>8);
    w5500_spi_write_creg(GAR3, (unsigned char)ip&0xff);
    eth_reginit ();
	return 0;
}

void set_S0TxFsr(unsigned short data)
{
    w5500_spi_write_sreg(SR0, Sn_TX_FSR0, (unsigned char)(data >> 8));
    w5500_spi_write_sreg(SR0, Sn_TX_FSR1, (unsigned char)(data & 0xff));
}

void set_S0TxRd(unsigned short data)
{
    w5500_spi_write_sreg(SR0, Sn_TX_RD0, (unsigned char)(data >> 8));
    w5500_spi_write_sreg(SR0, Sn_TX_RD1, (unsigned char)(data & 0xff));
}

void set_S0TxWr(unsigned short data)
{
    w5500_spi_write_sreg(SR0, Sn_TX_WR0, (unsigned char)(data >> 8));
    w5500_spi_write_sreg(SR0, Sn_TX_WR1, (unsigned char)(data & 0xff));
}

void set_S0RxRd(unsigned short data)
{
    w5500_spi_write_sreg(SR0, Sn_RX_RD0, (unsigned char)(data >> 8));
    w5500_spi_write_sreg(SR0, Sn_RX_RD1, (unsigned char)(data & 0xff));
}

void set_S0RxWr(unsigned short data)
{
    w5500_spi_write_sreg(SR0, Sn_RX_WR0, (unsigned char)(data >> 8));
    w5500_spi_write_sreg(SR0, Sn_RX_WR1, (unsigned char)(data & 0xff));
}

unsigned short get_S0TxFsr(void)
{
    return TOWORD(w5500_spi_read_sreg(SR0, Sn_TX_FSR0), w5500_spi_read_sreg(SR0, Sn_TX_FSR1));
}

unsigned short get_S0TxWr(void)
{
    return TOWORD(w5500_spi_read_sreg(SR0, Sn_TX_WR0), w5500_spi_read_sreg(SR0, Sn_TX_WR1));
}

unsigned short get_S0Rsr(void)
{
    return TOWORD(w5500_spi_read_sreg(SR0, Sn_RX_RSR0), w5500_spi_read_sreg(SR0, Sn_RX_RSR1));
}

unsigned short get_S0Rd(void)
{
    return TOWORD(w5500_spi_read_sreg(SR0, Sn_RX_RD0), w5500_spi_read_sreg(SR0, Sn_RX_RD1));
}

/* Get a data block via Ethernet */
static int _eth_rx (void)
{
	unsigned char *addr;
	unsigned short offset;
	unsigned short length;
	unsigned char buf[2];
	unsigned short received_rx_size;

	received_rx_size = get_S0Rsr();
	if (received_rx_size == 0)
	    return 0;
	addr = (unsigned char *)NetRxPackets[0];
	offset = get_S0Rd();
	w5500_spi_readbuf(offset, buf, 2);
	length = TOWORD(buf[0], buf[1]) - 2;
	offset += 2;
	w5500_spi_readbuf(offset, addr, length);
    DPRINTF("RX off:%04x len:%04x\n", offset, length);
    offset += length;
    set_S0RxRd(offset);
	w5500_spi_write_sreg(SR0, Sn_CR, Sn_CR_RECV);
	NetReceive (NetRxPackets[0], (int)length);
	return length;
}

/* Send a data block via Ethernet. */
static int _eth_send (volatile void *packet, int length)
{
    unsigned short offset;
	unsigned short free_tx_size;
    unsigned short s;
    int tmo;

	free_tx_size = get_S0TxFsr();
	if (free_tx_size < length) {
#ifdef DEBUG
		printf("Not enough free size.\n");
#endif
		eth_reset();
		eth_reginit();
		return 0;
	}
	if ((length < 0) || (length > 8192))
	    DPRINTF("eth_send:exceed length!\n");
	//Memory_Dump((unsigned long)packet, length);
	offset = get_S0TxWr();
	w5500_spi_writebuf(offset, (unsigned char *)packet, length);
	offset += (unsigned short)length;
    set_S0TxWr(offset);
	w5500_spi_write_sreg(SR0, Sn_CR, Sn_CR_SEND);
	while (w5500_spi_read_sreg(SR0, Sn_CR));
    DPRINTF("TX off:%04x len:%04x\n", offset, length);
	while (1) {
		unsigned short s = w5500_spi_read_sreg(SR0, Sn_IR);
		if (s & Sn_IR_SEND_OK) {
			w5500_spi_write_sreg(SR0, Sn_IR, Sn_IR_SEND_OK);
			break;
		} else if (s & Sn_IR_TIMEOUT) {
			w5500_spi_write_sreg(SR0, Sn_IR, Sn_IR_TIMEOUT);
			DPRINTF("TX timeout\n");
			break;
		}
	}
#if 0
#define CFG_HZ 1000
    tmo = get_timer (0) + 5 * CFG_HZ;
    while ((s = w5500_spi_read_sreg(SR0, Sn_IR) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK) {
        if (get_timer (0) >= tmo) {
            DPRINTF("TX NG\n");
            return 0;
        }
    }
    w5500_spi_write_sreg(SR0, Sn_IR, Sn_IR_SEND_OK);
#endif
	return 0;
}

static int w5500_eth_init(struct eth_device *dev, bd_t *bd);
static int w5500_eth_send(struct eth_device *dev, volatile void *pkt, int len);
static int w5500_eth_recv(struct eth_device *dev);
static void w5500_eth_halt(struct eth_device *dev);

int w5500_eth_init(struct eth_device *dev, bd_t *bd)
{
    return _eth_init(bd);
}

int w5500_eth_send(struct eth_device *dev, volatile void *pkt, int len)
{
    return _eth_send (pkt, len);
}

int w5500_eth_recv(struct eth_device *dev)
{
    return _eth_rx();
}

void w5500_eth_halt(struct eth_device *dev)
{
    // ToDo
    _eth_halt();
}

struct eth_device w5500_netdev;

struct eth_device w5500_netdev;

int w5500_eth_driver_init(bd_t *bd)
{
    int ret = 0;
    unsigned int bus = W5500_BUS;
    unsigned int cs = W5500_CS;
    unsigned int max_hz = DEF_BAUDRATE;
    unsigned int mode = 0;

    spi = spi_setup_slave(bus, cs, max_hz, 0);
    if (!spi) {
        debug("SF: Failed to set up slave\n");
        return -1;
    }
    ret = spi_claim_bus(spi);
    if (ret) {
        debug("SF: Failed to claim SPI bus: %d\n", ret);
        return -1;
    }

    sprintf(w5500_netdev.name, "w5500_eth");

    w5500_netdev.init = w5500_eth_init;
    w5500_netdev.halt = w5500_eth_halt;
    w5500_netdev.send = w5500_eth_send;
    w5500_netdev.recv = w5500_eth_recv;

    ret = eth_register(&w5500_netdev);
    return ret;
}

#endif	/* COMMANDS & CFG_NET */

#endif	/* CONFIG_DRIVER_W5500 */
