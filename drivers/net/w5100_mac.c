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
#include <w5100_mac.h>
#include <net.h>
#include <netdev.h>
#include <asm/errno.h>
#include <spi.h>

#ifdef CONFIG_DRIVER_W5100

//#define DEBUG_W5100
#define DEF_BAUDRATE    8000000	// 8MHz, unit=Hz

#if defined(MB9BF506N)
#define W5100_BUS   14      // CH4_2 -> 3*4+2
#define W5100_CS    0x39    // P39
#elif defined(MB9BF568R)
#define W5100_BUS   21      // CH7_0 -> 3*7+0
#define W5100_CS    0x74    // P74
#else

#endif

#ifndef TOWORD
#define TOWORD(h,l) ((((unsigned short)h) << 8) + (((unsigned short)l) & 0xff))
#endif

#if defined(CONFIG_CMD_NET)

//#undef DEBUG

#ifdef DEBUG_W5100
    #define DPRINTF(args...) printf(args)
#else
    #define DPRINTF(args...) { }
#endif /* DEBUG_W5100 */

static void wait(volatile int count)
{
	while (count-- > 0);
}

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

unsigned char w5100_spi_read(unsigned short addr)
{
    unsigned char cmd[4];
    unsigned char data[4];
    cmd[0]= (addr & 0xff00) >> 8;
    cmd[1] = addr & 0x00ff;
    cmd[2] = 0x0f;
    cmd[3] = 0xff;
    unsigned char idata;
    spi_cs_activate(spi);
    spi_xfer(spi, 8*4, (const void *)&cmd, &data, 0);
    spi_cs_deactivate(spi);
    return data[3];
}

void w5100_spi_write(unsigned short addr, unsigned char data)
{
    unsigned char cmd[4];
    cmd[0] = (addr & 0xff00) >> 8;
    cmd[1] = addr & 0x00ff;
    cmd[2] = 0xf0;
    cmd[3] = data;
    spi_cs_activate(spi);
    spi_xfer(spi, 8*4, (const void *)&cmd, NULL, 0);
    spi_cs_deactivate(spi);
}

#if 0
static u16 swap16(u16 x)
{
	return ((((x) & 0x00ff) << 8) | ( (x) >> 8));
}
#endif

/* packet page register access functions */
static void eth_reset (void)
{
	w5100_spi_write(MR, MR_RST);
	/* wait for 200ms */
	udelay (200000);
	/* Wait until the chip is reset */
}

#define SOCK_MACRAW        0x42 // mac raw mode socket

static void eth_reginit (void)
{
    int i;

    w5100_spi_write(RMSR, 0x03);  // use only socket 0
    w5100_spi_write(TMSR, 0x03);  // use only socket 0

    // channel 0 : MACRAW mode
    i = 0;
MACRAW:
    if(i++ > 10000) {
        DPRINTF("MACRAW open failed...");
        return;
    }
    w5100_spi_write(S0_BASE + Sn_MR, Sn_MR_MACRAW);
    w5100_spi_write(S0_BASE + Sn_CR, Sn_CR_OPEN);
    wait(100000);
    //udelay(10000);      // 10ms
    //DPRINTF("Sn_SSR = 0x%04x\n", IINCHIP_READ(Sn_SSR(0)));
    if((w5100_spi_read(S0_BASE + Sn_SR) & 0xff) != SOCK_MACRAW) {
        w5100_spi_write(S0_BASE + Sn_CR, Sn_CR_CLOSE);
        goto MACRAW;
    }
}

static void _eth_halt (void)
{
	w5100_spi_write(MR, MR_RST);
}

char MACAddr[6] = {0x00, 0x22, 0x33, 0x44, 0x55, 0x44};

static int _eth_init (bd_t * bd)
{
	IPaddr_t ip;

	eth_reset();
	/* MR, IMR, ICFGR */
	w5100_spi_write(MR, 0x0);
	w5100_spi_write(IMR, 0x0);	// no interrupt
	/* set the ethernet address */
    w5100_spi_write(SHAR0, MACAddr[0]);
    w5100_spi_write(SHAR1, MACAddr[1]);
    w5100_spi_write(SHAR2, MACAddr[2]);
    w5100_spi_write(SHAR3, MACAddr[3]);
    w5100_spi_write(SHAR4, MACAddr[4]);
    w5100_spi_write(SHAR5, MACAddr[5]);
	/* set the IP,Gateway,Subnet */
	ip = bd->bi_ip_addr;
	ip = ntohl(ip);
	w5100_spi_write(SIPR0, (unsigned char)ip>>24);
    w5100_spi_write(SIPR1, (unsigned char)ip>>16);
    w5100_spi_write(SIPR2, (unsigned char)ip>>8);
    w5100_spi_write(SIPR3, (unsigned char)ip&0xff);
	ip = getenv_IPaddr ("netmask");
	ip = ntohl(ip);
    w5100_spi_write(SUBR0, (unsigned char)ip>>24);
    w5100_spi_write(SUBR1, (unsigned char)ip>>16);
    w5100_spi_write(SUBR2, (unsigned char)ip>>8);
    w5100_spi_write(SUBR3, (unsigned char)ip&0xff);
	ip = getenv_IPaddr ("gatewayip");
	ip = ntohl(ip);
    w5100_spi_write(GAR0, (unsigned char)ip>>24);
    w5100_spi_write(GAR1, (unsigned char)ip>>16);
    w5100_spi_write(GAR2, (unsigned char)ip>>8);
    w5100_spi_write(GAR3, (unsigned char)ip&0xff);
    eth_reginit ();
	return 0;
}

void set_S0TxFsr(unsigned short data)
{
    w5100_spi_write(S0_BASE + Sn_TX_FSR0, (unsigned char)(data >> 8));
    w5100_spi_write(S0_BASE + Sn_TX_FSR1, (unsigned char)(data & 0xff));
}

void set_S0TxWr(unsigned short data)
{
    w5100_spi_write(S0_BASE + Sn_TX_WR0, (unsigned char)(data >> 8));
    w5100_spi_write(S0_BASE + Sn_TX_WR1, (unsigned char)(data & 0xff));
}

void set_S0RxRd(unsigned short data)
{
    w5100_spi_write(S0_BASE + Sn_RX_RD0, (unsigned char)(data >> 8));
    w5100_spi_write(S0_BASE + Sn_RX_RD1, (unsigned char)(data & 0xff));
}

unsigned short get_S0TxFsr(void)
{
    return TOWORD(w5100_spi_read(S0_BASE + Sn_TX_FSR0), w5100_spi_read(S0_BASE + Sn_TX_FSR1));
}

unsigned short get_S0TxWr(void)
{
    return TOWORD(w5100_spi_read(S0_BASE + Sn_TX_WR0), w5100_spi_read(S0_BASE + Sn_TX_WR1));
}

unsigned short get_S0Rsr(void)
{
    return TOWORD(w5100_spi_read(S0_BASE + Sn_RX_RSR0), w5100_spi_read(S0_BASE + Sn_RX_RSR1));
}

unsigned short get_S0Rd(void)
{
    return TOWORD(w5100_spi_read(S0_BASE + Sn_RX_RD0), w5100_spi_read(S0_BASE + Sn_RX_RD1));
}

void writebuf(unsigned short offset, unsigned char *packet, int length)
{
    int i;
    for (i = 0; i < length; i++) {
        w5100_spi_write(W5100_TXBUF + (offset & 0x1fff), *packet++);
        offset++;
    }
}

void readbuf(unsigned short offset, unsigned char *packet, int length)
{
    int i;
    for (i = 0; i < length; i++) {
        *packet++ = w5100_spi_read(W5100_RXBUF + (offset & 0x1fff));
        offset++;
    }
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
	readbuf(offset, buf, 2);
	length = TOWORD(buf[0], buf[1]) - 2;
	readbuf(offset+2, addr, length);
    DPRINTF("RX off:%04x len:%04x\n", offset, length);
    set_S0RxRd(offset + length + 2);
	w5100_spi_write(S0_BASE + Sn_CR, Sn_CR_RECV);
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
	writebuf(offset, (unsigned char *)packet, length);
    set_S0TxWr(offset + (unsigned short)length);
    w5100_spi_write(S0_BASE + Sn_CR, Sn_CR_SEND);
    DPRINTF("TX off:%04x len:%04x\n", offset, length);
#define CFG_HZ 1000
    tmo = get_timer (0) + 5 * CFG_HZ;
    while ((s = w5100_spi_read(S0_BASE + Sn_IR) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK) {
        if (get_timer (0) >= tmo) {
            DPRINTF("TX NG\n");
            return 0;
        }
    }
    w5100_spi_write(S0_BASE + Sn_IR, Sn_IR_SEND_OK);
	return 0;
}

static int w5100_eth_init(struct eth_device *dev, bd_t *bd);
static int w5100_eth_send(struct eth_device *dev, volatile void *pkt, int len);
static int w5100_eth_recv(struct eth_device *dev);
static void w5100_eth_halt(struct eth_device *dev);

int w5100_eth_init(struct eth_device *dev, bd_t *bd)
{
    return _eth_init(bd);
}

int w5100_eth_send(struct eth_device *dev, volatile void *pkt, int len)
{
    return _eth_send (pkt, len);
}

int w5100_eth_recv(struct eth_device *dev)
{
    return _eth_rx();
}

void w5100_eth_halt(struct eth_device *dev)
{
    // ToDo
    _eth_halt();
}

struct eth_device w5100_netdev;

/*
 * Initialize driver
 */
struct eth_device w5100_netdev;

int w5100_eth_driver_init(bd_t *bd)
{
    int ret = 0;
    unsigned int bus = W5100_BUS;
    unsigned int cs = W5100_CS;
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

    sprintf(w5100_netdev.name, "w5100_eth");

    w5100_netdev.init = w5100_eth_init;
    w5100_netdev.halt = w5100_eth_halt;
    w5100_netdev.send = w5100_eth_send;
    w5100_netdev.recv = w5100_eth_recv;

    ret = eth_register(&w5100_netdev);
    return ret;
}

#endif	/* COMMANDS & CFG_NET */

#endif	/* CONFIG_DRIVER_W5100 */
