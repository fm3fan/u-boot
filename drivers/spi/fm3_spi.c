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
#include <malloc.h>
#include <spi.h>

#include <asm/arch-fm3/fm3.h>

#define DEF_BAUDRATE    4000    // 1MHz, unit=KHz
#define CONFIG_FM3_SPI_IDLE_VAL 0xff

struct fm3_spi_slave {
    struct spi_slave slave;
    u32 baud;
    u32 flag;
};

#define to_fm3_spi_slave(s) container_of(s, struct fm3_spi_slave, slave)

__attribute__((weak))
void spi_cs_activate(struct spi_slave *slave)
{
    struct fm3_spi_slave *fss = to_fm3_spi_slave(slave);
    switch (slave->cs) {
    case 0x39:
        FM3_GPIO->PDOR3_f.P9 = 0;
        break;
    default:
        FM3_GPIO->PDOR4_f.P0 = 0;
        break;
    }
}

__attribute__((weak))
void spi_cs_deactivate(struct spi_slave *slave)
{
    struct fm3_spi_slave *fss = to_fm3_spi_slave(slave);
    switch (slave->cs) {
    case 0x39:
        FM3_GPIO->PDOR3_f.P9 = 1;
        break;
    default:
        FM3_GPIO->PDOR4_f.P0 = 1;
        break;
    }
}

void spi_init()
{
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs, unsigned int max_hz, unsigned int mode)
{
    struct fm3_spi_slave *fss;
    fss = malloc(sizeof(*fss));
    if (!fss)
        return NULL;
    fss->slave.bus = bus;
    fss->slave.cs = cs;
    fss->baud = max_hz;
    fss->flag = 0;
    return &fss->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
    struct fm3_spi_slave *fss = to_fm3_spi_slave(slave);
    free(fss);
}

static void spi_portmux(struct spi_slave *slave)
{
    struct fm3_spi_slave *fss = to_fm3_spi_slave(slave);
    switch (slave->bus) {
    case FM3_SPI_4_2:
#ifdef MB9BF506N
    // channel 4-2
    FM3_GPIO->PFR0 |= 0x00E0;       // SIN4_2, SOT4_2, SCK4_2
    FM3_GPIO->EPFR08 |= 0x000003F0; // SIN4_2, SOT4_2, SCK4_2
#endif
        break;
    default:
        break;
    }
    switch (slave->cs) {
    case 0x39:
        FM3_GPIO->PFR3 &= 0xFDFF;   // P39 GPIO
        FM3_GPIO->DDR3 |= 0x0200;   // P39 output
        break;
    default:
#ifdef MB9BF506N
    FM3_GPIO->PFR4 &= 0xFFFE;   // A22 P40 GPIO
    FM3_GPIO->DDR4 |= 0x0001;   // A22 P40 output
#endif
        break;
    }
}

FM3_MFS_CSIO_TypeDef *FM3_SPI[] = {
    (FM3_MFS_CSIO_TypeDef *)FM3_MFS0_CSIO,
    (FM3_MFS_CSIO_TypeDef *)FM3_MFS1_CSIO,
    (FM3_MFS_CSIO_TypeDef *)FM3_MFS2_CSIO,
    (FM3_MFS_CSIO_TypeDef *)FM3_MFS3_CSIO,
    (FM3_MFS_CSIO_TypeDef *)FM3_MFS4_CSIO,
    (FM3_MFS_CSIO_TypeDef *)FM3_MFS5_CSIO,
    (FM3_MFS_CSIO_TypeDef *)FM3_MFS6_CSIO,
    (FM3_MFS_CSIO_TypeDef *)FM3_MFS7_CSIO
};

int spi_claim_bus(struct spi_slave *slave)
{
    FM3_MFS_CSIO_TypeDef *spi;
    struct fm3_spi_slave *fss = to_fm3_spi_slave(slave);

    debug("%s: bus:%i cs:%i\n", __func__, slave->bus, slave->cs);
    spi_portmux(slave);
    spi = FM3_SPI[slave->bus/3];
    spi->SCR = SCR_UPCL;        // Initialize
    spi->SMR |= (SMR_MD_SIO     // SPI mode
                | SMR_SCINV     // Down edge(transfer)/Up edge(receive)
                | SMR_BDS       // MBS first
                | SMR_SCKE      // CLK enable
                | SMR_SOE);     // Data output enable
    uint16_t bgr = (uint16_t)((SysFrePCLK2 / (DEF_BAUDRATE * 1000)) - 1);
    spi->BGR = bgr;
    spi->ESCR = (ESCR_DATABITS_8);  // 8 bit
    spi->SCR |= (SCR_SPI |      // SPI mode
                SCR_RXE |       // Receive enable
                SCR_TXE);       // Transfer enable
    return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
    struct fm3_spi_slave *fss = to_fm3_spi_slave(slave);
    debug("%s: bus:%i cs:%i\n", __func__, slave->bus, slave->cs);
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
        void *din, unsigned long flags)
{
    FM3_MFS_CSIO_TypeDef *spi;
    struct fm3_spi_slave *bss = to_fm3_spi_slave(slave);
    const u8 *tx = dout;
    u8 *rx = din;
    uint bytes = bitlen / 8;
    int ret = 0;

    debug("%s: bus:%i cs:%i bitlen:%i bytes:%i flags:%lx\n", __func__,
        slave->bus, slave->cs, bitlen, bytes, flags);
    if (bitlen == 0)
    if (bitlen % 8) {
        flags |= SPI_XFER_END;
        goto done;
    }
    spi = FM3_SPI[slave->bus/3];
    if (flags & SPI_XFER_BEGIN)
        spi_cs_activate(slave);
    while (bytes--) {
        u8 value = (tx ? *tx++ : CONFIG_FM3_SPI_IDLE_VAL);
        debug("%s: tx:%x ", __func__, value);
        while ((spi->SSR & SSR_TDRE) == 0) {
            if (ctrlc()) {
                ret = -1;
                goto done;
            }
        }
        spi->TDR = value;
        while ((spi->SSR & SSR_RDRF) == 0) {
            if (ctrlc()) {
                ret = -1;
                goto done;
            }
        }
        value = (uint8_t)(spi->RDR);

        if (rx)
            *rx++ = value;
        debug("rx:%x\n", value);
    }
 done:
    if (flags & SPI_XFER_END)
        spi_cs_deactivate(slave);
    return ret;
}
