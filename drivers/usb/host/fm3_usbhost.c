/*
 * (C) Copyright 2013/11, Kentaro Sekimoto.
 *
 * This code is based on u-boot for sl811hs chip, source at
 * drivers/usb/host/sl811-hdc.c:
 *
 * FM3 USB HOST Interface driver for USB.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

//#define FM3_USB_DEBUG
//#define FM3_USB_DEBUG_HUB
//#define FM3_USB_DEBUG_PACKET

#include <common.h>
#include <usb.h>
#include <asm/arch-fm3/mb9bf618t.h>		/* only for indexing on MINGW */
#include "fm3_usbhost.h"

#ifdef FM3_USB_DEBUG
#define dprintk(fmt, ...)	printf("%s(): "fmt, __func__, ## __VA_ARGS__)
#else
#define	dprintk(fmt, ...)
#endif

//#define BUSRESET_INT
#ifdef BUSRESET_INT
static volatile int f_busreset_done = 0;
#endif
static volatile int transfer_speed=1;
static volatile int transfer_busy = 0;
static int root_hub_devnum = 0;
static struct usb_port_status rh_status = { 0 };/* root hub port status */

/*
 * FM3 Virtual Root Hub
 */

/* Device descriptor */
static __u8 FM3_rh_dev_des[] =
{
    0x12,       /*  __u8  bLength; */
    0x01,       /*  __u8  bDescriptorType; Device */
    0x10,       /*  __u16 bcdUSB; v1.1 */
    0x01,
    0x09,       /*  __u8  bDeviceClass; HUB_CLASSCODE */
    0x00,       /*  __u8  bDeviceSubClass; */
    0x00,       /*  __u8  bDeviceProtocol; */
    0x08,       /*  __u8  bMaxPacketSize0; 8 Bytes */
    0x00,       /*  __u16 idVendor; */
    0x00,
    0x00,       /*  __u16 idProduct; */
    0x00,
    0x00,       /*  __u16 bcdDevice; */
    0x00,
    0x00,       /*  __u8  iManufacturer; */
    0x02,       /*  __u8  iProduct; */
    0x01,       /*  __u8  iSerialNumber; */
    0x01        /*  __u8  bNumConfigurations; */
};

/* Configuration descriptor */
static __u8 FM3_rh_config_des[] =
{
    0x09,       /*  __u8  bLength; */
    0x02,       /*  __u8  bDescriptorType; Configuration */
    0x19,       /*  __u16 wTotalLength; */
    0x00,
    0x01,       /*  __u8  bNumInterfaces; */
    0x01,       /*  __u8  bConfigurationValue; */
    0x00,       /*  __u8  iConfiguration; */
    0x40,       /*  __u8  bmAttributes;
            Bit 7: Bus-powered, 6: Self-powered, 5 Remote-wakwup,
            4..0: resvd */
    0x00,       /*  __u8  MaxPower; */

    /* interface */
    0x09,       /*  __u8  if_bLength; */
    0x04,       /*  __u8  if_bDescriptorType; Interface */
    0x00,       /*  __u8  if_bInterfaceNumber; */
    0x00,       /*  __u8  if_bAlternateSetting; */
    0x01,       /*  __u8  if_bNumEndpoints; */
    0x09,       /*  __u8  if_bInterfaceClass; HUB_CLASSCODE */
    0x00,       /*  __u8  if_bInterfaceSubClass; */
    0x00,       /*  __u8  if_bInterfaceProtocol; */
    0x00,       /*  __u8  if_iInterface; */

    /* endpoint */
    0x07,       /*  __u8  ep_bLength; */
    0x05,       /*  __u8  ep_bDescriptorType; Endpoint */
    0x81,       /*  __u8  ep_bEndpointAddress; IN Endpoint 1 */
    0x03,       /*  __u8  ep_bmAttributes; Interrupt */
    0x08,       /*  __u16 ep_wMaxPacketSize; */
    0x00,
    0xff        /*  __u8  ep_bInterval; 255 ms */
};

/* root hub class descriptor*/
static __u8 FM3_rh_hub_des[] =
{
    0x09,           /*  __u8  bLength; */
    0x29,           /*  __u8  bDescriptorType; Hub-descriptor */
    0x01,           /*  __u8  bNbrPorts; */
    0x00,           /* __u16  wHubCharacteristics; */
    0x00,
    0x50,           /*  __u8  bPwrOn2pwrGood; 2ms */
    0x00,           /*  __u8  bHubContrCurrent; 0 mA */
    0xfc,           /*  __u8  DeviceRemovable; *** 7 Ports max *** */
    0xff            /*  __u8  PortPwrCtrlMask; *** 7 ports max *** */
};

static int ascii2utf (char *s, __u8 *utf, int utfmax)
{
    int retval;

    for (retval = 0; *s && utfmax > 1; utfmax -= 2, retval += 2) {
        *utf++ = *s++;
        *utf++ = 0;
    }
    return retval;
}

static int usb_root_hub_string (int id, int serial, char *type, __u8 *data, int len)
{
    char buf [30];

    /* assert (len > (2 * (sizeof (buf) + 1)));
       assert (strlen (type) <= 8);*/

    /* language ids */
    if (id == 0) {
        *data++ = 4; *data++ = 3;   /* 4 bytes data */
        *data++ = 0; *data++ = 0;   /* some language id */
        return 4;

    /* serial number */
    } else if (id == 1) {
        sprintf (buf, "%#x", serial);

    /* product description */
    } else if (id == 2) {
        sprintf (buf, "USB %s Root Hub", type);

    /* id 3 == vendor description */

    /* unsupported IDs --> "stall" */
    } else
        return 0;

    ascii2utf (buf, data + 2, len - 2);
    data [0] = 2 + strlen(buf) * 2;
    data [1] = 3;
    return data [0];
}

void __attribute__ ((interrupt)) fm3_usbh_int(void)
{
#ifdef FM3_USB_DEBUG
    uint8_t hcnt0;
    uint8_t hcnt1;
    uint8_t hirq;
    uint8_t herr;
    uint8_t hstate;
    uint8_t hfcomp;
    uint8_t hrtimer;
    uint8_t hadr;
    uint8_t heof;
    uint8_t hframe;
    uint8_t htoken;

    hcnt0 = FM3_USB1->HCNT0;
    hcnt1 = FM3_USB1->HCNT1;
    hirq = FM3_USB1->HIRQ;
    herr = FM3_USB1->HERR;
    hstate = FM3_USB1->HSTATE;
    hfcomp = FM3_USB1->HFCOMP;
    hrtimer = FM3_USB1->HRTIMER;
    hadr = FM3_USB1->HADR;
    heof = FM3_USB1->HEOF;
    hframe = FM3_USB1->HFRAME;
    htoken = FM3_USB1->HTOKEN;
#endif
    if (bFM3_USB1_HIRQ_CNNIRQ == 1) {
        bFM3_USB1_HIRQ_CNNIRQ = 0;
    }
    if (bFM3_USB1_HIRQ_CMPIRQ == 1) {
        bFM3_USB1_HIRQ_CMPIRQ = 0;
        transfer_busy = 0;
    }
    if (bFM3_USB1_HIRQ_SOFIRQ == 1) {
        bFM3_USB1_HIRQ_SOFIRQ = 0;
    }
#ifdef BUSRESET_INT
    if (bFM3_USB1_HIRQ_URIRQ == 1) {
        bFM3_USB1_HIRQ_URIRQ = 0;
        f_busreset_done = 1;
    }
#endif
}

void fm3_usbh_clock_init(void)
{
    //bFM3_CLK_GATING_CKEN2_USBCK0 = 1;
    bFM3_USBETHERNETCLK_UCCR_UCEN1 = 0;               /* disable USB clock */
    while (bFM3_USBETHERNETCLK_UCCR_UCEN1 != 0) ;     /* wait for USB clock stop */
    bFM3_USBETHERNETCLK_UPCR1_UPLLEN = 0;             /* disable USB-PLL clock */
    bFM3_USBETHERNETCLK_UCCR_UCSEL1 = 1;              /* select PLL macro clock */
    bFM3_USBETHERNETCLK_UPCR1_UPINC = 0;              /* select main clock as input clock */
    /* select clock stabilization time */
    FM3_USBETHERNETCLK->UPCR2 = UPCR2_UPOWT_INIT_VALUE;
    /* USB-PLL=Fin*N/K -> 96MHz=4MHz*24/1 */
    /* USB-PLL clock configuration register(K) initialize */
    FM3_USBETHERNETCLK->UPCR3 = UPCR3_UPLLK_INIT_VALUE;        // 4MHz:0, 16MHz:0
    /* USB-PLL clock configuration register(N) initialize */
    FM3_USBETHERNETCLK->UPCR4 = UPCR4_UPLLN_INIT_VALUE;        // 4MHz:60, 16Mz:24
    /* USB-PLL clock configuration register(N) initialize */
    FM3_USBETHERNETCLK->UPCR5 = UPCR5_UPLLN_INIT_VALUE;        // 4MHz:5,  16Mz:6
    bFM3_USBETHERNETCLK_UPINT_ENR_UPCSE = 0;          /* USB-PLL clock stabilize interrupt disable  */
    bFM3_USBETHERNETCLK_UPCR1_UPLLEN = 1;             /* enable USB-PLL clock */
    while (bFM3_USBETHERNETCLK_UP_STR_UPRDY == 0) ;   /* wait for USB-PLL clock ready */
    bFM3_USBETHERNETCLK_UCCR_UCEN1 = 1;               /* enable USB clock */
    /* wait for 5 cycle */
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    bFM3_USBETHERNETCLK_USBEN1_USBEN1 = 1;            /* enable USB controller */
    return;
}

#ifdef FM3_UNUSED
void fm3_usbh_disable_int_sof(void)
{
    bFM3_USB1_HCNT_CMPIRE = 0;
    bFM3_USB1_HCNT_SOFIRE = 0;
}

void fm3_usbh_enable_int_sof(void)
{
    bFM3_USB1_HCNT_CMPIRE = 1;
    bFM3_USB1_HCNT_SOFIRE = 1;
}
#endif

void fm3_usbh_out_set_fifo_size(uint16_t size)
{
    bFM3_USB1_EP2S_BFINI = 1;
    FM3_USB1->EP2C = ((FM3_USB1->EP2C & 0xFF80) | size);
    bFM3_USB1_EP2C_DIR = 1;
    bFM3_USB1_EP2S_BFINI = 0;
}

void fm3_usbh_in_set_fifo_size(uint16_t size)
{
    bFM3_USB1_EP1S_BFINI = 1;
    FM3_USB1->EP1C = ((FM3_USB1->EP1C & 0xFE00) | size);
    bFM3_USB1_EP1C_DIR = 0;
    bFM3_USB1_EP1S_BFINI = 0;
}

static inline void fm3_usbh_clear_fifo(void)
{
    bFM3_USB1_EP2S_BFINI = 1;   // initialize fifo
    bFM3_USB1_EP1S_BFINI = 1;   // initialize fifo
}

static inline void fm3_usbh_set_fifo(void)
{
    bFM3_USB1_EP2S_BFINI = 0;   // initialize fifo
    bFM3_USB1_EP1S_BFINI = 0;   // initialize fifo
}

void fm3_usbh_read_fifo(__u8 *buffer, int len)
{
    uint32_t i;
    if (buffer != NULL) {
        for (i = 0; i < len; i++) {
            if ((i & 0x01) == 0x00) {
                *buffer = FM3_USB1->EP1DTL;
            } else {
                *buffer = FM3_USB1->EP1DTH;
            }
            buffer++;
        }
    }
}

void fm3_usbh_write_fifo(__u8 *buffer, int len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        if ((i & 0x01) == 0x00) {
            FM3_USB1->EP2DTL = *buffer;
        } else {
            FM3_USB1->EP2DTH = *buffer;
        }
        buffer++;
    }
}

static inline void fm3_usbh_clear_out_ep_drq(void)
{
    bFM3_USB1_EP2S_DRQ = 0;
}

static inline void fm3_usbh_clear_in_ep_drq(void)
{
    bFM3_USB1_EP1S_DRQ = 0;
}

static void fm3_usbh_set_token(uint8_t addr, uint8_t toggle, uint8_t token, uint8_t num)
{
    uint8_t reg;
    reg = (token << 4) | (num & 0x0f);
    if (toggle)
        reg |= 0x80;
    else
        reg &= ~0x80;
    FM3_USB1->HADR = addr;
    FM3_USB1->HTOKEN = reg;
}

static inline void fm3_usbh_disable_all_int(void)
{
    bFM3_USB1_HCNT_RWKIRE = 0;  /* remote wake up interrupt */
    bFM3_USB1_HCNT_URIRE = 0;   /* USB bus reset interrupt */
    bFM3_USB1_HCNT_CMPIRE = 0;  /* completion interrupt */
    bFM3_USB1_HCNT_CNNIRE = 0;  /* connection interrupt */
    bFM3_USB1_HCNT_DIRE = 0;    /* disconnection interrupt */
    bFM3_USB1_HCNT_SOFIRE = 0;  /* SOF interrupt */
}

/* The function works only if interrupt is disabled. */
static int fm3_usbh_is_device_connected(void)
{
    int timeout = FM3_USBH_TIMEOUT;
    while (bFM3_USB1_HIRQ_CNNIRQ) {
        if (timeout-- <= 0) {
            return 0;
        }
    }
    return 1;
}

static void fm3_usbh_set_tmode(int fmessage)
{
    bFM3_USB1_UDCC_RST = 1;
    if (bFM3_USB1_HSTATE_TMODE == 1) {
        transfer_speed = 1;
        bFM3_USB1_HSTATE_ALIVE = 0;
        if (fmessage)
        	printf("FULL SPEED detected\n");
    } else {
        transfer_speed = 0;
        bFM3_USB1_HSTATE_ALIVE = 1;
        if (fmessage)
        	printf("LOW SPEED detected\n");
    }
    bFM3_USB1_HSTATE_CLKSEL = transfer_speed;
    while (bFM3_USB1_HSTATE_CLKSEL != transfer_speed) ;
    bFM3_USB1_UDCC_RST = 0;
}

static void fm3_usbh_force_tmode(int mode, int fmessage)
{
    bFM3_USB1_UDCC_RST = 1;
    if (mode == 1) {
        transfer_speed = 1;
        bFM3_USB1_HSTATE_ALIVE = 0;
        if (fmessage)
        	printf("Changed to FULL SPEED\n");
    } else {
        transfer_speed = 0;
        bFM3_USB1_HSTATE_ALIVE = 1;
        if (fmessage)
        	printf("Changed to LOW SPEED\n");
    }
    bFM3_USB1_HSTATE_CLKSEL = transfer_speed;
    while (bFM3_USB1_HSTATE_CLKSEL != transfer_speed) ;
    bFM3_USB1_UDCC_RST = 0;
    udelay(10000);
    fm3_usbh_set_token(0, 0, USBH_MH_TOKEN_SOF, 0);
}

static void fm3_usbh_busreset(void)
{
    int timeout;
    bFM3_USB1_HIRQ_CNNIRQ = 0;
    if (bFM3_USB1_HSTATE_CSTAT == 1) {
#ifdef BUSRESET_INT
        f_busreset_done = 0;
        bFM3_USB1_HCNT_URIRE = 1;
        bFM3_USB1_HCNT_URST = 1;
        __enable_irq();
        timeout = FM3_USBH_TIMEOUT * 10;
        while (f_busreset_done == 0) {
            if (timeout-- <= 0)
                break;
        }
        __disable_irq();
        bFM3_USB1_HCNT_URIRE = 0;
#else
        bFM3_USB1_HCNT_URST = 1;
        udelay(100000);
        timeout = FM3_USBH_TIMEOUT*5;
        while (bFM3_USB1_HIRQ_URIRQ == 0) {
            if (timeout-- <= 0) {
                printf("err: busreset timeout\n");
                bFM3_USB1_HCNT_URST = 0;
                break;
            }
        }
        bFM3_USB1_HIRQ_URIRQ = 0;
        //bFM3_USB1_HCNT_URST = 0;
#endif
        udelay(100);
        timeout = FM3_USBH_TIMEOUT;
        while (bFM3_USB1_HIRQ_CNNIRQ != 1) {
            if (timeout-- <= 0) {
                printf("err: connection timeout\n");
                break;
            }
        }
        fm3_usbh_set_tmode(1);
    } else {
    	printf("warn: device not connected\n");
    }
}

int fm3_usbh_init(void)
{
    __disable_irq();
    transfer_busy = 0;
    fm3_usbh_clock_init();
    //FM3_GPIO->PFR8 |= 0x00000003;
    //FM3_GPIO->PFR8 |= 0x0000000C;
    bFM3_GPIO_SPSR_USB1C = 1;
    /* ToDO set USB host interrupt level */
    /* initialize IO port related VBUS */
    //bFM3_GPIO_PDOR6_P62 = (~USBH_VBUS_IO_VALID_LEVEL);
    //bFM3_GPIO_DDR6_P62 = 1;

    bFM3_USB1_HCNT_HOST=0;
    bFM3_USB1_UDCC_HCONX=1;     /* set host mode */
    bFM3_USB1_UDCC_RST = 1;     /* reset USB function */

    bFM3_USB1_EP1C_EPEN = 0;
    bFM3_USB1_EP2C_EPEN = 0;
    fm3_usbh_clear_fifo();
    FM3_USB1->EP1C = 0x4000  | (uint16_t)USBH_MH_PKS_INIT;
    FM3_USB1->EP2C = 0x4000  | (uint16_t)USBH_MH_PKS_INIT;
    bFM3_USB1_EP1C_DIR = 0;  /* endpoint1 for IN-direction transfer */
    bFM3_USB1_EP2C_DIR = 1;  /* endpoint2 for OUT-direction transfer */
    bFM3_USB1_EP1C_EPEN = 1;
    bFM3_USB1_EP2C_EPEN = 1;

    FM3_USB1->HFCOMP = 0;
    FM3_USB1->HRTIMER0 = 0;
    FM3_USB1->HRTIMER1 = 0;
    FM3_USB1->HRTIMER2 = 0;
    bFM3_USB1_HSTATE_CSTAT = 0;
    bFM3_USB1_HSTATE_ALIVE = 0;

    FM3_USB1->HADR = 0;         /* set host address to 0 */
    bFM3_USB1_HCNT_HOST=1;      /* set host mode */
    while (bFM3_USB1_HCNT_HOST != 1) ;

    fm3_usbh_force_tmode(0, 0);

    if (fm3_usbh_is_device_connected()) {
        fm3_usbh_set_tmode(0);
    }
    FM3_USB1->HIRQ = 0;
	FM3_USB1->HEOF = USBH_MH_EOF_TIME;
    bFM3_USB1_HCNT_RETRY = 0;   /* do not retry the operation when an error occurs     */
    fm3_usbh_clear_fifo();
    fm3_usbh_set_fifo();

    bFM3_USB1_UDCC_RST = 0;     /* cancel reset condition of USB function */
    bFM3_USB1_HCNT_SOFSTEP = 1; /* generate an interrupt request during SOF processing */
    bFM3_USB1_HCNT_CANCEL = 0;  /* continue with the token in the EOF area             */

//#ifdef FM3_USBH_USE_INT
    bFM3_USB1_HCNT_RWKIRE = 1;  /* remote wake up interrupt */
    bFM3_USB1_HCNT_CMPIRE = 1;  /* token completion interrupt */
    //bFM3_USB1_HCNT_CNNIRE = 1;  /* connection interrupt of USB device */
    bFM3_USB1_HCNT_DIRE = 0;    /* disconnection interrupt of USB device */
    bFM3_USB1_HCNT_SOFIRE = 1;  /* SOF token interrupt */
    /* ToDo DMA init */
    /* ToDo NVIC init */
//#endif
    NVIC_EnableIRQ(USB1F_USB1H_IRQn);
    //bFM3_GPIO_PDOR6_P62 = USBH_VBUS_IO_VALID_LEVEL;
    udelay(10000);
    fm3_usbh_busreset();
    __enable_irq();
    fm3_usbh_set_token(0, 0, USBH_MH_TOKEN_SOF, 0);
    return 0;
}

int usb_lowlevel_init(void)
{
    root_hub_devnum = 0;
    return fm3_usbh_init();
}

int usb_lowlevel_stop(void)
{
    bFM3_USB1_UDCC_RST = 1;     /* reset USB function */
    //bFM3_GPIO_PDOR6_P62 = (~USBH_VBUS_IO_VALID_LEVEL);
    return 0;
}

int fm3_send_packet(struct usb_device *dev, int addr, unsigned long pipe, int token, __u8 *buffer, int len)
{
    int toggle;
    int ep;
    volatile int timeout;
    int handshake;
    int ret = len;
    int error = 0;

    toggle = usb_gettoggle(dev, usb_pipeendpoint(pipe), usb_pipeout(pipe));
    ep = usb_pipeendpoint(pipe);
    PDEBUG(7, "addr(devnum)=%d(%d) pipe=%0lx ep=%d token=%d len=%3d dir=%ld toggle=%d\n",
    		addr, dev->devnum, pipe, ep, token, len, usb_pipeout(pipe), toggle);
#ifdef FM3_USB_DEBUG_PACKET
    printf("addr(devnum)=%d(%d) pipe=%0lx ep=%d token=%d len=%3d dir=%ld toggle=%d\n",
    		addr, dev->devnum, pipe, ep, token, len, usb_pipeout(pipe), toggle);
#endif
    while (error < FM3_USBH_ERR_MAX) {
    	if (transfer_busy == 0) {
    		bFM3_USB1_HCNT_CMPIRE = 0;
    		bFM3_USB1_HCNT_SOFIRE = 0;
    		if (usb_pipeout(pipe)) {
    			if (len) {
    				fm3_usbh_out_set_fifo_size(len);
    				fm3_usbh_in_set_fifo_size(DEFAULT_MAX_PKS);
    				fm3_usbh_write_fifo(buffer, len);
    			}
    			fm3_usbh_clear_out_ep_drq();
    		} else {
    			fm3_usbh_in_set_fifo_size(usb_maxpacket(dev, pipe));
    			fm3_usbh_out_set_fifo_size(DEFAULT_MAX_PKS);
    		}
    		fm3_usbh_set_token(addr, toggle, token, ep);
    		transfer_busy = 1;
    		bFM3_USB1_HCNT_CMPIRE = 1;
    		bFM3_USB1_HCNT_SOFIRE = 1;
    		timeout = FM3_USBH_TIMEOUT;
    		while (transfer_busy == 1) {
    			if (timeout-- == 0) {
    				printf("Err: tx timeout\n");
    				transfer_busy = 0;
    				break;
    			}
    		}
    	} else {
            udelay(100);
    		//fm3_usbh_wait(10000);
    		error++;
    		continue;
    	}
    	//if (timeout == 0)
    	//	continue;
#if 0
        if (bFM3_USB1_HERR_LSTOF == 1) {
            FM3_USB1->HERR = 0x03;  // clear error;
            continue;
        }
        if (bFM3_USB1_HERR_TOUT == 1) {
            FM3_USB1->HERR = 0x03;  // clear error;
            break;
        }
        if (bFM3_USB1_HERR_RERR == 1) {
            FM3_USB1->HERR = 0x03;  // clear error;
            continue;
        }
#endif
        handshake = FM3_USB1->HERR;
        handshake &= 0x03;
        FM3_USB1->HERR = 0x03;  // clear error
        if (handshake == USBH_MH_HANDSHAKE_ACK) {
            if (usb_pipein(pipe)) {
                len = FM3_USB1->EP1S & 0x01FF;
                if (len) {
                	fm3_usbh_read_fifo(buffer, len);
                	fm3_usbh_clear_in_ep_drq();
                }
                ret = len;
            } else
                ret = len;
            break;
        } else if (handshake == USBH_MH_HANDSHAKE_NAK) {
    		timeout = FM3_USBH_TIMEOUT;
        	while (timeout-- > 0) ;
            ret = -USB_ST_CRC_ERR;
        } else if (handshake == USBH_MH_HANDSHAKE_STALL) {
            ret = -USB_ST_STALLED;
            break;
        } else {
        	PDEBUG(7, "NULL...");
    		timeout = FM3_USBH_TIMEOUT;
        	while (timeout-- > 0) ;
    		transfer_busy = 0;
            ret = -USB_ST_CRC_ERR;
        }
        error++;
    }
    if (error == FM3_USBH_ERR_MAX) {
        printf("Err: packet timeout\n");
        ret = -USB_ST_CRC_ERR;
    }
//fm3_send_packet_exit:
    return ret;
}

int submit_control_msg(struct usb_device *dev, unsigned long pipe, void *buffer,
            int len, struct devrequest *setup)
{
	int addr;
    int token;
    int done = 0;
#ifdef FM3_USB_DEBUG_HUB
    int devnum = usb_pipedevice(pipe);
#endif
    int ep = usb_pipeendpoint(pipe);
    unsigned long ctrlpipe = usb_sndctrlpipe(dev, ep);
    unsigned long rcvctrlpipe = usb_rcvctrlpipe(dev, ep);
    unsigned long sndctrlpipe = usb_sndctrlpipe(dev, ep);

    dev->status = 0;
    addr = ((int)setup->request == 5) ? 0: dev->devnum;
#ifdef FM3_USB_DEBUG_HUB
    PDEBUG(7, "addr(devnum)=%d(%d) pipe=%0lx ep=%d buf=%0x len=%d rt=%#x req=%#x\n",
        addr, devnum, pipe, ep, buffer, len, (int)setup->requesttype, (int)setup->request);
    if (usb_pipedevice(pipe) == root_hub_devnum)
        return fm3_rh_submit_urb(dev, pipe, buffer, len, setup);
#endif
    usb_settoggle(dev, ep, 1, 0);   /* setup phase start */
    if (fm3_send_packet(dev, addr, ctrlpipe, USBH_MH_TOKEN_SETUP,
            (__u8*)setup, sizeof(*setup)) == sizeof(*setup)) {
        int dir_in = usb_pipein(pipe);
        int max = usb_maxpacket(dev, pipe);
        /* data phase */
        if (dir_in)
            token = USBH_MH_TOKEN_IN;
        else
            token = USBH_MH_TOKEN_OUT;
        usb_settoggle(dev, ep, usb_pipeout(pipe), 1);
        while (done < len) {
            int res = fm3_send_packet(dev, addr, pipe, token,
                    (__u8*)buffer+done, max > len - done ? len - done : max);
            if (res < 0) {
                PDEBUG(0, "status data failed!\n");
                dev->status = -res;
                return 0;
            }
            done += res;
            usb_dotoggle(dev, ep, usb_pipeout(pipe));
            if (dir_in && res < max) /* short packet */
                break;
        }
        /* status phase */
        if (dir_in)
            token = USBH_MH_TOKEN_OUT;
        else
            token = USBH_MH_TOKEN_IN;
        usb_settoggle(dev, ep, !usb_pipeout(pipe), 1);
        if (fm3_send_packet(dev, addr, !dir_in ? rcvctrlpipe : sndctrlpipe, token,
                0, 0) < 0) {
            PDEBUG(0, "status phase failed!\n");
            dev->status = -1;
        }
    } else {
        PDEBUG(0, "setup phase failed!\n");
        dev->status = -1;
    }
    dev->act_len = done;
    return done;
}

int submit_bulk_msg(struct usb_device *dev, unsigned long pipe, void *buffer, int len)
{
    int token;
    int dir_out = usb_pipeout(pipe);
    int ep = usb_pipeendpoint(pipe);
    int max = usb_maxpacket(dev, pipe);
    int done = 0;

    //PDEBUG(7, "addr = %ld pipe = %ld ep=%d buf = %p size = %d dir_out = %d\n",
    //        usb_pipedevice(pipe), pipe, ep, buffer, len, dir_out);
    dev->status = 0;
    if (dir_out)
        token = USBH_MH_TOKEN_OUT;
    else
        token = USBH_MH_TOKEN_IN;
    while (done < len) {
        int res = fm3_send_packet(dev, usb_pipedevice(pipe), pipe, token, (__u8*)buffer+done, max > len - done ? len - done : max);
        if (res < 0) {
            dev->status = -res;
            return res;
        }
        usb_dotoggle(dev, ep, dir_out);
        if (!dir_out && res < max) /* short packet */
            break;
        done += res;
    }
    dev->act_len = done;
    return 0;
}

int submit_int_msg(struct usb_device *dev, unsigned long pipe, void *buffer,
            int len, int interval)
{
    int token;
    int dir_out = usb_pipeout(pipe);
    int ep = usb_pipeendpoint(pipe);
    int max = usb_maxpacket(dev, pipe);
    int done = 0;

    //PDEBUG(7, "addr = %ld pipe = %ld ep=%d buf = %p size = %d dir_out = %d\n",
    //        usb_pipedevice(pipe), pipe, ep, buffer, len, dir_out);
    dev->status = 0;
    if (dir_out)
        token = USBH_MH_TOKEN_OUT;
    else
        token = USBH_MH_TOKEN_IN;
    while (done < len) {
        int res = fm3_send_packet(dev, usb_pipedevice(pipe), pipe, token, (__u8*)buffer+done, max > len - done ? len - done : max);
        if (res < 0) {
            dev->status = -res;
            return res;
        }
        usb_dotoggle(dev, ep, dir_out);
        if (!dir_out && res < max) /* short packet */
            break;
        done += res;
    }
	dev->irq_status = 0;
	dev->irq_act_len = done;
	if (dev->irq_handle)
		dev->irq_handle(dev);
	dev->status = 0;
	dev->act_len = done;
    return 0;
}

int fm3_rh_submit_urb(struct usb_device *usb_dev, unsigned long pipe, void *data, int buf_len, struct devrequest *cmd)
{
    __u8 data_buf[16];
    __u8 *bufp = data_buf;
    int len = 0;
    int status = 0;

    __u16 bmRType_bReq;
    __u16 wValue;
    __u16 wIndex;
    __u16 wLength;

    if (usb_pipeint(pipe)) {
        PDEBUG(0, "interrupt transfer unimplemented!\n");
        return 0;
    }

    bmRType_bReq  = cmd->requesttype | (cmd->request << 8);
    wValue        = le16_to_cpu (cmd->value);
    wIndex        = le16_to_cpu (cmd->index);
    wLength       = le16_to_cpu (cmd->length);

    PDEBUG(5, "submit rh urb, req=%04x val=%04x index=%04x len=%d\n",
           bmRType_bReq, wValue, wIndex, wLength);

    /* Request Destination:
           without flags: Device,
           USB_RECIP_INTERFACE: interface,
           USB_RECIP_ENDPOINT: endpoint,
           USB_TYPE_CLASS means HUB here,
           USB_RECIP_OTHER | USB_TYPE_CLASS  almost ever means HUB_PORT here
    */
    switch (bmRType_bReq) {
    case RH_GET_STATUS:
        *(__u16 *)bufp = cpu_to_le16(1);
        OK(2);

    case RH_GET_STATUS | USB_RECIP_INTERFACE:
        *(__u16 *)bufp = cpu_to_le16(0);
        OK(2);

    case RH_GET_STATUS | USB_RECIP_ENDPOINT:
        *(__u16 *)bufp = cpu_to_le16(0);
        OK(2);

    case RH_GET_STATUS | USB_TYPE_CLASS:
        *(__u32 *)bufp = cpu_to_le32(0);
        OK(4);

    case RH_GET_STATUS | USB_RECIP_OTHER | USB_TYPE_CLASS:
        *(__u32 *)bufp = cpu_to_le32(rh_status.wPortChange<<16 | rh_status.wPortStatus);
        OK(4);

    case RH_CLEAR_FEATURE | USB_RECIP_ENDPOINT:
        switch (wValue) {
        case 1:
            OK(0);
        }
        break;

    case RH_CLEAR_FEATURE | USB_TYPE_CLASS:
        switch (wValue) {
        case C_HUB_LOCAL_POWER:
            OK(0);

        case C_HUB_OVER_CURRENT:
            OK(0);
        }
        break;

    case RH_CLEAR_FEATURE | USB_RECIP_OTHER | USB_TYPE_CLASS:
        switch (wValue) {
        case USB_PORT_FEAT_ENABLE:
            rh_status.wPortStatus &= ~USB_PORT_STAT_ENABLE;
            OK(0);

        case USB_PORT_FEAT_SUSPEND:
            rh_status.wPortStatus &= ~USB_PORT_STAT_SUSPEND;
            OK(0);

        case USB_PORT_FEAT_POWER:
            rh_status.wPortStatus &= ~USB_PORT_STAT_POWER;
            OK(0);

        case USB_PORT_FEAT_C_CONNECTION:
            rh_status.wPortChange &= ~USB_PORT_STAT_C_CONNECTION;
            OK(0);

        case USB_PORT_FEAT_C_ENABLE:
            rh_status.wPortChange &= ~USB_PORT_STAT_C_ENABLE;
            OK(0);

        case USB_PORT_FEAT_C_SUSPEND:
            rh_status.wPortChange &= ~USB_PORT_STAT_C_SUSPEND;
            OK(0);

        case USB_PORT_FEAT_C_OVER_CURRENT:
            rh_status.wPortChange &= ~USB_PORT_STAT_C_OVERCURRENT;
            OK(0);

        case USB_PORT_FEAT_C_RESET:
            rh_status.wPortChange &= ~USB_PORT_STAT_C_RESET;
            OK(0);
        }
        break;

    case RH_SET_FEATURE | USB_RECIP_OTHER | USB_TYPE_CLASS:
        switch (wValue) {
        case USB_PORT_FEAT_SUSPEND:
            rh_status.wPortStatus |= USB_PORT_STAT_SUSPEND;
            OK(0);

        case USB_PORT_FEAT_RESET:
            rh_status.wPortStatus |= USB_PORT_STAT_RESET;
            rh_status.wPortChange = 0;
            rh_status.wPortChange |= USB_PORT_STAT_C_RESET;
            rh_status.wPortStatus &= ~USB_PORT_STAT_RESET;
            rh_status.wPortStatus |= USB_PORT_STAT_ENABLE;
            OK(0);

        case USB_PORT_FEAT_POWER:
            rh_status.wPortStatus |= USB_PORT_STAT_POWER;
            OK(0);

        case USB_PORT_FEAT_ENABLE:
            rh_status.wPortStatus |= USB_PORT_STAT_ENABLE;
            OK(0);
        }
        break;

    case RH_SET_ADDRESS:
        root_hub_devnum = wValue;
        OK(0);

    case RH_GET_DESCRIPTOR:
        switch ((wValue & 0xff00) >> 8) {
        case USB_DT_DEVICE:
            len = sizeof(FM3_rh_dev_des);
            bufp = FM3_rh_dev_des;
            OK(len);

        case USB_DT_CONFIG:
            len = sizeof(FM3_rh_config_des);
            bufp = FM3_rh_config_des;
            OK(len);

        case USB_DT_STRING:
            len = usb_root_hub_string(wValue & 0xff, (int)(long)0,  "FM3_USBH", data, wLength);
            if (len > 0) {
                bufp = data;
                OK(len);
            }

        default:
            status = -32;
        }
        break;

    case RH_GET_DESCRIPTOR | USB_TYPE_CLASS:
        len = sizeof(FM3_rh_hub_des);
        bufp = FM3_rh_hub_des;
        OK(len);

    case RH_GET_CONFIGURATION:
        bufp[0] = 0x01;
        OK(1);

    case RH_SET_CONFIGURATION:
        OK(0);

    default:
        PDEBUG(1, "unsupported root hub command\n");
        status = -32;
    }
    len = min(len, buf_len);
    if (data != bufp)
        memcpy(data, bufp, len);
    PDEBUG(5, "len=%d, status=%d\n", len, status);
    usb_dev->status = status;
    usb_dev->act_len = len;
    return status == 0 ? len : status;
}

#ifdef CONFIG_SYS_USB_EVENT_POLL

#ifdef CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
int overwrite_console (void)
{
	return (1);
}
#endif

void usb_event_poll(void)
{
	struct stdio_dev *dev;
	struct usb_device *usb_kbd_dev;
	struct usb_interface *iface;
	struct usb_endpoint_descriptor *ep;
	int pipe;
	int maxp;

	/* Get the pointer to USB Keyboard device pointer */
	dev = stdio_get_by_name("usbkbd");
	usb_kbd_dev = (struct usb_device *)dev->priv;
	iface = &usb_kbd_dev->config.if_desc[0];
	ep = &iface->ep_desc[0];
	pipe = usb_rcvintpipe(usb_kbd_dev, ep->bEndpointAddress);

	/* Submit a interrupt transfer request */
	maxp = usb_maxpacket(usb_kbd_dev, pipe);
	usb_submit_int_msg(usb_kbd_dev, pipe, &new[0],
			maxp > 8 ? 8 : maxp, ep->bInterval);
}

#endif
