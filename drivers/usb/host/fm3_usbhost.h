/*
 * (C) Copyright 2013/11, Kentaro Sekimoto.
 *
 * This code is based on u-boot for sl811hs chip, source at
 * drivers/usb/host/sl811.h:
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

#ifndef FM3_USBHOST_H_
#define FM3_USBHOST_H_

#ifdef CONFIG_USB_KEYBOARD
#include <stdio_dev.h>
extern unsigned char new[];
#endif

#define FM3_USBH_TIMEOUT    100000
#define FM3_USBH_ERR_MAX    10
#define DEFAULT_MAX_PKS	64

#define OK(x)   len = (x); break
#define RH_GET_STATUS           0x0080
#define RH_CLEAR_FEATURE        0x0100
#define RH_SET_FEATURE          0x0300
#define RH_SET_ADDRESS      0x0500
#define RH_GET_DESCRIPTOR   0x0680
#define RH_SET_DESCRIPTOR       0x0700
#define RH_GET_CONFIGURATION    0x0880
#define RH_SET_CONFIGURATION    0x0900
#define RH_GET_STATE            0x0280
#define RH_GET_INTERFACE        0x0A80
#define RH_SET_INTERFACE        0x0B00
#define RH_SYNC_FRAME           0x0C80

#define USBH_EP_DISABLE 0
#define USBH_EP_ENABLE  1
#define USBH_EP_TYPE_BULK   2
#define USBH_EP_TYPE_INT    3
#define USBH_EP_DIR_IN      0
#define USBH_EP_DIR_OUT     1

#define USBH_VBUS_IO_VALID_LEVEL    ((uint8_t)0U)
#define USBH_MH_EP_INIT         ((uint16_t)0x4000)
#define USBH_MH_EOF_TIME        ((uint16_t)0x02C9)
#define USBH_MH_PKS_INIT        ((uint8_t)0x40)
#define USBH_MH_HRTIMER10_INIT  ((uint16_t)0xFFFF)
#define USBH_MH_HRTIMER2_INIT   ((uint8_t)0x03)
#define USBH_MH_EOF_TIME        ((uint16_t)0x02C9)

#define bFM3_USBETHERNETCLK_UCCR_UCEN0      *((volatile unsigned int*)(0x426C0000UL))
#define bFM3_USBETHERNETCLK_UCCR_UCSEL0     *((volatile unsigned int*)(0x426C0004UL))
#define bFM3_USBETHERNETCLK_UPCR1_UPLLEN    *((volatile unsigned int*)(0x426C0080UL))
#define bFM3_USBETHERNETCLK_UPCR1_UPINC     *((volatile unsigned int*)(0x426C0084UL))
#define bFM3_USBETHERNETCLK_UP_STR_UPRDY    *((volatile unsigned int*)(0x426C0280UL))
#define bFM3_USBETHERNETCLK_UPINT_ENR_UPCSE *((volatile unsigned int*)(0x426C0300UL))
#define bFM3_USBETHERNETCLK_USBEN0_USBEN0   *((volatile unsigned int*)(0x426C0600UL))
#define bFM3_CLK_GATING_CKEN2_USBCK0        *((volatile unsigned int*)(0x42782400UL))

#define UPCR2_UPOWT_INIT_VALUE  (0x07)  /* initial value of UPCR2 register's UPOWT bits    */
#define UPCR3_UPLLK_INIT_VALUE  (0x00)  /* initial value of UPCR3 register's UPLLK bits    */
#define UPCR4_UPLLN_INIT_VALUE  (0x3B)  /* initial value of UPCR4 register's UPLLN bits    */
#define UPCR5_UPLLN_INIT_VALUE  (0x04)  /* initial value of UPCR4 register's UPLLM bits    */

#define USBH_MH_HANDSHAKE_ACK         (0x00)  /* ACK   */
#define USBH_MH_HANDSHAKE_NAK         (0x01)  /* NAK   */
#define USBH_MH_HANDSHAKE_STALL       (0x02)  /* STALL */
#define USBH_MH_HANDSHAKE_NULL        (0x03)  /* NULL  */
/* token type */
#define USBH_MH_TOKEN_SETUP           (1)     /* SETUP token */
#define USBH_MH_TOKEN_IN              (2)     /* IN token    */
#define USBH_MH_TOKEN_OUT             (3)     /* OUT token   */
#define USBH_MH_TOKEN_SOF             (4)     /* SOF token   */

#ifdef FM3_USB_DEBUG
    #define PDEBUG(level, fmt, args...) \
        printf(fmt, ## args)
/*
        if (debug >= (level)) printf("[%s:%d] " fmt, \
        __PRETTY_FUNCTION__, __LINE__ , ## args)
*/
#else
    #define PDEBUG(level, fmt, args...) do {} while(0)
#endif

#endif /* FM3_USBHOST_H_ */
