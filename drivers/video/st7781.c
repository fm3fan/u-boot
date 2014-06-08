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
#include <lcd.h>

#ifdef CONFIG_LCD_ST7781

#define XRES		240
#define YRES		320
#define BIT_PER_PIXEL	16

vidinfo_t panel_info = {
	vl_col:		XRES,
	vl_row:		YRES,
	vl_bpix:	4,
};

void *lcd_base;
void *lcd_console_address;

int lcd_line_length;
int lcd_color_fg;
int lcd_color_bg;

short console_col;
short console_row;

static volatile u8 *st7781_reg = NULL;
static volatile u8 *st7781_val = NULL;

static inline void st7781_write_cmd(u16 index)
{
	*st7781_reg = (u8)(index >> 8);
	*st7781_reg = (u8)(index >> 0);
}

static inline void st7781_write_dat(u16 data)
{
	*st7781_val = (u8)(data >> 8);
	*st7781_val = (u8)(data >> 0);
}

static u16 st7781_read_dat(void)
{
	u16 dat;
	dat = (u16)*st7781_val;
	dat = (dat << 8) | (u16)*st7781_val;
	return dat;
}

static u16 st7781_read_reg(u16 index)
{
	st7781_write_cmd(index);
	return st7781_read_dat();
}

static void st7781_write_reg(u16 index, u16 data)
{
	st7781_write_cmd(index);
	st7781_write_dat(data);
}

void lcd_enable(void)
{
	st7781_write_reg(0x0007, 0x0133);
}

void lcd_disable(void)
{
	st7781_write_reg(0x0007, 0x0000);
}

ulong calc_fbsize(void)
{
#ifdef CONFIG_LCD_ST7781_DOUBLE_BUFFER
	int buffers_cnt = 2;
#else
	int buffers_cnt = 1;
#endif
	return ((panel_info.vl_col * panel_info.vl_row *
		NBITS(panel_info.vl_bpix)) / 8) * buffers_cnt;
}

#ifndef CONFIG_USB_KEYBOARD
int overwrite_console(void)
{
	return 1;
}
#endif

void st7781_update(void)
{
	u16 *b = lcd_base;
	int i;

	st7781_write_reg(0x0020, 0);
	st7781_write_reg(0x0021, 0);
	st7781_write_cmd(0x22);
	for (i = 0; i < XRES * YRES; i++) {
		st7781_write_dat(*b);
		b++;
	}
}

static void lcd_splash(void)
{
	st7781_write_reg(0x0020, 0);
	st7781_write_reg(0x0021, 0);
	st7781_write_cmd(0x22);
    u32 i,n,m;
    u16 color[8]={0xf800,0x07e0,0x001f,0x07ff,0xf81f,0xffe0,0x0000,0xffff};
    for(n=0;n<8;n++)
        for(i=0;i<(XRES/8);i++)
            for(m=0;m<YRES;m++)
            	st7781_write_dat(0xffff);
    udelay(30000);
    for(n=0;n<8;n++)
        for(i=0;i<(XRES/8);i++)
            for(m=0;m<YRES;m++)
            	st7781_write_dat(color[n]);
}

void lcd_setcolreg(u16 regno, u16 red, u16 green, u16 blue)
{
	return;
}

void lcd_ctrl_init(void *lcdbase)
{
	u16 id = 0;
	u32 mem_len = XRES * YRES * BIT_PER_PIXEL / 8;
	st7781_reg = (volatile u8 *)CONFIG_LCD_ST7781_BASE;
	st7781_val = (volatile u8 *)CONFIG_LCD_ST7781_BASE + 2;

	//if (!lcdbase)
	//	return;

	id = st7781_read_reg(0);
//	if (st7781_read_reg(0) != 0x9325)
//		return;

    st7781_write_reg(0x00ff, 0x0001);
    st7781_write_reg(0x00f3, 0x0008);
    //LCD_CtrlRead_ST7781 (0x00f3);         //Read Parameter
    st7781_write_reg(0x0001, 0x0100);  //Output Direct
    st7781_write_reg(0x0002, 0x0700);  //Line Inversion
    st7781_write_reg(0x0003, 0x1030);  //Entry Mode (65K, BGR)
    st7781_write_reg(0x0008, 0x0807);  //Porch Setting
    st7781_write_reg(0x0009, 0x0000);  //Scan Cycle
    st7781_write_reg(0x000A, 0x0000);  //FMARK off
    st7781_write_reg(0x0010, 0x0790);  //Power Control1
    st7781_write_reg(0x0011, 0x0005);  //Power Control2
    st7781_write_reg(0x0012, 0x0000);  //Power Control3
    st7781_write_reg(0x0013, 0x0000);  //Power Control4
    udelay(100000);;                              //Delay 100ms
    st7781_write_reg(0x0010, 0x1490);  //Power Control1
    udelay(50000);                                //Delay 50ms
    st7781_write_reg(0x0011, 0x0227);  //Power Control2
    udelay(50000);                                //Delay 50ms
    st7781_write_reg(0x0012, 0x0089);  //Power Control3
    st7781_write_reg(0x0013, 0x1900);  //Power Control4
    st7781_write_reg(0x0029, 0x0021);  //VCOMH setting
    udelay(50000);                                //Delay 50ms
    st7781_write_reg(0x0030, 0x0000);
    st7781_write_reg(0x0031, 0x0006);
    st7781_write_reg(0x0032, 0x0100);
    st7781_write_reg(0x0035, 0x0001);
    st7781_write_reg(0x0036, 0x0000);
    st7781_write_reg(0x0037, 0x0000);
    st7781_write_reg(0x0038, 0x0406);
    st7781_write_reg(0x0039, 0x0202);
    st7781_write_reg(0x003c, 0x0001);
    st7781_write_reg(0x003d, 0x0000);
    st7781_write_reg(0x0050, 0x0000);  //Horizontal Address Start Position
    st7781_write_reg(0x0051, 0x00ef);  //Horizontal Address End Position
    st7781_write_reg(0x0052, 0x0000);  //Vertical Address Start Position
    st7781_write_reg(0x0053, 0x013f);  //Vertical Address End Position
    st7781_write_reg(0x0060, 0xa700);  //Gate scan control
    st7781_write_reg(0x0061, 0x0001);  //Non-display Area setting
    st7781_write_reg(0x0090, 0x0033);  //RTNI setting
    st7781_write_reg(0x0007, 0x0133);  //Display Control1
    udelay(5000);                                 //Delay 50ms
    st7781_write_cmd(0x0022);

    lcd_splash();
	memset(lcdbase, 0, mem_len);
	st7781_update();
}

void st7781_bitblt(u16 x, u16 y, u16 width, u16 height, u8 *data)
{
    u16 i, j;
    u16 v1, v2;
    u16 *pdata = (u16 *)data;
    for (j = 0; j < height; j ++) {
    	st7781_write_reg(0x0020, x);
    	st7781_write_reg(0x0021, y + j);
        st7781_write_cmd(0x0022);
        for (i = 0; i < width; i ++) {
            st7781_write_dat(*(pdata + i));
        }
        pdata += XRES;
    }
}

void st7781_putc_xyc(u8 c, u8 *font, u16 fwx, u16 fwy, u16 x, u16 y, u16 fgcol, u16 bgcol)
{
    u8 i, j;
    u16 wx, wy, col;

    wx = fwx;
    if (x >= XRES)
    	wx = 0;
    else if (fwx  >= (XRES - x))
    	wx = XRES - x;
    wy = fwy;
    if (y >= YRES)
    	wy = 0;
    else if (fwy  >= (YRES - y))
    	wy = YRES - y;
    if (wx == 0 || wy == 0)
    	return;
    for (j = 0; j < wy; j++) {
    	st7781_write_reg(0x0020, x);
    	st7781_write_reg(0x0021, y + j);
        st7781_write_cmd(0x0022);
        for (i = 0; i < wx; i++) {
            if (font[(u16)c * fwy + j] & (0x80 >> i)) {
                col = fgcol;
            } else {
                col = bgcol;
            }
            st7781_write_dat(col);
        }
    }
}

static u16 line[XRES];

void st7781_scrollup(u16 fwx, u16 fwy, u16 bgcol)
{
    u16 i, j;
    u16 dummy;
    for (j = 0; j < YRES - fwy; j ++) {
    	st7781_write_reg(0x0020, 0);
    	st7781_write_reg(0x0021, j+fwy);
        st7781_write_cmd(0x0022);
        dummy = st7781_read_dat();
        for (i = 0; i < XRES; i ++) {
            line[i] = st7781_read_dat();
        }
    	st7781_write_reg(0x0020, 0);
    	st7781_write_reg(0x0021, j);
        st7781_write_cmd(0x0022);
        for (i = 0; i < XRES; i ++) {
            st7781_write_dat(line[i]);
        }
    }
    for (j = YRES - fwy; j < YRES; j ++) {
    	st7781_write_reg(0x0020, 0);
    	st7781_write_reg(0x0021, j);
        st7781_write_cmd(0x0022);
        for (i = 0; i < XRES; i ++) {
            st7781_write_dat(bgcol);
        }
    }

}
#endif /* CONFIG_LCD_ST7781 */
