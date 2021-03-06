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

#ifndef __ST7781_H
#define __ST7781_H

#define NOT_USED_SO_FAR

void st7781_update(void);
void st7781_putc_xyc(u8 c, u8 *font, u16 fwx, u16 fwy, u16 x, u16 y, u16 fgcol, u16 bgcol);

ulong calc_fbsize (void);

#endif /* __ST7781_H */
