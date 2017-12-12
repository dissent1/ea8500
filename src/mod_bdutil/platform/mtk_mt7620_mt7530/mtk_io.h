/*
 * Copyright (c) 2011,    Cisco Systems, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA 
 */

#ifndef __BDUTIL_MV_IO_H__
#define __BDUTIL_MV_IO_H__


#define RALINK_GPIO_LED_INFINITY        4000

typedef struct {
        int gpio;                       //gpio number (0 ~ 23)
        unsigned int on;                //interval of led on
        unsigned int off;               //interval of led off
        unsigned int blinks;            //number of blinking cycles
        unsigned int rests;             //number of break cycles
        unsigned int times;             //blinking times
} ralink_gpio_led_info;

extern int ralink_gpio_led_set(ralink_gpio_led_info led);

uint32_t inline MTK_READREG(uint32_t reg)
{
	return le32_to_cpu(*(volatile u32 *)(reg));
}

void inline MTK_WRITEREG(uint32_t reg, uint32_t val)
{
	(*(volatile u32 *)(reg))=val;
}

#endif /* __BDUTIL_MV_IO_H__ */
