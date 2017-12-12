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
/************************************************************************
  @file gpio.h
  @brief board gpio access routines 
*************************************************************************/

#ifndef __BDUTIL_GPIO_H__
#define __BDUTIL_GPIO_H__

#include <linux/types.h>
#include <asm-generic/ioctl.h>

extern int  init_gpio(void);
extern void exit_gpio(void);

void gpio_write_bit(u_int32_t pin, u_int32_t val);
int  gpio_read_bit(u_int32_t pin);
void gpioRegSet(u_int32_t group, u_int32_t regOffs, u_int32_t mask, u_int32_t value);
int gpioPolaritySet(u_int32_t group, u_int32_t mask, u_int32_t value);
u_int32_t gpioPolarityGet(u_int32_t group, u_int32_t mask);

/* Marvell GPIO API struct */
#define MV_GPIO_CHIP_LABEL	"mv_gpio"

#define MV_GPIO_PORT_REGS_OFFSET	0x10100

#define MV_GPP41		BIT9
#define MV_GPP42		BIT10
#define MV_GPP43		BIT11
#define MV_GPP47		BIT15
#define MV_GPP48		BIT16

#define MPP_16			16
#define MPP_17			17 
#define MPP_41			41
#define MPP_42			42
#define MPP_43			43
#define MPP_47			47
#define MPP_48			48

#define BDGPIO_INPUT_GET	_IOR('g', 1, unsigned int)
#define BDGPIO_INPUT_SET	_IOW('g', 2, unsigned int)
#define BDGPIO_OUTPUT_GET	_IOR('g', 3, unsigned int)
#define BDGPIO_OUTPUT_SET	_IOW('g', 4, unsigned int)
	
typedef struct {
	int pin;		/* gpio pin */
	int value;		/* gpio pin value 0 or 1 */
} bdgpio_t;

#endif /* __BDUTIL_GPIO_H__ */
