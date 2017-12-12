/*
 * Copyright (c) 2013,    Belkin International, Inc.
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

#include <common/mvCommon.h>
#include <gpp/mvGpp.h>
#include <gpp/mvGppRegs.h>

extern int  init_gpio(void);
extern void exit_gpio(void);

void gpio_write_bit(u_int32_t pin, u_int32_t val);
int  gpio_read_bit(u_int32_t pin);
void gpioRegSet(u_int32_t regOffs, u_int32_t mask, u_int32_t value);
int gpioPolaritySet(u_int32_t group, u_int32_t mask, u_int32_t value);
u_int32_t gpioPolarityGet(u_int32_t group, u_int32_t mask);
unsigned int mv_gpp_value_real_get(unsigned int gpp_group, unsigned int mask);

void gpio_toggle_bit_high(u_int32_t offset, u_int32_t mask);
void gpio_toggle_bit_low(u_int32_t offset, u_int32_t mask);
void gpio_set_bit_high(u_int32_t offset, u_int32_t mask);
void gpio_set_bit_low(u_int32_t offset, u_int32_t mask);

/* Marvell GPIO API struct */
#define MV_GPIO_CHIP_LABEL	"mv_gpio"

#define MV_GPIO_PORT_REGS_OFFSET	0x10100

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

#define GPP_CONTROL_CLEAR_REG(grp)       (MV_GPP_REGS_BASE(grp) + 0x2c)
#define GPP_BLINK_CTR_A_ON_DUR_REG(grp)  (MV_GPP_REGS_BASE(grp) + 0x80)
#define GPP_BLINK_CTR_A_OFF_DUR_REG(grp) (MV_GPP_REGS_BASE(grp) + 0x84)
	
typedef struct {
	int pin;		/* gpio pin */
	int value;		/* gpio pin value 0 or 1 */
} bdgpio_t;

/*
 * BEGIN
 * copied with GPL2 permission from Marvell LSP
 * file: arch/arm/plat-armada/mv_drivers_lsp/mv_btns/btns_driver.c
 *
 * Copyright included below:
 */
/*******************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
*******************************************************************************/
/* MACROS */
#define GPP_GROUP(gpp)  gpp/32
#define GPP_ID(gpp)     gpp%32
#define GPP_BIT(gpp)    0x1 << GPP_ID(gpp)
/*
 * copied with GPL2 permission from Marvell LSP
 * file: arch/arm/plat-armada/mv_drivers_lsp/mv_btns/btns_driver.c
 * END
 */

#endif /* __BDUTIL_GPIO_H__ */
