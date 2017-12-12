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
  @file gpio.c
  @brief board driver gpio access kernel module
*************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <asm/io.h>
#include <asm-generic/errno-base.h>
#include <linux/gpio.h>

#include <linux_oss/mvOs.h>

#include "../../debug.h"
#include "gpio.h"

#define DEVICE_NAME     "bdutil board utility gpio kernel module"
#define DRV_VERSION     "1.3"

#define GPIO_BASE_ADDR	(INTER_REGS_BASE + MV_GPIO_PORT_REGS_OFFSET)

static int bdgpio_open(struct inode *, struct file *);
static int bdgpio_release(struct inode *, struct file *);
static long bdgpio_unlocked_ioctl(struct file *fp, unsigned int cmd, unsigned long arg);

static void __iomem * gpio_base = NULL;

static int bdutil_gpio_open_count = 0;
static int major = 0;
static DEFINE_MUTEX(bdgpio_lock);

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
unsigned int
mv_gpp_value_real_get(unsigned int gpp_group, unsigned int mask)
{
	unsigned int temp, in_out, gpp_val;
	/* in ->1,  out -> 0 */
	in_out = MV_REG_READ(GPP_DATA_OUT_EN_REG(gpp_group)) & mask;

	gpp_val = MV_REG_READ(GPP_DATA_IN_REG(gpp_group)) & mask;

	/* outputs values */
	temp = (gpp_val & ~in_out);

	/* input */
	temp |= (( gpp_val ^ MV_REG_READ(GPP_DATA_IN_POL_REG(gpp_group)) ) & in_out) & mask;

	return temp;
}
/* 
 * copied with GPL2 permission from Marvell LSP
 * file: arch/arm/plat-armada/mv_drivers_lsp/mv_btns/btns_driver.c
 * END 
 */

void gpioRegSet(u_int32_t regOffs, u_int32_t mask, u_int32_t value)
{
	u_int32_t gppData;

	gppData = MV_REG_READ(regOffs);
	gppData &= ~mask;
	gppData |= (value & mask);
	MV_REG_WRITE(regOffs, gppData);
}

int gpioPolaritySet(u_int32_t group, u_int32_t mask, u_int32_t value)
{
	gpioRegSet(GPP_DATA_IN_POL_REG(group), mask, value);
	return 0;
}

u_int32_t gpioPolarityGet(u_int32_t group, u_int32_t mask)
{
	u_int32_t regVal;
	regVal = MV_REG_READ(GPP_DATA_IN_POL_REG(group));
	return (regVal & mask);
}

void gpio_write_bit(u_int32_t pin, u_int32_t val)
{
	u_int32_t reg_grp, reg_bit, reg_value;

	reg_grp = (pin/32); // 0: LSB, 1: MSB
	reg_bit = (pin % 32);

	reg_value = MV_REG_READ(GPP_DATA_OUT_REG(reg_grp));

	if (val) // high active
		reg_value |= (1 << reg_bit);
	else // low active
		reg_value &= ~(1 << reg_bit);

	MV_REG_WRITE(GPP_DATA_OUT_REG(reg_grp), reg_value);
}

int gpio_read_bit(u_int32_t pin)
{
	u_int32_t reg_grp, reg_bit, reg_value, btn_gpio;

	reg_grp = (pin/32); // 0: LSB, 1: MSB
	reg_bit = (pin % 32);

	reg_value = MV_REG_READ(GPP_DATA_IN_REG(reg_grp));
        DBG("%s:  reg_value = 0x%08x\n", __FUNCTION__, reg_value);
	btn_gpio = reg_value & (1 << reg_bit);

	return ((btn_gpio) ? 1 : 0);
}

int gpio_read_output_bit(u_int32_t pin)
{
	u_int32_t reg_grp, reg_bit, reg_value;

	reg_grp = (pin/32); // 0: LSB, 1: MSB
	reg_bit = (pin % 32);

	reg_value = MV_REG_READ(GPP_DATA_OUT_REG(reg_grp));
        DBG("%s:  reg_value = 0x%08x\n", __FUNCTION__, reg_value);

	return ((reg_value & (1 << reg_bit)) ? 1 : 0);
}

static struct file_operations bdgpio_fops = {
        .owner = THIS_MODULE,
	.open = bdgpio_open,
	.unlocked_ioctl = bdgpio_unlocked_ioctl,
	.release = bdgpio_release
};

static int bdgpio_open(struct inode *inode, struct file *filp)
{
	DBGX(1, "%s: entering, bdutil_gpio_open_count = %d\n", __FUNCTION__, bdutil_gpio_open_count);

	if (bdutil_gpio_open_count >= 1) return -EFAULT;

	bdutil_gpio_open_count = 1;

	return 0;
}

static int bdgpio_release(struct inode *inode, struct file *filp)
{
	DBGX(1, "%s: entering, bdutil_gpio_open_count = %d\n", __FUNCTION__, bdutil_gpio_open_count);

	bdutil_gpio_open_count = 0;

	return 0;
}

static long bdgpio_unlocked_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    bdgpio_t user_bdgpio;
    int rc = 1;

    mutex_lock(&bdgpio_lock);

    switch (cmd)
    {
    case BDGPIO_OUTPUT_GET:
        if (copy_from_user(&user_bdgpio,
                          (bdgpio_t*) arg,
                          sizeof(bdgpio_t)) == 0)
        {
            user_bdgpio.value = gpio_read_output_bit(user_bdgpio.pin);
            if (copy_to_user((void*) arg,
                             (void*) &user_bdgpio,
                             sizeof(bdgpio_t)) == 0)
            {
                /* success */
                break;
            }
        }

        ERR("%s: - bad copy\n", __FUNCTION__);
        rc = -EFAULT;
        break;

    case BDGPIO_OUTPUT_SET:
        if (copy_from_user(&user_bdgpio,
                           (bdgpio_t*) arg,
                           sizeof(bdgpio_t)) == 0)
        {
            gpio_write_bit(user_bdgpio.pin, user_bdgpio.value);
            /* success */
            break;
        }

        ERR("%s: - bad copy\n", __FUNCTION__);
        rc = -EFAULT;
        break;

    default:
	break;
    }

    mutex_unlock(&bdgpio_lock);

    return rc;
}

/* ---------------------------------------------------------------------------
 * Toggles the bit to high as indicated by the mask, does not write
 * to register if it's already high.
 * ------------------------------------------------------------------------ */
void gpio_toggle_bit_high(u_int32_t offset, u_int32_t mask)
{
    u_int32_t value;

    value = MV_REG_READ(offset);
    if ((value & mask) == 0)
    {
        value |= mask;
        MV_REG_WRITE(offset, value);
    }
}

/* ---------------------------------------------------------------------------
 * Toggles the bit to low as indicated by the mask, does not write
 * to register if it's already low.
 * ------------------------------------------------------------------------ */
void gpio_toggle_bit_low(u_int32_t offset, u_int32_t mask)
{
    u_int32_t value;

    value = MV_REG_READ(offset);
    if (value & mask)
    {
        value &= ~mask;
        MV_REG_WRITE(offset, value);
    }
}

/* ---------------------------------------------------------------------------
 * Sets the bit to high as indicated by the mask
 * ------------------------------------------------------------------------ */
void gpio_set_bit_high(u_int32_t offset, u_int32_t mask)
{
    u_int32_t value;

    value = MV_REG_READ(offset);
    value |= mask;
    MV_REG_WRITE(offset, value);
}

/* ---------------------------------------------------------------------------
 * Sets the bit to low as indicated by the mask
 * ------------------------------------------------------------------------ */
void gpio_set_bit_low(u_int32_t offset, u_int32_t mask)
{
    u_int32_t value;

    value = MV_REG_READ(offset);
    value &= ~mask;
    MV_REG_WRITE(offset, value);
}

int init_gpio(void)
{
        LOG(DEVICE_NAME ", version " DRV_VERSION "\n");

        gpio_base = ioremap(GPIO_BASE_ADDR, 0x200);
        if (!gpio_base)
                return -ENOMEM;

	major = register_chrdev(0, "bdgpio", &bdgpio_fops);

        return 0;
}

void exit_gpio(void)
{
        if (gpio_base)
                iounmap(gpio_base);

	unregister_chrdev(major, "bdgpio");

        LOG(DEVICE_NAME ", unloaded " DRV_VERSION "\n");
}
