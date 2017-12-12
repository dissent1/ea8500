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
  @file gpio.c
  @brief board driver gpio access kernel module
*************************************************************************/

#include <linux/proc_fs.h>
#include <asm/io.h>
#include <asm-generic/errno-base.h>

#include <linux/gpio.h>

#ifdef CONFIG_HW_VENDOR_MRVL
#include <gpp/mvGpp.h>

#include "bdutil.h"
#include "mv_io.h"

#define DEVICE_NAME     "bdutil board utility gpio kernel module"
#define DRV_VERSION     "1.1"

#define GPIO_BASE_ADDR	(INTER_REGS_BASE + MV_GPIO_PORT_REGS_OFFSET)

static int bdgpio_open(struct inode *, struct file *);
static int bdgpio_release(struct inode *, struct file *);
static int bdgpio_ioctl(struct inode *inode, struct file *filp, u_int32_t cmd, unsigned long arg);

static void __iomem * gpio_base = NULL;

static int bdutil_gpio_open_count = 0;
static int major = 0;

void gpioRegSet(u_int32_t group, u_int32_t regOffs, u_int32_t mask, u_int32_t value)
{
	u_int32_t gppData;

	gppData = MV_REG_READ(regOffs);
	gppData &= ~mask;
	gppData |= (value & mask);
	MV_REG_WRITE(regOffs, gppData);
}

int gpioPolaritySet(u_int32_t group, u_int32_t mask, u_int32_t value)
{
	gpioRegSet(group, GPP_DATA_IN_POL_REG(group), mask, value);
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
	.ioctl = bdgpio_ioctl,
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

static int bdgpio_ioctl(struct inode *inode, struct file *filp, u_int32_t cmd, unsigned long arg)
{
	bdgpio_t user_bdgpio;

	switch (cmd) {
	case BDGPIO_OUTPUT_GET:
		copy_from_user(&user_bdgpio, (bdgpio_t *)arg, sizeof(bdgpio_t));
		user_bdgpio.value = gpio_read_output_bit(user_bdgpio.pin);
		if (copy_to_user((void *)arg, (void *)&user_bdgpio, sizeof(bdgpio_t))) {
			ERR("%s: - bad copy\n", __FUNCTION__);
			return -EFAULT;
		}	
		break;
	case BDGPIO_OUTPUT_SET:
		copy_from_user(&user_bdgpio, (bdgpio_t *)arg, sizeof(bdgpio_t));
		gpio_write_bit(user_bdgpio.pin, user_bdgpio.value);
		break;
	default:
		break;
	}

	return 1;
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
#endif
