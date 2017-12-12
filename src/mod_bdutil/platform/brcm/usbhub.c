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
  @file usbhub.c
  @brief board driver usbhub control 
*************************************************************************/

#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>

#include <linux/sched.h>
#include <linux/stat.h>

#include <linux/timer.h>
#include <linux/stat.h>

#include <osl.h>
#include <bcmgpio.h>
#include <typedefs.h>
#include <siutils.h>

#include "../../debug.h"
#include "usbhub.h"

#define DRV_DESC	"bdutil board utility usbhub kernel module"
#define DRV_VERSION	"1.1"
#define DEVICE_NAME	"usbhub"

extern struct proc_dir_entry *proc_bdutil;
static struct proc_dir_entry *usbhub_proc_node;

si_t *gpio_usbhub_sih;

static int gpio_set_value(int gpio_pin, int value)
{
	unsigned int gpio_mask = ((unsigned int) 1 << gpio_pin);

	si_gpiosetcore(gpio_usbhub_sih);
	si_gpioouten(gpio_usbhub_sih, gpio_mask, gpio_mask, GPIO_HI_PRIORITY);
	si_gpioout(gpio_usbhub_sih, gpio_mask, (value ? gpio_mask : 0), GPIO_HI_PRIORITY);

	return 0;
}

#if defined( _CARRERA )
static int reset_usbhub(void)
{
	gpio_set_value(BOARD_GPIO_USBHUB_RESET_PIN, 1);

	return 0;
}

#else

static int enable_usbport(void)
{
#if defined( _HONDA ) || defined( _F70 )
	gpio_set_value(BOARD_GPIO_USB0_PORT_ENABLE, 1);
	gpio_set_value(BOARD_GPIO_USB1_PORT_ENABLE, 1);
/*Active is low*/
#elif defined( _SPYDER )
	gpio_set_value(BOARD_GPIO_USB0_PORT_ENABLE, 1);
	gpio_set_value(BOARD_GPIO_USB1_PORT_ENABLE, 1);
#else
	gpio_set_value(BOARD_GPIO_USB1_PORT_ENABLE, 1);
#endif

	return 0;
}
#endif

static int usbhub_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    return 0;
}

static int usbhub_proc_write(struct file * file, const char __user * buffer, unsigned long count, void *data)
{
	char proc_buff[64];
	int idx;

	if (copy_from_user(proc_buff, buffer, count))
		return -EFAULT;

	for (idx = 0; idx < 64; idx++)
		if (proc_buff[idx] == '\n')
			break;
	proc_buff[idx] = '\0';

	if (strncmp("high", proc_buff, strlen("high")) == 0) 
    {
#if defined( _CARRERA )
    	gpio_set_value(BOARD_GPIO_USBHUB_RESET_PIN, 1);
#elif defined( _HONDA ) || defined( _F70 )
	    gpio_set_value(BOARD_GPIO_USB0_PORT_ENABLE, 1);
	    gpio_set_value(BOARD_GPIO_USB1_PORT_ENABLE, 1);
#elif defined( _SPYDER )
		gpio_set_value(BOARD_GPIO_USB0_PORT_ENABLE, 0);
	    gpio_set_value(BOARD_GPIO_USB1_PORT_ENABLE, 0);
#else
	    gpio_set_value(BOARD_GPIO_USB1_PORT_ENABLE, 1);
#endif
	} 
    else if (strncmp("low", proc_buff, strlen("low")) == 0) 
	{
#if defined( _CARRERA )
    	gpio_set_value(BOARD_GPIO_USBHUB_RESET_PIN, 0);
#elif defined( _HONDA ) || defined( _F70 )
	    gpio_set_value(BOARD_GPIO_USB0_PORT_ENABLE, 0);
	    gpio_set_value(BOARD_GPIO_USB1_PORT_ENABLE, 0);
#elif defined( _SPYDER )
		gpio_set_value(BOARD_GPIO_USB0_PORT_ENABLE, 1);
	    gpio_set_value(BOARD_GPIO_USB1_PORT_ENABLE, 1);
#else
	    gpio_set_value(BOARD_GPIO_USB1_PORT_ENABLE, 0);
#endif
	}

    return count;
}

int init_usbhub(void)
{
	int status = 0;

    LOG(DRV_DESC ", version " DRV_VERSION"\n");

	usbhub_proc_node = create_proc_entry(BDUTIL_USBHUB_PROC_DEVICE_NAME, 0666, proc_bdutil);
	if (usbhub_proc_node == NULL) {
		ERR("bdutil: Failed to create usbhub proc entry\n");
		return ENOENT;
	}
	usbhub_proc_node->read_proc = usbhub_proc_read;
	usbhub_proc_node->write_proc = usbhub_proc_write;

	if (!(gpio_usbhub_sih = si_kattach(SI_OSH)))
	{
		printk(KERN_ERR "Error connecting GPIO %d for usbhub reset\n", BOARD_GPIO_USBHUB_RESET_PIN);
		return -ENODEV;
	}

#if defined( _CARRERA )
	reset_usbhub();
#else
	enable_usbport();
#endif

	return status;
}

void exit_usbhub(void)
{
    if (usbhub_proc_node)
        remove_proc_entry(BDUTIL_USBHUB_PROC_DEVICE_NAME, proc_bdutil);

    si_detach(gpio_usbhub_sih);

    LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}
