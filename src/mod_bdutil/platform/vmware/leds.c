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
  @file leds.c
  @brief board driver led control 
*************************************************************************/

#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>

#include <linux/gpio.h>

#include "../../debug.h"
#include "leds.h"

#define DRV_DESC	"bdutil board utility LEDs kernel module"
#define DRV_VERSION	"1.3"
#define DEVICE_NAME	"leds"

extern struct proc_dir_entry *proc_bdutil;
static struct proc_dir_entry *leds_proc_node;

extern u_int32_t bdutil_gboardId;

static int leds_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return sprintf(page, "Board type not supported.");
}

static int leds_proc_write(struct file * file, const char __user * buffer, unsigned long count, void *data)
{
        return count;
}

int init_leds(void)
{
	int status = 0;
        LOG(DRV_DESC ", version " DRV_VERSION"\n");

	/*
	 * U-Boot setup SoC GPIO registers for LEDs
	 */
	
	leds_proc_node = create_proc_entry(BDUTIL_LEDS_PROC_DEVICE_NAME, 0666, proc_bdutil);
	if (leds_proc_node == NULL) {
		ERR("bdutil_gpio: Failed to create LEDs proc entry\n");
		return ENOENT;
	}
	leds_proc_node->read_proc = leds_proc_read;
	leds_proc_node->write_proc = leds_proc_write;

	return status;
}

void exit_leds(void)
{
        if (leds_proc_node)
                remove_proc_entry(BDUTIL_LEDS_PROC_DEVICE_NAME, proc_bdutil);

        LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}
