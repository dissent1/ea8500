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
  @file buzzer.c
  @brief board driver for board buzzer support
*************************************************************************/

#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>

#include <boardEnv/mvBoardEnvSpec.h>

#include "../../debug.h"
#include "buzzer.h"
#include "gpio.h"
#include "ch_buzzer.h"

#define DRV_DESC	"bdutil board utility buzzer kernel module"
#define DRV_VERSION	"1.2"
#define DEVICE_NAME	"buzzer"

extern struct proc_dir_entry *proc_bdutil;
static struct proc_dir_entry *buzzer_proc_node;

extern u_int32_t bdutil_gboardId;
extern buzzer_t ch_buzzer;

static buzzer_t *buzzer = NULL;

static int buzzer_init_data(void)
{
	int retVal = 0;
	switch (bdutil_gboardId) {
	case CH_88F6282A_BP_ID:	/* CandyHouse board */
		DBGX(1, "%s: CH_88F6282A_BP_ID \n", __FUNCTION__);
		buzzer = &ch_buzzer;
		retVal = bdutil_gboardId;
		break;	
	default:	
		buzzer = NULL;
		ERR("ERROR: Buzzer not supported (%d)", bdutil_gboardId);
		retVal = -ENODEV;
		break;
	}

	return retVal;
}

static int buzzer_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	if (buzzer)
		return sprintf(page, buzzer->usage_str);
	else
		return sprintf(page, "Board type Not Supported");
}

static int buzzer_proc_write(struct file * file, const char __user * buffer, unsigned long count, void *data)
{
	int idx;
	char proc_buff[64];

	if (copy_from_user(proc_buff, buffer,count))
		return -EFAULT;

	for (idx = 0; idx < 64; idx++)
		if (proc_buff[idx] == '\n')
			break;
	proc_buff[idx] = '\0';

	if (!buzzer) return count;

	if (buzzer->request)
		buzzer->request((void *)proc_buff);

        return count;
}

int init_buzzer(void)
{
	int status = 0;

	if (bdutil_gboardId != CH_88F6282A_BP_ID) return;

        LOG(DRV_DESC ", version " DRV_VERSION"\n");

	/*
	 * U-Boot setup SoC GPIO registers for LEDs
	 */
	buzzer_proc_node = create_proc_entry(BDUTIL_BUZZER_PROC_DEVICE_NAME, 0666, proc_bdutil);
	if (buzzer_proc_node == NULL) {
		ERR("bdutil_gpio: Failed to create buzzer proc entry\n");
		return ENOENT;
	}
	buzzer_proc_node->read_proc = buzzer_proc_read;
	buzzer_proc_node->write_proc = buzzer_proc_write;

	buzzer_init_data();

	return status;
}

void exit_buzzer(void)
{
	if (bdutil_gboardId != CH_88F6282A_BP_ID) return;

        if (buzzer_proc_node)
                remove_proc_entry(BDUTIL_BUZZER_PROC_DEVICE_NAME, proc_bdutil);

        LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}
