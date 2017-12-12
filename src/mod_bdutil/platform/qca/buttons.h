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
  @file buttons.h
  @brief board button(s) support
*************************************************************************/

#ifndef __BDUTIL_BUTTONS_H__
#define __BDUTIL_BUTTONS_H__

#include <linux/completion.h>

extern int init_buttons(void);
extern void exit_buttons(void);

#define GPIO_RESET_PIN			17
#define GPIO_WPS_PIN            16

#define OPTION_DBG                      0x10000000
#define OPTION_BUTTONPRESS              0x01000000
#define OPTION_BUTTONRELEASE            0x00100000

#define BDUTIL_IOCTL_BASE               'B'
#define BDUTIL_BUTTONPRESS              (BDUTIL_IOCTL_BASE + 0)
#define BDUTIL_BUTTONRELEASE            (BDUTIL_IOCTL_BASE + 1)
#define BDUTIL_SETRSTTIMEOUT		(BDUTIL_IOCTL_BASE + 2)

/* UNKNOWN */
//#define UNKNOWN_GPIO_RESET_PIN		0		

struct buttons_private
{
	struct task_struct *thread;
	struct completion thread_done;
	char *name;

	int my_pid;
	u_int32_t eventmask;
	unsigned long thread_event_id;
	wait_queue_head_t waitq;

	wait_queue_head_t longpress_waitq;
	struct list_head list;
};


#define EVENT_BUTTON_FACTORY_RESET	    (1<<0)
#define EVENT_BUTTON_WPS		        (1<<1)
#define EVENT_BUTTON_WIFI		        (1<<2)
#define EVENT_BUTTON_LONGPRESS          (1<<8)

#define BUTTON_PRESSED			0
#define BUTTON_RELEASED			1

#define POWER_SHUTDOWN			0xf0
#define LEAVE_THREAD			0xff

#define LEAVE_THREAD			0xff
#define RESET_REMAP			0xfe

#define BOARD_GPIO_FACTORY_RESET_PIN	GPIO_RESET_PIN
#define BOARD_GPIO_WPS_PIN		GPIO_WPS_PIN

#endif /* __BDUTIL_BUTTONS_H__ */
