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
  @file bdutil.c
  @brief board utility (button(s), buzzer, led(s), reset, etc.) kernel module 
*************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "bdutil.h"

#define DRV_DESC    "bdutil board utility kernel module"
#define DRV_VERSION "1.4"

MODULE_AUTHOR("Adam Graham <adagraha@cisco.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRV_DESC);

const char* platform = 0;
struct proc_dir_entry *proc_bdutil=NULL;
static struct proc_dir_entry *boardid_proc_node;

u_int32_t bdutil_gboardId = 0;

/* ---------------------------------------------------------------------------
 * Read function for boardid proc interface, returns name of board
 * ------------------------------------------------------------------------- */
static int boardid_proc_read(char *page, char **start, off_t off, int count,
                             int *eof, void *data)
{
    char boardid_str[BOARDID_MAX_STR];

    memset(boardid_str, 0, sizeof(boardid_str));
    if (get_board_type(boardid_str, sizeof(boardid_str)))
    {
        strcpy(boardid_str, "unknown");
    }

    return sprintf(page, "%s\n", boardid_str); 
}

/* ---------------------------------------------------------------------------
 * Module init
 * ------------------------------------------------------------------------- */
static int __init bdutil_init(void)
{
    platform = BDUTIL_PROC_DEVICE_NAME;
    if (proc_bdutil == NULL)
    {
        proc_bdutil = proc_mkdir(platform, NULL);
        if (!proc_bdutil)
            return ENOENT;
    }    
    LOG(DRV_DESC ", version " DRV_VERSION"\n");

    boardid_proc_node = create_proc_entry(BDUTIL_BOARDID_PROC_DEVICE_NAME,
                                          0666,
                                          proc_bdutil);
    if (boardid_proc_node == NULL)
    {
        ERR("bdutil_gpio: Failed to create boardid proc entry\n");
    }
    boardid_proc_node->read_proc = boardid_proc_read;

    bdutil_gboardId = get_boardId();
    LOG(DRV_DESC " boardId (%d)\n", bdutil_gboardId);

#if defined(GPIO)
    init_gpio();
#endif
#if defined(BUTTONS)
    init_buttons();
#endif
#if defined(LEDS)
    init_leds();
#endif
#if defined(BUZZER)
    init_buzzer();
#endif
#if defined(PWMLED)
    init_pwmled();
#endif
#if defined(EEPROM)
    init_eeprom();
#endif
#if defined(USBHUB)
    init_usbhub();
#endif

    return 0;
}

/* ---------------------------------------------------------------------------
 * Module exit
 * ------------------------------------------------------------------------- */
static void __exit bdutil_exit(void)
{
#if defined(EEPROM)
    exit_eeprom();
#endif
#if defined(PWMLED)
    exit_pwmled();
#endif
#if defined(BUZZER)
    exit_buzzer();
#endif
#if defined(LEDS)
    exit_leds();
#endif
#if defined(BUTTONS)
    exit_buttons();
#endif
#if defined(GPIO)
    exit_gpio();
#endif
#if defined(USBHUB)
    exit_usbhub();
#endif

    if (boardid_proc_node)
    {
        remove_proc_entry(BDUTIL_BOARDID_PROC_DEVICE_NAME, proc_bdutil);
    }

    remove_proc_entry(platform, NULL);
    LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}

module_init(bdutil_init);
module_exit(bdutil_exit);
