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
  @file ch_buzzer.c
  @brief CandyHouse board specific driver buzzer support
*************************************************************************/

#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/delay.h>

#ifdef CONFIG_HW_VENDOR_MRVL
#include <boardEnv/mvBoardEnvSpec.h>

#include "bdutil.h"
#include "ch_buzzer.h"

#if defined(_CANDYHOUSE)

char usage_str_buzzer_candyhouse[] =
{	/* CandyHouse */ 
	"Usage: echo \"tone=<tone_number 0-10>\" > /proc/bdutil/buzzer\n"
};

static int ch_buzzer_act(int pin, int act, int freq, int dur)
{
	u_int32_t i, period, times;

	period = 1000000/freq;
	times = (1000000*dur)/period;
	times = times/8;

	for ( i=0 ; i < times ; i++) {
		gpio_write_bit(pin, !act); // inactive
		udelay(period/2);

		gpio_write_bit(pin, act); // active
		udelay(period/2);
	}

	gpio_write_bit(pin, !act); // inactive

	return 0;
}

int ch_buzzer_tone(void *user_str)
{
	int tone = -ENODEV;

	if (strncmp("tone=", (char *)user_str, 5) == 0)
		sscanf((char *)user_str, "tone=%d", &tone);
	
	if (tone == 100) {
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 2000, 1);
	} else if (tone == 0) {
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 2);
		mdelay(100);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 2);
		mdelay(100);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 2);
	} else if (tone == 1) {
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 2000, 1);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 2400, 1);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 2800, 1);
	} else if (tone == 2) {
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1200, 4);
	} else if (tone == 3) {
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1000, 1);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 2000, 1);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1400, 1);
	} else if (tone == 4) {
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1200, 4);
		mdelay(100);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1200, 1);
	} else if (tone == 5) {
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1200, 1);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1600, 1);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 2000, 1);
	} else if (tone == 6) {
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 2200, 4);
		mdelay(100);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 2200, 2);
		mdelay(10);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 2200, 2);
	} else if (tone == 7) {
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 1);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1200, 1);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1600, 1);
	} else if (tone == 8) { 
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 1);
		mdelay(10);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1400, 1);
	} else if (tone == 9) { 
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 1);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1600, 1);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1200, 1);
	} else if (tone == 10) { 
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1400, 1);
		mdelay(10);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 1);
	} else {
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1200, 2);
		mdelay(10);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 1);
		mdelay(10);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 1);
		mdelay(10);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 2);
		mdelay(10);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 800, 2);
		mdelay(100);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1200, 1);
		mdelay(10);
		ch_buzzer_act(CH_BUZZER_GPIO_PIN, 1, 1600, 2);
	}

        return 0;
}

/* CandyHouse per board buzzer struct */
buzzer_t ch_buzzer = {
	.label		= "ch_buzzer",
	.mv_boardId	= CH_88F6282A_BP_ID,
	.usage_str	= usage_str_buzzer_candyhouse,
	.request	= ch_buzzer_tone,
};

#else /* !_CANDYHOUSE */

char usage_str_buzzer_candyhouse[] =
{	/* CandyHouse */ 
	"Usage: Not supported.\n"
};

/* CandyHouse per board buzzer struct */
buzzer_t ch_buzzer = {
	.label		= "ch_buzzer",
	.mv_boardId	= CH_88F6282A_BP_ID,
	.usage_str	= usage_str_buzzer_candyhouse,
};

#endif /* _CANDYHOUSE */
#endif
