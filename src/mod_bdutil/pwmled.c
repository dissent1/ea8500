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
  @file pwmled.c
  @brief board driver for pwm LED 
*************************************************************************/

#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>

#include <linux/delay.h>

#include <linux/gpio.h>

#ifdef CONFIG_HW_VENDOR_MRVL
#include <gpp/mvGpp.h>
#include <boardEnv/mvBoardEnvSpec.h>
#include <ctrlEnv/mvCtrlEnvRegs.h>

#include "bdutil.h"
#include "mv_io.h"

#define DRV_DESC	"bdutil board utility pwm LED kernel module"
#define DRV_VERSION	"1.1"
#define DEVICE_NAME	"pwmled"

extern struct proc_dir_entry *proc_bdutil;
static struct proc_dir_entry *pwm_proc_node;

extern u_int32_t bdutil_gboardId;
static char *usage_str;

extern void mvPwmLedInit(void);
extern u_int32_t mvPwmLedGet(void);
extern void mvPwmLedSet(u_int32_t);

static int pwm_init_data(void);
static void mvBoardMfgPwmLedSet(u_int8_t);
static inline MV_VOID wncPwmLedBlink(void);
static inline MV_VOID wncPwmLedOff(void);
static inline MV_VOID PwmLedOn(void);
static MV_VOID wncPwmLedOn(void);
static inline void wncLedTurnOn(void);
static void wncLedTurnOnFull(void);
static void VpPwmLedInit(void);
static void VpPwmLedOn(void);
static void VpPwmLedOff(void);
static void VpPwmLedBlinkOn(void);
static void VpPwmLedBlinkOff(void);
static void VpPwmLedBlink(int);
static void VpPwmLedPulseOn(void);
static void VpPwmLedPulseOff(void);
static void VpPwmLedPulse(int);
static void mfg_led_pwm_test(void);
static int pwm_proc_read(char *, char **, off_t, int, int *, void *);
static int pwm_proc_write(struct file *, const char __user *, unsigned long, void *);

typedef struct timer_list Timer;
static Timer blink_timer;
static int blink_decisec = 0; /* 1 = 100ms */
static int blink_toggle = 0;
static int status_blink_timer = 0;

static void blink_set_expire(void);
static void blink_counter(void);
static void blink_set_timer(void);
static void blink_free_timer(void);

static void blink_set_expire(void)
{
	blink_timer.expires = (jiffies + (blink_decisec  * (HZ) / 10));
	/*DBGX(1, "%s: blink_timer.expires =%d\n", __FUNCTION__, blink_timer.expires );*/
}

static void blink_counter(void)
{
	if (blink_toggle) {
		VpPwmLedOn();
	} else {
		VpPwmLedOff();
	}

	blink_toggle = !blink_toggle;

	blink_set_expire();
	/*DBGX(1, "%s: blink_timer.expires =%d\n", __FUNCTION__, blink_timer.expires );*/
	add_timer(&blink_timer);
}

static void blink_free_timer(void)
{
	if (status_blink_timer == 1) {
		status_blink_timer = 0;
		del_timer_sync(&blink_timer);	
	}
}

static void blink_set_timer(void)
{
	blink_free_timer();

	status_blink_timer = 1;
	init_timer(&blink_timer);
	blink_timer.function = blink_counter;
	blink_timer.data = (unsigned int)NULL;

	blink_set_expire();
	add_timer(&blink_timer);
}

static char usage_str_vp_pwmled[] =
{
	"Usage: echo on > /proc/bdutil/pwmled\n"
	"       echo off > /proc/bdutil/pwmled\n"
	"       echo blinking=[on|off] > /proc/bdutil/pwmled\n"
	"       echo pulsing=[on|off] > /proc/bdutil/pwmled\n"
	"       echo \"level=[0-15]\" > /proc/bdutil/pwmled\n"
	"       echo \"timeron=[1-50]\" > /proc/bdutil/pwmled\n"
	"       echo timeroff > /proc/bdutil/pwmled\n"
	"\n"
};
static char usage_str_ad_pwmled[] =
{
	"Usage: echo on > /proc/bdutil/pwmled\n"
	"       echo off > /proc/bdutil/pwmled\n"
	"       echo blinking=[on|off] > /proc/bdutil/pwmled\n"
	"       echo pulsing=[on|off] > /proc/bdutil/pwmled\n"
	"       echo \"level=[0-15]\" > /proc/bdutil/pwmled\n"
	"       echo \"timeron=[1-50]\" > /proc/bdutil/pwmled\n"
	"       echo timeroff > /proc/bdutil/pwmled\n"
	"\n"
};
static char usage_str_pwmled[] =
{
	"Usage: echo on > /proc/bdutil/pwmled\n"
	"       echo stop > /proc/bdutil/pwmled\n"
	"       echo full > /proc/bdutil/pwmled\n"
	"       echo off > /proc/bdutil/pwmled\n"
	"       echo blink > /proc/bdutil/pwmled\n"
	"       echo \"level=[0-15]\" > /proc/bdutil/pwmled\n"
	"       echo test > /proc/bdutil/pwmled\n"
	"\n"
	"PWM value: %s.\n"
};

static int pwm_init_data(void)
{
	int retVal = 0;

	switch (bdutil_gboardId) {
	case VP_88F6282A_BP_ID: /* Viper board */
		DBGX(1, "%s: VP_88F6282A_BP_ID \n", __FUNCTION__);
		usage_str = usage_str_vp_pwmled; 
		retVal = 0;
		break;
	case AD_88F6282A_BP_ID: /* Audi board */
		DBGX(1, "%s: AD_88F6282A_BP_ID \n", __FUNCTION__);
		usage_str = usage_str_ad_pwmled; 
		retVal = 0;
		break;		
	case GS_88F6282A_BP_ID:	/* Gobstopper board */
	case FB_88F6282A_BP_ID:	/* Fireball board */
	case P2_88F6282A_BP_ID:	/* P2 Caramello board */
		DBGX(1, "%s: P2_88F6282A_BP_ID \n", __FUNCTION__);
		usage_str = usage_str_pwmled; 
		retVal = 0;
		break;	
	case DB_88F6282A_BP_ID:	/* P1 Caramello board */
		DBGX(1, "%s: DB_88F6282A_BP_ID \n", __FUNCTION__);
		usage_str = '\0'; 
		retVal = 1;
		break;	
	case CH_88F6282A_BP_ID:	/* CandyHouse board */
		DBGX(1, "%s: CH_88F6282A_BP_ID \n", __FUNCTION__);
		usage_str = '\0';
		retVal = 1;
		break;	
	default:	
		usage_str = '\0';
		ERR("ERROR: Board Type not supported (%d)", bdutil_gboardId);
		retVal = -1;
		break;
	}

	return retVal;
}

static void mvBoardMfgPwmLedSet(u_int8_t level)
{
	u_int32_t reg_data;

	reg_data = MV_REG_READ(0xc00fc);
	reg_data |= 0x00100000;
	reg_data &= 0x0fffffff;
	reg_data |= level << 28;
	MV_REG_WRITE(0xc00fc, reg_data);
}

#if 0
static MV_VOID wncPwmInit(void)
{
	u_int32_t reg_data = 0xf0100000;

	MV_REG_WRITE(MPP_CONTROL_REG0, 0xB1111111);/* MPP7 (0xb) for PWM */

	MV_REG_WRITE(0xc01b8, 0x60000001);
	MV_REG_WRITE(0xc01a8, 0x80000001);

	MV_REG_WRITE(0xcf0a0, 0x80000000); /* set bit 31 = 0 , power down */
	MV_REG_WRITE(0xcf0a0, 0x84500004); /* programming the value */

	MV_REG_WRITE(0xcf0a4, 0x80FC0228);
	MV_REG_WRITE(0xcf0a4, 0x80FE0228); /* enable the power bit 17 */

	MV_REG_WRITE(0xcf0a0, 0x4500004); /* enable the PLL */

	MV_REG_WRITE(0xc00fc, reg_data);
}
#endif

static inline MV_VOID wncPwmLedBlink(void)
{
	/* 88F6282 SoC LCD_CFG_GRA_PITCH Reg., offset 0xC00FC */
	MV_REG_WRITE(0xc00fc, 0xffff0000);
}

static inline MV_VOID wncPwmLedOff(void)
{
	/* 88F6282 SoC LCD_CFG_GRA_PITCH Reg., offset 0xC00FC */
	MV_REG_WRITE(0xc00fc, 0x10000000);
}

static inline MV_VOID PwmLedOn(void)
{
	/* 88F6282 SoC LCD_CFG_GRA_PITCH Reg., offset 0xC00FC */
	MV_REG_WRITE(0xc00fc, 0xf0110000);
}

static MV_VOID wncPwmLedOn(void)
{
	int i;
	u_int32_t reg_data;

	/*
	 * 88F6282 SoC LCD_CFG_GRA_PITCH Reg., offset 0xC00FC
	 *
	 * bits:    | 31:27 |    27:16     |      15:0       |
	 *            duty       clock         don't care
	 *            cycle    frequency
	 *                      divisor
	 *
	 * Clock freq divisor value 0xfff = generates 32kHz divided by 4096.
	 *
	 * To achive LED fade-on/fade-out ramp up then ramp down duty cycle
	 * in a loop.
	 */
	for (i = 0; i < 16; i++) {
		reg_data = MV_REG_READ(0xc00fc);
		reg_data &= 0x0fffffff;
		reg_data |= i << 28;	/* duty cycle bits */
		MV_REG_WRITE(0xc00fc, reg_data);
		mvOsDelay(50);
	}
	for (i = 15; i >= 0; i--) {
		reg_data = MV_REG_READ(0xc00fc);
		reg_data &= 0x0fffffff;
		reg_data |= i << 28;	/* duty cycle bits */
		MV_REG_WRITE(0xc00fc, reg_data);
		mvOsDelay(50);
	}
}

static inline void wncLedTurnOn(void)
{
	wncPwmLedOn();
}

static void wncLedTurnOnFull(void)
{
	u_int32_t reg_data;

	reg_data = MV_REG_READ(0xc00fc);
	reg_data &= 0x0fffffff;
	MV_REG_WRITE(0xc00fc, reg_data);
}

static void VpPwmLedInit(void)
{
	u_int32_t reg_data;

	/* MPP_CONTROL_REG0[31-28] = 0 for GPIO7 */
	reg_data = MV_REG_READ(0x10000);
	reg_data &= 0x0fffffff;
	MV_REG_WRITE(0x10000, reg_data);
}

static void VpPwmLedOn(void)
{
	u_int32_t reg_data;

	VpPwmLedInit();

	/* GPP_DATA_OUT_REG[7] = 1 for GPIO7 output on */
	/* GPP_DATA_OUT_REG[14] = 0 for GPIO14 output off */
	reg_data = MV_REG_READ(0x10100);
	reg_data |= 0x00000080;
	reg_data &= 0xffffbfff;
	MV_REG_WRITE(0x10100, reg_data);
}

static void VpPwmLedOff(void)
{
	u_int32_t reg_data;

	VpPwmLedInit();

	/* GPP_DATA_OUT_REG[7] = 0 for GPIO7 output off */
	/* GPP_DATA_OUT_REG[14] = 0 for GPIO14 output off */
	reg_data = MV_REG_READ(0x10100);
	reg_data &= 0xffffbf7f;
	MV_REG_WRITE(0x10100, reg_data);
}

static void VpPwmLedBlinkOn(void)
{
	u_int32_t reg_data;

	VpPwmLedInit();

	/* GPP_DATA_BLINK_REG[7] = 1 for GPIO7 blink enable */
	reg_data = MV_REG_READ(0x10108);
	reg_data |= 0x00000080;
	MV_REG_WRITE(0x10108, reg_data);
}

static void VpPwmLedBlinkOff(void)
{
	u_int32_t reg_data;

	VpPwmLedInit();

	/* GPP_DATA_BLINK_REG[7] = 0 for GPIO7 blink disable */
	reg_data = MV_REG_READ(0x10108);
	reg_data &= 0xffffff7f;
	MV_REG_WRITE(0x10108, reg_data);
}

static void VpPwmLedBlink(int OnOff)
{
	if (OnOff) {
		/* on */
		VpPwmLedOn();
		VpPwmLedBlinkOn();
	} else {
		/* off */
		VpPwmLedOn();
		VpPwmLedBlinkOff();
	}
}

static void VpPwmLedPulseOn(void)
{
	u_int32_t reg_data;

	VpPwmLedInit();

	VpPwmLedOn();

	/* GPP_DATA_OUT_REG[7] = 0 for GPIO7 */
	/* GPP_DATA_OUT_REG[14] = 1 for GPIO14 */
	reg_data = MV_REG_READ(0x10100);
	reg_data &= 0xffffff7f;
	reg_data |= 0x00004000;
	MV_REG_WRITE(0x10100, reg_data);
}

static void VpPwmLedPulseOff(void)
{
	u_int32_t reg_data;

	VpPwmLedInit();

	VpPwmLedOn();

	/* GPP_DATA_OUT_REG[7] = 0 for GPIO7 */
	/* GPP_DATA_OUT_REG[14] = 0 for GPIO14 */
	reg_data = MV_REG_READ(0x10100);
	reg_data &= 0xffff4f7f;
	MV_REG_WRITE(0x10100, reg_data);

	VpPwmLedOn();
}

static void VpPwmLedPulse(int OnOff)
{
	if (OnOff)
		VpPwmLedPulseOn();
	else
		VpPwmLedPulseOff();
}

static void mfg_led_pwm_test(void)
{
	int i, j;

	printk("\n");
	wncPwmLedOff();
	for (i = 0; i < 5; i++) {
		printk("**LED PWM Test: become dark...\n");
		for (j = 0; j < 16; j++) {
			mvBoardMfgPwmLedSet(j);
			mvOsDelay(40);
		}
		wncPwmLedOff();
		mvOsDelay(100);
		printk("**LED PWM Test: become light...\n");
		for (j = 15; j >= 0; j--) {
			mvOsDelay(80);
			mvBoardMfgPwmLedSet(j);
		}
		mvOsDelay(100);
	}
	wncPwmLedOff();
	printk("\n");
}

static int pwm_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *pwmstr;

	switch((int)mvPwmLedGet()) {
	case 0:
		pwmstr = "off";
		break;
	case 1:
		pwmstr = "on";
		break;
	default:
		pwmstr = "unknown";
		break;
	}

	if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID)
		return sprintf(page, usage_str);
	else
		return sprintf(page, usage_str, pwmstr);
}

static int pwm_proc_write(struct file * file, const char __user * buffer, unsigned long count, void *data)
{
	int idx, level;
	char proc_buff[64];

	if (copy_from_user(proc_buff, buffer,count))
		return -EFAULT;

	for (idx = 0; idx < 64; idx++)
		if (proc_buff[idx] == '\n')
			break;
	proc_buff[idx] = '\0';

	if (strncmp("on", proc_buff, strlen("on")) == 0) {
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID) {
			VpPwmLedOn();
		} else if (bdutil_gboardId == GS_88F6282A_BP_ID) {
			mvPwmLedSet(1);
		}
	} else if (strncmp("stop", proc_buff, strlen("stop")) == 0) {
		if (bdutil_gboardId == GS_88F6282A_BP_ID) {
			mvPwmLedSet(0);
			mvOsDelay(200);
		}
	} else if (strncmp("full", proc_buff, strlen("full")) == 0) {
		if (bdutil_gboardId == GS_88F6282A_BP_ID) {
			mvPwmLedSet(0);
			mvOsDelay(200);
			wncLedTurnOnFull();
		}
	} else if (strncmp("off", proc_buff, strlen("off")) == 0) {
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID) {
			VpPwmLedOff();
		} else if (bdutil_gboardId == GS_88F6282A_BP_ID) {
			mvPwmLedSet(0);
			mvOsDelay(200);
			wncPwmLedOff();
		}
	} else if (strncmp("blinking=on", proc_buff, strlen("blinking=on")) == 0) {
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID) {
			VpPwmLedBlink(1);
		}
	} else if (strncmp("blinking=off", proc_buff, strlen("blinking=off")) == 0) {
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID) {
			VpPwmLedBlink(0);
		}
	} else if (strncmp("blink", proc_buff, strlen("blink")) == 0) {
		if (bdutil_gboardId == GS_88F6282A_BP_ID) {
			mvPwmLedSet(0);
			mvOsDelay(200);
			wncPwmLedBlink();
		}
	} else if (strncmp("pulsing=on", proc_buff, strlen("pulsing=on")) == 0){
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID) {
			VpPwmLedPulse(1);
		}
	} else if (strncmp("pulsing=off", proc_buff, strlen("pulsing=off")) == 0) {
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID) {
			VpPwmLedPulse(0);
		}
	} else if (strncmp("level=", (char *)proc_buff, strlen("level=")) == 0){
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID) {
			mvPwmLedInit();
		} else if (bdutil_gboardId == GS_88F6282A_BP_ID) {
			mvPwmLedSet(0);
		}
		sscanf((char *)proc_buff, "level=%d", &level);
		/* level values 0 - 15 are valid */
		if ((level >= 0) && (level <= 15)) {
			DBGX(1, "%s: level=%d\n", __FUNCTION__, level);
			level = 15 - level;
			mvBoardMfgPwmLedSet(level);
		}
	} else if (strncmp("test", proc_buff, strlen("test")) == 0) {
		if (bdutil_gboardId == GS_88F6282A_BP_ID) {
			mvPwmLedSet(0);
			mvOsDelay(500);
			mfg_led_pwm_test();
		}
	} else if (strncmp("timeron=", proc_buff, strlen("timeron=")) == 0) {
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID) {
			int usr_input = 0;
			DBGX(1, "%s: timeron \n", __FUNCTION__);
			sscanf((char *)proc_buff, "timeron=%d", &usr_input);
			if ((usr_input >= 1) && (usr_input  <= 50)) {
				DBGX(1, "%s: usr_input =%d\n", __FUNCTION__, usr_input );
				blink_decisec = usr_input;
				blink_set_timer();
			}
		}
	} else if (strncmp("timeroff", proc_buff, strlen("timeroff")) == 0) {
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID) {
			blink_free_timer();
			DBGX(1, "%s: timeroff \n", __FUNCTION__);
		}
	}

        return count;
}

int init_pwmled(void)
{
	int status = 0;

	/*
	 * the extra check of SoC reg. offset 0xc00fc is to exclude
	 * non-PWM_LED P2_Caramello boards.
	 */
	pwm_init_data();
	//if ((pwm_init_data() != 0) || (MV_REG_READ(0xc00fc) == 0))
	//	return;
        LOG(DRV_DESC ", version " DRV_VERSION"\n");

	/*
	 * U-Boot setup SoC GPIO registers for LEDs
	 */
	
	pwm_proc_node = create_proc_entry(BDUTIL_PWM_PROC_DEVICE_NAME, 0666, proc_bdutil);
	if (pwm_proc_node == NULL) {
		ERR("bdutil_gpio: Failed to create PWM proc entry\n");
		return ENOENT;
	}
	pwm_proc_node->read_proc = pwm_proc_read;
	pwm_proc_node->write_proc = pwm_proc_write;

	if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID) {
		VpPwmLedInit();
		VpPwmLedPulseOn();
	} else if (bdutil_gboardId == GS_88F6282A_BP_ID) {
		mvPwmLedInit();
		wncLedTurnOnFull();
		mvPwmLedSet(1);
	}

	return status;
}

void exit_pwmled(void)
{
	/*
	 * the extra check of reg. offset 0xc00fc is to exclude
	 * non-PWM_LED P2_Caramello boards.
	 */
	pwm_init_data();
	//if ((pwm_init_data() != 0) || (MV_REG_READ(0xc00fc) == 0))
	//	return;

        if (pwm_proc_node)
                remove_proc_entry(BDUTIL_PWM_PROC_DEVICE_NAME, proc_bdutil);

        LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}
#endif
