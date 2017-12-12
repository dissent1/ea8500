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
#include <gpp/mvGpp.h>
#include <boardEnv/mvBoardEnvSpec.h>

#include "../../debug.h"
#include "leds.h"
#include "gpio.h"
#include "mv_io.h"


#define DRV_DESC	"bdutil board utility LEDs kernel module"
#define DRV_VERSION	"1.3"
#define DEVICE_NAME	"leds"

extern void mvEthSwitchRegWrite(u_int32_t ethPortNum, u_int32_t phyAddr, u_int32_t regOffs, u_int16_t data);
extern struct proc_dir_entry *proc_bdutil;
static struct proc_dir_entry *leds_proc_node;

extern u_int32_t bdutil_gboardId;
static leds_t *ledtblp;
static char *usage_str;

static leds_t leds_gpio_viper[] =
{  /* Viper */
};
static leds_t leds_gpio_audi[] =
{  /* Audi */
};
static leds_t leds_gpio_p2_caramello[] =
{  /* P2 Caramello */
    /* Name	Input	 MPP Pin	GPIO Pin	Act.	Pair	State */
    { "blue1",	S_INPUT, MPP_16,	MV_GPP16,	0,	0, 	0 },
    { "white",	D_INPUT, MPP_17,	MV_GPP17,	0,	MPP_43, 0 },
    { "amber",	D_INPUT, MPP_17,	MV_GPP17,	0,	MPP_43, 1 },
    { "blue",	D_INPUT, MPP_43,	MV_GPP43,	0,	MPP_17, 1 },
    { NULL,	0,	0,		0,		0,	0, 	0 },
};

static leds_t leds_gpio_p1_caramello[] =
{  /* P1 Caramello */
    /* Name	Input	 MPP Pin	GPIO Pin	Act.	Pair	State */
    { "green",	S_INPUT, MPP_16,	MV_GPP16,	1,	0,	0 },
    { "blue",	S_INPUT, MPP_17,	MV_GPP17,	1,	0,	0 },
    { "amber",	S_INPUT, MPP_43,	MV_GPP43,	1,	0,	0 },
    { NULL,	0,	0,		0,		0,	0,	0 },
};

static leds_t leds_gpio_candyhouse[] =
{  /* CandyHouse */
    /* Name	Input	 MPP Pin	GPIO Pin	Act. 	Pair	State */
    { "green1",	S_INPUT, MPP_16,	MV_GPP16,	1,	0,	0 },
    { "green2",	S_INPUT, MPP_17,	MV_GPP17,	1,	0,	0 },
    { "green",	D_INPUT, MPP_41,	MV_GPP41,	1,	MPP_42,	0 },
    { "amber",	D_INPUT, MPP_42,	MV_GPP42,	1,	MPP_41,	0 },
    { "amber1",	S_INPUT, MPP_43,	MV_GPP43,	1,	0,	0 },
    { NULL,	0,	 0,		0,		0,	0,	0 }
};

static char usage_str_viper[] =
{	/* Viper */
	"Usage: echo \"[on|off|default]\" > /proc/bdutil/leds\n"
};
static char usage_str_audi[] =
{	/* Audi */
	"Usage: echo \"[on|off|default]\" > /proc/bdutil/leds\n"
};
static char usage_str_p2_caramello[] =
{	/* P2 Caramello */
	"Usage: echo \"blue=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"white=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"amber=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"blue1=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"[blue|white|amber|blue1]=blinkon\" > /proc/bdutil/leds\n"
	"       echo \"[blue|white|amber|blue1]=blinkoff\" > /proc/bdutil/leds\n"
	"       echo off > /proc/bdutil/leds\n"
};

static char usage_str_p1_caramello[] =
{	/* P1 Caramello */
	"Usage: echo \"green=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"blue=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"amber=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"[green|blue|amber]=blinkon\" > /proc/bdutil/leds\n"
	"       echo \"[green|blue|amber]=blinkoff\" > /proc/bdutil/leds\n"
	"       echo off > /proc/bdutil/leds\n"
};
   
static char usage_str_candyhouse[] =
{	/* CandyHouse */ 
	"Usage: echo \"green1=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"green2=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"green=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"amber=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"amber1=[on|off]\" > /proc/bdutil/leds\n"
	"       echo \"[green1|green2|amber1]=blinkon\" > /proc/bdutil/leds\n"
	"       echo \"[green1|green2|amber1]=blinkoff\" > /proc/bdutil/leds\n"
	"       echo off > /proc/bdutil/leds\n"
};

static int leds_init_data(void)
{
	int retVal = 0;

	switch (bdutil_gboardId) {
	case VP_88F6282A_BP_ID:	/* Viper board */
		DBGX(1, "%s: VP_88F6282A_BP_ID \n", __FUNCTION__);
		ledtblp = leds_gpio_viper;	
		usage_str = usage_str_viper;
		break;
	case AD_88F6282A_BP_ID:	/* Audi board */
		DBGX(1, "%s: AD_88F6282A_BP_ID \n", __FUNCTION__);
		ledtblp = leds_gpio_audi;	
		usage_str = usage_str_audi;
		break;
	case GS_88F6282A_BP_ID:	/* Gobstopper board */
	case FB_88F6282A_BP_ID:	/* Fireball board */
	case P2_88F6282A_BP_ID:	/* P2 Caramello board */
		DBGX(1, "%s: P2_88F6282A_BP_ID \n", __FUNCTION__);
		ledtblp = leds_gpio_p2_caramello;	
		usage_str = usage_str_p2_caramello;
		retVal = 0;
		break;	
	case P1_88F6282A_BP_ID:	/* P1 Caramello board */
		DBGX(1, "%s: DB_88F6282A_BP_ID \n", __FUNCTION__);
		ledtblp = leds_gpio_p1_caramello;	
		usage_str = usage_str_p1_caramello;
		retVal = 0;
		break;	
	case CH_88F6282A_BP_ID:	/* CandyHouse board */
		DBGX(1, "%s: CH_88F6282A_BP_ID \n", __FUNCTION__);
		ledtblp = leds_gpio_candyhouse;	
		usage_str = usage_str_candyhouse;
		retVal = 0;
		break;	
	default:	
		ledtblp = NULL; 
		usage_str = '\0';
		ERR("ERROR: Board Type not supported (%d)", bdutil_gboardId);
		retVal = -1;
		break;
	}

	return retVal;
}

static void set_blink_LED(leds_t *LEDtable, int blink) {
	gpioRegSet((LEDtable->mpp_pin / 32),
		   GPP_BLINK_EN_REG((LEDtable->mpp_pin / 32)),
		   LEDtable->gpio_pin,
		   (blink & LEDtable->gpio_pin));
}

static void set_LED(leds_t *LEDtable, int on_off, int blink)
{
	gpio_set_value(LEDtable->mpp_pin, on_off);
	set_blink_LED(LEDtable, blink);
}

static void set_dual_input_LED(leds_t *LEDtable, int on_off, int blink)
{
	gpio_set_value(LEDtable->mpp_pin, on_off);
	if (LEDtable->gpio_polarity == on_off) /* on */
	 	gpio_set_value(LEDtable->mpp_pair, LEDtable->mpp_pair_val);
	else /* off */
		gpio_set_value(LEDtable->mpp_pair, !LEDtable->gpio_polarity);
	set_blink_LED(LEDtable, blink);
}

static int set_leds(leds_t *ledp, int on_off, int blink)
{
	DBGX(1, "%s: name(%s), reg_group(%d), mpp_pin(%d), on-off(%d) blink(%d)\n", __FUNCTION__, ledp->name, (ledp->mpp_pin / 32), ledp->mpp_pin, on_off, ((blink == MV_GPP_IN_INVERT) ? 1:0));

	if (ledp->type == D_INPUT)
		set_dual_input_LED((leds_t *)ledp, on_off, blink);
	else
		set_LED((leds_t *)ledp, on_off, blink);

	return 0;
}

static int ledact(char *proc_buff)
{
	int blink, on_off = 0;
	char *tok, *x;
	leds_t *ledp;

        if (!ledtblp)
		return EINVAL;

	if (strstr(proc_buff, "=") == NULL) 
		return EINVAL;

	blink = MV_GPP_IN_ORIGIN;
	x = proc_buff;
	tok = strsep(&x, "=");
	for (ledp = ledtblp; ledp->name != NULL; ledp++) {
		if (strcmp(ledp->name, tok) == 0) {
			tok = strsep(&x, "=");
			if (strncmp("blinkon", tok, 7) == 0) {
				blink = MV_GPP_IN_INVERT;
				on_off = ledp->gpio_polarity;
			} else if (strncmp("blinkoff", tok, 8) == 0) {
				blink = MV_GPP_IN_ORIGIN;
				on_off =
				   gpio_get_value(ledp->mpp_pin);
			} else if (strncmp(tok , "on", 2) == 0) {
				on_off = ledp->gpio_polarity;
			} else if (strncmp(tok, "off", 3) == 0) {
				on_off = !(ledp->gpio_polarity);
			} else {
				return EINVAL;
			}	
			break;
		}
	}

	if (ledp->name != NULL)
		set_leds(ledp, on_off, blink);

	return 0;
}

int leds_blue_on(void)
{
        if (!ledtblp) return EINVAL;
	ledact("blue=on");
	return 0;
}

int leds_blue_off(void)
{
        if (!ledtblp) return EINVAL;
	ledact("blue=off");
	return 0;
}

int leds_white_off(void)
{
        if (!ledtblp) return EINVAL;
	ledact("white=off");
	return 0;
}

int leds_off_ethernet_ports(void)
{
	mvEthSwitchRegWrite(0, 0x10, 0x16, 0x80ee);
	mvEthSwitchRegWrite(0, 0x11, 0x16, 0x80ee);
	mvEthSwitchRegWrite(0, 0x12, 0x16, 0x80ee);
	mvEthSwitchRegWrite(0, 0x13, 0x16, 0x80ee);
	mvEthSwitchRegWrite(0, 0x14, 0x16, 0x80ee);

	return 0;	
}

int leds_on_ethernet_ports(char *proc_buff)
{
	int retVal = 0;

	if (strncmp("on", proc_buff, strlen("on")) == 0) {
		mvEthSwitchRegWrite(0, 0x10, 0x16, 0x80ff);
		mvEthSwitchRegWrite(0, 0x11, 0x16, 0x80ff);
		mvEthSwitchRegWrite(0, 0x12, 0x16, 0x80ff);
		mvEthSwitchRegWrite(0, 0x13, 0x16, 0x80ff);
		mvEthSwitchRegWrite(0, 0x14, 0x16, 0x80ff);
	} else if (strncmp("default", proc_buff, strlen("default")) == 0) {
		mvEthSwitchRegWrite(0, 0x10, 0x16, 0x8088);
		mvEthSwitchRegWrite(0, 0x11, 0x16, 0x8088);
		mvEthSwitchRegWrite(0, 0x12, 0x16, 0x8088);
		mvEthSwitchRegWrite(0, 0x13, 0x16, 0x8088);
		mvEthSwitchRegWrite(0, 0x14, 0x16, 0x8088);
	} else {
		retVal = 1;	
	}

	return retVal;
}

int leds_off_all(void)
{
	int on_off;
	leds_t *ledp;

        if (!ledtblp) return EINVAL;

	for (ledp = ledtblp; ledp->name != NULL; ledp++) {
		if (ledp->name != NULL) {
			on_off = !(ledp->gpio_polarity); /* off */	
			set_leds(ledp, on_off, MV_GPP_IN_ORIGIN);
		}
	}
}

static int leds_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	if (ledtblp)
		return sprintf(page, usage_str);
	else
		return sprintf(page, "Board type not supported.");
}

static int leds_proc_write(struct file * file, const char __user * buffer, unsigned long count, void *data)
{
	char proc_buff[64];
	int idx;

	if (copy_from_user(proc_buff, buffer,count))
		return -EFAULT;

	for (idx = 0; idx < 64; idx++)
		if (proc_buff[idx] == '\n')
			break;
	proc_buff[idx] = '\0';

        if (!ledtblp) return count;

	if (strncmp("off", proc_buff, strlen("off")) == 0)
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID)
			leds_off_ethernet_ports();
		else
			leds_off_all();
	else
		if (bdutil_gboardId == VP_88F6282A_BP_ID || bdutil_gboardId == AD_88F6282A_BP_ID)
			leds_on_ethernet_ports(proc_buff);
		else
			ledact(proc_buff);

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

	leds_init_data();

	if (bdutil_gboardId == P2_88F6282A_BP_ID) {
		leds_blue_off();
		leds_white_off();
	}

	return status;
}

void exit_leds(void)
{
        if (leds_proc_node)
                remove_proc_entry(BDUTIL_LEDS_PROC_DEVICE_NAME, proc_bdutil);

        LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}
