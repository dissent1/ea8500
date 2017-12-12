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

#include <linux/sched.h>
#include <linux/stat.h>

#include <linux/timer.h>
#include <linux/stat.h>

#include <osl.h>
#include <bcmgpio.h>
#include <typedefs.h>
#include <siutils.h>

#include "../../debug.h"
#include "leds.h"

#define DRV_DESC	"bdutil board utility LEDs kernel module"
#define DRV_VERSION	"1.3"
#define DEVICE_NAME	"leds"

#define ON 		1
#define OFF 		0

extern struct proc_dir_entry *proc_bdutil;
static struct proc_dir_entry *leds_proc_node;

si_t *gpio_led_sih;

static leds_t *ledtblp;
static char *usage_str;

typedef struct timer_list Timer;
static Timer blink_timer;
static int blink_toggle = 0;
static int status_blink_timer = 0;

static int led_on(void);
static int led_off(void);
static void blink_set_expire(void);
static void blink_counter(void);
static void blink_set_timer(void);
static void blink_free_timer(void);

#define MODE_BLINK 	   0
#define MODE_PULSATE   1
#define MODE_WIFILED   4

static int blink_mode = MODE_PULSATE;
static int wifi_led_status = OFF;

#if defined( _CARRERA )

#define DEFAULT_BLINK_DECISEC 4   /* 1 = 100ms */
#define POWERUP_PULSATE_DECISEC 13   /* 1 = 100ms */
static int blink_decisec = POWERUP_PULSATE_DECISEC; /* 1 = 100ms */

static leds_t leds_gpio_carrera[] =
{  /* Carrera */
};

static char usage_str_carrera[] =
{	/* Carrera */
	"Usage: echo \"[on|off|blink_on|blink_off]\" > /proc/bdutil/leds\n"
	"       echo \"timeron=[1-50]\" > /proc/bdutil/leds\n"
	"       echo timeroff > /proc/bdutil/leds\n"
};

static int leds_init_data(void)
{
	DBGX(1, "%s: enter\n", __FUNCTION__);
	ledtblp = leds_gpio_carrera;	
	usage_str = usage_str_carrera;

	return 0;
}

static void start_flash(int type)
{
	u32 val_out, val_en, val_ctrl, real_out;
	u32 ctr_mask, out_mask;

	val_out = si_gpioout(gpio_led_sih, 0, 0, GPIO_DRV_PRIORITY);
	val_en = si_gpioouten(gpio_led_sih, 0, 0, GPIO_DRV_PRIORITY);
	val_ctrl = si_gpiocontrol(gpio_led_sih, 0, 0, GPIO_DRV_PRIORITY);

	//GPIO 1, 2
	ctr_mask= ~(1 << 1| (1 << 2));
	out_mask= (1 << 1 | (1 << 2));
	real_out = ~(1 << 2);

	si_gpioouten(gpio_led_sih, ~0, val_en | out_mask, GPIO_DRV_PRIORITY);
	si_gpiocontrol(gpio_led_sih, ~0, val_ctrl & ctr_mask, GPIO_DRV_PRIORITY);

	if(type == ON) {
		si_gpioout(gpio_led_sih, ~0, ((val_out | out_mask) & real_out), GPIO_DRV_PRIORITY);
	} else{ 
		si_gpioout(gpio_led_sih, ~0, val_out | out_mask, GPIO_DRV_PRIORITY);
	}
}

#elif defined(_HONDA) || defined(_F70) || defined(_SPYDER)
/* honda */
#define DEFAULT_BLINK_DECISEC 7   /* 1 = 100ms */
#define POWERUP_PULSATE_DECISEC 13   /* 1 = 100ms */
static int blink_decisec = POWERUP_PULSATE_DECISEC; /* 1 = 100ms */

static leds_t leds_gpio_honda[] =
{
};

static char usage_str_honda[] =
{
	"Usage: echo \"[on|off|blink_on|blink_off]\" > /proc/bdutil/leds\n"
	"       echo \"[pulsate_on|pulsate_off]\" > /proc/bdutil/leds\n"
	"       echo \"timeron=[1-50]\" > /proc/bdutil/leds\n"
	"       echo timeroff > /proc/bdutil/leds\n"
	"       echo \"gpio_on=[n]\" > /proc/bdutil/leds\n"
	"       echo \"gpio_off=[n]\" > /proc/bdutil/leds\n"
};

static int leds_init_data(void)
{
	DBGX(1, "%s: enter\n", __FUNCTION__);
	ledtblp = leds_gpio_honda;
	usage_str = usage_str_honda;

	return 0;
}

#else
/* bentley */
/* lemans */
/* esprit */
/* r8 */
#define DEFAULT_BLINK_DECISEC 7   /* 1 = 100ms */
#define POWERUP_PULSATE_DECISEC 13   /* 1 = 100ms */
static int blink_decisec = DEFAULT_BLINK_DECISEC; /* 1 = 100ms */

static leds_t leds_gpio_bentley[] =
{
};

static char usage_str_bentley[] =
{
	"Usage: echo \"[on|off|blink_on|blink_off]\" > /proc/bdutil/leds\n"
	"       echo \"timeron=[1-50]\" > /proc/bdutil/leds\n"
	"       echo timeroff > /proc/bdutil/leds\n"
	"       echo \"gpio_on=[n]\" > /proc/bdutil/leds\n"
	"       echo \"gpio_off=[n]\" > /proc/bdutil/leds\n"
};

static int leds_init_data(void)
{
	DBGX(1, "%s: enter\n", __FUNCTION__);
	ledtblp = leds_gpio_bentley;
	usage_str = usage_str_bentley;

	return 0;
}

#endif

static void blink_set_expire(void)
{
	blink_timer.expires = jiffies + (blink_decisec  * (HZ) / 10);
	/*DBGX(1, "%s: blink_timer.expires =%d\n", __FUNCTION__, blink_timer.expires );*/
}

static void blink_counter(void)
{
	if (blink_toggle) {
		/*VpPwmLedOn()*/
		led_on();
	} else {
		/*VpPwmLedOff();*/
		led_off();
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

static int gpio_set_value(int gpio_pin, int value)
{
	unsigned int gpio_mask = ((unsigned int) 1 << gpio_pin);
	/*unsigned int gpio_mask = ((unsigned int) 1 << BOARD_GPIO_WPS_LED_PIN);*/

	si_gpiosetcore(gpio_led_sih);
	si_gpioouten(gpio_led_sih, gpio_mask, gpio_mask, GPIO_HI_PRIORITY);
	si_gpioout(gpio_led_sih, gpio_mask, (value ? gpio_mask : 0), GPIO_HI_PRIORITY);

	return 0;
}

static int led_on(void)
{
#if defined( _CARRERA )
	start_flash(ON);
#elif defined(_HONDA) || defined(_F70) 
	if (blink_mode == MODE_PULSATE) {
		gpio_set_value(8, 0);
		gpio_set_value(6, 0);
	} else { /* MODE_BLINK */
		gpio_set_value(6, 1);
		gpio_set_value(8, 1);
	}
#elif defined(_SPYDER)
	if (blink_mode == MODE_PULSATE) {
		gpio_set_value(BOARD_GPIO_WPS_LED_PIN, 1);
	} else { /* MODE_BLINK */
		gpio_set_value(BOARD_GPIO_WPS_LED_PIN, 1);
	}
#else
	gpio_set_value(BOARD_GPIO_WPS_LED_PIN, 1);
#endif
	return 0;
}

static int led_on_wifi(void)
{
#if defined(_SPYDER)
	if(blink_mode == MODE_WIFILED) {
		gpio_set_value(BOARD_GPIO_WIFI_PIN, 0);
	}
#endif
	return 0;
}

static int led_off(void)
{
#if defined( _CARRERA )
	start_flash(OFF);
#elif defined(_HONDA) || defined(_F70) 
	if (blink_mode == MODE_PULSATE) {
		gpio_set_value(8, 0);
		gpio_set_value(6, 1);
	} else { /* MODE_BLINK */
		gpio_set_value(6, 1);
		gpio_set_value(8, 0);
	}
#elif defined(_SPYDER)
	if (blink_mode == MODE_PULSATE) {
		gpio_set_value(BOARD_GPIO_WPS_LED_PIN, 0);
	} else { /* MODE_BLINK */
		gpio_set_value(BOARD_GPIO_WPS_LED_PIN, 0);
	}
#else
	gpio_set_value(BOARD_GPIO_WPS_LED_PIN, 0);
#endif
	return 0;
}

static int led_off_wifi(void)
{
#if defined(_SPYDER)
	if(blink_mode == MODE_WIFILED) {
		gpio_set_value(BOARD_GPIO_WIFI_PIN, 1);
	}
#endif
	return 0;
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

	if (strncmp("on", proc_buff, strlen("on")) == 0) {
		led_on();
	} else if (strncmp("off", proc_buff, strlen("off")) == 0) {
		led_off();
	} else if (strncmp("blink_on", proc_buff, strlen("blink_on")) == 0) {
		blink_mode = MODE_BLINK;
		blink_decisec = DEFAULT_BLINK_DECISEC; /* 1 = 100ms */
		blink_set_timer();
	} else if (strncmp("blink_off", proc_buff, strlen("blink_off")) == 0) {
		blink_mode = MODE_BLINK;
		blink_free_timer();
	} else if (strncmp("wifi_led_on", proc_buff, strlen("wifi_led_on")) == 0) {
		blink_mode = MODE_WIFILED;
		wifi_led_status = ON;
		led_on_wifi();
	} else if (strncmp("wifi_led_off", proc_buff, strlen("wifi_led_off")) == 0) {
		blink_mode = MODE_WIFILED;
		wifi_led_status = OFF;
		led_off_wifi();
	} else if(strncmp("wifi_led_reverse", proc_buff, strlen("wifi_led_off")) == 0){
		blink_mode = MODE_WIFILED;
		if(wifi_led_status == ON)
		{
			wifi_led_status = OFF;
			led_off_wifi();
		}
		else
		{
			wifi_led_status = ON;
			led_on_wifi();
		}
	} else if (strncmp("pulsate_on", proc_buff, strlen("pulsate_on")) == 0) {
		blink_mode = MODE_PULSATE;
		blink_decisec = POWERUP_PULSATE_DECISEC;
		blink_set_timer();
	} else if (strncmp("pulsate_off", proc_buff, strlen("pulsate_off")) == 0) {
		blink_mode = MODE_PULSATE;
		blink_free_timer();
	} else if (strncmp("timeron=", proc_buff, strlen("timeron=")) == 0) {
		int usr_input = 0;
		DBGX(1, "%s: timeron \n", __FUNCTION__);
		sscanf((char *)proc_buff, "timeron=%d", &usr_input);
		if ((usr_input >= 1) && (usr_input  <= 50)) {
			DBGX(1, "%s: usr_input =%d\n", __FUNCTION__, usr_input );
			blink_decisec = usr_input;
			blink_set_timer();
		}
	} else if (strncmp("timeroff", proc_buff, strlen("timeroff")) == 0) {
		blink_free_timer();
		DBGX(1, "%s: timeroff \n", __FUNCTION__);
	} else if (strncmp("gpio_on=", proc_buff, strlen("gpio_on=")) == 0) {
		int usr_input = 0;
		DBGX(1, "%s: gpio_on \n", __FUNCTION__);
		sscanf((char *)proc_buff, "gpio_on=%d", &usr_input);
		if ((usr_input >= 1) && (usr_input  <= 50)) {
			DBGX(1, "%s: usr_input =%d\n", __FUNCTION__, usr_input );
			gpio_set_value(usr_input, 1);
		}
	} else if (strncmp("gpio_off=", proc_buff, strlen("gpio_off=")) == 0) {
		int usr_input = 0;
		DBGX(1, "%s: gpio_off \n", __FUNCTION__);
		sscanf((char *)proc_buff, "gpio_off=%d", &usr_input);
		if ((usr_input >= 1) && (usr_input  <= 50)) {
			DBGX(1, "%s: usr_input =%d\n", __FUNCTION__, usr_input );
			gpio_set_value(usr_input, 0);
		}
	}

        return count;
}

int init_leds(void)
{
	int status = 0;
        LOG(DRV_DESC ", version " DRV_VERSION"\n");

	leds_proc_node = create_proc_entry(BDUTIL_LEDS_PROC_DEVICE_NAME, 0666, proc_bdutil);
	if (leds_proc_node == NULL) {
		ERR("bdutil_gpio: Failed to create LEDs proc entry\n");
		return ENOENT;
	}
	leds_proc_node->read_proc = leds_proc_read;
	leds_proc_node->write_proc = leds_proc_write;

	leds_init_data();

	if (!(gpio_led_sih = si_kattach(SI_OSH)))
	{
		printk(KERN_ERR "Error connecting GPIO %d for wps led\n", BOARD_GPIO_WPS_LED_PIN);
		return -ENODEV;
	}

	blink_set_timer();

	return status;
}

void exit_leds(void)
{
        if (leds_proc_node)
                remove_proc_entry(BDUTIL_LEDS_PROC_DEVICE_NAME, proc_bdutil);

	si_detach(gpio_led_sih);

        LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}
