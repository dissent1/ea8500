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
  @brief board driver for LEDs
*************************************************************************/

#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>

#include <linux/delay.h>

#include <linux/gpio.h>

#include "../../debug.h"
#include "leds.h"
#include "mtk_io.h"

#define DRV_DESC	"bdutil board utility LED kernel module"
#define DRV_VERSION	"1.0"
#define DEVICE_NAME	"leds"

#define SWITCH_LED_ON      0
#define SWITCH_LED_OFF     1
#define SWITCH_LED_DEFAULT 2
#define GPIO_PWMLED        9
#define WPS_LED_BLINKING_TIME  120000   /* 120 seconds */

extern struct proc_dir_entry *proc_bdutil;
static struct proc_dir_entry *leds_proc_node;

static int pwmled_is_on = 1;
static struct timer_list timer_pwmled_cleanup;

static inline void PwmLedOn(void);
static int leds_proc_read(char *, char **, off_t, int, int *, void *);
static int leds_proc_write(struct file *, const char __user *, unsigned long, void *);

static char usage_str[] =
{
  "Usage: echo pwm=[on|off|blink] > /proc/bdutil/leds\n"
  "       echo wps=[on|off|altblink|blink] > /proc/bdutil/leds\n"
  "       echo lan=[on|off|default] > /proc/bdutil/leds\n"
  "\n"
};

struct proc_cmd_definition {
  char str_cmd[32];
  int  id_cmd;
};

enum enum_led_cmds {
  CMD_LED_PWM_ON,
  CMD_LED_PWM_OFF,
  CMD_LED_PWM_BLINK,
  CMD_LED_ALL_ON,
  CMD_LED_ALL_OFF,
  CMD_LED_WPS_ON,
  CMD_LED_WPS_OFF,
  CMD_LED_WPS_BLINK,
  CMD_LED_WPS_ALT_BLINK,
  CMD_LED_LAN_ON,
  CMD_LED_LAN_OFF,
  CMD_LED_LAN_DEFAULT,
};

#define CMD_LED_MAX 99

static struct proc_cmd_definition proc_cmds[] = {
  { "pwm=on", CMD_LED_PWM_ON },
  { "pwm=off", CMD_LED_PWM_OFF },
  { "pwm=blink", CMD_LED_PWM_BLINK },
  { "all=on", CMD_LED_ALL_ON },
  { "all=off", CMD_LED_ALL_OFF },
  { "wps=on", CMD_LED_WPS_ON },
  { "wps=off", CMD_LED_WPS_OFF },
  { "wps=blink", CMD_LED_WPS_BLINK },
  { "wps=altblink", CMD_LED_WPS_ALT_BLINK },
  { "lan=on", CMD_LED_LAN_ON },
  { "lan=off", CMD_LED_LAN_OFF },
  { "lan=default", CMD_LED_LAN_DEFAULT },
  { "finalcommand", CMD_LED_MAX},
};

static inline void PwmLedOn(void)
{
  ralink_gpio_led_info led_info;

  led_info.gpio = GPIO_PWMLED;
  led_info.on = RALINK_GPIO_LED_INFINITY;
  led_info.off = 0;
  led_info.blinks = 0;
  led_info.rests = 0;
  led_info.times = 0;
  ralink_gpio_led_set(led_info);

  pwmled_is_on = 1;
}

static void PwmLedOff(void)
{
  ralink_gpio_led_info led_info;

  led_info.gpio = GPIO_PWMLED;
  led_info.on = 0;
  led_info.off = RALINK_GPIO_LED_INFINITY;
  led_info.blinks = 0;
  led_info.rests = 0;
  led_info.times = 0;
  ralink_gpio_led_set(led_info);

  pwmled_is_on = 0;
}

static void PwmLedBlink(void) /* blink always */
{
  ralink_gpio_led_info led_info;
  
  led_info.gpio = GPIO_PWMLED;
  led_info.on = 0;
  led_info.off = 0;
  led_info.blinks = RALINK_GPIO_LED_INFINITY;
  led_info.rests = 0;
  led_info.times = 0;
  ralink_gpio_led_set(led_info);
}

static void timer_func_pwmled(unsigned long arg)
{
  printk("Timer function setting the power LED to ");
  if (pwmled_is_on) {
    printk("ON.\n");
    PwmLedOn();
  } else {
    printk("OFF.\n");
    PwmLedOff();
  }

  return;
}

static void WpsBlink(void)   /* blink once a second */
{
  ralink_gpio_led_info led_info;

  led_info.gpio = 9;
  led_info.on = 7;
  led_info.off = 3;
  led_info.blinks = 0;
  led_info.rests = 0;
  led_info.times = 120;
  ralink_gpio_led_set(led_info);

  if (timer_pending(&timer_pwmled_cleanup))
    mod_timer(&timer_pwmled_cleanup, jiffies + msecs_to_jiffies(WPS_LED_BLINKING_TIME));
  else {
    timer_pwmled_cleanup.expires = jiffies + msecs_to_jiffies(WPS_LED_BLINKING_TIME); 
    add_timer(&timer_pwmled_cleanup);
  }

  return;
}

static void WpsAltBlink(void)  /* blink twice a second */
{
  ralink_gpio_led_info led_info;

  led_info.gpio = 9;
  led_info.on = 3;
  led_info.off = 2;
  led_info.blinks = 0;
  led_info.rests = 0;
  led_info.times = 240;
  ralink_gpio_led_set(led_info);

  if (timer_pending(&timer_pwmled_cleanup)) 
    mod_timer(&timer_pwmled_cleanup, jiffies + msecs_to_jiffies(WPS_LED_BLINKING_TIME));
  else {
    timer_pwmled_cleanup.expires = jiffies + msecs_to_jiffies(WPS_LED_BLINKING_TIME);
    add_timer(&timer_pwmled_cleanup);
  }
  return;
}

static void SwitchLedControl(int type)
{
  uint32_t tmp;

  switch (type) {
  case SWITCH_LED_ON:
  case SWITCH_LED_DEFAULT:
    tmp = MTK_READREG(0xb0000060);
    tmp = tmp & ~(1<<15);
    MTK_WRITEREG(0xb0000060, tmp);
    break;
  case SWITCH_LED_OFF:
    tmp = MTK_READREG(0xb0000060);
    tmp |= (1<<15);
    MTK_WRITEREG(0xb0000060, tmp);
    
    tmp = MTK_READREG(0xb0000674);
    tmp |= 0x1f;
    MTK_WRITEREG(0xb0000674, tmp);
    
    tmp = MTK_READREG(0xb000067C);
    tmp |= 0x1f;
    MTK_WRITEREG(0xb000067C, tmp);
    break;
  default:
    break;
  }

  return;
}
    
static void AllLedOn(void) /* Turn on all LEDs */
{
  PwmLedOn();
  SwitchLedControl(SWITCH_LED_ON);
}

static void AllLedOff(void) /* Turn off all LEDs */
{
  PwmLedOff();
  SwitchLedControl(SWITCH_LED_OFF);
}

static void cmd_led_control(int type)
{
  switch (type) {
  case CMD_LED_PWM_ON:
    PwmLedOn();
    break;
  case CMD_LED_PWM_OFF:
    PwmLedOff();
    break;
  case CMD_LED_PWM_BLINK:
    PwmLedBlink();
    break;
  case CMD_LED_ALL_ON:
    AllLedOn();
    break;
  case CMD_LED_ALL_OFF:
    AllLedOff();
    break;
  case CMD_LED_WPS_ON:
    PwmLedOn();
    break;
  case CMD_LED_WPS_OFF:
    PwmLedOn();
    break;
  case CMD_LED_WPS_BLINK:
    WpsBlink();
    break;
  case CMD_LED_WPS_ALT_BLINK:
    WpsAltBlink();
    break;
  case CMD_LED_LAN_ON:
    SwitchLedControl(SWITCH_LED_ON);
    break;
  case CMD_LED_LAN_OFF:
    SwitchLedControl(SWITCH_LED_OFF);
    break;
  case CMD_LED_LAN_DEFAULT:
    SwitchLedControl(SWITCH_LED_DEFAULT);
    break;
  default:
    printk("Unsupported LED control command.");
    break;
  }
}

static int leds_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
  return sprintf(page, usage_str);
}

static int leds_proc_write(struct file * file, const char __user * buffer, unsigned long count, void *data)
{
  int idx;
  char proc_buff[64];
  struct proc_cmd_definition *pcd;
  
  if (copy_from_user(proc_buff, buffer,count))
    return -EFAULT;
  
  for (idx = 0; idx < 64; idx++)
    if (proc_buff[idx] == '\n')
      break;
  proc_buff[idx] = '\0';

  pcd = proc_cmds;
  while (pcd->id_cmd != CMD_LED_MAX) {
    if (strncmp(pcd->str_cmd, proc_buff, strlen(pcd->str_cmd))==0) {
      cmd_led_control(pcd->id_cmd);
      break;
    }
    pcd++;
  }
  
  return count;
}

int init_leds(void)
{
  int status = 0;
  
  LOG(DRV_DESC ", version " DRV_VERSION"\n");

  init_timer(&timer_pwmled_cleanup);
  timer_pwmled_cleanup.function = timer_func_pwmled;
  timer_pwmled_cleanup.data = (unsigned int)NULL;
  pwmled_is_on = 1;
  
  leds_proc_node = create_proc_entry(BDUTIL_LEDS_PROC_DEVICE_NAME, 0666, proc_bdutil);
  if (leds_proc_node == NULL) {
    ERR("bdutil: Failed to create LEDS proc entry\n");
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
