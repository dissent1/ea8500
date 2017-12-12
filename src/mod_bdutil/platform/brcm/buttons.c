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
  @file buttons.c
  @brief board driver for board button support 
*************************************************************************/

#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/smp_lock.h>

#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/spinlock.h>

#include <linux/sched.h>
#include <linux/stat.h>

#include <linux/timer.h>
#include <linux/stat.h>

#include <osl.h>
#include <bcmgpio.h>
#include <typedefs.h>
#include <siutils.h>

#include "../../debug.h"
#include "buttons.h"

#define DRV_DESC	"bdutil board utility buttons kernel module"
#define DRV_VERSION	"1.3"
#define DEVICE_NAME	"buttons"

#ifdef _DEBUG
static int counter = 0;
#endif

/* Prototypes */
static long buttons_ioctl(struct file *, unsigned int, unsigned long);
static int buttons_open(struct inode *, struct file *);
static ssize_t buttons_read(struct file *, char __user *, size_t, loff_t *);
static u_int32_t buttons_poll(struct file *, poll_table *);
static int buttons_release(struct inode *, struct file *);
static irqreturn_t factory_reset_irq_handler(int, void *);
#ifdef WIFIBUTTON
static irqreturn_t WIFI_irq_handler(int irq, void *dev);
#endif
static irqreturn_t buttons_irq_handler(int, void *);
static inline int buttons_signal_read(struct buttons_private *);
static inline int buttons_lookup_pin(struct buttons_dev *);
static int gpio_get_value(int);

extern u_int32_t bdutil_gboardId;
static u_int32_t board_gpio_reset_pin = 0;
static int major = 0;
static int bdutil_reset_irq = 0;
static LIST_HEAD(buttons);

//extern void gpio_free(unsigned int gpio);
#ifndef NO_IRQ
#define NO_IRQ  ((unsigned int)(-1))
#endif
int rst_btn_irq_handled = -1;
si_t *gpio_sih;
int mod_bdutil_opened = 0;

typedef struct timer_list Timer;
static Timer rst_btn_timer;
//static int rst_timer = 0;
static int rst_btn_check_freq = 4;        /* 2 sec. default*/
#ifdef WIFIBUTTON
static Timer rst_btnwifi_timer;
static int rst_btnwifi_check_freq = 1;     /* 100 ms, default*/
#endif
static struct buttons_dev button_irqs[] = {
	{ 0,				/* irq */
	  "factory_reset",		/* irq name */
	  0,				/* assigned_irq */
	  0,				/* irq_state */
	  IRQ_TYPE_LEVEL_LOW,		/* irq_setting */
	  BOARD_GPIO_FACTORY_RESET_PIN,	/* gpio_pin */
	  0,				/* buttons_flags */
	  factory_reset_irq_handler },	/* irq_handler_func() */
#ifdef WIFIBUTTON
	{ 1,				/* irq */
	  "WIFI_ONOFF",		/* irq name */
	  0,				/* assigned_irq */
	  0,				/* irq_state */
	  IRQ_TYPE_LEVEL_LOW,		/* irq_setting */
	  BOARD_GPIO_WIFI_PIN,	/* gpio_pin */
	  0,				/* buttons_flags */
	  WIFI_irq_handler }	/* irq_handler_func() */
#endif
};

static void rst_btn_counter(void)
{
	int value;

	DBGX(1, "%s: check interval %d\n", __FUNCTION__, rst_btn_check_freq);

	value = gpio_get_value(BOARD_GPIO_FACTORY_RESET_PIN);
	if (!value) {
		DBGX(1, "%s: (%d) Reset button pressed!\n", __FUNCTION__, value);
		buttons_irq_handler(BOARD_GPIO_FACTORY_RESET_PIN, NULL);
		rst_btn_irq_handled = 1;
	} else {
		rst_btn_irq_handled = 0;
		DBGX(1, "%s: (%d) Reset button not detected!\n", __FUNCTION__, value);
	}

	rst_btn_timer.expires = jiffies + rst_btn_check_freq*(HZ);
	add_timer(&rst_btn_timer);
}
#ifdef WIFIBUTTON
static void rst_btnwifi_counter(void)
{
	int value;

	DBGX(1, "%s: check interval %d\n", __FUNCTION__, rst_btn_check_freq);

	value = gpio_get_value(BOARD_GPIO_WIFI_PIN);
	if (!value) {
		DBGX(1, "%s: (%d) Reset button pressed!\n", __FUNCTION__, value);
		buttons_irq_handler(BOARD_GPIO_WIFI_PIN, NULL);
		rst_btn_irq_handled = 1;
	} else {
		rst_btn_irq_handled = 0;
		DBGX(1, "%s: (%d) Reset button not detected!\n", __FUNCTION__, value);
	}

	rst_btnwifi_timer.expires = jiffies + (rst_btnwifi_check_freq  * (HZ) / 10);//100ms
	add_timer(&rst_btnwifi_timer);
}
#endif
static void free_rst_btn_resources(void)
{
        del_timer_sync(&rst_btn_timer);
#ifdef WIFIBUTTON
		del_timer_sync(&rst_btnwifi_timer);
#endif
}

static void rst_btn_set_timer(void)
{
	//if (rst_timer == 1) {
	//	rst_timer = 0;
	//	free_rst_btn_resources();
	//}
	//rst_timer = 1;
	init_timer(&rst_btn_timer);
	rst_btn_timer.function = rst_btn_counter;
	rst_btn_timer.data = (unsigned int)NULL;
	rst_btn_timer.expires = jiffies + rst_btn_check_freq*(HZ);
#ifdef WIFIBUTTON
	init_timer(&rst_btnwifi_timer);
	rst_btnwifi_timer.function = rst_btnwifi_counter;
	rst_btnwifi_timer.data = (unsigned int)NULL;
	rst_btnwifi_timer.expires = jiffies + (rst_btnwifi_check_freq  * (HZ) / 10);//100ms

	add_timer(&rst_btnwifi_timer);
#endif
	add_timer(&rst_btn_timer);
}

static int gpio_get_value(int gpio_pin)
{
	int value;
	unsigned int gpio_mask = ((unsigned int) 1 << gpio_pin);
	unsigned int get;

	si_gpiosetcore(gpio_sih);
	si_gpioouten(gpio_sih, gpio_mask, 0, GPIO_DRV_PRIORITY);
	get = si_gpioin(gpio_sih);
	get &= gpio_mask;
	//value = get ? 0 : 1;
	value = get ? 1 : 0;

	DBGX(1, "%s: value %d\n", __FUNCTION__, value);

	return value;
}

static inline int buttons_signal_read(struct buttons_private *priv)
{
	int i;
	int j = -1;

	for (i = 0; i < (sizeof(button_irqs) / sizeof(struct buttons_dev)); i++)
	{
		if (priv->thread_event_id == button_irqs[i].assigned_irq) {
			
			j = gpio_get_value(button_irqs[i].gpio_pin);
			break;
		}
	}

	DBGX(1, "%s: gpio_get_value is %d\n", __FUNCTION__, j);

	return j;
}

static inline int buttons_lookup_pin(struct buttons_dev *bdev)
{
	switch (bdev->irq) {
	case 0:
		return BOARD_GPIO_FACTORY_RESET_PIN;
#ifdef WIFIBUTTON
	case 1:
		return BOARD_GPIO_WIFI_PIN;
#endif
	default:
		return -1;
	}
}

static int buttons_longpress_kernel_thread(void *arg)
{
	struct buttons_private *priv = (struct buttons_private *)arg;
	unsigned long start_jiffies = 0;
	u_int32_t bsignal;

	DBGX(1, "started kernel thread for pid: %d\n", priv->my_pid);
	lock_kernel();
	daemonize(priv->name);
	allow_signal(SIGKILL);
	priv->thread = current;
	unlock_kernel();

	complete(&priv->thread_done);

	/*
	 * callers processes kernel thread sleeps waiting for a button
	 * event to wake it up
	 */
	do {
		priv->thread_event_id = 0;
		DBGX(1, "thread going to sleep\n");
		wait_event_interruptible(priv->longpress_waitq,
					 priv->thread_event_id);
		DBGX(1, "%s: wake_thread -> event_id = %ld\n",
					__FUNCTION__, priv->thread_event_id);
		if (priv->thread_event_id &&
		    priv->thread_event_id != LEAVE_THREAD) {
			priv->thread_event_id = (priv->thread_event_id == RESET_REMAP ? 0 : priv->thread_event_id);
			DBGX(1, "%s: do for pid: %d, thread_event_id %lu\n",
					__FUNCTION__, priv->my_pid,
					priv->thread_event_id);
			start_jiffies = jiffies;

			/*
			 * Check to see if RESET button is held-down for greater than 5 sec.
			 * indicating a factory_reset operation.
			 */
			bsignal = buttons_signal_read(priv);
			if (bsignal < 0) {
				ERR("%s() - ERROR reading button pin.\n",
					__FUNCTION__);
				break;
			}
			for (;bsignal != BUTTON_RELEASED;) {
				bsignal = buttons_signal_read(priv);
				if (bsignal < 0) {
					ERR("%s() -ERROR reading button pin.\n",
						__FUNCTION__);
					break;
				}
				if (bsignal == BUTTON_RELEASED) break;
				DBGX(2, "%s longpress BUTTON_PRESSED(%d).\n",
						__FUNCTION__, bsignal);
				
				/* check for a button press and held down for 10 seconds */
				if (time_after(jiffies, start_jiffies + msecs_to_jiffies(8500))) {
					DBGX(1, "*********** Got LONGPRESS for pid: %d ************\n", priv->my_pid);
					priv->eventmask |=
							EVENT_BUTTON_LONGPRESS;
					break;
				} else {
					DBGX(1, "%s longpress wait >10 sec. for pid %d\n", __FUNCTION__, priv->my_pid);
					set_current_state(TASK_INTERRUPTIBLE);
					schedule_timeout_interruptible(50 * HZ / 1000);
				}
			}

			/* record the button event */
			priv->eventmask |= 
				(priv->thread_event_id == button_irqs[0].assigned_irq) ? EVENT_BUTTON_FACTORY_RESET :
				(priv->thread_event_id == button_irqs[1].assigned_irq) ? EVENT_BUTTON_WIFI : 0;

			DBGX(2, "%s: **** eventmask 0x%08x ****\n",
					__FUNCTION__, priv->eventmask);

			if (priv->eventmask)
				wake_up_interruptible(&priv->waitq);
		} else {
			DBGX(1, "%s: else for pid: %d, thread_event_id %lu\n",
					__FUNCTION__, priv->my_pid,
					priv->thread_event_id);
			priv->thread_event_id = LEAVE_THREAD;
		}
	} while (priv->thread_event_id != LEAVE_THREAD);

	DBGX(1, "leaving kernel thread for pid: %d\n", priv->my_pid);

	complete_and_exit(&priv->thread_done, 0);
}

static irqreturn_t buttons_irq_handler(int irq, void *dev)
{
	int i; 
	struct list_head *pos, *temp;

#ifdef _DEBUG
	int cnt = 0;

	counter += 1;
	DBGX(2, "counter: %d\n", counter );
#endif

	DBGX(2, "%s: entered\n", __FUNCTION__);

	if (rst_btn_irq_handled == 1) return;

	rst_btn_irq_handled = 1;

	/*
	 * service the button signal
	 */
	for (i = 0; i < (sizeof(button_irqs) / sizeof(struct buttons_dev)); i++)
	{
		if (irq == button_irqs[i].assigned_irq) {
			if (!(button_irqs[i].irq_handler_func(irq, dev)))
				return IRQ_NONE;
			break;
		}
	}

	/*
	 * Send the type of button "event" (eventmask) to the reader/poller
	 * kernel threads
	 */
	list_for_each_safe(pos, temp, &buttons) {
		struct buttons_private* priv =
			list_entry(pos, struct buttons_private, list);
#ifdef _DEBUG
		DBGX(2,"%s: member=(%d) NAME=%s\n",__FUNCTION__,cnt,priv->name);
		cnt++;
		DBGX(2, "irq %d sent to pid %d\n", irq, priv->my_pid);
#endif
		priv->thread_event_id = irq;

		if (priv->thread_event_id) {
			DBGX(1, "%s: thread_event_id %lu\n",
				__FUNCTION__, priv->thread_event_id );
			wake_up_interruptible(&priv->longpress_waitq);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t factory_reset_irq_handler(int irq, void *dev)
{
	u_int32_t irq_polarity;

	DBGX(2, "%s: entered\n", __FUNCTION__);

	return IRQ_HANDLED; 
}
#ifdef WIFIBUTTON
static irqreturn_t WIFI_irq_handler(int irq, void *dev)
{
	u_int32_t irq_polarity;

	DBGX(2, "%s: entered\n", __FUNCTION__);

	return IRQ_HANDLED; 
}
#endif
static struct file_operations buttons_fops = {
	.owner = THIS_MODULE,
	.read = buttons_read,
	.poll = buttons_poll,
	.unlocked_ioctl = buttons_ioctl,
	.open = buttons_open,
	.release = buttons_release
};

static long buttons_ioctl(struct file *file, unsigned int cmd,
                                unsigned long arg)
{
	int ret = -ENOTTY;

	switch (cmd) {
	case BDUTIL_SETRSTTIMEOUT:
		DBGX(1, "%s: BDUTIL_SETRSTTIMEOUT switch\n", __FUNCTION__);
		free_rst_btn_resources();
		ret = 0;
		break;
	default:
		DBGX(1, "%s: default switch\n", __FUNCTION__);
		break;
	}

	return ret;
}


/* creates a kernel thread to read and report button "events" */
static int buttons_open(struct inode *inode, struct file *filp)
{
	struct buttons_private *priv;
	int status;

	DBGX(1, "%s: entering\n", __FUNCTION__); 
	if (mod_bdutil_opened) return;

	priv = kzalloc(sizeof(struct buttons_private), GFP_KERNEL);

	if (priv == NULL) {
		ERR("%s: could not allocate memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	priv->name   = "button_thread";
	init_completion(&priv->thread_done);
	priv->thread = NULL;
	priv->my_pid = current->pid;
	init_waitqueue_head(&priv->waitq);
	init_waitqueue_head(&priv->longpress_waitq);

	filp->private_data = priv;
	list_add_tail(&priv->list, &buttons);
	DBGX(1, "%s: buttons_open(pid: %d)\n", __FUNCTION__, current->pid);
	kernel_thread(buttons_longpress_kernel_thread,
		      (void*)priv, CLONE_FS|CLONE_FILES);

	wait_for_completion(&priv->thread_done);

	rst_btn_set_timer();
	mod_bdutil_opened = 1;
	DBGX(1, "%s: exiting\n", __FUNCTION__); 
	return 0;
}

static ssize_t buttons_read(struct file *filp, char __user *buf,
			    size_t count, loff_t *f_pos)
{
	struct buttons_private* priv = filp->private_data;

	DBGX(1, "%s: (pid: %d)!\n", __FUNCTION__, priv->my_pid);
	wait_event_interruptible(priv->waitq, priv->eventmask);
	DBGX(1, "%s: -> wake-up, eventmask: 0x%08x\n", __FUNCTION__, priv->eventmask);

	if (copy_to_user(buf, &priv->eventmask,  sizeof(unsigned long)) ) {
		ERR("%s: - bad copy\n", __FUNCTION__);
		return -EFAULT;
	}
	priv->eventmask = 0;

	return sizeof(unsigned long);
}

static u_int32_t buttons_poll(struct file *filp, poll_table *wait)
{
	struct buttons_private* priv = filp->private_data;

	DBGX(1, "%s: (pid: %d)\n", __FUNCTION__, priv->my_pid);
	poll_wait(filp, &priv->waitq, wait);
	if (priv->eventmask)
		return (POLLIN | POLLRDNORM);
	return 0;
}

static int buttons_release(struct inode *inode, struct file *filp)
{
	struct buttons_private* priv = filp->private_data;

	DBGX(1, "%s: (pid: %d)\n", __FUNCTION__, priv->my_pid);
	free_rst_btn_resources();
	priv->thread_event_id = LEAVE_THREAD;
	wake_up_interruptible(&priv->longpress_waitq);
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout_interruptible(50 * HZ / 1000);

	list_del(&priv->list);
	kfree(priv);

	return 0;
}

int init_buttons(void)
{
	int i;
	int status = 0;

	unsigned long gpio_mask;

    LOG(DRV_DESC ", version " DRV_VERSION"\n");

	major = register_chrdev(0, DEVICE_NAME, &buttons_fops);
	DBGX(1, "%s: dev entry =  %s\n", __FUNCTION__, DEVICE_NAME);
	DBGX(1, "%s: major =  %d\n", __FUNCTION__, major);

	gpio_mask = ((unsigned long)1 << BOARD_GPIO_FACTORY_RESET_PIN);
	if (!(gpio_sih = si_kattach(SI_OSH))) 
	{
		printk(KERN_ERR "Error connecting GPIO %d to reset button\n", BOARD_GPIO_FACTORY_RESET_PIN);
		return -ENODEV;
	}

	button_irqs[0].assigned_irq = BOARD_GPIO_FACTORY_RESET_PIN;
#ifdef WIFIBUTTON
	button_irqs[1].assigned_irq = BOARD_GPIO_WIFI_PIN;
#endif	

	/*
	 * At system boot up, mv_gpio_init(); called by
	 * arch/arm/mach-feroceon-kw/core.c, initialized the Marvell
	 * SoC GPIO subsystem and registered the GPIO API struct
	 * (struct gpio_chip.label = "mv_gpio")
	 */
	
	return status;
}

void exit_buttons(void)
{
	int i;

#if 0
	struct list_head *pos, *temp;

	list_for_each_safe(pos, temp, &buttons) {
		struct buttons_private* priv =
			list_entry(pos, struct buttons_private, list);
	        list_del(&priv->list);
		kfree(priv);
	}
#endif

	free_rst_btn_resources();
	for (i = 0; i < (sizeof(button_irqs) / sizeof(struct buttons_dev)); i++)
	{
		if (button_irqs[i].assigned_irq != NO_IRQ) {	
			;
			//free_irq(button_irqs[i].assigned_irq, NULL);
			//gpio_free(button_irqs[i].gpio_pin);
		}
	}

	unregister_chrdev(major, DEVICE_NAME);

	si_detach(gpio_sih);

        LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}
