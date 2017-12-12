/*
 * Copyright (c) 2013,    Belkin International, Inc.
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
#include <linux/gpio.h>
#include <mach/irqs.h>

#include <common/mvCommon.h>
#include <gpp/mvGpp.h>
#include <boardEnv/mvBoardEnvSpec.h>
#include <linux_oss/mvOs.h>

#include "../../debug.h"
#include "buttons.h"
#include "gpio.h"

#define DRV_DESC    "bdutil board utility buttons kernel module"
#define DRV_VERSION "1.4"
#define DEVICE_NAME "buttons"

#ifdef _DEBUG
static int counter = 0;
#endif

extern u_int32_t bdutil_gboardId;
static int major = 0;
static LIST_HEAD(ButtonsList);
static DEFINE_MUTEX(BDButtonsLock);

/* ---------------------------------------------------------------------------
 * Prototypes
 * ------------------------------------------------------------------------- */
static inline int  buttons_signal_read(struct buttons_private* priv);
static inline int  buttons_lookup_pin(struct buttons_dev* bdev);
static int         buttons_longpress_kernel_thread(void* arg);
static irqreturn_t buttons_irq_handler(int irq, void* dev);
static irqreturn_t factory_reset_irq_handler(int irq, void* dev);
static irqreturn_t WPS_irq_handler(int irq, void* dev);

static long      buttons_ioctl(struct file* file, unsigned int cmd,
                               unsigned long arg);
static int       buttons_open(struct inode* inode, struct file* filp);
static ssize_t   buttons_read(struct file* filp, char __user* buf, size_t count,
                              loff_t* f_pos);
static u_int32_t buttons_poll(struct file* filp, poll_table* wait);
static int       buttons_release(struct inode* inode, struct file* filp);

/* ---------------------------------------------------------------------------
 * Buttons List
 * ------------------------------------------------------------------------- */
static struct buttons_dev ButtonIrqs[] = 
{
    {
        0,                            /* irq */
        "factory_reset",              /* irq name */
        0,                            /* assigned_irq */
        0,                            /* irq_state */
        IRQ_TYPE_LEVEL_LOW,           /* irq_setting */
        BOARD_GPIO_FACTORY_RESET_PIN, /* gpio_pin */
        0,                            /* buttons_flags */
        factory_reset_irq_handler     /* irq_handler_func() */
    },   
    {
        1,                            /* irq */
        "WPS",                        /* irq name */
        0,                            /* assigned_irq */
        0,                            /* irq_state */
        IRQ_TYPE_LEVEL_LOW,           /* irq_setting */
        BOARD_GPIO_WPS_PIN,           /* gpio_pin */
        0,                            /* buttons_flags */
        WPS_irq_handler               /* irq_handler_func() */
    }
};

/* ---------------------------------------------------------------------------
 * Read button IRQ
 * ------------------------------------------------------------------------- */
static inline int buttons_signal_read(struct buttons_private* priv)
{
    int i;
    u32 gpp = -1;
    u32 gppVal = -1;

    for (i = 0; i < (sizeof(ButtonIrqs) / sizeof(struct buttons_dev)); i++)
    {
        if (priv->thread_event_id == ButtonIrqs[i].assigned_irq)
        {
            gpp = ButtonIrqs[i].assigned_irq - IRQ_GPP_START;
            gppVal = mv_gpp_value_real_get(GPP_GROUP(gpp), GPP_BIT(gpp));
            break;
        }

    }

    return gppVal;
}

/* ---------------------------------------------------------------------------
 * Given the IRQ, return associated GPIO pin
 * ------------------------------------------------------------------------- */
static inline int buttons_lookup_pin(struct buttons_dev* bdev)
{
    switch (bdev->irq)
    {
    case 0:
        return BOARD_GPIO_FACTORY_RESET_PIN;
        break;

    case 1:
        return BOARD_GPIO_WPS_PIN;
        break;

    default:
        return -1;
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Detect & handle button long press
 * ------------------------------------------------------------------------- */
static int buttons_longpress_kernel_thread(void* arg)
{
    struct buttons_private *priv = (struct buttons_private *)arg;
    unsigned long start_jiffies = 0;
    u_int32_t bsignal;

    DBGX(1, "started kernel thread for pid: %d\n", priv->my_pid);

    mutex_lock(&BDButtonsLock);
    daemonize(priv->name);
    allow_signal(SIGKILL);
    priv->thread = current;
    mutex_unlock(&BDButtonsLock);

    complete(&priv->thread_done);

    /*
     * callers processes kernel thread sleeps waiting for a button
     * event to wake it up
     */
    do
    {
        priv->thread_event_id = 0;
        DBGX(1, "thread going to sleep\n");
        wait_event_interruptible(priv->longpress_waitq,
                                 priv->thread_event_id);
        DBGX(1, "%s: wake_thread -> event_id = %ld\n",
                __FUNCTION__, priv->thread_event_id);

        if (priv->thread_event_id &&
            priv->thread_event_id != LEAVE_THREAD)
        {
            priv->thread_event_id = (priv->thread_event_id == RESET_REMAP ? 0 : priv->thread_event_id);
            DBGX(1, "%s: do for pid: %d, thread_event_id %lu\n",
                    __FUNCTION__, priv->my_pid, priv->thread_event_id);
            start_jiffies = jiffies;

            /*
             * Check to see if RESET button is held-down for greater than 5 sec.
             * indicating a factory_reset operation.
             */
            bsignal = buttons_signal_read(priv);
            if (bsignal < 0)
            {
                ERR("%s() - ERROR reading button pin.\n", __FUNCTION__);
                break;
            }

            while ((bsignal != BUTTON_RELEASED) &&
                   (!(bsignal & GPP_BIT(GPIO_RESET_PIN))))
            {
                DBGX(2, "%s longpress BUTTON_PRESSED(%d).\n",
                        __FUNCTION__, bsignal);

                /* check for a button press and held down for 10 seconds */
                if (time_after(jiffies, start_jiffies + msecs_to_jiffies(10000)))
                {
                    DBGX(1, "*********** Got LONGPRESS for pid: %d ************\n", priv->my_pid);
                    priv->eventmask |= EVENT_BUTTON_LONGPRESS;
                    break;
                }
                else
                {
                    DBGX(1, "%s longpress wait >= 10 sec. for pid %d\n", __FUNCTION__, priv->my_pid);
                    set_current_state(TASK_INTERRUPTIBLE);
                    schedule_timeout_interruptible(50 * HZ / 1000);
                }

                bsignal = buttons_signal_read(priv);
                if (bsignal < 0)
                {
                    ERR("%s() -ERROR reading button pin.\n", __FUNCTION__);
                    break;
                }
            } 

            /* record the button event */
            priv->eventmask |= 
                (priv->thread_event_id == ButtonIrqs[0].assigned_irq) ? EVENT_BUTTON_FACTORY_RESET :
                (priv->thread_event_id == ButtonIrqs[1].assigned_irq) ? EVENT_BUTTON_WPS : 0;

            DBGX(2, "%s: **** eventmask 0x%08x ****\n",
                    __FUNCTION__, priv->eventmask);

            if (priv->eventmask)
            {
                wake_up_interruptible(&priv->waitq);
            }
        }
        else
        {
            DBGX(1, "%s: else for pid: %d, thread_event_id %lu\n",
                    __FUNCTION__, priv->my_pid, priv->thread_event_id);
            priv->thread_event_id = LEAVE_THREAD;
        }
    } while (priv->thread_event_id != LEAVE_THREAD);

    DBGX(1, "leaving kernel thread for pid: %d\n", priv->my_pid);

    complete_and_exit(&priv->thread_done, 0);
}

/* ---------------------------------------------------------------------------
 * Buttons IRQ handler
 * ------------------------------------------------------------------------- */
static irqreturn_t buttons_irq_handler(int irq, void* dev)
{
    int i; 
    struct list_head *pos, *temp;

#ifdef _DEBUG
    int cnt = 0;

    counter += 1;
    DBGX(2, "counter: %d\n", counter );
#endif

    DBGX(2, "%s: entered\n", __FUNCTION__);

    /* service the button signal */
    for (i = 0; i < (sizeof(ButtonIrqs) / sizeof(struct buttons_dev)); i++)
    {
        if (irq == ButtonIrqs[i].assigned_irq)
        {
            if (!(ButtonIrqs[i].irq_handler_func(irq, dev)))
                return IRQ_NONE;
            break;
        }
    }

    /* Send the type of button "event" (eventmask) to the reader/poller
       kernel threads */
    list_for_each_safe(pos, temp, &ButtonsList)
    {
        struct buttons_private* priv;
            
        priv = list_entry(pos, struct buttons_private, list);

#ifdef _DEBUG
        DBGX(2,"%s: member=(%d) NAME=%s\n",__FUNCTION__,cnt,priv->name);
        cnt++;
        DBGX(2, "irq %d sent to pid %d\n", irq, priv->my_pid);
#endif
        priv->thread_event_id = irq;

        if (priv->thread_event_id)
        {
            DBGX(1, "%s: thread_event_id %lu\n",
                __FUNCTION__, priv->thread_event_id );
            wake_up_interruptible(&priv->longpress_waitq);
        }
    }

    return IRQ_HANDLED;
}

/* ---------------------------------------------------------------------------
 * Factory Reset button handler
 * ------------------------------------------------------------------------- */
static irqreturn_t factory_reset_irq_handler(int irq, void* dev)
{
    u_int32_t irq_polarity;
    u32 gpp = (u32) irq - IRQ_GPP_START;
    u32 gppVal;

    DBGX(2, "%s: entered\n", __FUNCTION__);

    gppVal = mv_gpp_value_real_get(GPP_GROUP(gpp), GPP_BIT(gpp));

    irq_polarity = gpioPolarityGet(1, MV_GPP33);
    if (gppVal == 0) /* button pressed */
    {
        DBGX(1, "%s: button pressed\n", __FUNCTION__);
        if (irq_polarity == 0) 
            gpioPolaritySet(1, MV_GPP33, (MV_GPP_IN_INVERT & MV_GPP33));
        else
            gpioPolaritySet(1, MV_GPP33, (MV_GPP_IN_ORIGIN & MV_GPP33));
    }
    else /* button not pressed  */
    {
        DBGX(1, "%s: button released\n", __FUNCTION__);
        if (irq_polarity == 0) 
            gpioPolaritySet(1, MV_GPP33, (MV_GPP_IN_INVERT & MV_GPP33));
        else
            gpioPolaritySet(1, MV_GPP33, (MV_GPP_IN_ORIGIN & MV_GPP33));
    }

    return IRQ_HANDLED; 
}

/* ---------------------------------------------------------------------------
 * WPS button handler
 * ------------------------------------------------------------------------- */
static irqreturn_t WPS_irq_handler(int irq, void* dev)
{
    u_int32_t irq_polarity;
    u32 gpp = (u32) irq - IRQ_GPP_START;
    u32 gppVal;

    DBGX(2, "%s: entered\n", __FUNCTION__);

    gppVal = mv_gpp_value_real_get(GPP_GROUP(gpp), GPP_BIT(gpp));

    irq_polarity = gpioPolarityGet(1, MV_GPP32);
    if (gppVal == 0) /* button pressed */
    {
        DBGX(1, "%s: button pressed\n", __FUNCTION__);
        if (irq_polarity == 0) 
            gpioPolaritySet(1, MV_GPP32, (MV_GPP_IN_INVERT & MV_GPP32));
        else
            gpioPolaritySet(1, MV_GPP32, (MV_GPP_IN_ORIGIN & MV_GPP32));
    }
    else /* button not pressed  */
    {
        DBGX(1, "%s: button released\n", __FUNCTION__);
        if (irq_polarity == 0) 
            gpioPolaritySet(1, MV_GPP32, (MV_GPP_IN_INVERT & MV_GPP32));
        else
            gpioPolaritySet(1, MV_GPP32, (MV_GPP_IN_ORIGIN & MV_GPP32));
    }

    return IRQ_HANDLED; 
}

/* ---------------------------------------------------------------------------
 * buttons user space interface operations
 * ------------------------------------------------------------------------- */
static struct file_operations buttons_fops = {
    .owner = THIS_MODULE,
    .read = buttons_read,
    .poll = buttons_poll,
    .unlocked_ioctl = buttons_ioctl,
    .open = buttons_open,
    .release = buttons_release
};

/* ---------------------------------------------------------------------------
 * buttons IOCTL
 * ------------------------------------------------------------------------- */
static long buttons_ioctl(struct file* file, unsigned int cmd,
                          unsigned long arg)
{
    int ret = -ENOTTY;

    switch (cmd)
    {
    case BDUTIL_SETRSTTIMEOUT:
        DBGX(1, "%s: BDUTIL_SETRSTTIMEOUT switch\n", __FUNCTION__);
        ret = 0;
        break;
    default:
        DBGX(1, "%s: default switch\n", __FUNCTION__);
        break;
    }

    return ret;
}

/* ---------------------------------------------------------------------------
 * buttons open
 * creates a kernel thread to read and report button "events" 
 * ------------------------------------------------------------------------- */
static int buttons_open(struct inode* inode, struct file* filp)
{
    struct buttons_private* priv;

    DBGX(1, "%s: entering\n", __FUNCTION__); 

    priv = kzalloc(sizeof(struct buttons_private), GFP_KERNEL);

    if (priv == NULL)
    {
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
    list_add_tail(&priv->list, &ButtonsList);
    DBGX(1, "%s: buttons_open(pid: %d)\n", __FUNCTION__, current->pid);

    kernel_thread(buttons_longpress_kernel_thread,
                  (void*) priv,
                  CLONE_FS|CLONE_FILES);

    wait_for_completion(&priv->thread_done);

    DBGX(1, "%s: exiting\n", __FUNCTION__); 
    return 0;
}

/* ---------------------------------------------------------------------------
 * buttons read
 * ------------------------------------------------------------------------- */
static ssize_t buttons_read(struct file* filp, char __user* buf, size_t count,
                            loff_t* f_pos)
{
    struct buttons_private* priv = filp->private_data;

    DBGX(1, "%s: (pid: %d)!\n", __FUNCTION__, priv->my_pid);
    wait_event_interruptible(priv->waitq, priv->eventmask);
    DBGX(1, "%s: -> wake-up, eventmask: 0x%08x\n",
            __FUNCTION__, priv->eventmask);

    if (copy_to_user(buf, &priv->eventmask,  sizeof(unsigned long)) )
    {
        ERR("%s: - bad copy\n", __FUNCTION__);
        return -EFAULT;
    }
    priv->eventmask = 0;

    return sizeof(unsigned long);
}

/* ---------------------------------------------------------------------------
 * buttons poll
 * ------------------------------------------------------------------------- */
static u_int32_t buttons_poll(struct file* filp, poll_table* wait)
{
    struct buttons_private* priv = filp->private_data;

    DBGX(1, "%s: (pid: %d)\n", __FUNCTION__, priv->my_pid);
    poll_wait(filp, &priv->waitq, wait);
    if (priv->eventmask)
    {
        return (POLLIN | POLLRDNORM);
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * buttons release
 * ------------------------------------------------------------------------- */
static int buttons_release(struct inode* inode, struct file* filp)
{
    struct buttons_private* priv = filp->private_data;

    DBGX(1, "%s: (pid: %d)\n", __FUNCTION__, priv->my_pid);
    priv->thread_event_id = LEAVE_THREAD;
    wake_up_interruptible(&priv->longpress_waitq);
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout_interruptible(50 * HZ / 1000);

    list_del(&priv->list);
    kfree(priv);
    return 0;
}

/* ---------------------------------------------------------------------------
 * initialization
 * ------------------------------------------------------------------------- */
int init_buttons(void)
{
    int i;
    int status = 0;
#if defined DEBUG
    u32 gpp = -1;
#endif

    LOG(DRV_DESC ", version " DRV_VERSION"\n");

    major = register_chrdev(0, DEVICE_NAME, &buttons_fops);
    DBGX(1, "%s: dev entry =  %s\n", __FUNCTION__, DEVICE_NAME);
    DBGX(1, "%s: major =  %d\n", __FUNCTION__, major);

    for (i = 0; i < (sizeof(ButtonIrqs) / sizeof(struct buttons_dev)); i++)
    {
#if defined DEBUG
        gpp = ButtonIrqs[i].assigned_irq - IRQ_GPP_START;
        status = mv_gpp_value_real_get(GPP_GROUP(gpp), GPP_BIT(gpp));
        DBGX(2, "%s: gpio get value(%d) returned %d\n", 
                __FUNCTION__, ButtonIrqs[i].gpio_pin, status);
#endif

        /* set polarity bit */
        gpioPolaritySet(1, MV_GPP33, (MV_GPP_IN_INVERT & MV_GPP33));
        gpioPolaritySet(1, MV_GPP32, (MV_GPP_IN_INVERT & MV_GPP32));

        ButtonIrqs[i].assigned_irq = gpio_to_irq(ButtonIrqs[i].gpio_pin);

        DBGX(2, "%s: gpio_to_irq(%d) returned %d\n",
                __FUNCTION__, ButtonIrqs[i].gpio_pin, ButtonIrqs[i].assigned_irq);

        status = request_irq(ButtonIrqs[i].assigned_irq,
                             buttons_irq_handler,
                             0,
                             ButtonIrqs[i].name,
                             NULL);

        if (status)
        {
            ERR("%s: failed to request_irq for %s, gpio board_gpio_pin %d\n",
                    __FUNCTION__, ButtonIrqs[i].name, ButtonIrqs[i].gpio_pin);
            free_irq(ButtonIrqs[i].assigned_irq, NULL);
            return -ENODEV;
        }

        DBGX(2, "%s: request_irq(%d) returned %d\n", 
                __FUNCTION__, ButtonIrqs[0].gpio_pin, status);
    }
    return status;
}

/* ---------------------------------------------------------------------------
 * exit
 * ------------------------------------------------------------------------- */
void exit_buttons(void)
{
    // TODO do we need to free up ButtonsList?

    int i;

    for (i = 0; i < (sizeof(ButtonIrqs) / sizeof(struct buttons_dev)); i++)
    {
        if (ButtonIrqs[i].assigned_irq != NO_IRQ)
        {    
            free_irq(ButtonIrqs[i].assigned_irq, NULL);
            gpio_free(ButtonIrqs[i].gpio_pin);
        }
    }

    unregister_chrdev(major, DEVICE_NAME);

    LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}
