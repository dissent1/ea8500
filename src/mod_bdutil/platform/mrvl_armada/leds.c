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
  @file leds.c
  @brief board driver led control 
*************************************************************************/

/*
   Notes on locking:
   All globals and I2C register operations are protected and serialized
   by LedLock mutex. All the LED action functions called from procfs are
   protected. In addition blink activity threads and WPS workqueue also use
   LedLock.
   
   However the interrupt routines also needs to access some of the variables. 
   Changing LedLock to spin lock is one way, but the interrupt may spend
   quite a bit of time waiting for LED actions to finish. In the interest
   of minimzing the time spend in interrupt, the following short cuts are taken:

   LedEnabled is read but not written to by the interrupt routine, so we cheat
   by not having the interrupt routine acquire LedLock first. But we made 
   LedEnabled into an atomic. Possibly a blink could be missed by the
   interrupt routine when switch states due to lack of locking, but that
   should not be noticible.

   LED activity related data is written to by the interrupt routing, so we
   created an activity spin lock (activity_lock) to protect that. Activity
   blink thread will try to acquire both LedLock and activity_lock, but the
   interrupt routine does not acquire the LedLock, and no other routine will
   acquire the activity_lock, so there shouldn't be any deadlock condition.
*/


#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>

#include <boardEnv/mvBoardEnvSpec.h>
#include <linux_oss/mvOs.h>
#include <mv_drivers_lsp/mv_switch/mv_switch.h>
#include <ctrlEnv/mvCtrlEnvRegs.h>

#include "tlc_io_expander.h"

#include "../../debug.h"
#include "leds.h"
#include "gpio.h"

#define DRV_DESC    "bdutil board utility LEDs kernel module"
#define DRV_VERSION "2.5"
#define DEVICE_NAME "leds"

/* Procfs input buffer size */
#define PROC_BUF_SIZE (64)

/* Power LED blink rate */
#define GPIO_LED_BLINK_NOT_READY     0x08000000
#define GPIO_LED_BLINK_FACTORY_RESET 0x05B60000

/* Activity blinking interrups */
#define USB1_IRQ    45
#define USB2_IRQ    58
#define ESATA_IRQ   55
#define WIFI24G_IRQ 59
#define WIFI5G_IRQ  60

/* Belkin spec, blink time is 25ms */
#define ACTIVITY_BLINK_TIME (25)

/* I2C LEDs definitions */
#define LED_WAN_AMBER_REG   TLC_LEDOUT0
#define LED_WAN_AMBER_MASK  TLC_LED0_MASK
#define LED_WAN_AMBER_SHIFT TLC_LED0_SHIFT
#define LED_WAN_AMBER_PWM   TLC_PWM0

#define LED_WAN_WHITE_REG   TLC_LEDOUT0
#define LED_WAN_WHITE_MASK  TLC_LED1_MASK
#define LED_WAN_WHITE_SHIFT TLC_LED1_SHIFT
#define LED_WAN_WHITE_PWM   TLC_PWM1

#define LED_USB1_REG        TLC_LEDOUT1
#define LED_USB1_MASK       TLC_LED1_MASK
#define LED_USB1_SHIFT      TLC_LED1_SHIFT
#define LED_USB1_PWM        TLC_PWM5

#define LED_USB2A_REG       TLC_LEDOUT1
#define LED_USB2A_MASK      TLC_LED2_MASK
#define LED_USB2A_SHIFT     TLC_LED2_SHIFT
#define LED_USB2A_PWM       TLC_PWM6

#define LED_USB2B_REG       TLC_LEDOUT1
#define LED_USB2B_MASK      TLC_LED3_MASK
#define LED_USB2B_SHIFT     TLC_LED3_SHIFT
#define LED_USB2B_PWM       TLC_PWM7

#define LED_WPS_WHITE_REG   TLC_LEDOUT2
#define LED_WPS_WHITE_MASK  TLC_LED0_MASK
#define LED_WPS_WHITE_SHIFT TLC_LED0_SHIFT
#define LED_WPS_WHITE_PWM   TLC_PWM8

#define LED_WPS_AMBER_REG   TLC_LEDOUT2
#define LED_WPS_AMBER_MASK  TLC_LED1_MASK
#define LED_WPS_AMBER_SHIFT TLC_LED1_SHIFT
#define LED_WPS_AMBER_PWM   TLC_PWM9

/* I2C Temperature sensors */
#define TEMPERATURE_STATUS_REG      (0x08)
#define TEMPERATURE_CONFIG1_REG     (0x09)
#define TEMPERATURE_CONFIG2_REG     (0x0A)
#define TEMPERATURE_CONV_REG        (0x0B)

#define TEMPERATURE_CONFIG1_DEFAULT (0x00) /* don't use extended range */
#define TEMPERATURE_CONFIG2_DEFAULT (0x1C) /* 2 sensors TMP421 */
#define TEMPERATURE_CONV_DEFAULT    (0x01) /* checks one every 8 seconds */

#define TEMPERATURE_DDR_HIGH_REG    (0x00)
#define TEMPERATURE_DDR_LOW_REG     (0x10)
#define TEMPERATURE_WIFI_HIGH_REG   (0x01)
#define TEMPERATURE_WIFI_LOW_REG    (0x11)

#define TEMPERATURE_INVALID         (-1000) /* our indicator for unable to
                                               read temperature */

/* certain LEDs can be disabled while other cannot */
#define LED_CANNOT_DISABLE false
#define LED_CAN_DISABLE    true

/* number of threads handling LED blinking */
#define LED_ACTIVITY_BLINK_THREADS 2

/* the wifi device names (for controllnig wifi LEDs) */
#define WIFI24G_DEV_NAME "wdev0"
#define WIFI5G_DEV_NAME  "wdev1"

/*
    There are 48 MPPs. Each MPP can have up to 8 possible functions.
    The MPPs are split into groups of 8:
      0 : 0 - 7
      1 : 8 - 15
      2 : 16 - 23
      3 : 24 - 31
      4 : 32 - 39
      5 : 40 - 47 
    The macro MPP_CONTROL_REG(group) can be used to calculate the offset
    of the group register. For user space, add 0xF1000000 to the offset. 

    The 32-bit register represents 8 MPPs, 4 bits each, for a total of
    8 possible functions per MPP.
*/

/*
  eSata LED (MPP46)
  MPP46 is in group 5. Group 5 contains 40 - 47, so 46 is in the 7th
  position, bits 24:27.

  Setting MPP46 to 0x0 will select GPIO control.
  Setting MPP46 to 0x4 will select eSata HW control.
*/
#define LED_ESATA_MPP46_OFFSET MPP_CONTROL_REG(5)
#define LED_ESATA_MPP46_MASK   (0x0F000000)
#define LED_ESATA_GPIO_CONTROL (0x00000000)
#define LED_ESATA_HW_CONTROL   (0x04000000)

/*
  Fan (MPP24)
  MPP24 is in group 3. Group 3 contains 24 - 31, so 24 is in the first
  position, bits 0:4.

  We need to set this to GPIO control (0x0).
*/
#define FAN_MPP24_OFFSET       MPP_CONTROL_REG(3)
#define FAN_MPP24_MASK         (0x0000000F)
#define FAN_GPIO_CONTROL       (0x00000000)

/*
  GPIO are split into groups of 32:
      0 : 0 - 31,  offset 0x00018100 (output), 0x00018104 (enable)
      1 : 32 - 63, offset 0x00018140 (output), 0x00018144 (enable)
  For user space, add 0xF1000000 to the offset.

  Each GPIO is 1 bit.

  Enable is active low, meaning 0 - enable, 1 - disable.
  Output is 0 - off, 1 - on.
*/

#define GPIO_GROUP0_BASE          (0x00018100)
#define GPIO_GROUP1_BASE          (0x00018140)

/* GPIO register offsets */
#define GPIO_DATA_OUT             (0x00)
#define GPIO_DATA_OUT_ENABLE      (0x04)
#define GPIO_BLINK_ENABLE         (0x08)
#define GPIO_DATA_IN_POLARITY     (0x0c)
#define GPIO_DATA_IN              (0x10)
#define GPIO_INTERRUPT_CAUSE      (0x14)
#define GPIO_INTERRUPT_MASK       (0x18)
#define GPIO_INTERRUPT_LEVEL      (0x1C)
#define GPIO_BLINK_COUNTER_SELECT (0x20)
#define GPIO_CONTROL_SET          (0x28)
#define GPIO_CONTROL_CLEAR        (0x2C)
#define GPIO_DATA_OUT_SET         (0x30)
#define GPIO_DATA_OUT_CLEAR       (0x34)

/* GPIO registers */
#define GPIO_BLINK_COUNTER_B_ON_DUTY  (0x000181c8)
#define GPIO_BLINK_COUNTER_B_OFF_DUTY (0x000181cc)

/*
  eSata GPIO (GPIO46)
  GPIO46 is group 1, bit 14.
*/
#define LED_GPIO46_MASK          (0x00004000)
#define LED_GPIO46_OUTPUT_OFFSET (GPIO_GROUP1_BASE + GPIO_DATA_OUT)
#define LED_GPIO46_ENABLE_OFFSET (GPIO_GROUP1_BASE + GPIO_DATA_OUT_ENABLE)

/*
  Fan GPIO (GPIO24) 
  GPIO24 is group 0, bit 24
*/
#define FAN_GPIO24_MASK          (0x01000000)
#define FAN_GPIO24_OUTPUT        (GPIO_GROUP0_BASE + GPIO_DATA_OUT)
#define FAN_GPIO24_OUTPUT_ENABLE (GPIO_GROUP0_BASE + GPIO_DATA_OUT_ENABLE)
#define FAN_GPIO24_BLINK_ENABLE  (GPIO_GROUP0_BASE + GPIO_BLINK_ENABLE)
#define FAN_GPIO24_BLINK_COUNTER (GPIO_GROUP0_BASE + GPIO_BLINK_COUNTER_SELECT)

/* sata0 register is at offset 0x000A002C, setting bit 0 to 1 enables
   activity blinking */
#define SATA0_LED_CONFIG_REG      (0x0000A002C)
#define SATA0_ACTIVITY_BLINK_MASK (0x000000001)
#define SATA0_SET_ACTIVITY_BLINK  (0x000000001)

/* FAN control */
#define FAN_PROC_NAME      "fan"
#define FAN_CONTROL_PERIOD (30) /* poll time in seconds */

/* 10Hz 60% duty is fan low speed */
#define FAN_LOW_SPEED_ON_DUTY  (0x00001770)
#define FAN_LOW_SPEED_OFF_DUTY (0x00000fa0)

#define FAN_CPU_OFF2LOW   (85)
#define FAN_CPU_LOW2OFF   (80)
#define FAN_CPU_LOW2HIGH  (95)
#define FAN_CPU_HIGH2LOW  (90)

#define FAN_DDR_OFF2LOW   (65)
#define FAN_DDR_LOW2OFF   (60)
#define FAN_DDR_LOW2HIGH  (75)
#define FAN_DDR_HIGH2LOW  (70)

#define FAN_WIFI_OFF2LOW  (105)
#define FAN_WIFI_LOW2OFF  (100)
#define FAN_WIFI_LOW2HIGH (115)
#define FAN_WIFI_HIGH2LOW (110)

/* ---------------------------------------------------------------------------
 * I2C kernel driver was patched to expose certain symbols for LED control.
 * The following are for accessing the I2C driver. (from agraham@marvell.com)
 * ------------------------------------------------------------------------- */
typedef signed int     s32;
typedef unsigned short u16;
typedef unsigned char  u8;

struct i2c_dev
{
    struct list_head list;
    struct i2c_adapter *adap;
    struct device *dev;
};

extern struct i2c_dev*     i2c_dev_get_by_minor(unsigned index);
extern struct i2c_adapter* i2c_get_adapter(int nr);
extern s32                 i2c_smbus_xfer(struct i2c_adapter *adapter,
                                          u16 addr, unsigned short flags,
                                          char read_write, u8 command,
                                          int protocol,
                                          union i2c_smbus_data *data);

/* ---------------------------------------------------------------------------
 * mod_bdutil externs
 * ------------------------------------------------------------------------- */
extern u_int32_t              bdutil_gboardId;
extern struct proc_dir_entry* proc_bdutil;

/* ---------------------------------------------------------------------------
 * Structures
 * ------------------------------------------------------------------------- */
/* LED actions, not all are supported by individual LEDs */
typedef enum
{
    LED_ACTION_OFF,
    LED_ACTION_ON,
    LED_ACTION_BLINK,
    LED_ACTION_FAST_BLINK,
    LED_ACTION_ALT_ON,
    LED_ACTION_ALT_BLINK,

    /* the following are not Procfs commands but are used internally */
    LED_ACTION_DEFAULT,
    LED_ACTION_FADE, 
} LedAction;

/* LED control - name and action function */
typedef int (*LedActionFunc)(LedAction act);
typedef struct 
{
    char*         name;
    bool          can_disable;
    LedActionFunc do_action;
} LedControl;

/* I2C info */
typedef struct
{
    struct i2c_adapter* adapter;

    /* register values are doubled in memory for save/restore purposes */
    unsigned char tlc_ledout0;
    unsigned char tlc_ledout1;
    unsigned char tlc_ledout2;
    unsigned char tlc_pwm8; /* this is the WPS LED that needs fading */

    /* additional usb data for save/restore */
    bool usb1_enabled;
    bool usb2_enabled;
} I2C_Info;

/* WPS LED work queue data structure */
typedef struct
{
    struct workqueue_struct* wq;
    struct delayed_work      work;
    LedAction                cmd;
    unsigned char            fade;
} WPSLedWQ;

/* activity blink data per LED */
typedef struct
{
    bool             enabled;  
    unsigned char    reg;
    unsigned char    mask;
    unsigned char    off;
    unsigned char    on;
    bool             is_blinking;

    /* protected by activity lock */
    bool             has_activity; 
} LedBlinkData;

/* LEDs that will be managed by activity blinking */
enum
{
    LED_ACTIVITY_USB1 = 0,
    LED_ACTIVITY_USB2,
    LED_ACTIVITY_MAX
};

/* activity irq data */
typedef struct
{
    struct semaphore* sem;
    spinlock_t*       activity_lock;
    bool*             has_activity;
} LedActivityIrqData;

/* global activity blink data */
typedef struct
{
    struct semaphore    sem;

    /* activity lock protects activity variables that are modified
       by the interrupt handler */
    spinlock_t          activity_lock; 

    LedBlinkData        led[LED_ACTIVITY_MAX];
    struct task_struct* blinker[LED_ACTIVITY_BLINK_THREADS];

    unsigned int        irq[LED_ACTIVITY_MAX];
    bool                irq_initialized[LED_ACTIVITY_MAX];
    LedActivityIrqData  irq_data[LED_ACTIVITY_MAX];
} LedActivityData;

/* fan control data */
typedef struct
{
    int cpu_last_temp;
    int ddr_last_temp;
    int wifi_last_temp;

    int cpu_off2low;
    int cpu_low2off;
    int cpu_low2high;
    int cpu_high2low;

    int ddr_off2low;
    int ddr_low2off;
    int ddr_low2high;
    int ddr_high2low;

    int wifi_off2low;
    int wifi_low2off;
    int wifi_low2high;
    int wifi_high2low;
} FanControlData;

/* fan speed, the lowest priority should have the lowest integer equivalent
   value. I.e. if sensor 1 says low, sensor 2 say high, the fan should be high,
   therefore high should have larger integer equivalent value */
typedef enum
{
    FAN_SPEED_OFF = 0,
    FAN_SPEED_LOW,
    FAN_SPEED_HIGH
} FanSpeed;

/* ---------------------------------------------------------------------------
 * Prototypes
 * ------------------------------------------------------------------------- */
static int  i2c_led_reg_get(unsigned int reg, unsigned char* val);
static int  i2c_led_reg_set(unsigned int reg, unsigned char val);
static int  i2c_led_control(unsigned int reg, unsigned char mask,
                            unsigned char val);
static void i2c_buffered_reg_get(unsigned int reg, unsigned char* val);
static void i2c_buffered_reg_set(unsigned int reg, unsigned char val);
static void i2c_buffered_led_control(unsigned int reg, unsigned char mask,
                                     unsigned char val);
static int  i2c_led_save_state(void);
static int  i2c_led_restore_state(void);

static int         activity_blink_thread(void* data);
static irqreturn_t blink_activity_irq_handler(int irq, void* dev);
static void        wps_led_wq_handler(struct work_struct* ws);

static int  led_pwr_action(LedAction act);
static int  led_usb1_action(LedAction act);
static int  led_usb2a_action(LedAction act);
static int  led_usb2b_action(LedAction act);
static int  led_wan_action(LedAction act);
static int  led_wps_action(LedAction act);
static int  led_lan_action(LedAction act);
static int  led_esata_action(LedAction act);
static int  led_all_action(LedAction act);

static int  i2c_temperature_reg_get(unsigned int reg, unsigned char* val);
static int  i2c_temperature_reg_set(unsigned int reg, unsigned char val);
static int  fan_get_speed(int temperature, FanSpeed fan_state, int off2low, 
                          int low2off, int low2high, int high2low);
static int  fan_control_thread(void* data);

static int  parse_led(char* proc_buff);
static int  help_proc_read(char* page, char** start, off_t off, int count,
                           int* eof, void* data);
static int  leds_proc_write(struct file* file, const char __user* buffer,
                            unsigned long count, void *data);
static int  fan_proc_read(char* page, char** start, off_t off, int count,
                          int* eof, void* data);
static int  fan_proc_write(struct file* file, const char __user* buffer,
                           unsigned long count, void *data);
static int  do_init_i2c_leds(void);
static void do_cleanup_i2c_leds(void);
static int  do_init_gpio_leds(void);
static void do_cleanup_gpio_leds(void);
static int  leds_init_data(void);
static void cleanup_leds(void);

int  init_leds(void);
void exit_leds(void);

/* ---------------------------------------------------------------------------
 * Globals
 * ------------------------------------------------------------------------- */
static struct proc_dir_entry* ProcHelpNode = NULL;
static struct proc_dir_entry* ProcLedsNode = NULL;
static struct proc_dir_entry* ProcFanNode = NULL;

static DEFINE_MUTEX(LedLock);
static atomic_t LedEnabled;

static I2C_Info I2C;

/* WPS LED workqueue */
static WPSLedWQ WPSLed_wq;

/* LED activity blink */
static LedActivityData LedActivity;

/* FAN Control */
static FanControlData FanData =
{
    .cpu_last_temp = TEMPERATURE_INVALID,
    .ddr_last_temp = TEMPERATURE_INVALID,
    .wifi_last_temp = TEMPERATURE_INVALID,

    .cpu_off2low = FAN_CPU_OFF2LOW,
    .cpu_low2off = FAN_CPU_LOW2OFF,
    .cpu_low2high = FAN_CPU_LOW2HIGH,
    .cpu_high2low = FAN_CPU_HIGH2LOW,

    .ddr_off2low = FAN_DDR_OFF2LOW,
    .ddr_low2off = FAN_DDR_LOW2OFF,
    .ddr_low2high = FAN_DDR_LOW2HIGH,
    .ddr_high2low = FAN_DDR_HIGH2LOW,

    .wifi_off2low = FAN_WIFI_OFF2LOW,
    .wifi_low2off = FAN_WIFI_LOW2OFF,
    .wifi_low2high = FAN_WIFI_LOW2HIGH,
    .wifi_high2low = FAN_WIFI_HIGH2LOW,
};
struct task_struct* FanControlThread = NULL;

static char* UsageStr;
static char* UsageStrMamba =
    "Usage: echo \"<led>=<action>\" > /proc/bdutil/leds\n"
    "    LED    ACTION\n"
    "    all    off|on\n"
    "    pwr    off|on|blink|fastblink\n"
    "    usb1   off|on\n"
    "    usb2a  off|on\n"
    "    usb2b  off|on\n"
    "    wan    off|on|blink|alton|altblink\n"
    "    wps    off|on|blink|altblink\n";

/* ---------------------------------------------------------------------------
 * LED Definitions
 * - Belkin spec is that PWR and WPS LEDs cannot be disabled
 * - eSata LED is controlled by HW and we can enable/disable it here
 * - 2.4G/5G LEDs are controlled by HW and we enable/diable them from user
 *   space iwpriv command
 * ------------------------------------------------------------------------- */
static LedControl LedCtlList[] =
{
    { "pwr",   LED_CANNOT_DISABLE, led_pwr_action, },
    { "usb1",  LED_CAN_DISABLE,    led_usb1_action, },
    { "usb2a", LED_CAN_DISABLE,    led_usb2a_action, },
    { "usb2b", LED_CAN_DISABLE,    led_usb2b_action, },
    { "wan",   LED_CAN_DISABLE,    led_wan_action, },
    { "wps",   LED_CANNOT_DISABLE, led_wps_action, },
};
unsigned int LedCtlListSize = sizeof(LedCtlList) / sizeof(LedCtlList[0]);

/* ===========================================================================
 * I2C register read/write functions
 * ==========================================================================*/
/* ---------------------------------------------------------------------------
 * Wrapper for reading from I2C LED register
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static int i2c_led_reg_get(unsigned int reg, unsigned char* val)
{
    union i2c_smbus_data data;
    int rc;

    rc = i2c_smbus_xfer(I2C.adapter,
                        I2C_LED_ADDR,
                        0,
                        I2C_SMBUS_READ,
                        reg,
                        2,
                        &data);
    if (rc < 0)
    {
        return rc;
    }

    *val = data.byte;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Wrapper for writing to I2C LED register
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static int i2c_led_reg_set(unsigned int reg, unsigned char val)
{
    union i2c_smbus_data data;

    /* If LED is not disabled, then do normal write */
    data.byte = val;
    return i2c_smbus_xfer(I2C.adapter,
                          I2C_LED_ADDR,
                          0,
                          I2C_SMBUS_WRITE,
                          reg,
                          2,
                          &data);
}

/* ---------------------------------------------------------------------------
 * controls the I2C LED registers
 * Caller must hold LedLock
 * reg - the output register
 * mask - mask of the bits that will be written
 * val - value to write to register (this is not checked, it should only
 *       have bits inside the mask)
 * ------------------------------------------------------------------------- */
static int i2c_led_control(unsigned int reg, unsigned char mask,
                           unsigned char val)
{
    unsigned char reg_data = 0;
    int rc, err = 0;

    /* read from LED register */
    rc = i2c_led_reg_get(reg, &reg_data);
    if (rc < 0)
    {
        err = rc;
    }

    /* mask out the LED control */
    reg_data &= ~(mask);
    /* apply new value */
    reg_data |= val;

    /* write to LED register */
    rc = i2c_led_reg_set(reg, reg_data);
    if (rc < 0)
    {
        err = rc;
    }

    return err;
}

/* ---------------------------------------------------------------------------
 * read from buffered I2C register, this is for keeping state while LEDs
 * are disabled
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static void i2c_buffered_reg_get(unsigned int reg, unsigned char* val)
{
    switch (reg)
    {
    case TLC_LEDOUT0:
        *val = I2C.tlc_ledout0;
        break;

    case TLC_LEDOUT1:
        *val = I2C.tlc_ledout1;
        break;

    case TLC_LEDOUT2:
        *val = I2C.tlc_ledout2;
        break;

    case TLC_PWM8:
        *val = I2C.tlc_pwm8;
        break;
    }
}

/* ---------------------------------------------------------------------------
 * write to buffered I2C register, this is for keeping state while LEDs
 * are disabled
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static void i2c_buffered_reg_set(unsigned int reg, unsigned char val)
{
    switch (reg)
    {
    case TLC_LEDOUT0:
        I2C.tlc_ledout0 = val;
        break;

    case TLC_LEDOUT1:
        I2C.tlc_ledout1 = val;
        break;

    case TLC_LEDOUT2:
        I2C.tlc_ledout2 = val;
        break;

    case TLC_PWM8:
        I2C.tlc_pwm8 = val;
        break;
    }
}

/* ---------------------------------------------------------------------------
 * controls the I2C LED registers, using buffered registers for when the LEDs
 * are disabled
 * Caller must hold LedLock
 * reg - the output register
 * mask - mask of the bits that will be written
 * val - value to write to register (this is not checked, it should only
 *       have bits inside the mask)
 * ------------------------------------------------------------------------- */
static void i2c_buffered_led_control(unsigned int reg, unsigned char mask,
                                     unsigned char val)
{
    unsigned char reg_data = 0;

    /* read buffered LED registers */
    i2c_buffered_reg_get(reg, &reg_data);

    /* mask out the LED control */
    reg_data &= ~(mask);
    /* apply new value */
    reg_data |= val;

    /* write to buffered LED register */
    i2c_buffered_reg_set(reg, reg_data);
}

/* ---------------------------------------------------------------------------
 * Saves I2C LED register state
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static int i2c_led_save_state(void)
{
    int rc, err = 0;

    /* save LEDOUT0 LEDs */
    rc = i2c_led_reg_get(TLC_LEDOUT0, &I2C.tlc_ledout0);
    if (rc != 0)
    {
        err = rc;
    }

    /* save LEDOUT1 LEDs */
    rc = i2c_led_reg_get(TLC_LEDOUT1, &I2C.tlc_ledout1);
    if (rc != 0)
    {
        err = rc;
    }

    /* save LEDOUT2 LEDs */
    rc = i2c_led_reg_get(TLC_LEDOUT2, &I2C.tlc_ledout2);
    if (rc != 0)
    {
        err = rc;
    }

    /* save PWM8 */
    rc = i2c_led_reg_get(TLC_PWM8, &I2C.tlc_pwm8);
    if (rc != 0)
    {
        err = rc;
    }

    /* save LED activity state */
    I2C.usb1_enabled = LedActivity.led[LED_ACTIVITY_USB1].enabled;
    I2C.usb2_enabled = LedActivity.led[LED_ACTIVITY_USB2].enabled;

    return err;
}

/* ---------------------------------------------------------------------------
 * Restores I2C LED register state
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static int i2c_led_restore_state(void)
{
    int rc, err = 0;

    /* restore LEDOUT0 */
    rc = i2c_led_reg_set(TLC_LEDOUT0, I2C.tlc_ledout0);
    if (rc < 0)
    {
        err = rc;
    }

    /* restore LEDOUT1 */
    rc = i2c_led_reg_set(TLC_LEDOUT1, I2C.tlc_ledout1);
    if (rc < 0)
    {
        err = rc;
    }

    /* restore LEDOUT2 */
    rc = i2c_led_reg_set(TLC_LEDOUT2, I2C.tlc_ledout2);
    if (rc < 0)
    {
        err = rc;
    }

    /* restore PWM8 */
    rc = i2c_led_reg_set(TLC_PWM8, I2C.tlc_pwm8);
    if (rc < 0)
    {
        err = rc;
    }

    /* restore USB state */
    LedActivity.led[LED_ACTIVITY_USB1].enabled = I2C.usb1_enabled;
    LedActivity.led[LED_ACTIVITY_USB2].enabled = I2C.usb2_enabled;

    return err;
}

/* ===========================================================================
 * LED activity blink/fade functions
 * ==========================================================================*/
/* ---------------------------------------------------------------------------
 * LED activity blink thread
 * Certain LEDs need to blink from activity, it's not supported in HW, so
 * we do it in SW
 * This thread runs on its own, so it needs locking
 * ------------------------------------------------------------------------- */
static int activity_blink_thread(void* data)
{
    unsigned long sem_flags, flags;
    bool activity[LED_ACTIVITY_MAX];
    int i;

    (void) data;

    while (!kthread_should_stop())
    {
        /* wait to be woken up */
        if (down_interruptible(&LedActivity.sem) != 0)
        {
            continue;
        }

        /* check if we should exit */
        if (kthread_should_stop())
        {
            break;
        }

        /* We really don't want to service every interrupt that comes in,
           since interrupts will happen way faster than LED blinking, and
           it'll pile up and keep going long after the interrupts have stopped.
           We reset the semaphore count here because we've serviced all
           interrupts up to this point in one shot (blink once).
           However, there's no API to set the semaphore count, so we roll our
           own.
           */
        raw_spin_lock_irqsave(&LedActivity.sem.lock, sem_flags);
        LedActivity.sem.count = 0;
        raw_spin_unlock_irqrestore(&LedActivity.sem.lock, sem_flags);

        mutex_lock(&LedLock);
        /* should we actually blink? */
        if (atomic_read(&LedEnabled) == 0)
        {
            /* if LED is disabled, don't bother blinking */
            mutex_unlock(&LedLock);
            continue;
        }

        /* store and consume which LEDs have activity */
        spin_lock_irqsave(&LedActivity.activity_lock, flags);
        for (i = 0; i < LED_ACTIVITY_MAX; i++)
        {
            if (LedActivity.led[i].is_blinking == false)
            {
                /* only blink when there's activity and is enabled */
                activity[i] = LedActivity.led[i].has_activity &&
                              LedActivity.led[i].enabled;

                /* consume the activity */
                LedActivity.led[i].has_activity = false;
            }
            else
            {
                /* we don't touch LEDs that are being blinked
                   by another thread */
                activity[i] = false;
            }
        }
        spin_unlock_irqrestore(&LedActivity.activity_lock, flags);

        /* blink - off */
        for (i = 0; i < LED_ACTIVITY_MAX; i++)
        {
            if (activity[i])
            {
                LedActivity.led[i].is_blinking = true;
                (void) i2c_led_control(LedActivity.led[i].reg,
                                       LedActivity.led[i].mask,
                                       LedActivity.led[i].off);
            }
        }
        mutex_unlock(&LedLock);

        /* sleep for a bit */
        msleep(ACTIVITY_BLINK_TIME);

        /* blink - on */
        mutex_lock(&LedLock);
        /* in case LED got turned off just now, we don't turn
           anything back on */
        if (atomic_read(&LedEnabled))
        {
            for (i = 0; i < LED_ACTIVITY_MAX; i++)
            {
                if (activity[i])
                {
                    (void) i2c_led_control(LedActivity.led[i].reg,
                                           LedActivity.led[i].mask,
                                           LedActivity.led[i].on);
                }
            }
        }
        mutex_unlock(&LedLock);

        /* sleep some more before we can blink again */
        msleep(ACTIVITY_BLINK_TIME);

        /* blink done */
        mutex_lock(&LedLock);
        for (i = 0; i < LED_ACTIVITY_MAX; i++)
        {
            if (activity[i])
            {
                LedActivity.led[i].is_blinking = false;
            }
        }
        mutex_unlock(&LedLock);
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * LED activity blink work around
 * certain LED need to blink from activity, it's not supported in HW, so
 * we do it in SW
 * ------------------------------------------------------------------------- */
static irqreturn_t blink_activity_irq_handler(int irq, void* dev)
{
    LedActivityIrqData* data = (LedActivityIrqData*) dev;
    unsigned long flags;

    /* LedEnabled normally is protected by LedLock. However, since we
       only do a read here, and the worst that can happen is that we'll
       lose one blink, we skip the lock */
    if (atomic_read(&LedEnabled))
    {
        /* this LED has activity */
        spin_lock_irqsave(data->activity_lock, flags);
        *data->has_activity = true;
        spin_unlock_irqrestore(data->activity_lock, flags);

        /* wake up blink thread */
        up(data->sem);
    }

    /* We actually didn't handle anything. We're just hooking into the 
       IRQ to blink the LED, we didn't actually handle any data */
    return IRQ_NONE;
}

/* ---------------------------------------------------------------------------
 * WPS LED workqueue
 * WPS LED needs fade, fade is not support in HW, so we do it in SW
 * This workqueue runs on its own, so it needs locking
 * ------------------------------------------------------------------------- */
static void wps_led_wq_handler(struct work_struct* ws)
{
    (void) ws;

    mutex_lock(&LedLock);
    switch (WPSLed_wq.cmd)
    {
    case LED_ACTION_OFF:
        /* this is for turning off WPS in 1 minute */
        (void) led_wps_action(LED_ACTION_OFF);
        break;

    case LED_ACTION_FADE:
        /*
           First wait is 1 second, where LED remains at max,
           Then for the next 4 seconds, every 80ms, we decrease the brightness
           by 5, so by the end, brightness will go from 255 to 5, then we
           turn it off
         */
        if (WPSLed_wq.fade > 5)
        {
            WPSLed_wq.fade -= 5;

            /* WPS cannot be disabled, so we always write to LED */
            (void) i2c_led_reg_set(LED_WPS_WHITE_PWM, WPSLed_wq.fade); 

            /* but if LED is disabled, we also want to write to
               the buffered registers, so when restore happens, the
               wrong state won't be written to LED */
            if (atomic_read(&LedEnabled) == 0)
            {
                i2c_buffered_reg_set(LED_WPS_WHITE_PWM, WPSLed_wq.fade); 
            }
        }
        else
        {
            WPSLed_wq.cmd = LED_ACTION_OFF;
        }

        /* schedule next work in 80ms */
        queue_delayed_work(WPSLed_wq.wq,
                           &WPSLed_wq.work,
                           msecs_to_jiffies(80));
        break;

    default:
        break;
    }
    mutex_unlock(&LedLock);
}

/* ===========================================================================
 * LED actions
 * ==========================================================================*/
/* ---------------------------------------------------------------------------
 * POWER LED action (GPIO MV_GPP40)
 * POWER LED does not check LedEnabled because it cannot be disabled
 * ------------------------------------------------------------------------- */
static int led_pwr_action(LedAction act)
{
    switch (act)
    {
    case LED_ACTION_OFF:
        /* LED off */
        gpioRegSet(GPP_DATA_OUT_EN_REG(1),
                   MV_GPP40,
                   ~(MV_GPP_IN_INVERT & MV_GPP40));

        /* blink off */
        gpioRegSet(GPP_BLINK_EN_REG(1),
                   MV_GPP40,
                   ~(MV_GPP_IN_INVERT & MV_GPP40));
        break;

    case LED_ACTION_ON:
        /* LED on */
        gpioRegSet(GPP_DATA_OUT_EN_REG(1),
                   MV_GPP40,
                   MV_GPP40);

        /* blink off */
        gpioRegSet(GPP_BLINK_EN_REG(1),
                   MV_GPP40,
                   ~(MV_GPP_IN_INVERT & MV_GPP40));
        break;

    case LED_ACTION_BLINK:
        /* set not  ready blink */
        gpioRegSet(GPP_BLINK_CTR_A_ON_DUR_REG(1),
                   MV_GPP_IN_INVERT,
                   GPIO_LED_BLINK_NOT_READY);
        gpioRegSet(GPP_BLINK_CTR_A_OFF_DUR_REG(1),
                   MV_GPP_IN_INVERT,
                   GPIO_LED_BLINK_NOT_READY);

        /* LED off */
        gpioRegSet(GPP_DATA_OUT_EN_REG(1),
                   MV_GPP40,
                   ~(MV_GPP_IN_INVERT & MV_GPP40));

        /* blink on */
        gpioRegSet(GPP_BLINK_EN_REG(1),
                   MV_GPP40,
                   MV_GPP40);
        break;

    case LED_ACTION_FAST_BLINK:
        /* set factory reset blink */
        gpioRegSet(GPP_BLINK_CTR_A_ON_DUR_REG(1),
                   MV_GPP_IN_INVERT,
                   GPIO_LED_BLINK_FACTORY_RESET);
        gpioRegSet(GPP_BLINK_CTR_A_OFF_DUR_REG(1),
                   MV_GPP_IN_INVERT,
                   GPIO_LED_BLINK_FACTORY_RESET);

        /* LED off */
        gpioRegSet(GPP_DATA_OUT_EN_REG(1),
                   MV_GPP40,
                   ~(MV_GPP_IN_INVERT & MV_GPP40));

        /* blink on */
        gpioRegSet(GPP_BLINK_EN_REG(1),
                   MV_GPP40,
                   MV_GPP40);
        break;

    default:
        break;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * USB1 LED action
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static int led_usb1_action(LedAction act)
{
    unsigned char val;
    bool enabled;

    switch (act)
    {
    case LED_ACTION_OFF:
        enabled = false;
        val = TLC_LED_DRIVER_DEF_OFF << LED_USB1_SHIFT;
        break;

    case LED_ACTION_ON:
        enabled = true;
        val = TLC_LED_DRIVER_FULL_ON << LED_USB1_SHIFT;
        break;

    default:
        return 0;
        break;
    }

    if (atomic_read(&LedEnabled))
    {
        /* start/stop activity blink */
        LedActivity.led[LED_ACTIVITY_USB1].enabled = enabled;

        /* set LED action */
        return i2c_led_control(LED_USB1_REG,
                               LED_USB1_MASK,
                               val);
    }
    else
    {
        /* start/stop activity blink (buffered) */
        I2C.usb1_enabled = enabled;

        /* set LED action (buffered) */
        i2c_buffered_led_control(LED_USB1_REG,
                                 LED_USB1_MASK,
                                 val);
        return 0;
    }
}

/* ---------------------------------------------------------------------------
 * USB2A LED action
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static int led_usb2a_action(LedAction act)
{
    unsigned char val;
    bool enabled;

    switch (act)
    {
    case LED_ACTION_OFF: 
        enabled = false;
        val = TLC_LED_DRIVER_DEF_OFF << LED_USB2A_SHIFT;
        break;

    case LED_ACTION_ON:
        enabled = true;
        val = TLC_LED_DRIVER_FULL_ON << LED_USB2A_SHIFT;
        break;

    default:
        return 0;
        break;
    }

    if (atomic_read(&LedEnabled))
    {
        /* start/stop activity blink */
        LedActivity.led[LED_ACTIVITY_USB2].enabled = enabled;

        /* set LED action */
        return i2c_led_control(LED_USB2A_REG,
                               LED_USB2A_MASK,
                               val);
    }
    else
    {
        /* start/stop activity blink (buffered) */
        I2C.usb2_enabled = enabled;
    
        /* set LED action (buffered) */
        i2c_buffered_led_control(LED_USB2A_REG,
                                 LED_USB2A_MASK,
                                 val);
        return 0;
    }
}

/* ---------------------------------------------------------------------------
 * USB2B LED action (indicates USB3.0 if solid)
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static int led_usb2b_action(LedAction act)
{
    unsigned char val;

    switch (act)
    {
    case LED_ACTION_OFF:
        val = TLC_LED_DRIVER_DEF_OFF << LED_USB2B_SHIFT;
        break;

    case LED_ACTION_ON:
        val = TLC_LED_DRIVER_FULL_ON << LED_USB2B_SHIFT;
        break;

    default:
        return 0;
        break;
    }

    if (atomic_read(&LedEnabled))
    {
        /* set LED action */
        return i2c_led_control(LED_USB2B_REG,
                               LED_USB2B_MASK,
                               val);
    }
    else
    {
        /* set LED action buffered */
        i2c_buffered_led_control(LED_USB2B_REG,
                                 LED_USB2B_MASK,
                                 val);
        return 0;
    }
}

/* ---------------------------------------------------------------------------
 * WAN LED action
 * Caller must hold LedLock
 * WAN LED is both white and amber, but only 1 should be active at any
 * given time 
 * ------------------------------------------------------------------------- */
static int led_wan_action(LedAction act)
{
    unsigned char val;

    switch (act)
    {
    case LED_ACTION_OFF:
        /* turn off both LEDs */
        val = (TLC_LED_DRIVER_DEF_OFF << LED_WAN_AMBER_SHIFT) |
              (TLC_LED_DRIVER_DEF_OFF << LED_WAN_WHITE_SHIFT);
        break;

    case LED_ACTION_ON:
        /* turn off LED0 (amber), turn on LED1 (white) */
        val = (TLC_LED_DRIVER_DEF_OFF << LED_WAN_AMBER_SHIFT) |
              (TLC_LED_DRIVER_FULL_ON << LED_WAN_WHITE_SHIFT);
        break;

    case LED_ACTION_BLINK:
        /* turn off LED0 (amber), blink LED1 (white) */
        val = (TLC_LED_DRIVER_DEF_OFF << LED_WAN_AMBER_SHIFT) |
              (TLC_LED_DRIVER_DIMM_BLINK << LED_WAN_WHITE_SHIFT);
        break;

    case LED_ACTION_ALT_ON:
        /* turn on LED0 (amber), turn off LED1 (white) */
        val = (TLC_LED_DRIVER_FULL_ON << LED_WAN_AMBER_SHIFT) |
              (TLC_LED_DRIVER_DEF_OFF << LED_WAN_WHITE_SHIFT);
        break;

    case LED_ACTION_ALT_BLINK:
        /* blink LED0 (amber), turn off LED1 (white) */
        val = (TLC_LED_DRIVER_DIMM_BLINK << LED_WAN_AMBER_SHIFT) |
              (TLC_LED_DRIVER_DEF_OFF << LED_WAN_WHITE_SHIFT);
        break;

    default:
        return 0;
        break;
    }

    /* WHITE and AMBER both use the same register */
    if (atomic_read(&LedEnabled))
    {
        return i2c_led_control(LED_WAN_AMBER_REG,
                               LED_WAN_AMBER_MASK | LED_WAN_WHITE_MASK,
                               val);
    }
    else
    {
        i2c_buffered_led_control(LED_WAN_AMBER_REG,
                                 LED_WAN_AMBER_MASK | LED_WAN_WHITE_MASK,
                                 val);
        return 0;
    }
}

/* ---------------------------------------------------------------------------
 * WPS LED action
 * Caller must hold LedLock
 * WPS LED is both white and amber, but only 1 should be active at any
 * given time 
 * ------------------------------------------------------------------------- */
static int led_wps_action(LedAction act)
{
    unsigned char val;
    int rc, err = 0;
    bool set_white_pwm;

    switch (act)
    {
    case LED_ACTION_OFF:
        /* cancel any pending work, since latest event will override
           previous ones */
        cancel_delayed_work(&WPSLed_wq.work);

        /* turn off both LEDs */
        val = (TLC_LED_DRIVER_DEF_OFF << LED_WPS_WHITE_SHIFT) |
              (TLC_LED_DRIVER_DEF_OFF << LED_WPS_AMBER_SHIFT);

        /* don't need to set white LED pwm */
        set_white_pwm = false;
        break;

    case LED_ACTION_ON:
        /* cancel any pending work, since latest event will override
           previous ones */
        cancel_delayed_work(&WPSLed_wq.work);

        /* schedule workqueue to fade, remain at max for 1 second first */
        WPSLed_wq.cmd = LED_ACTION_FADE;
        WPSLed_wq.fade = 0xff;

        queue_delayed_work(WPSLed_wq.wq,
                           &WPSLed_wq.work,
                           msecs_to_jiffies(1000));


        /* set PWM for LED0 (white), turn off LED1 (amber) */
        val = (TLC_LED_DRIVER_PWM_CRTL << LED_WPS_WHITE_SHIFT) |
              (TLC_LED_DRIVER_DEF_OFF << LED_WPS_AMBER_SHIFT);

        /* set start brightness */
        set_white_pwm = true;
        break;

    case LED_ACTION_BLINK:
        /* cancel any pending work, since latest event will override
           previous ones */
        cancel_delayed_work(&WPSLed_wq.work);

        (void) i2c_led_reg_set(TLC_PWM8, 0xff);

        /* blink LED0 (white), turn off LED1 (amber) */
        val = (TLC_LED_DRIVER_DIMM_BLINK << LED_WPS_WHITE_SHIFT) |
              (TLC_LED_DRIVER_DEF_OFF << LED_WPS_AMBER_SHIFT);

        /* set pwm, since the fade may have been cancelled
           and left this in a less than full brightness state */
        set_white_pwm = true;
        break;

    case LED_ACTION_ALT_BLINK:
        /* cancel any pending work, since latest event will override
           previous ones */
        cancel_delayed_work(&WPSLed_wq.work);

        /* schedule workqueue to turn off at 1 minute */
        WPSLed_wq.cmd = LED_ACTION_OFF;

        queue_delayed_work(WPSLed_wq.wq,
                           &WPSLed_wq.work,
                           msecs_to_jiffies(60000));

        /* turn off LED0 (white), blink LED1 (amber) */
        val = (TLC_LED_DRIVER_DEF_OFF << LED_WPS_WHITE_SHIFT) |
              (TLC_LED_DRIVER_DIMM_BLINK << LED_WPS_AMBER_SHIFT);

        /* don't need to set white LED pwm */
        set_white_pwm = false;
        break;

    default:
        return 0;
        break;
    }

    /* since WPS cannot be disabled, always write to LED */
    /* set white LED pwm if needed */
    if (set_white_pwm)
    {
        rc = i2c_led_reg_set(LED_WPS_WHITE_PWM, 0xff);
        if (rc < 0)
        {
            err = rc;
        }
    }

    /* write to register -- both WHITE and AMBER use the same register */
    rc = i2c_led_control(LED_WPS_WHITE_REG,
                         LED_WPS_WHITE_MASK | LED_WPS_AMBER_MASK,
                         val);
    if (rc < 0)
    {
        err = rc;
    }

    /* if LED is disabled, we want to update the buffered states anyway,
       so when restore happens, the wrong state won't be applied */
    if (atomic_read(&LedEnabled) == 0)
    {
        if (set_white_pwm)
        {
            i2c_buffered_reg_set(LED_WPS_WHITE_PWM, 0xff);
        }

        i2c_buffered_led_control(LED_WPS_WHITE_REG,
                                 LED_WPS_WHITE_MASK | LED_WPS_AMBER_MASK,
                                 val);
    }

    return err;
}

/* ---------------------------------------------------------------------------
 * Handle LAN (switch ports) LED actions 
 * Caller must hold LedLock
 * LAN LEDs are controlled via Marvell switch registers
 *
 * LED0 - bits 0:3
 * 0x00 off=link down, on=link up, blink=activity, blink rate=link speed
 * 0x01 off=link down, on=100/1000 link up, blink=activity
 * 0x02 off=link down, on=1000 link up, blink=activity
 * 0x03 off=link down, on=link up, blink=activity
 * 0x04 Port 0's special LED
 * 0x05 Reserved
 * 0x06 off=half-duplex, on=full-duplex, blink=collision
 * 0x07 off=link down, on=10/1000 link up, blink=activity
 * 0x08 off=link down, on=link up
 * 0x09 off=link down, on=10 link up
 * 0x0A off=link down, on=10 link up, blink=activity
 * 0x0B off=link down, on=100/1000 link up
 * 0x0C blink=PTP activity
 * 0x0D force blink
 * 0x0E force off
 * 0x0F force on
 *
 * LED1 - bits 4:7
 * 0x00 Port 2's special LED
 * 0x01 off=link down, on=10/100 link up, blink=activity
 * 0x02 off=link down, on=10/100 link up, blink=activity
 * 0x03 off=link down, on=1000 link up
 * 0x04 Port 1's special LED
 * 0x05 Reserved
 * 0x06 off=link down, on=10/1000 link up, blink=activity
 * 0x07 off=link down, on=10/1000 link up
 * 0x08 off=link down, blink=activity
 * 0x09 off=link down, on=100 link up
 * 0x0A off=link down, on=100 link up, blink=activity
 * 0x0B off=link down, on=10/100 link up
 * 0x0C blink=PTP activity
 * 0x0D force blink
 * 0x0E force off
 * 0x0F force on
 * ------------------------------------------------------------------------- */
static int led_lan_action(LedAction act)
{
    unsigned int val;
    int i;

    /* Exported from Marvell mv_switch.c, writes to Ethernet registers */
    extern int mv_switch_reg_write(int port, int reg, int type, MV_U16 value);

    switch (act)
    {
    case LED_ACTION_DEFAULT:
        val = 0x30;
        break;

    case LED_ACTION_OFF:
        val = 0xEE;
        break;

    case LED_ACTION_ON:
        val = 0xFF;
        break;

    case LED_ACTION_BLINK:
        val = 0xDD;
        break;

    default:
        return 0;
        break;
    }

    val |= 0x8000;

    for (i=0; i < LAN_PORTS_NUM; i++)
    {
        (void) mv_switch_reg_write(i, 22, 2, val);    
    }

    return 0;
}

#if 0
WiFi LED will be controlled via iwpriv user-space commands instead of
through this module

/* ---------------------------------------------------------------------------
 * WIFI LED action 
 * Caller must hold LedLock
 * WIFI is control by hardware, we can enable or disable it
 * ------------------------------------------------------------------------- */
static int led_wifi_action(LedAction act)
{
    struct net_device* dev;
    struct net_device* wifi24g = NULL;
    struct net_device* wifi5g = NULL;

    // Note: need to patch marvell driver to export this 
    /* Wifi driver loads after bdutil, so we have to load the symbol */
    int (*led_func)(struct net_device* netdev, UINT8 led_on);

    led_func = __symbol_get("wlFwLedon");
    if (led_func == NULL)
    {
        /* don't do anything, wifi driver isn't loaded, so this
           command doesn't mean anything */ 
        return 0;
    }
 
    /* We're not expected to be turning these LEDs on/off all the time,
       so this is fine. If this ever changes (I don't see it), we can
       optimize this by storing the net_device corresponding to the WIFI
       devices, so we don't have to find it every time */
    dev = first_net_device(&init_net);
    while (dev)
    {
        if ((wifi24g == NULL) &&
            (strcmp(dev->name, WIFI24G_DEV_NAME) == 0))
        {
            wifi24g = dev;
        }
        else if ((wifi5g == NULL) &&
                 (strcmp(dev->name, WIFI5G_DEV_NAME) == 0))
        {
            wifi5g = dev;
        }
        dev = next_net_device(dev);
    }

    switch (act)
    {
    case LED_ACTION_DEFAULT:
        (*led_func)(wifi24g, 1);
        (*led_func)(wifi5g, 1);
        break;

    case LED_ACTION_OFF:
        (*led_func)(wifi24g, 0);
        (*led_func)(wifi5g, 0);
        break;

    default:
        break;
    }

    symbol_put_addr(led_func);

    return 0;
}
#endif

/* ---------------------------------------------------------------------------
 * ESATA LED action 
 * Caller must hold LedLock
 * eSata is control by hardware, we can also set it to always off/on
 * ------------------------------------------------------------------------- */
static int led_esata_action(LedAction act)
{
    switch (act)
    {
    case LED_ACTION_DEFAULT:
        /* Set MPP46 to SATA_0_PRESENT_AC */
        gpioRegSet(LED_ESATA_MPP46_OFFSET,
                   LED_ESATA_MPP46_MASK,
                   LED_ESATA_HW_CONTROL);

        /* Set SATA0 to activity blink */
        gpioRegSet(SATA0_LED_CONFIG_REG,
                   SATA0_ACTIVITY_BLINK_MASK,
                   SATA0_SET_ACTIVITY_BLINK);
        break;

    case LED_ACTION_OFF:
        /* Set MPP46 to GPIO */
        gpioRegSet(LED_ESATA_MPP46_OFFSET,
                   LED_ESATA_MPP46_MASK,
                   LED_ESATA_GPIO_CONTROL);

        /* Enable GPIO46 (active low) */
        gpio_set_bit_low(LED_GPIO46_ENABLE_OFFSET,
                         LED_GPIO46_MASK);

        /* Set GPIO46 to off */
        gpio_set_bit_low(LED_GPIO46_OUTPUT_OFFSET,
                         LED_GPIO46_MASK);
        break;

    case LED_ACTION_ON:
        /* Set MPP46 to GPIO */
        gpioRegSet(LED_ESATA_MPP46_OFFSET,
                   LED_ESATA_MPP46_MASK,
                   LED_ESATA_GPIO_CONTROL);

        /* Enable GPIO46 (active low) */
        gpio_set_bit_low(LED_GPIO46_ENABLE_OFFSET,
                         LED_GPIO46_MASK);

        /* Set GPIO46 to on */
        gpio_set_bit_high(LED_GPIO46_OUTPUT_OFFSET,
                          LED_GPIO46_MASK);
        break;

    default:
        break;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * Enables/Disables LEDs
 * Caller must hold LedLock
 *
 * This function does not handle 2.4G/5G LEDs, as those LEDs are handled by
 * user space iwpriv command.
 * ------------------------------------------------------------------------- */
static int led_all_action(LedAction act)
{
    int i;

    switch (act)
    {
    case LED_ACTION_OFF:
        if (atomic_read(&LedEnabled))
        {
            /* Saves LED states */
            (void) i2c_led_save_state();

            /* do the off command for all LEDs */
            for (i = 0; i < LedCtlListSize; i++)
            {
                if (LedCtlList[i].can_disable)
                {
                    (void) LedCtlList[i].do_action(LED_ACTION_OFF);
                }
            }

            /* Turn off HW controlled LEDs */
            (void) led_lan_action(LED_ACTION_OFF);
            (void) led_esata_action(LED_ACTION_OFF);

            /* disable LED */
            atomic_set(&LedEnabled, 0);
        }
        break;

    case LED_ACTION_ON: 
        if (atomic_read(&LedEnabled) == 0)
        {
            /* Restore LED state */
            (void) i2c_led_restore_state();

            /* Restore LEDs to HW control */
            (void) led_lan_action(LED_ACTION_DEFAULT);
            (void) led_esata_action(LED_ACTION_DEFAULT);

            /* Turn LED back on */
            atomic_set(&LedEnabled, 1);
        }
        break;
    
    default:
        break;
    }

    return 0;
}

/* ===========================================================================
 * FAN CONTROL
 * Technically this is not related to LEDs, but this is a one-off thing
 * for this product only, so we just slide it into leds.c
 * ==========================================================================*/
 /* ---------------------------------------------------------------------------
 * Wrapper for reading from I2C temperature register
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static int i2c_temperature_reg_get(unsigned int reg, unsigned char* val)
{
    union i2c_smbus_data data;
    int rc;

    rc = i2c_smbus_xfer(I2C.adapter,
                        I2C_TEMPERATURE_ADDR,
                        0,
                        I2C_SMBUS_READ,
                        reg,
                        2,
                        &data);
    if (rc < 0)
    {
        return rc;
    }

    *val = data.byte;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Wrapper for writing to I2C temperature register
 * Caller must hold LedLock
 * ------------------------------------------------------------------------- */
static int i2c_temperature_reg_set(unsigned int reg, unsigned char val)
{
    union i2c_smbus_data data;

    /* If LED is not disabled, then do normal write */
    data.byte = val;
    return i2c_smbus_xfer(I2C.adapter,
                          I2C_TEMPERATURE_ADDR,
                          0,
                          I2C_SMBUS_WRITE,
                          reg,
                          2,
                          &data);
}

/* ---------------------------------------------------------------------------
 * Determines the state the fan should be
 * INPUT
 *    temperature - current temperature
 *    fan_state   - current fan state
 *    off2low, low2off, low2high, high2low - thresholds
 * RETURN
 *    new fan speed
 * ------------------------------------------------------------------------- */
static int fan_get_speed(int temperature, FanSpeed fan_state, int off2low, 
                         int low2off, int low2high, int high2low)
{
    FanSpeed new_fan_state = fan_state;

    if (temperature == TEMPERATURE_INVALID)
    {
        /* We can ALWAYS read CPU junction temperature. In the case
           where DDR and/or WIFI junction temperatures are not available,
           we default to use the CPU. Since higher fan speed has priority,
           we set it to off (lowest priority), this way the CPU junction
           temperature (never invalid) will be the only deciding factor. */
        return FAN_SPEED_OFF;
    }

    switch (fan_state)
    {
    case FAN_SPEED_OFF:
        if (temperature >= low2high)
        {
            new_fan_state = FAN_SPEED_HIGH;
        }
        else if (temperature >= off2low)
        {
            new_fan_state = FAN_SPEED_LOW;
        }
        break;

    case FAN_SPEED_LOW:
        if (temperature >= low2high)
        {
            new_fan_state = FAN_SPEED_HIGH;
        }
        else if (temperature <= low2off)
        {
            new_fan_state = FAN_SPEED_OFF;
        }
        break;

    case FAN_SPEED_HIGH:
        if (temperature <= low2off)
        {
            new_fan_state = FAN_SPEED_OFF;
        }
        else if (temperature <= high2low)
        {
            new_fan_state = FAN_SPEED_LOW;
        }
        break;
    }

    return new_fan_state;
}

/* ---------------------------------------------------------------------------
 * Fan control thread (MV_GPP24)
 * ------------------------------------------------------------------------- */
static int fan_control_thread(void* data)
{
    FanControlData* fandata = (FanControlData*) data;
    FanControlData localData;
    unsigned char value;
    FanSpeed cpu_fan, ddr_fan, wifi_fan;
    FanSpeed cur_fan_speed, new_fan_speed;

    /* Exported from Marvell hwmon.c, reads CPU junction temperature */
    extern int axptemp_read_temp(void);

    while (!kthread_should_stop())
    {
        /* read temperature */
        localData.cpu_last_temp = axptemp_read_temp();

        if (i2c_temperature_reg_get(TEMPERATURE_DDR_HIGH_REG, &value) == 0)
        {
            localData.ddr_last_temp = (((value & 0xf0) >> 4) * 16) + (value & 0x0f);
        }
        else
        {
            localData.ddr_last_temp = TEMPERATURE_INVALID;
        }

        if (i2c_temperature_reg_get(TEMPERATURE_WIFI_HIGH_REG, &value) == 0)
        {
            localData.wifi_last_temp = (((value & 0xf0) >> 4) * 16) + (value & 0x0f);
        }
        else
        {
            localData.wifi_last_temp = TEMPERATURE_INVALID;
        }

        /* read current fan speed */
        if ((MV_REG_READ(FAN_GPIO24_OUTPUT) & FAN_GPIO24_MASK) == 0)
        {
            cur_fan_speed = FAN_SPEED_OFF;
        }
        else
        {
            if (MV_REG_READ(FAN_GPIO24_BLINK_ENABLE) & FAN_GPIO24_MASK)
            {
                cur_fan_speed = FAN_SPEED_LOW;
            }
            else
            {
                cur_fan_speed = FAN_SPEED_HIGH;
            }
        }

        /* sync with fandata */
        mutex_lock(&LedLock);
        localData.cpu_off2low = fandata->cpu_off2low;
        localData.cpu_low2off = fandata->cpu_low2off;
        localData.cpu_low2high = fandata->cpu_low2high;
        localData.cpu_high2low = fandata->cpu_high2low;
 
        localData.ddr_off2low = fandata->ddr_off2low;
        localData.ddr_low2off = fandata->ddr_low2off;
        localData.ddr_low2high = fandata->ddr_low2high;
        localData.ddr_high2low = fandata->ddr_high2low;

        localData.wifi_off2low = fandata->wifi_off2low;
        localData.wifi_low2off = fandata->wifi_low2off;
        localData.wifi_low2high = fandata->wifi_low2high;
        localData.wifi_high2low = fandata->wifi_high2low;

        fandata->cpu_last_temp = localData.cpu_last_temp;
        fandata->ddr_last_temp = localData.ddr_last_temp;
        fandata->wifi_last_temp = localData.wifi_last_temp;
        mutex_unlock(&LedLock);

        /* check CPU temperature */
        cpu_fan = fan_get_speed(localData.cpu_last_temp,
                                cur_fan_speed,
                                localData.cpu_off2low,
                                localData.cpu_low2off,
                                localData.cpu_low2high,
                                localData.cpu_high2low);

        /* check DDR junction temperature */
        ddr_fan = fan_get_speed(localData.ddr_last_temp,
                                cur_fan_speed,
                                localData.ddr_off2low,
                                localData.ddr_low2off,
                                localData.ddr_low2high,
                                localData.ddr_high2low);

        /* check WIFI junction temperature */
        wifi_fan = fan_get_speed(localData.wifi_last_temp,
                                 cur_fan_speed,
                                 localData.wifi_off2low,
                                 localData.wifi_low2off,
                                 localData.wifi_low2high,
                                 localData.wifi_high2low);

        /* check all the fan decisions and figure out what speed
           the fan should be */
        new_fan_speed = cpu_fan;
        if (ddr_fan > cpu_fan)
        {
            new_fan_speed = ddr_fan;
        }
        if (wifi_fan > cpu_fan)
        {
            new_fan_speed = wifi_fan;
        }

        if (new_fan_speed != cur_fan_speed)
        {
            switch (new_fan_speed)
            {
            case FAN_SPEED_OFF:
                gpio_toggle_bit_low(FAN_GPIO24_BLINK_ENABLE,
                                    FAN_GPIO24_MASK);
                gpio_toggle_bit_low(FAN_GPIO24_OUTPUT,
                                    FAN_GPIO24_MASK);
                break;

            case FAN_SPEED_LOW:
                gpio_toggle_bit_high(FAN_GPIO24_BLINK_ENABLE,
                                     FAN_GPIO24_MASK);
                gpio_toggle_bit_high(FAN_GPIO24_OUTPUT,
                                     FAN_GPIO24_MASK);
                break;

            case FAN_SPEED_HIGH:
                gpio_toggle_bit_low(FAN_GPIO24_BLINK_ENABLE,
                                    FAN_GPIO24_MASK);
                gpio_toggle_bit_high(FAN_GPIO24_OUTPUT,
                                     FAN_GPIO24_MASK);
                break;
            }
        }

        ssleep(FAN_CONTROL_PERIOD);
    }

    return 0;
}

/* ===========================================================================
 * procfs handler functions
 * ==========================================================================*/
/* ---------------------------------------------------------------------------
 * Parses proc commands for LED action
 * return EINVAL on error, 0 on success
 * ------------------------------------------------------------------------- */
static int parse_led(char* proc_buff)
{
    char* led_str;
    char* action_str;
    char* delimiter;
    int i, rc;
    LedAction act;

    delimiter = strchr(proc_buff, '=');
    if (delimiter == NULL)
    {
        return -EINVAL;
    }

    *delimiter = '\0';
    led_str = proc_buff;
    action_str = delimiter + 1;

    LOG("mod_bdutil (led): led = %s, action = %s\n", led_str, action_str);

    /* parse the action */
    if (strcmp(action_str, "off") == 0)
    {
        act = LED_ACTION_OFF;
    }
    else if (strcmp(action_str, "on") == 0)
    {
        act = LED_ACTION_ON;
    }
    else if (strcmp(action_str, "blink") == 0)
    {
        act = LED_ACTION_BLINK;
    }
    else if (strcmp(action_str, "fastblink") == 0)
    {
        act = LED_ACTION_FAST_BLINK;
    }
    else if (strcmp(action_str, "alton") == 0)
    {
        act = LED_ACTION_ALT_ON;
    }
    else if (strcmp(action_str, "altblink") == 0)
    {
        act = LED_ACTION_ALT_BLINK;
    }
    else
    {
        /* unknown action */
        return 0;
    }

    /* find the LED & execute associated action */
    for (i = 0; i < LedCtlListSize; i++)
    {
        if (strcmp(led_str, LedCtlList[i].name) == 0)
        {
            mutex_lock(&LedLock);
            rc = LedCtlList[i].do_action(act);
            mutex_unlock(&LedLock);
            return rc;
        }
    }

    /* check the "all" command, it's separate because it's not an LED */
    if (strcmp(led_str, "all") == 0)
    {
        mutex_lock(&LedLock);
        rc = led_all_action(act);
        mutex_unlock(&LedLock);
        return rc;
    }
    
    /* unknown LED name */
    return 0;
}

/* ---------------------------------------------------------------------------
 * Display help
 * ------------------------------------------------------------------------- */
static int help_proc_read(char* page, char** start, off_t off, int count,
                          int* eof, void* data)
{
    return sprintf(page, UsageStr);
}

/* ---------------------------------------------------------------------------
 * Write handler for proc interface
 * ------------------------------------------------------------------------- */
static int leds_proc_write(struct file* file, const char __user* buffer,
                           unsigned long count, void* data)
{
    char  proc_buff[PROC_BUF_SIZE];
    char* ptr;

    if (count > PROC_BUF_SIZE)
    {
        count = PROC_BUF_SIZE;
    }

    if (copy_from_user(proc_buff, buffer, count))
    {
        return -EFAULT;
    }

    ptr = strchr(proc_buff, '\n');
    if (ptr == NULL)
    {
        ptr = &proc_buff[PROC_BUF_SIZE - 1];
    }
    *ptr = '\0';

    parse_led(proc_buff);

    return count;
}

/* ---------------------------------------------------------------------------
 * Display fan control settigns
 * ------------------------------------------------------------------------- */
static int fan_proc_read(char* page, char** start, off_t off, int count,
                         int* eof, void* data)
{
    int rc;
    char cpu_txt[10], ddr_txt[10], wifi_txt[10];
    FanControlData localData;

    mutex_lock(&LedLock);
    memcpy(&localData, &FanData, sizeof(localData));
    mutex_unlock(&LedLock);
    
    if (localData.cpu_last_temp != TEMPERATURE_INVALID)
    {
        sprintf(cpu_txt, "%d", localData.cpu_last_temp);
    }
    else
    {
        sprintf(cpu_txt, "Err");
    }

    if (localData.ddr_last_temp != TEMPERATURE_INVALID)
    {
        sprintf(ddr_txt, "%d", localData.ddr_last_temp);
    }
    else
    {
        sprintf(ddr_txt, "Err");
    }

    if (localData.wifi_last_temp != TEMPERATURE_INVALID)
    {
        sprintf(wifi_txt, "%d", localData.wifi_last_temp);
    }
    else
    {
        sprintf(wifi_txt, "Err");
    }

    rc = sprintf(page,
                 "Last reading: cpu %s, ddr %s, wifi %s\n"
                 "Settings:\n"
                 "   JUNCTION OFF->LOW LOW->OFF LOW->HIGH HIGH->LOW\n"
                 "     cpu      %3d      %3d      %3d       %3d\n"
                 "     ddr      %3d      %3d      %3d       %3d\n"
                 "     wifi     %3d      %3d      %3d       %3d\n"
                 "To set:\n"
                 "   Example: echo \"cpu=%d %d %d %d\" > /proc/bdutil/fan\n",
                 cpu_txt, ddr_txt, wifi_txt,
                 localData.cpu_off2low, localData.cpu_low2off,
                 localData.cpu_low2high, localData.cpu_high2low,
                 localData.ddr_off2low, localData.ddr_low2off,
                 localData.ddr_low2high, localData.ddr_high2low,
                 localData.wifi_off2low, localData.wifi_low2off,
                 localData.wifi_low2high, localData.wifi_high2low,
                 localData.cpu_off2low, localData.cpu_low2off,
                 localData.cpu_low2high, localData.cpu_high2low);

    return rc;
}

/* ---------------------------------------------------------------------------
 * Set fan control settings
 * ------------------------------------------------------------------------- */
static int fan_proc_write(struct file* file, const char __user* buffer,
                          unsigned long count, void *data)
{
    char proc_buff[PROC_BUF_SIZE];
    char* junction_str;
    char* value_str;
    char* delimiter;
    int off2low, low2off, low2high, high2low;

    /* prepare the input buffer */
    if (count > PROC_BUF_SIZE)
    {
        count = PROC_BUF_SIZE;
    }

    if (copy_from_user(proc_buff, buffer, count))
    {
        return -EFAULT;
    }

    delimiter = strchr(proc_buff, '\n');
    if (delimiter == NULL)
    {
        delimiter = &proc_buff[PROC_BUF_SIZE - 1];
    }
    *delimiter = '\0';

    /* split the input to A/V pair */
    delimiter = strchr(proc_buff, '=');
    if (delimiter == NULL)
    {
        return -EINVAL;
    }
    *delimiter = '\0';

    junction_str = proc_buff;
    value_str = delimiter + 1;

    /* parse the value */
    if (sscanf(value_str,
               "%d %d %d %d",
               &off2low, &low2off, &low2high, &high2low) != 4)
    {
        return -EINVAL;
    }

    /* set the settings */
    if (strcmp(junction_str, "cpu") == 0)
    {
        mutex_lock(&LedLock);
        FanData.cpu_off2low = off2low;
        FanData.cpu_low2off = low2off;
        FanData.cpu_low2high = low2high;
        FanData.cpu_high2low = high2low;
        mutex_unlock(&LedLock);
    }
    else if (strcmp(junction_str, "ddr") == 0)
    {
        mutex_lock(&LedLock);
        FanData.ddr_off2low = off2low;
        FanData.ddr_low2off = low2off;
        FanData.ddr_low2high = low2high;
        FanData.ddr_high2low = high2low;
        mutex_unlock(&LedLock);
    }
    else if (strcmp(junction_str, "wifi") == 0)
    {
        mutex_lock(&LedLock);
        FanData.wifi_off2low = off2low;
        FanData.wifi_low2off = low2off;
        FanData.wifi_low2high = low2high;
        FanData.wifi_high2low = high2low;
        mutex_unlock(&LedLock);
    }

    return count;
}

/* ===========================================================================
 * Init/Cleanup functions
 * ==========================================================================*/
/* ---------------------------------------------------------------------------
 * Initialize I2C LED
 * ------------------------------------------------------------------------- */
static int do_init_i2c_leds(void)
{
    unsigned int minor = 0;
    struct i2c_dev *dev;
    int rc, i;

    /* setup activity blink global data */
    /* -------------------------------- */
    memset(&LedActivity, 0x00, sizeof(LedActivity));

    /* init locks */
    sema_init(&LedActivity.sem, 0);
    spin_lock_init(&LedActivity.activity_lock);

    /* populate LED activty blink data, other struct members are already set
       to 0x00 */
    LedActivity.led[LED_ACTIVITY_USB1].reg = LED_USB1_REG;
    LedActivity.led[LED_ACTIVITY_USB1].mask = LED_USB1_MASK;
    LedActivity.led[LED_ACTIVITY_USB1].off = TLC_LED_DRIVER_DEF_OFF << 
                                             LED_USB1_SHIFT;
    LedActivity.led[LED_ACTIVITY_USB1].on = TLC_LED_DRIVER_FULL_ON << 
                                            LED_USB1_SHIFT;

    LedActivity.led[LED_ACTIVITY_USB2].reg = LED_USB2A_REG;
    LedActivity.led[LED_ACTIVITY_USB2].mask = LED_USB2A_MASK;
    LedActivity.led[LED_ACTIVITY_USB2].off = TLC_LED_DRIVER_DEF_OFF << 
                                             LED_USB2A_SHIFT;
    LedActivity.led[LED_ACTIVITY_USB2].on = TLC_LED_DRIVER_FULL_ON << 
                                            LED_USB2A_SHIFT;

    /* connect to I2C */
    /* -------------- */
    dev = i2c_dev_get_by_minor(minor);
    I2C.adapter = i2c_get_adapter(dev->adap->nr);

    /* initialize LED */
    /* -------------- */
    /* Set allcall */
    rc = i2c_led_reg_set(TLC_MODE1, TLC_MODE1_ALLCALL);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C LED (MODE1)\n");
        goto err_exit;
    }

    /* set enable blinking */
    rc = i2c_led_reg_set(TLC_MODE2, TLC_MODE2_DMBLNK);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C LED (MODE2)\n");
        goto err_exit;
    }

    /* set blink duty */
    rc = i2c_led_reg_set(TLC_GRPPWM, 0x7f /* 50% */);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C LED (GRPPWM)\n");
        goto err_exit;
    }

    /* set blink frequency */
    rc = i2c_led_reg_set(TLC_GRPFREQ, 16 /* (16 + 1) / 25 = .708 sec */);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C LED (GRPFREQ)\n");
        goto err_exit;
    }

    /* set WAN amber PWM to full for blinking */
    rc = i2c_led_reg_set(LED_WAN_AMBER_PWM, 0xff);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C LED (PWM0)\n");
        goto err_exit;
    }

    /* set WAN white PWM to full for blinking */
    rc = i2c_led_reg_set(LED_WAN_WHITE_PWM, 0xff);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C LED (PWM1)\n");
        goto err_exit;
    }

    /* set WPS white PWM to full for blinking */
    rc = i2c_led_reg_set(LED_WPS_WHITE_PWM, 0xff);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C LED (PWM8)\n");
        goto err_exit;
    }

    /* set WPS amber PWM to full for blinking */
    rc = i2c_led_reg_set(LED_WPS_AMBER_PWM, 0xff);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C LED (PWM9)\n");
        goto err_exit;
    }

    /* setup temperature sensors */
    /* ------------------------- */
    rc = i2c_temperature_reg_set(TEMPERATURE_CONFIG1_REG,
                                 TEMPERATURE_CONFIG1_DEFAULT);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C TEMP (CONFIG1)\n");
        goto err_exit;
    }
    
    rc = i2c_temperature_reg_set(TEMPERATURE_CONFIG2_REG, 
                                 TEMPERATURE_CONFIG2_DEFAULT);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C TEMP (CONFIG2)\n");
        goto err_exit;
    }

    rc = i2c_temperature_reg_set(TEMPERATURE_CONV_REG, 
                                 TEMPERATURE_CONV_DEFAULT);
    if (rc < 0)
    {
        ERR("mod_bdutil (led): failed to init I2C TEMP (CONV)\n");
        goto err_exit;
    }

    /* setup fan speed (blink duty) */
    /* ---------------------------- */
    gpioRegSet(GPIO_BLINK_COUNTER_B_ON_DUTY,
               0xffffffff,
               FAN_LOW_SPEED_ON_DUTY);
    gpioRegSet(GPIO_BLINK_COUNTER_B_OFF_DUTY,
               0xffffffff,
               FAN_LOW_SPEED_OFF_DUTY);
    /* select counter B for blink duty */
    gpio_toggle_bit_high(FAN_GPIO24_BLINK_COUNTER,
                         FAN_GPIO24_MASK);

    /* setup irq */
    /* --------- */
    for (i = 0; i < LED_ACTIVITY_MAX; i++)
    {
        LedActivity.irq_data[i].sem = &LedActivity.sem;
        LedActivity.irq_data[i].activity_lock = &LedActivity.activity_lock;
        LedActivity.irq_data[i].has_activity = &LedActivity.led[i].has_activity;
    }

    /* initialize irq */
    rc = request_irq(USB1_IRQ,
                     blink_activity_irq_handler,
                     IRQF_SHARED,
                     "mod_bdutil:led_usb1",
                     &LedActivity.irq_data[LED_ACTIVITY_USB1]);
    if (rc != 0)
    {
        ERR("mod_bdutil: request_irq %d failed\n", USB1_IRQ);
        goto err_exit;   
    }
    LedActivity.irq[LED_ACTIVITY_USB1] = USB1_IRQ;
    LedActivity.irq_initialized[LED_ACTIVITY_USB1] = true;

    rc = request_irq(USB2_IRQ,
                     blink_activity_irq_handler,
                     IRQF_SHARED,
                     "mod_bdutil:led_usb2",
                     &LedActivity.irq_data[LED_ACTIVITY_USB2]);
    if (rc != 0)
    {
        ERR("mod_bdutil: request_irq %d failed\n", USB2_IRQ);
        goto err_exit;
    }
    LedActivity.irq[LED_ACTIVITY_USB2] = USB1_IRQ;
    LedActivity.irq_initialized[LED_ACTIVITY_USB2] = true;

    /* start blink threads */
    for (i = 0; i < LED_ACTIVITY_BLINK_THREADS; i++)
    {
        LedActivity.blinker[i] = kthread_run(activity_blink_thread,
                                             NULL,
                                             "mod_bdutil:activity");
        if (LedActivity.blinker[i] == NULL)
        {
            ERR("mod_bdutil (led): cannot create activity thread");
            goto err_exit;
        }
    }

    /* create WPS LED fade workqueue */
    /* ----------------------------- */
    memset(&WPSLed_wq, 0x00, sizeof(WPSLed_wq));

    WPSLed_wq.wq = create_singlethread_workqueue("wpsled_wq");
    if (WPSLed_wq.wq == NULL)
    {
        ERR("mod_bdutil (led): failed to create LED fade workqueue\n");
        rc = -ENOMEM;
        goto err_exit;
    }
    INIT_DELAYED_WORK(&WPSLed_wq.work, wps_led_wq_handler);

    /* start fan control thread */
    /* ------------------------ */
    FanControlThread = kthread_run(fan_control_thread,
                                   (void*) &FanData,
                                   "mod_bdutil:fan");
    if (FanControlThread == NULL)
    {
        ERR("mod_bdutil (leds): cannot create fan thread");
        goto err_exit;
    }

    LOG("mod_bdutil (leds): i2c-dev (%d) LEDs driver initialized\n",
        I2C.adapter->nr);
    return 0;    

err_exit:
    do_cleanup_i2c_leds();
    ERR("mod_bdutil (leds): I2C LED init failed\n");
    return rc;
}

/* ---------------------------------------------------------------------------
 * Cleanup I2C LED
 * ------------------------------------------------------------------------- */
static void do_cleanup_i2c_leds(void)
{
    int i;

    for (i = 0; i < LED_ACTIVITY_MAX; i++)
    {
        if (LedActivity.irq_initialized[i])
        {
            free_irq(LedActivity.irq[i], &LedActivity.irq_data[i]);
            LedActivity.irq_initialized[i] = false;
        }
    }

    if (FanControlThread != NULL)
    {
        kthread_stop(FanControlThread);
        FanControlThread = NULL;
    }

    for (i = 0; i < LED_ACTIVITY_BLINK_THREADS; i++)
    {
        if (LedActivity.blinker[i] != NULL)
        {
            kthread_stop(LedActivity.blinker[i]);
            LedActivity.blinker[i] = NULL;
        }
    }

    if (WPSLed_wq.wq)
    {
        destroy_workqueue(WPSLed_wq.wq);
        WPSLed_wq.wq = NULL;
    }
}

/* ---------------------------------------------------------------------------
 * Initialize gpio LED
 * ------------------------------------------------------------------------- */
static int do_init_gpio_leds(void)
{
    /* set blink rate, defaults to not ready blink */
    gpioRegSet(GPP_BLINK_CTR_A_ON_DUR_REG(1),
               MV_GPP_IN_INVERT,
               GPIO_LED_BLINK_NOT_READY);
    gpioRegSet(GPP_BLINK_CTR_A_OFF_DUR_REG(1),
               MV_GPP_IN_INVERT,
               GPIO_LED_BLINK_NOT_READY);

    /* set fan control */
    gpioRegSet(FAN_MPP24_OFFSET,
               FAN_MPP24_MASK,
               FAN_GPIO_CONTROL);

    /* enable fan GPIO24, enable is active low */
    gpio_set_bit_low(FAN_GPIO24_OUTPUT_ENABLE,
                     FAN_GPIO24_MASK);

    return 0;
}

/* ---------------------------------------------------------------------------
 * Cleanup gpio LED
 * ------------------------------------------------------------------------- */
static void do_cleanup_gpio_leds(void)
{
}

/* ---------------------------------------------------------------------------
 * Initialize LED data
 * ------------------------------------------------------------------------- */
static int leds_init_data(void)
{
    int rc = 0;

    switch (bdutil_gboardId)
    {
    case DB_784MP_GP_ID: /* mamba board */
        DBGX(1, "%s: DB_88F78XX0_BP_ID\n", __FUNCTION__);
        UsageStr = UsageStrMamba;
        break;

    default:    
        UsageStr = '\0';
        ERR("mod_bdutil (leds): board Type not supported (%d)",
             bdutil_gboardId);
        rc = -1;
        break;
    }

    return rc;
}

/* ---------------------------------------------------------------------------
 * cleanup
 * ------------------------------------------------------------------------- */
static void cleanup_leds(void)
{
    do_cleanup_gpio_leds();
    do_cleanup_i2c_leds();

    if (ProcLedsNode)
    {
        remove_proc_entry(BDUTIL_LEDS_PROC_DEVICE_NAME, proc_bdutil);
        ProcLedsNode = NULL;
    }

    if (ProcHelpNode)
    {
        remove_proc_entry(BDUTIL_HELP_PROC_DEVICE_NAME, proc_bdutil);
        ProcHelpNode = NULL;
    }

    if (ProcFanNode)
    {
        remove_proc_entry(FAN_PROC_NAME, proc_bdutil);
        ProcFanNode = NULL;
    }
}

/* ---------------------------------------------------------------------------
 * Initialize
 * ------------------------------------------------------------------------- */
int init_leds(void)
{
    int rc;

    LOG(DRV_DESC ", version " DRV_VERSION "\n");

    atomic_set(&LedEnabled, 1);

    /* Create proc interface */
    ProcHelpNode = create_proc_entry(BDUTIL_HELP_PROC_DEVICE_NAME,
                                     0666,
                                     proc_bdutil);
    if (ProcHelpNode == NULL)
    {
        ERR("mod_bdutil (leds): failed to create help proc entry\n");
        rc = ENOENT;
        goto err_exit;
    }
    ProcHelpNode->read_proc = help_proc_read;
    
    ProcLedsNode = create_proc_entry(BDUTIL_LEDS_PROC_DEVICE_NAME,
                                     0666,
                                     proc_bdutil);
    if (ProcLedsNode == NULL)
    {
        ERR("mod_bdutil (leds): failed to create LEDs proc entry\n");
        rc = ENOENT;
        goto err_exit;
    }
    ProcLedsNode->write_proc = leds_proc_write;

    ProcFanNode = create_proc_entry(FAN_PROC_NAME,
                                    0666,
                                    proc_bdutil);
    if (ProcFanNode == NULL)
    {
        ERR("mod_bdutil (leds): failed to create fan proc entry\n");
        rc = ENOENT;
        goto err_exit;
    }
    ProcFanNode->read_proc = fan_proc_read;
    ProcFanNode->write_proc = fan_proc_write;

    /* init */
    rc = do_init_i2c_leds();
    if (rc != 0)
    {
        goto err_exit;
    }

    rc = do_init_gpio_leds();
    if (rc != 0)
    {
        goto err_exit;
    }

    rc = leds_init_data();
    if (rc != 0)
    {
        goto err_exit;
    }

    return 0;

err_exit:
    ERR("mod_bdutil (leds): init failed\n");
    cleanup_leds();
    return 0;
}

/* ---------------------------------------------------------------------------
 * Exit
 * ------------------------------------------------------------------------- */
void exit_leds(void)
{
    cleanup_leds();
    LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}

