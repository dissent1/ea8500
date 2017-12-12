/*
 * Copyright (c) 2013,   Belkin International Inc.
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
  @file tlc_io_expander.h
  @brief TLC io_expander kernel module 
*************************************************************************/

#ifndef __TLC_IO_EXPANDER_H__
#define __TLC_IO_TXPANDER_H__

/****************************************************************
 * Texas Instruments
 * TLC59116 - 16-Channel I2C-Bus LED I/O Expander
 * 6 LED Drivers (Each Output Programmable at
 * Off, On, Programmable LED Brightness, or
 * Programmable Group Dimming/Blinking Mixed
 * With Individual LED Brightness)
 *****************************************************************/

/*
 * Registers
 */
#define TLC_BASE   0x00

#define TLC_MODE1   (TLC_BASE + 0x00)    /* R/W    Mode 1 */    
#define TLC_MODE2   (TLC_BASE + 0x01)    /* R/W    Mode 2 */
#define TLC_PWM0    (TLC_BASE + 0x02)    /* R/W    Brightness control LED0 */
#define TLC_PWM1    (TLC_BASE + 0x03)    /* R/W    Brightness control LED1 */
#define TLC_PWM2    (TLC_BASE + 0x04)    /* R/W    Brightness control LED2 */
#define TLC_PWM3    (TLC_BASE + 0x05)    /* R/W    Brightness control LED3 */
#define TLC_PWM4    (TLC_BASE + 0x06)    /* R/W    Brightness control LED4 */
#define TLC_PWM5    (TLC_BASE + 0x07)    /* R/W    Brightness control LED5 */
#define TLC_PWM6    (TLC_BASE + 0x08)    /* R/W    Brightness control LED6 */
#define TLC_PWM7    (TLC_BASE + 0x09)    /* R/W    Brightness control LED7 */
#define TLC_PWM8    (TLC_BASE + 0x0A)    /* R/W    Brightness control LED8 */
#define TLC_PWM9    (TLC_BASE + 0x0B)    /* R/W    Brightness control LED9 */
#define TLC_PWM10    (TLC_BASE + 0x0C)    /* R/W    Brightness control LED10 */
#define TLC_PWM11    (TLC_BASE + 0x0D)    /* R/W    Brightness control LED11 */
#define TLC_PWM12    (TLC_BASE + 0x0E)    /* R/W    Brightness control LED12 */
#define TLC_PWM13    (TLC_BASE + 0x0F)    /* R/W    Brightness control LED13 */
#define TLC_PWM14    (TLC_BASE + 0x10)    /* R/W    Brightness control LED14 */
#define TLC_PWM15    (TLC_BASE + 0x11)    /* R/W    Brightness control LED15 */
#define TLC_GRPPWM   (TLC_BASE + 0x12)    /* R/W    Group duty cycle control */
#define TLC_GRPFREQ    (TLC_BASE + 0x13)    /* R/W    Group frequency */
#define TLC_LEDOUT0    (TLC_BASE + 0x14)    /* R/W    LED output state 0 */
#define TLC_LEDOUT1    (TLC_BASE + 0x15)    /* R/W    LED output state 1 */
#define TLC_LEDOUT2    (TLC_BASE + 0x16)    /* R/W    LED output state 2 */
#define TLC_LEDOUT3    (TLC_BASE + 0x17)    /* R/W    LED output state 3 */
#define TLC_SUBADR1    (TLC_BASE + 0x18)    /* R/W    I2C bus subaddress 1 */
#define TLC_SUBADR2    (TLC_BASE + 0x19)    /* R/W    I2C bus subaddress 2 */
#define TLC_SUBADR3    (TLC_BASE + 0x1A)    /* R/W    I2C bus subaddress 3 */
#define TLC_ALLCALLADR (TLC_BASE + 0x1B)    /* R/W    LED Alt Call I2C bus address */
#define TLC_IREF    (TLC_BASE + 0x1C)    /* R/W    IREF configuration */
#define TLC_EFLAG1  (TLC_BASE + 0x1D)    /* R    Error flags 1 */
#define TLC_EFLAG2  (TLC_BASE + 0x1E)    /* R    Error flags 2 */

/* MODE1 Register Bit Definition */
#define TLC_MODE1_AL2        BIT7
#define TLC_MODE1_AL1        BIT6
#define TLC_MODE1_AL0        BIT5
#define TLC_MODE1_OSC        BIT4
#define TLC_MODE1_SUB1       BIT3
#define TLC_MODE1_SUB2       BIT2
#define TLC_MODE1_SUB3       BIT1
/* TLC_MODE1_ALLCALL R/W    1: Device responds to LED All Call I2C bus address */
/*                0: Device does not respond to LED All Call I2C bus address */
#define TLC_MODE1_ALLCALL    BIT0     

/* MODE2 Register Bit Definition */
#define TLC_MODE2_EFCLR      BIT7
#define TLC_MODE2_DMBLNK     BIT5
#define TLC_MODE2_OCH        BIT3

/* LEDOUT0 to LEDOUT3 States */ 
#define TLC_LED_DRIVER_DEF_OFF    0x0  /* driver x is off - default power-up state */
#define TLC_LED_DRIVER_FULL_ON    0x1  /* driver x is fully on */ 
#define TLC_LED_DRIVER_PWM_CRTL   0x2  /* driver x brightness controlled via PWM */
#define TLC_LED_DRIVER_DIMM_BLINK 0x3  /* driver x brightness dimming/blinking controlled through PWM register and the GRPPWM registers */ 

/* For managing individual LED contol */
#define TLC_LED0_MASK 0x03
#define TLC_LED1_MASK 0x0c
#define TLC_LED2_MASK 0x30
#define TLC_LED3_MASK 0xc0

#define TLC_LED0_SHIFT 0
#define TLC_LED1_SHIFT 2
#define TLC_LED2_SHIFT 4
#define TLC_LED3_SHIFT 6

#endif /* __TLC_IO_EXPANDER_H__ */
