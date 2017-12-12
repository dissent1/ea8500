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
  @file leds.h
  @brief board led(s) support
*************************************************************************/

#ifndef __BDUTIL_LEDS_H__
#define __BDUTIL_LEDS_H__

extern int init_leds(void);
extern void exit_leds(void);
int leds_blue_on(void);
int leds_blue_off(void);


/*
 * Broadcom platforms GPIO used for LEDs
 *
 * bentely 		GPIO 9
 * carrera 		GPIO 1,2
 * honda 		GPIO 6,8
 * lemans 		GPIO 8
 * r8 		GPIO 8
 * esprit 		GPIO 8
 *
 */

#define BDUTIL_LEDS_PROC_DEVICE_NAME	"leds"
#if defined( _HONDA ) || defined(_LEMANS) || defined(_ESPRIT) || defined(_F70) || defined(_R8)
#define BOARD_GPIO_WPS_LED_PIN		8
#elif defined(_SPYDER)
#define BOARD_GPIO_WPS_LED_PIN		8
#define BOARD_GPIO_WIFI_PIN		    0
#else
/* bentley */
#define BOARD_GPIO_WPS_LED_PIN		9
#endif

#define	S_INPUT			1
#define D_INPUT			2

#define ON  1
#define OFF 0
	
typedef struct {
	char	*name;
	int	type;		/* LED type: single-input, dual-input */ 
	int	mpp_pin;	/* MPP pin number */
	int	gpio_pin;	/* gpio pin number */
	int	gpio_polarity;	/* gpio polarity active */
	int	mpp_pair;	/* MPP pair for dual-input LED */
	int	mpp_pair_val;	/* MPP pair value for dual-input LED */
} leds_t;

#endif /* __BDUTIL_LEDS_H__ */
