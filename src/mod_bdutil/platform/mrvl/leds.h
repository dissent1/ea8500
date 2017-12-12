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

#define BDUTIL_LEDS_PROC_DEVICE_NAME	"leds"

#define	S_INPUT			1
#define D_INPUT			2
	
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
