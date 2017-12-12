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

#ifndef __BDUTIL_USBHUB_H__
#define __BDUTIL_USBHUB_H__

extern int init_usbhub(void);
extern void exit_usbhub(void);

#define BDUTIL_USBHUB_PROC_DEVICE_NAME	"usbhub"

/*** Carrera ***/
#define BOARD_GPIO_USBHUB_RESET_PIN		13

#if defined( _SPYDER )
#define BOARD_GPIO_USB0_PORT_ENABLE		13
#define BOARD_GPIO_USB1_PORT_ENABLE		14
#else
/*** Honda, Esprit, Lemans, R8 ***/
#define BOARD_GPIO_USB0_PORT_ENABLE		9
#define BOARD_GPIO_USB1_PORT_ENABLE		10
#endif
#endif /* __BDUTIL_USBHUB_H__ */
