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

#define BDUTIL_LEDS_PROC_DEVICE_NAME	"leds"

extern int init_leds(void);
extern void exit_leds(void);

#endif /* __BDUTIL_LEDS_H__ */
