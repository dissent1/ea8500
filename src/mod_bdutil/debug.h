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
  @file debug.h
  @brief board debug 
*************************************************************************/

#ifndef __BDUTIL_DEBUG_H__
#define __BDUTIL_DEBUG_H__

#include <linux/kernel.h>

#define LOG(x...) printk(KERN_INFO x)

#ifdef _DEBUG
#define ERR(x...) printk(KERN_ERR x)
#define DBG(x...) printk(KERN_INFO x)
#define DBGX(level,x...) if (_DEBUG>=level) printk(KERN_INFO x)
#else
#define ERR(x...) ;
#define DBG(x...) ;
#define DBGX(level,x...) ;
#endif

#endif /* __BDUTIL_DEBUG_H__ */
