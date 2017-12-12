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
  @file buzzer.h
  @brief board buzzer support
*************************************************************************/

#ifndef __BDUTIL_BUZZER_H__
#define __BDUTIL_BUZZER_H__

extern int init_buzzer(void);
extern void exit_buzzer(void);

#define BDUTIL_BUZZER_PROC_DEVICE_NAME	"buzzer"

/* per board buzzer struct */
typedef struct {
	const char	*label;	
	const char	*usage_str;
	int		mv_boardId;
	int		(*set)(void *);
	int		(*get)(void *);
	int		(*request)(void *);
	int		(*init)(void *);
} buzzer_t;

#endif /* __BDUTIL_BUZZER_H__ */
