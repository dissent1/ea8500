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
  @file board_id.c
  @brief board driver board_id access kernel module
*************************************************************************/

#include "linux/string.h"
#include "board_id.h"

int get_boardId(void)
{
	return 0;
}

int get_board_type(char *boardid_str, int str_size)
{
	if (str_size != BOARDID_MAX_STR) return 1;
	strcpy(boardid_str, "viper");
	return 0;
}
