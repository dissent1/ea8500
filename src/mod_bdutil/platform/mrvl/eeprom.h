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
  @file eeprom.h
  @brief board eeprom support 
*************************************************************************/

#ifndef __BDUTIL_EEPROM_H__
#define __BDUTIL_EEPROM_H__

extern int init_eeprom(void);
extern void exit_eeprom(void);

#define	MFG_EEPROM_SIZE		256
#define MFG_EEPROM_I2C_DEV_ADDR	0x56	

#define MANUFACTURER_DATA_MAX_LEN	256

/* ioctl interface to eeprom */
#define MFG_DATA_GET		_IOR('e', 1, unsigned int)
#define MFG_DATA_SET		_IOW('e', 2, unsigned int)

typedef struct {
	int field;				/* field see enum choices */
	int layout;				/* specify eeprom layout */
	size_t count;				/* data size */
	size_t ioctl_data;			/* reserved for ioctl usage */
	unsigned char eeData[MANUFACTURER_DATA_MAX_LEN];/* returned data */
} mfg_data_t;


#endif /* __BDUTIL_EEPROM_H__ */
