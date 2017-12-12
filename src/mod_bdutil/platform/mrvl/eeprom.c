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
  @file eeprom.c
  @brief board driver eeprom support
*************************************************************************/

#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/string.h>

#include <linux/gpio.h>
#include <gpp/mvGpp.h>
#include <boardEnv/mvBoardEnvSpec.h>
#include <ctrlEnv/mvCtrlEnvRegs.h>
#include <twsi/mvTwsi.h>

#include "../../debug.h"
#include "eeprom.h"
#include "mv_io.h"

#define DRV_DESC	"bdutil board utility eeprom kernel module"
#define DRV_VERSION	"1.3"
#define DEVICE_NAME	"mfgeeprom"

extern u_int32_t bdutil_gboardId;

static int bdutil_eeprom_open_count = 0;
static int major = 0;

static int eeprom_open_fs(struct inode *, struct file *);
static int eeprom_release_fs(struct inode *, struct file *);
static ssize_t eeprom_read_fs(struct file *, char __user *, size_t, loff_t *);
static ssize_t eeprom_write_fs(struct file *file, const char __user * buffer, size_t count, loff_t * ppos);
static int eeprom_ioctl_fs(struct inode *inode, struct file *filp, u_int32_t cmd, unsigned long arg);
static int read_eeprom(unsigned char *data, size_t offset, size_t len);
static int eeprom_write(unsigned char *data, int offset, size_t len);

static struct file_operations eeprom_fops = {
        .owner = THIS_MODULE,
        .read = eeprom_read_fs,
	.write = eeprom_write_fs,
	.ioctl = eeprom_ioctl_fs,
        .open = eeprom_open_fs,
        .release = eeprom_release_fs 
};

static int eeprom_open_fs(struct inode *inode, struct file *filp)
{
	DBGX(1, "%s: entering, bdutil_eeprom_open_count = %d\n", __FUNCTION__, bdutil_eeprom_open_count);

	if (bdutil_eeprom_open_count >= 1) return -EFAULT;

	bdutil_eeprom_open_count = 1;		

	return 0;
}

static int eeprom_release_fs(struct inode *inode, struct file *filp)
{
	DBGX(1, "%s: entering, bdutil_eeprom_open_count = %d\n", __FUNCTION__, bdutil_eeprom_open_count);

	bdutil_eeprom_open_count = 0;		

	return 0;
}

static ssize_t eeprom_read_fs(struct file *filp, char __user *buf,
                            size_t count, loff_t *f_pos)
{
	char eeprom_data[MFG_EEPROM_SIZE];

	DBGX(1, "%s: entering, count = %d\n", __FUNCTION__, count);

	if (count != MFG_EEPROM_SIZE) return -EFAULT;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	if ((count > MFG_EEPROM_SIZE) || (count < 0x10))
		return -EFAULT;

	memset(eeprom_data, 0xff, sizeof(eeprom_data));

	if (read_eeprom(eeprom_data, 0, count) != 0) {
		return -EFAULT;
	}

#define ENDLINE_STRING	'\0'
	//strcpy(eeprom_data, TEST_STRING);
	// to make panic and then verbose: strcpy(eeprom_data[14], ENDLINE_STRING);

        if (copy_to_user(buf, &eeprom_data,  count) ) {
                ERR("%s: - bad copy\n", __FUNCTION__);
                return -EFAULT;
        }

        return count;
}

static ssize_t eeprom_write_fs(struct file *file, const char __user * buffer, size_t count, loff_t * ppos)
{
	char *dataptr = NULL;
	char eeprom_data[MFG_EEPROM_SIZE];
	int offset = 0;

	DBGX(1, "%s: entering, count = %d\n", __FUNCTION__, count);

	if (!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;

	if (count > MFG_EEPROM_SIZE)
		return -EFAULT;

	memset(eeprom_data, 0, sizeof(eeprom_data));

	if (copy_from_user(eeprom_data, buffer, count))
		return -EFAULT;

	dataptr = eeprom_data;
	offset = count;

	if (dataptr != NULL)
		/* FIXME:
		 * arch/arm/plat-feroceon/mv_hal/mvTwsi.c
		 * mvTwsi.c mvTwsiWrite() routine returns error after writing
		 * the 16th byte.
		 * On the write of byte 17 twsiAddr7BitSet() routine will write
		 * the eeprom address and write bit to twsi data reg. then wait
		 * for twsi status which returns status 0x20 - "Address + write
		 * bit trnasmitted, ackowledge not received".  Should be status
		 * 0x28-"Address + write bit transmitted, acknowledge received"
		 */	
		if (eeprom_write(dataptr, offset, 16) != 0)
			return -EFAULT;

        return count;
}


static int eeprom_ioctl_fs(struct inode *inode, struct file *filp, u_int32_t cmd, unsigned long arg)
{
	mfg_data_t user_mfg_data;
	size_t count;
	size_t offset;
	char eeprom_data[MFG_EEPROM_SIZE];
	int rc = 0;

	switch (cmd) {
	case MFG_DATA_GET:
		copy_from_user(&user_mfg_data, (mfg_data_t *)arg, sizeof(mfg_data_t));
		offset = user_mfg_data.ioctl_data;
		count = user_mfg_data.count;
		memset(eeprom_data, 0xff, sizeof(eeprom_data));
		if (read_eeprom(eeprom_data, offset, count) != 0) {
			rc = -EFAULT;
		} else {
			memcpy((char *)&user_mfg_data.eeData, (char *)eeprom_data, count);
			if (copy_to_user((void *)arg, (void *)&user_mfg_data, sizeof(mfg_data_t))) {
				ERR("%s: - bad copy\n", __FUNCTION__);
				rc = -EFAULT;
			}
			DBGX(1, "%s: MFG_DATA_GET, offset = %d, count = %d, data=%s\n", __FUNCTION__, offset, count, eeprom_data);
		}
		break;
	case MFG_DATA_SET:
		copy_from_user(&user_mfg_data, (mfg_data_t *)arg, sizeof(mfg_data_t));
		offset = user_mfg_data.ioctl_data;
		count = user_mfg_data.count;
		memset(eeprom_data, 0xff, sizeof(eeprom_data));
		memcpy((char *)eeprom_data, (char *)&user_mfg_data.eeData, count);
		if (count > 16) count = 16;
		DBGX(1, "%s: MFG_DATA_SET, offset = %d, count = %d, data=%s\n", __FUNCTION__, offset, count, eeprom_data);
		if (eeprom_write(eeprom_data, offset, count) != 0)
			rc = -EFAULT;
		break;
	default:
		break;
	}

	return rc;
}

static int i2c_read(MV_U8 chanNum, MV_U8 dev_addr, unsigned int offset, int alen, MV_U8* data, int len)
{
	MV_TWSI_SLAVE twsiSlave;
	MV_TWSI_ADDR slave;

	DBGX(1, "%s: offset=%d, len=%d\n", __FUNCTION__, offset, len);

	/* TWSI init */
        slave.type = ADDR7_BIT;
        slave.address = 0;

	mvTwsiInit(chanNum, TWSI_SPEED, mvBoardTclkGet(), &slave, 0);

	twsiSlave.slaveAddr.type = ADDR7_BIT;	
	twsiSlave.slaveAddr.address = dev_addr;	
	if (alen != 0){
		twsiSlave.validOffset = MV_TRUE;
		twsiSlave.offset = offset;
		if (alen == 2) {
			twsiSlave.moreThen256 = MV_TRUE;
		} else {
			twsiSlave.moreThen256 = MV_FALSE;
		}
	}
	return mvTwsiRead(chanNum, &twsiSlave, data, len);
}

static int i2c_write(MV_U8 chanNum, MV_U8 dev_addr, unsigned int offset, int alen, MV_U8* data, int len)
{
	MV_TWSI_SLAVE twsiSlave;
	MV_TWSI_ADDR slave;

	DBGX(1, "%s: data=%s, offset=%d, len=%d\n", __FUNCTION__, data, offset, len);

	/* TWSI init */
	slave.type = ADDR7_BIT;
	slave.address = 0;

	twsiSlave.slaveAddr.type = ADDR7_BIT;
	twsiSlave.slaveAddr.address = dev_addr;
	if (alen != 0){
		twsiSlave.validOffset = MV_TRUE;
		twsiSlave.offset = offset;
		if (alen == 2) {
			twsiSlave.moreThen256 = MV_TRUE;
		} else {
			twsiSlave.moreThen256 = MV_FALSE;
		}
	}

	mvTwsiInit(chanNum, TWSI_SPEED, mvBoardTclkGet(), &slave, 0);

	return mvTwsiWrite(chanNum, &twsiSlave, data, len);
}

#define I2C_RXTX_LEN    128     /* maximum tx/rx buffer length */
static int read_eeprom(unsigned char *data, size_t offset, size_t len)
{
	int chanNum = 1;
	unsigned end = offset + len;
	unsigned blk_off;
	int rcode = 0;

	DBGX(1, "%s: offset=%d, len=%d, end=%d\n", __FUNCTION__, offset, len, end);

	while (offset < end) {
		unsigned alen, len;
		unsigned maxlen;
                MV_U8 addr[2];

		blk_off = offset & 0xFF;        /* block offset */

		addr[0] = offset >> 8;          /* block number */
		addr[1] = blk_off;              /* block offset */
		alen    = 2;

		addr[0] |= MFG_EEPROM_I2C_DEV_ADDR; /* insert device address */

		len = end - offset;

		maxlen = 0x100 - blk_off;
		if (maxlen > I2C_RXTX_LEN)
			maxlen = I2C_RXTX_LEN;
		if (len > maxlen)
			len = maxlen;

		if (i2c_read(chanNum, addr[0], offset, alen-1, data, len) != 0)
			rcode = 1;

		data += len;
		offset += len;
	}

	return rcode;
}

static int eeprom_write(unsigned char *data, int offset, size_t len)
{
	int chanNum = 1;
	unsigned end = offset + len;
	unsigned blk_off;
	int rcode = 0;

	DBGX(1, "%s: offset=%d, len=%d, end=%d\n", __FUNCTION__, offset, len, end);

	while (offset < end) {
		unsigned alen, len;
		unsigned maxlen;
		MV_U8 addr[2];

		blk_off = offset & 0xFF;	/* block offset */

		addr[0] = offset >> 8;		/* block number */
		addr[1] = blk_off;		/* block offset */
		alen    = 2;

		addr[0] |= MFG_EEPROM_I2C_DEV_ADDR; /* insert device address */

		len = end - offset;

		maxlen = 0x100 - blk_off;
		if (maxlen > I2C_RXTX_LEN)
			maxlen = I2C_RXTX_LEN;
		if (len > maxlen)
			len = maxlen;

		if (i2c_write(chanNum, addr[0], offset, alen-1, data, len) != 0)
			rcode = 1;

		data += len;
		offset += len;
	}

	return rcode;
}

int init_eeprom(void)
{
	int status = 0;

	LOG(DRV_DESC ", version " DRV_VERSION"\n");

	major = register_chrdev(0, DEVICE_NAME, &eeprom_fops);
	DBGX(1, "%s: dev entry =  %s\n", __FUNCTION__, DEVICE_NAME);
	DBGX(1, "%s: major =  %d\n", __FUNCTION__, major);

	return status;
}

void exit_eeprom(void)
{
	unregister_chrdev(major, DEVICE_NAME);
	LOG(DRV_DESC ", unloaded " DRV_VERSION"\n");
}
