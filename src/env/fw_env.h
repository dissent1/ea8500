/* Modifications were made by Linksys on or before Tue Aug 12 11:57:31 PDT 2014 */
/*
 * (C) Copyright 2014
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * To build the utility with the run-time configuration
 * uncomment the next line.
 * See included "fw_env.config" sample file (TRAB board)
 * for notes on configuration.
 */
//#define CONFIG_FILE     "/etc/fw_env.config"

#undef HAVE_REDUND	/* Define for systems with 2 env sectors else undef */
//#define DEVICE1_NAME	"/dev/mtd0"	/* U-Boot binary and U-Boot env. */	
//#define DEVICE1_OFFSET	0x84000		/* 512K + 16K */
//#define ENV1_SIZE	0x4000		/* 16K environment size */
//#define DEVICE1_ESIZE	0x4000		/* 16K erasesize */
//#define DEVICE2_NAME	"/dev/mtd2"
//#define DEVICE2_OFFSET	0x0000
//#define ENV2_SIZE	0x4000
//#define DEVICE2_ESIZE	0x4000

/* ---------------------------
 * Special case for Mamba
 * ------------------------- */
#if defined(PRODUCT_MAMBA_NOSPI)
/* SPI flash is for mamba hardware 2 only, it is no longer supported, and
   removed. No SPI is the only option available. */
#define MTD_UENV_NAME      "u_env"
#define MTD_UENV_OFFSET    0x0000
#define MTD_UENV_PART_SIZE 0x40000

/* ---------------------------
 * Special case for Cobra
 * ------------------------- */
#elif defined(PRODUCT_COBRA)
#define MTD_UENV_NAME      "u_env"
#define MTD_UENV_OFFSET    0x0000
#define MTD_UENV_PART_SIZE 0x0

/* ---------------------------
 * Special case for legacy (audi/viper)
 * ------------------------- */
#elif defined(PRODUCT_LEGACY)
#define MTD_UENV_NAME      "u_env"
#define MTD_UENV_OFFSET    0x0000
#define MTD_UENV_PART_SIZE 0x0        /* use erase block size */

/* ---------------------------
 * Special case for Impala
 * ------------------------- */
#elif defined(PRODUCT_IMPALA)
#define MTD_UENV_NAME      "u_env"
#define MTD_UENV_OFFSET    0x0000
#define MTD_UENV_PART_SIZE 0x0        /* use erase block size */

/* ---------------------------
 * Special case for Wraith
 * ------------------------- */
#elif defined(PRODUCT_WRAITH)
#define MTD_UENV_NAME      "u_env"
#define MTD_UENV_OFFSET    0x0000
#define MTD_UENV_PART_SIZE 0x0        /* use erase block size */

/* ---------------------------
 * Special case for Macan
 * ------------------------- */
#elif defined(PRODUCT_MACAN)
#define MTD_UENV_NAME      "u_env"
#define MTD_UENV_OFFSET    0x0000
#define MTD_UENV_PART_SIZE 0x0        /* use erase block size */

/* ---------------------------
 * Special case for Taurus
 * ------------------------- */
#elif defined(PRODUCT_TAURUS)
#define MTD_UENV_NAME      "u_env"
#define MTD_UENV_OFFSET    0x0000
#define MTD_UENV_PART_SIZE 0x1000     /* MTK has heavily modified uboot */

/* ---------------------------
 * Special case for Focus
 * ------------------------- */
#elif defined(PRODUCT_FOCUS)
#define MTD_UENV_NAME      "Config"
#define MTD_UENV_OFFSET    0x0000
#define MTD_UENV_PART_SIZE 0x1000     /* MTK has heavily modified uboot */

/* ---------------------------
 * Special case for IP Central build
 * ------------------------- */
#elif defined(IP_CENTRAL)
#define MTD_UENV_NAME      "u_env"
#define MTD_UENV_OFFSET    0x0000
#define MTD_UENV_PART_SIZE 0x0        /* use erase block size */

/* ---------------------------
 * DEFAULT
 * ------------------------- */
#else
/* Wrong defines in this area could potentially brick a unit, since it may
   cause env to write to the boot partition and corrupt it. Default to error
   so the defines must be explicitly set. */
#error "Model/platform not specified."
#endif

extern		void  fw_printenv(int argc, char *argv[]);
extern unsigned char *fw_getenv  (unsigned char *name);
extern		int   fw_setenv  (int argc, char *argv[]);

extern unsigned	long  crc32	 (unsigned long, const unsigned char *, unsigned);
