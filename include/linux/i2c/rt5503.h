/*
 *  include/linux/i2c/rt5503.h
 *  Include header file to Richtek RT5503 SPK Amp driver
 *
 *  Copyright (C) 2015 Richtek Technology Corp.
 *  cy_huang <cy_huang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __LINUX_I2C_RT5503_H
#define __LINUX_I2C_RT5503_H
#include <linux/workqueue.h>
#include <linux/atomic.h>

#define RT5503_DEVICE_NAME		"rt5503"
#define RT5503_DRV_VER			"1.0.0_C"

/* unit: milli-second */
#define RT5503_HDET_DEBOUNCE 100
/* total 13 bits : 12~0 */
#define RT5503_MICISADC_RESOL 8192
/* unit: % */
#define RT5503_MICDET_PERCENT 7

#ifdef CONFIG_RT_REGMAP
#define RT5503_SIMULATE_DEVICE	0
#if RT5503_SIMULATE_DEVICE
int rt5503_calculate_offset(int reg);
int rt5503_calculate_total_size(void);
#endif /* #if RT5503_SIMULATE_DEVICE */
#else
#define RT5503_SIMULATE_DEVICE	0
#endif /* #ifdef CONFIG_RT_REGMAP */

enum {
	RT_HEADSET_NONE,
	RT_STEREO_HEADPHONE,
	RT_MONO_HEADSET,
	RT_STEREO_HEADSET,
};

struct rt5503_pdata {
	int hdet_gpio;
	int hdet_active_level;
};

struct rt5503_chip {
	struct i2c_client *i2c;
	struct device *dev;
	struct rt5503_pdata *pdata;
	struct rt_regmap_device *rd;
	struct input_dev *input;
#if RT5503_SIMULATE_DEVICE
	void *sim;
#endif /* #if RT5503_SIMULATE_DEVICE */
	struct delayed_work dwork;
	int irq;
	struct semaphore io_semaphore;
	struct semaphore var_semaphore;
	atomic_t power_count;
	u8 headset_stat;
	u8 headset_in:1;
};

/* RT5503_REGISTER_LIST */
#define RT5503_REG_CHIPEN	0x00
#define RT5503_REG_DIGVOL	0x01
#define RT5503_REG_CFG1		0x02
#define RT5503_REG_IMSCFG1	0x03
#define RT5503_REG_IMSSTAT	0x04
#define RT5503_REG_LZRAW	0x05
#define RT5503_REG_RZRAW	0x06
#define RT5503_REG_ASRCCFG1	0x07
#define RT5503_REG_ASRCSTAT	0x08
#define RT5503_REG_MICCFG1	0x09
#define RT5503_REG_MICTH	0x0A
#define RT5503_REG_MICDETADCOUT	0x0B
#define RT5503_REG_ADCCFG1	0x0C
#define RT5503_REG_ADCDIGVOL	0x0D
#define RT5503_REG_DACCFG1	0x0E
#define RT5503_REG_DACAUDFMT	0x0F
#define RT5503_REG_ADCCFG2	0x10
#define RT5503_REG_ADCAUDFMT	0x11
#define RT5503_REG_BLOCKEN	0x12
#define RT5503_REG_VBBHODT	0x13
#define RT5503_REG_BBMODSEL	0x14
#define RT5503_REG_VBBDESSEL	0x15
#define RT5503_REG_OCOTCFG1	0x16
#define RT5503_REG_VBBCFG1	0x17
#define RT5503_REG_NGCFG1	0x18
#define RT5503_REG_NGTH		0x19
#define RT5503_REG_ALPHA	0x1A
#define RT5503_REG_GAALPHA	0x1B
#define RT5503_REG_GRALPHA	0x1C
#define RT5503_REG_INTCFG1	0x1D
#define RT5503_REG_PREAMPCFG1	0x1E
#define RT5503_REG_ADCDACSTAT	0x1F
#define RT5503_REG_FILEN1	0x20
#define RT5503_REG_COMPCOEF1	0x21
#define RT5503_REG_COMPCOEF2	0x22
#define RT5503_REG_COMPCOEF3	0x23
#define RT5503_REG_DVOLZ1	0x24
#define RT5503_REG_DVOLZ2	0x25
#define RT5503_REG_DVOLZ3	0x26
#define RT5503_REG_DVOLZ4	0x27
#define RT5503_REG_DVOLZ5	0x28
#define RT5503_REG_DVOLZ6	0x29
#define RT5503_REG_DVOLZ7	0x2A
#define RT5503_REG_DVOLZ8	0x2B
#define RT5503_REG_DVOLZ9	0x2C
#define RT5503_REG_LZOSLO	0x30
#define RT5503_REG_LZOSME	0x31
#define RT5503_REG_LZOSHI	0x32
#define RT5503_REG_RZOSLO	0x33
#define RT5503_REG_RZOSME	0x34
#define RT5503_REG_RZOSHI	0x35
#define RT5503_REG_ZMODFLG	0x36
#define RT5503_REG_CDACCFG1	0x40
#define RT5503_REG_CDACCFG2	0x41
#define RT5503_REG_BOOSTCFG1	0x42
#define RT5503_REG_BOOSTCFG2	0x43
#define RT5503_REG_BOOSTCFG3	0x44
#define RT5503_REG_BOOSTCFG4	0x45
#define RT5503_REG_BOOSTCFG5	0x46
#define RT5503_REG_BOOSTCFG6	0x47
#define RT5503_REG_SIGGAIN	0x48
#define RT5503_REG_VBATGAIN	0x49
#define RT5503_REG_FFGAIN	0x4A
#define RT5503_REG_RLDCOEFLR	0x4B
#define RT5503_REG_DAGAINLR	0x4C
#define RT5503_REG_PLLCFG1	0x50
#define RT5503_REG_PLLCFG2	0x51
#define RT5503_REG_PLLCFG3	0x52
#define RT5503_REG_DIGVOLADJ	0x80
#define RT5503_REG_DVOLRAMP	0x81
#define RT5503_REG_DIGVOLR	0x82
#define RT5503_REG_CFG2		0x83
#define RT5503_REG_HPDRECFG1	0x84
#define RT5503_REG_HPDRECFG2	0x85
#define RT5503_REG_DIGDREDELAY	0x86
#define RT5503_REG_ANADREDELAY	0x87
#define RT5503_REG_HPLDCCNT	0x88
#define RT5503_REG_HPRDCCNT	0x89
#define RT5503_REG_VDDHSEL	0x8A
#define RT5503_REG_DACCFG2	0x8C
#define RT5503_REG_DACCFG3	0x8D
#define RT5503_REG_TESTCFG1	0x8E
#define RT5503_REG_OTPCFG1	0x8F
#define RT5503_REG_CFG3		0x90
#define RT5503_REG_DBBZL	0x91
#define RT5503_REG_DBBZR	0x92
#define RT5503_REG_VBBMR	0x94
#define RT5503_REG_ZMTSWM	0x95
#define RT5503_REG_ZMTP		0x96
#define RT5503_REG_ZCFLY	0x97
#define RT5503_REG_CPSRSEL	0x99
#define RT5503_REG_CPNONOVLSEL	0x9A
#define RT5503_REG_ADCDACCLKINV	0x9B
#define RT5503_REG_HPJACKSEL	0x9C
#define RT5503_REG_OVPCFG1	0x9D
#define RT5503_REG_PNZEROEN	0x9E
#define RT5503_REG_CPFREQSEL	0x9F
#define RT5503_REG_CPFMBYP	0xA0
#define RT5503_REG_MICDETCFG1	0xA1
#define RT5503_REG_DVDDCFG1	0xA2
#define RT5503_REG_AVDDCFG1	0xA3
#define RT5503_REG_MICBSCFG1	0xA4
#define RT5503_REG_IMSCFG2	0xA7
#define RT5503_REG_ADCIMSDG	0xA9
#define RT5503_REG_ADCIMSDO	0xAA
#define RT5503_REG_IMSCFG3	0xAB
#define RT5503_REG_MICADCCFG1	0xAC
#define RT5503_REG_MICADCCFG2	0xAD
#define RT5503_REG_TESTCFG2	0xB0
#define RT5503_REG_RAMTESTSTAT	0xB1
#define RT5503_REG_TOCKSEL	0xB2
#define RT5503_REG_CLKEN	0xB3
#define RT5503_REG_VERID	0xBE
#define RT5503_REG_I2CSID	0xBF
#define RT5503_REG_VBBCFG2	0xC0
#define RT5503_REG_VBBCFG3	0xC1
#define RT5503_REG_OCOTCFG2	0xC2
#define RT5503_REG_ZCBOOST	0xC3
#define RT5503_REG_ZCBUCK	0xC4
#define RT5503_REG_IRRAMP	0xC5
#define RT5503_REG_VBBCFG4	0xC6
#define RT5503_REG_BGCFG1	0xC7
#define RT5503_REG_BGCFG2	0xC8
#define RT5503_REG_BIASSEL1	0xC9
#define RT5503_REG_BIASSEL2	0xCA
#define RT5503_REG_BIASSEL3	0xCB
#define RT5503_REG_BIASSEL4	0xCC
#define RT5503_REG_BIASSEL5	0xCD
#define RT5503_REG_BIASSEL6	0xCE
#define RT5503_REG_BIASSEL7	0xCF
#define RT5503_REG_BIASSEL8	0xD0
#define RT5503_REG_BIASSEL9	0xD1
#define RT5503_REG_BIASSEL10	0xD3
#define RT5503_REG_BIASSEL11	0xD4
#define RT5503_REG_VBATSCFG1	0xD5
#define RT5503_REG_VBATVAL	0xD6
#define RT5503_REG_TESTCFG3	0xD7
#define RT5503_REG_WLEN		0xD8
#define RT5503_REG_HPVSSCFG1	0xE0
#define RT5503_REG_HPVSSCFG2	0xE1
#define RT5503_REG_HODSEL	0xE2
#define RT5503_REG_OCPMSSEL	0xE3
#define RT5503_REG_PADDRV	0xE4
#define RT5503_REG_IDACTEST	0xF0
#define RT5503_REG_IDACTESTSTAT	0xF1
#define RT5503_REG_VBATDATATEST	0xF2
#define RT5503_REG_EFUSECFG1	0xF3
#define RT5503_REG_EFUSECFG2	0xF4
#define RT5503_REG_EFUSECFG3	0xF5
#define RT5503_REG_TESTCFG4	0xF6
#define RT5503_REG_TESTCFG5	0xF7
#define RT5503_REG_DACPMSBTEST	0xF8
#define RT5503_REG_DACNMSBTEST	0xF9
#define RT5503_REG_DACPLSBTEST	0xFA
#define RT5503_REG_DACNLSBTEST	0xFB

/* RT5503_REG_CHIPEN:0x00 */
#define RT5503_HPENL_MASK	0x80
#define RT5503_HPENL_SHFT	7
#define RT5503_HPENR_MASK	0x40
#define RT5503_HPENR_SHFT	6
#define RT5503_SWRST_MASK	0x04
#define RT5503_HPHQMOD_MASK	0x02
#define RT5503_CHIPEN_MASK	0x01

/* RT5503_REG_CFG1: 0x02 */
#define RT5503_MUTE_MASK	0xc0
#define RT5503_PLLSEL_MASK	0x30
#define RT5503_PLLSEL_SHFT	4

/* RT5503_REG_IMSCFG1:0x03 */
#define RT5503_IMSEN_MASK	0x80

/* RT5503_REG_MICCFG1:0x09 */
#define RT5503_MICBEN_MASK	0x20
#define RT5503_MICISEN_MASK	0x10
#define RT5503_MICISDEBEN_MASK	0x02

/* RT5503_REG_ADCCFG1:0x0c */
#define RT5503_ADCEN_MASK	0x80
#define RT5503_ADCEN_SHFT	7

/* RT5503_REG_DACCFG1:0x0e */
/* RT5503_REG_ADCCFG1:0x10 */
enum {
	RT5503_BCKMODE_32FS,
	RT5503_BCKMODE_48FS,
	RT5503_BCKMODE_64FS,
};
#define RT5503_BCKMODE_MASK	0x30
#define RT5503_BCKMODE_SHFT	4
enum {
	RT5503_SRMODE_8K,
	RT5503_SRMODE_12K,
	RT5503_SRMODE_16K,
	RT5503_SRMODE_24K,
	RT5503_SRMODE_32K,
	RT5503_SRMODE_48K,
	RT5503_SRMODE_96K,
	RT5503_SRMODE_192K,
	RT5503_SRMODE_384K,
};
#define RT5503_SRMODE_MASK	0x0f
#define RT5503_SRMODE_SHFT	0

/* RT5503_REG_DACAUDFMT:0x0f */
/* RT5503_REG_ADCAUDFMT:0x11 */
enum {
	RT5503_DSPMODE_A,
	RT5503_DSPMODE_B,
};
#define RT5503_DSPMODE_SHFT	4
#define RT5503_DSPMODE_MASK	0x10
enum {
	RT5503_AUDFMT_I2S,
	RT5503_AUDFMT_LEFTJ,
	RT5503_AUDFMT_RIGHTJ,
	RT5503_AUDFMT_DSPMODE,
};
#define RT5503_AUDFMT_SHFT	2
#define RT5503_AUDFMT_MASK	0x0c
enum {
	RT5503_AUDBIT_24,
	RT5503_AUDBIT_20,
	RT5503_AUDBIT_18,
	RT5503_AUDBIT_16,
};
#define RT5503_AUDBIT_MASK	0x03
#define RT5503_AUDBIT_SHFT	0

/* RT5503_REG_BLOCKEN: 0x12 */
#define RT5503_BBEN_MASK	0x80
#define RT5503_CPEN_MASK	0x40
#define RT5503_PLLEN_MASK	0x20
#define RT5503_PLLEN_SHFT	5

/* RT5503_REG_DACCFG2: 0x8C */
#define RT5503_DACLEN_MASK	0x80
#define RT5503_DACLEN_SHFT	7
#define RT5503_DACREN_MASK	0x40
#define RT5503_DACREN_SHFT	6

struct rt_regmap_device * rt5503_regmap_register(
	struct rt_regmap_fops *regmap_ops,
	struct device *parent, void *client, void *drvdata);

#endif /* #ifndef __LINUX_I2C_RT5503_H */