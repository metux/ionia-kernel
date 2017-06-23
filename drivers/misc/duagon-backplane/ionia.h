/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_IONIA_H
#define __DUAGON_IONIA_H

#include <linux/io.h>

#include "ionia-pdata.h"

#define IONIA_DRIVER_VERSION	"0.0.2"
#define IONIA_DRIVER_NAME	"ionia-backplane"
#define IONIA_DEVICE_NAME	"ionia-backplane"

struct platform_device;

int  ionia_backplane_looptest(struct platform_device *pdev);
void ionia_backplane_debugfs_init(struct platform_device *pdev);
void ionia_backplane_debugfs_fini(struct platform_device *pdev);

/* start of the register bank */
#define IONIA_BACKPLANE_BANK_OFFSET             0x00000800

/* registers are word-aligned - IOW: byte offset = regid * 2 */
#define IONIA_BACKPLANE_REG_BURNIN              0x04
#define IONIA_BACKPLANE_REG_LOOPBACK            0x16

static inline uint16_t ionia_backplane_getreg(struct ionia_backplane_platform_data *pdata, int reg) {
	uint16_t val = readw(pdata->registers + IONIA_BACKPLANE_BANK_OFFSET + reg*2);
	printk(KERN_INFO "ionia getreg %d -- %ld = %d\n", reg, (long)pdata->registers + IONIA_BACKPLANE_BANK_OFFSET + reg*2, val);
	return val;
}

static inline void ionia_backplane_setreg(struct ionia_backplane_platform_data *pdata, uint16_t reg, uint16_t val) {
	printk(KERN_INFO "ionia setreg %d -- %ld : %d\n", reg, (long)pdata->registers + IONIA_BACKPLANE_BANK_OFFSET + reg*2, val);
	writew(val, pdata->registers + IONIA_BACKPLANE_BANK_OFFSET + reg*2);
}

#endif /* __DUAGON_IONIA_H */
