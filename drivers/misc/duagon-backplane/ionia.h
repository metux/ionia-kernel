/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_IONIA_H
#define __DUAGON_IONIA_H

#include <linux/io.h>
#include <linux/delay.h>

#include "ionia-pdata.h"

#define IONIA_BACKPLANE_DRIVER_VERSION	"0.1.2"
#define IONIA_BACKPLANE_DRIVER_NAME	"ionia-backplane"
#define IONIA_BACKPLANE_DEVICE_NAME	"ionia-backplane"

#define IONIA_BACKPLANE_STATUS_DOWN     0
#define IONIA_BACKPLANE_STATUS_PROBED   1       // loop test went through

/* start of the register bank */
#define IONIA_BACKPLANE_BANK_OFFSET             0x00000800

#define IONIA_BACKPLANE_REG_BASE                0x00000400

/* registers are word-aligned - IOW: byte offset = regid * 2 */
#define IONIA_BACKPLANE_REG_LOOPBACK            IONIA_BACKPLANE_REG_BASE + 0x16


struct platform_device;

int  ionia_backplane_looptest(struct platform_device *pdev);
void ionia_backplane_debugfs_init(struct platform_device *pdev);
void ionia_backplane_debugfs_fini(struct platform_device *pdev);
int  ionia_backplane_start(struct platform_device *pdev);

// artificial wait between write and read cycles
static inline void ionia_backplane_waitreg(void) {
	udelay(1); //10
}

#endif /* __DUAGON_IONIA_H */
