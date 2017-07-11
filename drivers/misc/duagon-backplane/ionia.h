/*
 * Duagon Ionia backplane core Linux support
 *
 * Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
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
#include "ionia-rpc.h"

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

struct ionia_slot;

struct ionia_slot *ionia_backplane_slot_for_device(struct platform_device *pdev);
int ionia_backplane_devname_for_slot(char *buf, size_t sz, int slot);
int ionia_backplane_slot_for_devname(const char* name);
int ionia_backplane_probe_card(struct platform_device *pdev, struct ionia_slot *slot);
int ionia_backplane_probe_cards(struct platform_device *pdev);
ionia_rpc_t *ionia_backplane_rpc_for_device(struct platform_device *pdev);
int ionia_backplane_probe_slots(struct platform_device *pdev);

#endif /* __DUAGON_IONIA_H */
