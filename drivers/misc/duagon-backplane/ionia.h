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

#define I101_HOST_BASE_CHANNEL_MAX	2
#define I101_HOST_BASE_CHANNEL_NONE	0x0

/* backplane serial interface registers (2G) */
#define IONIA_BACKPLANE_SERIAL_REG_READWRITE	0x00	/* read/write */
#define IONIA_BACKPLANE_SERIAL_REG_READCUR	0x02	/* read current w/o consumption */
#define IONIA_BACKPLANE_SERIAL_REG_RX_SIZE	0x04
#define IONIA_BACKPLANE_SERIAL_REG_TX_SIZE	0x06
#define IONIA_BACKPLANE_SERIAL_REG_CONF		0x08
#define IONIA_BACKPLANE_SERIAL_REG_LINESTAT	0x0a
#define IONIA_BACKPLANE_SERIAL_REG_FIFO_SIZE	0x0e

#define IONIA_BACKPLANE_STATUS_DOWN     0
#define IONIA_BACKPLANE_STATUS_PROBED   1       // loop test went through

#define IONIA_BACKPLANE_IOMEM_ASYNC             0x18000000
#define IONIA_BACKPLANE_IOMEM_SYNC              0x1C000000

/* start of the register bank */
#define IONIA_BACKPLANE_BANK_OFFSET             0x00000800

#define IONIA_BACKPLANE_REG_BASE                0x00000400

/* registers are word-aligned - IOW: byte offset = regid * 2 */
#define IONIA_BACKPLANE_REG_LOOPBACK            IONIA_BACKPLANE_REG_BASE + 0x16


struct platform_device;
struct ionia_port;
struct uart_port;

struct i101_card
{
	/* register base IDs (not byte offsets) of the channels */
	int base;
	const char* name;
	struct uart_port *port;
};

extern struct i101_card i101_cards[];
const extern int i101_cards_max;

int  ionia_backplane_looptest(struct platform_device *pdev);
void ionia_backplane_debugfs_init(struct platform_device *pdev);
void ionia_backplane_debugfs_fini(struct platform_device *pdev);

// artificial wait between write and read cycles
static inline void ionia_backplane_waitreg(void) {
	udelay(1); //10
}

#endif /* __DUAGON_IONIA_H */
