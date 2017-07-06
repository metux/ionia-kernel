/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_IONIA_SERIAL_H
#define __DUAGON_IONIA_SERIAL_H

#include <linux/serial_core.h>

#include "ionia-fifo.h"

struct task_struct;
struct platform_device;
struct uart_port;

struct ionia_port {
	struct uart_port port;
	int card;
	struct platform_device *bp_pdev;
	struct task_struct *kthread;
	int kthread_running;
	const char* name;
	ionia_fifo_t fifo;
};

int  ionia_serial_init(struct platform_device *pdev);
void ionia_serial_dumpall(struct platform_device *pdev);

#endif /* __DUAGON_IONIA_SERIAL_H */
