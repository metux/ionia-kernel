/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_IONIA_SLOTS_H
#define __DUAGON_IONIA_SLOTS_H

#include "ionia-fifo.h"

struct platform_device;
struct ionia_port;

struct ionia_slot
{
	int base;
	int sz;
	const char* name;
	struct ionia_port *port;
	ionia_fifo_t fifo;
};

extern struct ionia_slot ionia_slots[];
const extern int ionia_slots_max;

void ionia_init_slots(struct platform_device *pdev);

#endif /* __DUAGON_IONIA_SLOTS_H */
