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

struct ionia_port;
struct device_node;

struct ionia_slot
{
	int base;
	int sz;
	const char* name;
	struct ionia_port *port;
	struct device_node *devnode;
	struct ionia_fifo fifo;
};

#endif /* __DUAGON_IONIA_SLOTS_H */
