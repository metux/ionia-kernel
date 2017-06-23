/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_IONIA_PLATFORM_DATA_H
#define __DUAGON_IONIA_PLATFORM_DATA_H

#include <linux/types.h>
#include <linux/ioport.h>

#define IONIA_BACKPLANE_STATUS_DOWN	0
#define IONIA_BACKPLANE_STATUS_PROBED	1	// loop test went through
#define IONIA_BACKPLANE_STATUS_FAILED	2

struct dentry;

struct ionia_backplane_platform_data {
	uint32_t status;
	void * __iomem registers;
	struct dentry *debugfs_dentry;
};

#endif /* __LINUX_DUAGON_IONIA_PDATA_H */
