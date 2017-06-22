/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#ifndef __LINUX_DUAGON_IONIA_PDATA_H
#define __LINUX_DUAGON_IONIA_PDATA_H

#include <linux/types.h>

struct dentry;

struct ionia_backplane_platform_data {
	uint32_t status;
	void * __iomem registers;
	struct dentry *debugfs_dentry;
};

#define IONA_PDATA(pdev)	((struct ionia_backplane_platform_data*)(pdev->dev.platform_data))

#endif /* __LINUX_DUAGON_IONIA_PDATA_H */
