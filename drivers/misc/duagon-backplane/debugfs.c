/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/errno.h>

#include "ionia.h"
#include "ionia-pdata.h"

#define IONIA_BACKPLANE_DBGCMD_LOOPTEST		666

static int cmd_write_op(void *data, u64 value)
{
	struct platform_device *pdev = data;

	printk(KERN_INFO "ionia_core cmd: %llu\n", value);

	switch (value) {
		case IONIA_BACKPLANE_DBGCMD_LOOPTEST:
			ionia_backplane_looptest(pdev);
		break;
		default:
			dev_info(&pdev->dev, "unhandled debug cmd: %llu\n", value);
		break;
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cmd_fops, NULL, cmd_write_op, "%llu\n");

void ionia_backplane_debugfs_init(struct platform_device* pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;

	struct dentry* dbg_dir = debugfs_create_dir(IONIA_BACKPLANE_DRIVER_NAME, NULL);
	debugfs_create_file("cmd", 0222, dbg_dir, pdev, &cmd_fops);
	debugfs_create_u32("state", 0444, dbg_dir, &(pdata->status));
	pdata->debugfs_dentry = dbg_dir;
}

void ionia_backplane_debugfs_fini(struct platform_device* pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	debugfs_remove_recursive(pdata->debugfs_dentry);
}
