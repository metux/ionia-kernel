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
#include <linux/init.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/kmemcheck.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/c2port.h>

#define DRIVER_VERSION		"0.1.0"
#define DRIVER_NAME		"ionia-backplane"
#define DEVICE_NAME		"ionia-backplane"

#include "ionia.h"
#include "ionia-pdata.h"
#include "hdio.h"

static struct of_device_id backplane_of_match[] = {
	{ .compatible = "duagon,i101-backplane", },
	{}
};
MODULE_DEVICE_TABLE(of, backplane_of_match);

struct ionia_backplane_pdata bp_pdata = {
	.status = 0,
};

struct platform_device ionia_backplane_device = {
	.name = DEVICE_NAME,
	.dev = {
		.platform_data = &bp_pdata,
	},
	.id = -1,
};

static int cmd_write_op(void *data, u64 value)
{
	struct platform_device *pdev = data;

	printk(KERN_INFO "ionia_core cmd: %llu\n", value);

	if (value == 666)
		ionia_backplane_looptest(pdev);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cmd_fops, NULL, cmd_write_op, "%llu\n");

static void init_debugfs(struct platform_device* pdev)
{
	struct ionia_backplane_pdata *pdata = pdev->dev.platform_data;

	struct dentry* dbg_dir = debugfs_create_dir(DRIVER_NAME, NULL);
	debugfs_create_file("cmd", 0222, dbg_dir, pdev, &cmd_fops);
	debugfs_create_u32("state", 0444, dbg_dir, &(pdata->status));
	pdata->debugfs_dentry = dbg_dir;
}

static int backplane_probe(struct platform_device* pdev)
{
	int rc;
	struct ionia_backplane_pdata *pdata = pdev->dev.platform_data;
	struct resource res;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	rc = of_address_to_resource(pdev->dev.of_node, 0, &res);
	if (rc) {
		dev_err(&pdev->dev, "fetching resource address failed: %d\n", rc);
		return rc;
	}

	pdata->registers = devm_ioremap_resource(&pdev->dev, &res);
	if (!pdata->registers) {
		dev_err(&pdev->dev, "ioremap resource failed\n");
		return -EINVAL;
	}

	pdev->dev.platform_data = pdata;
	dev_info(&pdev->dev, "probe succeed: phys %pK mem at %pK\n", (void*)res.start, pdata->registers);

	init_debugfs(pdev);

	return 0;
}

static int backplane_remove(struct platform_device* pdev)
{
	struct ionia_backplane_pdata *pdata = pdev->dev.platform_data;
	debugfs_remove_recursive(pdata->debugfs_dentry);
	return 0;
}

struct platform_driver ionia_backplane_driver = {
	.probe = backplane_probe,
	.remove = backplane_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = backplane_of_match,
	},
};

static int __init backplane_init(void)
{
	pr_info("loading module v" DRIVER_VERSION "\n");
	return platform_driver_register(&ionia_backplane_driver);
}

static void __exit backplane_exit(void)
{
	pr_info("unloading module");
	return platform_driver_unregister(&ionia_backplane_driver);
}

module_init(backplane_init);
module_exit(backplane_exit);

MODULE_AUTHOR("Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>");
MODULE_DESCRIPTION("Ionia backplane support v" DRIVER_VERSION);
MODULE_LICENSE("GPL");
