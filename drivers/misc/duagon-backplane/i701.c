/*
 *  Duagon Ionia i701 driver
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
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

#include "ionia.h"
#include "ionia-pdata.h"
#include "ionia-slots.h"


struct ionia_i701_platform_data {
	int status;
};

static struct of_device_id i701_of_match[] = {
	{ .compatible = "duagon,ionia-i701", },
	{}
};
MODULE_DEVICE_TABLE(of, i701_of_match);

static int i701_probe(struct platform_device* pdev)
{
	pdev_info(pdev, "i701_probe\n");
#if 0
	int rc;
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
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
	dev_info(&pdev->dev, "probe succeed: phys 0x%pK mem at 0x%pK\n", (void*)res.start, pdata->registers);

	ionia_backplane_debugfs_init(pdev);
	ionia_backplane_looptest(pdev);

	probe_slots(pdev);
#endif
	return -EINVAL;
}

static int i701_remove(struct platform_device* pdev)
{
	return 0;
}

struct platform_driver i701_driver = {
	.probe = i701_probe,
	.remove = i701_remove,
	.driver = {
		.name = "ionia-i701",
		.owner = THIS_MODULE,
		.of_match_table = i701_of_match,
	},
};

static int __init i701_init(void)
{
	pr_info("loading i701 driver\n");
	return platform_driver_register(&i701_driver);
}

static void __exit i701_exit(void)
{
	pr_info("unloading module");
	return platform_driver_unregister(&i701_driver);
}

module_init(i701_init);
module_exit(i701_exit);

MODULE_AUTHOR("Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>");
MODULE_DESCRIPTION("Ionia i701 driver");
MODULE_LICENSE("GPL");
