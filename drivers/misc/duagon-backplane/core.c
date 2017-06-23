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

static int backplane_probe(struct platform_device* pdev)
{
	int rc;
	const struct of_device_id *match;
	struct ionia_backplane_pdata *pdata = pdev->dev.platform_data;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	match = of_match_device(backplane_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "dt compatible mismatch\n");
		return -EINVAL;
	}

	rc = of_address_to_resource(pdev->dev.of_node, 0, &pdata->res);
	if (rc) {
		dev_err(&pdev->dev, "fetching resource address failed: %d\n", rc);
		return rc;
	}

	pdata->registers = devm_ioremap_resource(&pdev->dev, &pdata->res);
	if (!pdata->registers) {
		dev_err(&pdev->dev, "ioremap resource failed\n");
		goto failed;
	}

	pdev->dev.platform_data = pdata;

	dev_info(&pdev->dev, "probe succeed: phys %pK mem at %pK\n", pdata->res.start, pdata->registers);
	return 0;

failed:
	if (pdata->registers) iounmap(pdata->registers);
	return -EINVAL;
}

static int backplane_remove(struct platform_device* pdev)
{
	struct ionia_backplane_pdata *pdata = pdev->dev.platform_data;
	if (pdata->registers) iounmap(pdata->registers);
	release_mem_region(pdata->res.start, resource_size(&pdata->res));
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
