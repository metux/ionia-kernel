/*
 *  Duagon Ionia i202 driver
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "ionia.h"
#include "ionia-rpc.h"


struct ionia_i202_platform_data {
	ionia_rpc_t *rpc;
};

static struct of_device_id i202_of_match[] = {
	{ .compatible = "duagon,ionia-i202", },
	{}
};
MODULE_DEVICE_TABLE(of, i202_of_match);

static int i202_probe(struct platform_device* pdev)
{
	struct ionia_i202_platform_data *pdata;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pdata->rpc = ionia_backplane_rpc_for_device(pdev);
	if (!(pdata->rpc)) {
		pdev_err(pdev, "failed to retrieve rpc\n");
		kfree(pdata);
		return -ENOENT;
	}

	pdev->dev.platform_data = pdata;

	pdev_info(pdev, "i202 device initialized\n");

	return 0;
}

static int i202_remove(struct platform_device* pdev)
{
	return 0;
}

struct platform_driver i202_driver = {
	.probe = i202_probe,
	.remove = i202_remove,
	.driver = {
		.name = "ionia-i202",
		.owner = THIS_MODULE,
		.of_match_table = i202_of_match,
	},
};

static int __init i202_init(void)
{
	return platform_driver_register(&i202_driver);
}

static void __exit i202_exit(void)
{
	return platform_driver_unregister(&i202_driver);
}

module_init(i202_init);
module_exit(i202_exit);

MODULE_AUTHOR("Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>");
MODULE_DESCRIPTION("Ionia i202 driver");
MODULE_LICENSE("GPLv3");
