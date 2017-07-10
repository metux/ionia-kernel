/*
 *  Duagon Ionia backplane core Linux support
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
#include "ionia-serial.h"


static struct of_device_id backplane_of_match[] = {
	{ .compatible = "duagon,ionia-backplane", },
	{}
};
MODULE_DEVICE_TABLE(of, backplane_of_match);

static int probe_slots(struct platform_device *pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	struct device_node *backplane_node = pdev->dev.of_node;
	struct device_node *slots_node;
	struct device_node *pp;
	int nr_slots = 0;
	int cnt_slots = 0;

	if (pdata->nr_slots) {
		pdev_info(pdev, "slots already probed\n");
		return 0;
	}

	if (!(backplane_node = pdev->dev.of_node)) {
		pdev_err(pdev, "failed to get of node\n");
		return -ENOENT;
	}

	if (!(slots_node = of_get_child_by_name(backplane_node, "slots"))) {
		pdev_err(pdev, "no slots node\n");
		return -ENOENT;
	}

	for_each_child_of_node(slots_node, pp) {
		nr_slots++;
	}

	if (!nr_slots) {
		pdev_err(pdev, "no slots");
		return -ENOENT;
	}

	if (!(pdata->slots = devm_kzalloc(&pdev->dev, sizeof(struct ionia_slot)*nr_slots, GFP_KERNEL))) {
		pdev_err(pdev, "kzalloc() failed\n");
		return -ENOMEM;
	}

	cnt_slots = 0;
	for_each_child_of_node(slots_node, pp) {
		int a_cells = of_n_addr_cells(pp);
		int s_cells = of_n_addr_cells(pp);
		int len;
		const __be32 *reg = of_get_property(pp, "reg", &len);
		struct ionia_slot *slot = &(pdata->slots[cnt_slots]);

		slot->name = of_get_property(pp, "label", NULL);
		slot->devnode = pp;

		if (!reg) {
			pdev_warn(pdev, "   missing reg property in slot %d\n", cnt_slots);
			nr_slots--;
			continue;
		}

		if (len / 4 != a_cells + s_cells) {
			pdev_warn(pdev, "   reg property size mismatch in slot %d\n", cnt_slots);
			nr_slots--;
			continue;
		}

		slot->base = of_read_number(reg, a_cells);
		slot->sz = of_read_number(reg + a_cells, s_cells);

		ionia_fifo_init(&slot->fifo,
				slot->base,
				cnt_slots,
				pdata->registers + slot->base,
				slot->name,
				pdev);

		pdev_info(pdev, "slot %2d at 0x%04X:%02X: %s\n",
			 cnt_slots,
			 slot->base,
			 slot->sz,
			 slot->name);

		cnt_slots++;
	}

	pdata->nr_slots = nr_slots;

	return 0;
}

static int backplane_probe(struct platform_device* pdev)
{
	int rc;
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	struct resource res;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pdev_err(pdev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	rc = of_address_to_resource(pdev->dev.of_node, 0, &res);
	if (rc) {
		pdev_err(pdev, "fetching resource address failed: %d\n", rc);
		return rc;
	}

	pdata->registers = devm_ioremap_resource(&pdev->dev, &res);
	if (!pdata->registers) {
		pdev_err(pdev, "ioremap resource failed\n");
		return -EINVAL;
	}

	pdev->dev.platform_data = pdata;
	pdev_info(pdev, "probe succeed: phys 0x%pK mem at 0x%pK\n", (void*)res.start, pdata->registers);

	ionia_backplane_debugfs_init(pdev);
	ionia_backplane_start(pdev);

	return 0;
}

int ionia_backplane_start(struct platform_device *pdev)
{
	probe_slots(pdev);
	ionia_backplane_looptest(pdev);
	ionia_serial_init(pdev);
	return 0;
}

static int backplane_remove(struct platform_device* pdev)
{
	ionia_backplane_debugfs_fini(pdev);
	return 0;
}

struct platform_driver ionia_backplane_driver = {
	.probe = backplane_probe,
	.remove = backplane_remove,
	.driver = {
		.name = IONIA_BACKPLANE_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = backplane_of_match,
	},
};

static int __init backplane_init(void)
{
	pr_info("loading module v" IONIA_BACKPLANE_DRIVER_VERSION "\n");
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
MODULE_DESCRIPTION("Ionia backplane support v" IONIA_BACKPLANE_DRIVER_VERSION);
MODULE_LICENSE("GPL");
