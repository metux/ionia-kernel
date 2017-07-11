/*
 * Duagon Ionia backplane core Linux support
 *
 * Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "ionia.h"
#include "ionia-pdata.h"
#include "ionia-serial.h"
#include "ionia-fifo.h"
#include "ionia-rpc.h"
#include "ionia-proto-log.h"
#include "ionia-slots.h"

#define IONIA_DEBUG_CMD_LOOPTEST	666
#define IONIA_DEBUG_CMD_UART_DUMP	1
#define IONIA_DEBUG_CMD_UART_LOOPBACK	2

static void set_loopback(struct platform_device *pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	int x;
	dev_info(&pdev->dev, "setting devices to loopback\n");

	for (x=0; x<pdata->nr_slots; x++) {
		ionia_fifo_set_loopback(&(pdata->slots[x].fifo), 1);
	}
	ionia_serial_dumpall(pdev);
}

static int cmd_write_op(void *data, u64 value)
{
	struct platform_device *pdev = data;

	dev_info(&pdev->dev, "debug command: %lld\n", value);
	switch (value) {
		case IONIA_DEBUG_CMD_LOOPTEST:
			return ionia_backplane_start(pdev);
		break;
		case IONIA_DEBUG_CMD_UART_DUMP:
			ionia_serial_dumpall(pdev);
		break;
		case IONIA_DEBUG_CMD_UART_LOOPBACK:
			set_loopback(pdev);
		break;
		default:
			dev_info(&pdev->dev, "unhandled debug command: %lld\n", value);
		break;
	}

	return 0;
}

static int log_init_write_op(void *data, u64 value)
{
	struct platform_device *pdev = data;
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	ionia_rpc_t *rpc;

	dev_info(&pdev->dev, "test: %lld\n", value);

	if (value > pdata->nr_slots) {
		dev_warn(&pdev->dev, "slot number too large\n");
		return -EINVAL;
	}

	rpc = ionia_rpc_get_fifo(&(pdata->slots[value].fifo));

	ionia_log_init(rpc);
	ionia_log_channel_enable(rpc, 0x0f, 0x0f);

	ionia_rpc_put(rpc);

	return 0;
}

static int io_init_write_op(void *data, u64 value)
{
	struct platform_device *pdev = data;
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	ionia_rpc_t *rpc;

	dev_info(&pdev->dev, "test: %lld\n", value);

	if (value > pdata->nr_slots) {
		dev_warn(&pdev->dev, "slot number too large\n");
		return -EINVAL;
	}

	rpc = ionia_rpc_get_fifo(&(pdata->slots[value].fifo));

	ionia_log_init(rpc);
	ionia_log_channel_enable(rpc, 0x0f, 0x0f);

	ionia_rpc_put(rpc);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cmd_fops, NULL, cmd_write_op, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(log_init_fops, NULL, log_init_write_op, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(io_init_fops, NULL, io_init_write_op, "%llu\n");

void ionia_backplane_debugfs_init(struct platform_device* pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;

	struct dentry* dbg_dir = debugfs_create_dir(IONIA_BACKPLANE_DRIVER_NAME, NULL);
	debugfs_create_file("cmd", 0222, dbg_dir, pdev, &cmd_fops);
	debugfs_create_file("log_init", 0222, dbg_dir, pdev, &log_init_fops);
	debugfs_create_file("io_init", 0222, dbg_dir, pdev, &io_init_fops);
	debugfs_create_u32("state", 0444, dbg_dir, &(pdata->status));
	pdata->debugfs_dentry = dbg_dir;
}

void ionia_backplane_debugfs_fini(struct platform_device* pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	debugfs_remove_recursive(pdata->debugfs_dentry);
}
