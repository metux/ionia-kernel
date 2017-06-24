/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/kmemcheck.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/c2port.h>

#include <linux/platform_device.h>

#include "ionia.h"
#include "ionia-pdata.h"
#include "hdio.h"

/**
 * HACK: HACK: HACK:
 *
 * Until we've modeled GPMC parameter into DT, we need to rely on legacy
 * userland to do the gpmc init. Therefore the device cannot be used
 * before that's done - triggring device init via debugfs
 */
int ionia_backplane_looptest(struct platform_device* pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	int x=0;
	int fail=0;

	dev_info(&pdev->dev, "running loop test\n");
	for (x=0; x<0xFFFF; x+=0x7F) {
		ionia_backplane_setreg(pdata, IONIA_BACKPLANE_REG_LOOPBACK, x);
		if (ionia_backplane_getreg(pdata, IONIA_BACKPLANE_REG_LOOPBACK) != x) {
			dev_err(&pdev->dev, "loop test failed for value %X\n", x);
			fail++;
		}
	}

	if (fail) {
		pdata->status = IONIA_BACKPLANE_STATUS_DOWN;
		return -ENOENT;
	}

	dev_info(&pdev->dev, "loop test OKAY\n");
	pdata->status = IONIA_BACKPLANE_STATUS_PROBED;

	return 0;
}
