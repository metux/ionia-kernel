/*
 * Duagon Ionia backplane core Linux support
 *
 * Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/ctype.h>

#include <linux/platform_device.h>

#include "ionia.h"
#include "ionia-pdata.h"

static inline uint16_t ionia_backplane_getreg(struct platform_device *pdev, int reg) {
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	return readw(pdata->registers + reg*2);
}

static inline void ionia_backplane_setreg(struct platform_device *pdev, uint16_t reg, uint16_t val) {
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	writew(val, pdata->registers + reg*2);
}

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

	pdev_info(pdev, "running loop test\n");
	for (x=0; x<0xFFFF; x+=0x7F) {
		ionia_backplane_setreg(pdev, IONIA_BACKPLANE_REG_LOOPBACK, x);
		ionia_backplane_waitreg();
		if (ionia_backplane_getreg(pdev, IONIA_BACKPLANE_REG_LOOPBACK) != x) {
			pdev_err(pdev, "loop test failed for value %X\n", x);
			fail++;
		}
	}

	if (fail) {
		pdata->status = IONIA_BACKPLANE_STATUS_DOWN;
		return -ENOENT;
	}

	pdev_info(pdev, "loop test OKAY\n");
	pdata->status = IONIA_BACKPLANE_STATUS_PROBED;

	/** ping the cards **/
	
	return 0;
}
