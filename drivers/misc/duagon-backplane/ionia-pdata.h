#ifndef __LINUX_DUAGON_IONIA_PDATA_H
#define __LINUX_DUAGON_IONIA_PDATA_H

#include <linux/types.h>

struct dentry;

struct ionia_backplane_platform_data {
	uint32_t status;
	void * __iomem registers;
	struct dentry *debugfs_dentry;
};

#endif /* __LINUX_DUAGON_IONIA_PDATA_H */
