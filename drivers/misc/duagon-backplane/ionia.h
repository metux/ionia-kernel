#ifndef __DUAGON_IONIA_H
#define __DUAGON_IONIA_H

#define IONIA_BACKPLANE_DRIVER_VERSION	"0.1.1"
#define IONIA_BACKPLANE_DRIVER_NAME	"ionia-backplane"
#define IONIA_BACKPLANE_DEVICE_NAME	"ionia-backplane"

struct platform_device;

int ionia_backplane_looptest(struct platform_device *pdev);

void ionia_backplane_debugfs_init(struct platform_device *pdev);
void ionia_backplane_debugfs_fini(struct platform_device *pdev);

#endif /* __DUAGON_IONIA_H */
