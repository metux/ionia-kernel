#ifndef __LINUX_DUAGON_HDIO_H
#define __LINUX_DUAGON_HDIO_H

struct duagon_hdio_channel;

typedef int (*duagon_hdio_channel_timeout_func)(struct duagon_hdio_channel* ch);

struct duagon_hdio_channel {
	int channel_type;
	int rx_timeout;
	int tx_timeout;
	int blocking;
	int is_polling;
	duagon_hdio_channel_timeout_func op_timeout;
	duagon_hdio_channel_timeout_func op_timeout_default;
};

// move to own pdata header
struct ionia_backplane_pdata {
	int status;
	struct resource res;
	void * __iomem registers;
};

#define DUAGON_FPGA_IOMEM_ASYNC	0x18000000
#define DUAGON_FPGA_IOMEM_SYNC	0x1C000000

#endif /* __LINUX_DUAGON_HDIO_H */
