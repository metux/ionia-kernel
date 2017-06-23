#ifndef __LINUX_DUAGON_HDIO_H
#define __LINUX_DUAGON_HDIO_H

#include <linux/io.h>

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

#define IONIA_BACKPLANE_STATUS_DOWN	0
#define IONIA_BACKPLANE_STATUS_PROBED	1	// loop test went through

#define IONIA_BACKPLANE_IOMEM_ASYNC		0x18000000
#define IONIA_BACKPLANE_IOMEM_SYNC		0x1C000000

/* start of the register bank */
#define IONIA_BACKPLANE_BANK_OFFSET		0x00000800

/* registers are word-aligned - IOW: byte offset = regid * 2 */
#define IONIA_BACKPLANE_REG_LOOPBACK		0x16

static inline uint16_t ionia_backplane_getreg(struct ionia_backplane_pdata *pdata, int reg) {
	printk(KERN_INFO "ionia_backplane_getreg() addr=%pK\n", pdata->registers + IONIA_BACKPLANE_BANK_OFFSET + reg*2);
	return readw(pdata->registers + IONIA_BACKPLANE_BANK_OFFSET + reg*2);
}

static inline void ionia_backplane_setreg(struct ionia_backplane_pdata *pdata, uint16_t reg, uint16_t val) {
	writew(val, pdata->registers + IONIA_BACKPLANE_BANK_OFFSET + reg*2);
}

#endif /* __LINUX_DUAGON_HDIO_H */
