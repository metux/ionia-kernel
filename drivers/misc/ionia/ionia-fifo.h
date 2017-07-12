#ifndef __DUAGON_IONIA_FIFO_H
#define __DUAGON_IONIA_FIFO_H

#include <linux/io.h>

/* backplane serial interface registers (2G) */
#define IONIA_FIFO_REG_READWRITE	0x00	/* read/write */
#define IONIA_FIFO_REG_READCUR		0x02	/* read current w/o consumption */
#define IONIA_FIFO_REG_RX_SIZE		0x04
#define IONIA_FIFO_REG_TX_SIZE		0x06
#define IONIA_FIFO_REG_CONF		0x08
#define IONIA_FIFO_REG_LINESTAT		0x0a
#define IONIA_FIFO_REG_FIFO_SIZE	0x0e

/* bitmasks for linestat register */
#define IONIA_BITMASK_LSR_DR		0x01
#define IONIA_BITMASK_LSR_LOOPBACK	0x02
#define IONIA_BITMASK_LSR_FLUSHTX	0x04

#define IONIA_BITMASK_LSR_THRE		0x20
#define IONIA_BITMASK_LSR_DEVREADY	0x40
#define IONIA_BITMASK_LSR_HOSTREADY	0x80


struct platform_device;

struct ionia_fifo {
	int base;
	int slot;
	void * __iomem regs;
	const char *name;
	struct platform_device *pdev;
	u32 buf;
	u8 bufcnt;
};

void ionia_fifo_init(struct ionia_fifo *fifo, int base, int slot, void * __iomem registers, const char *name, struct platform_device *pdev);
void ionia_fifo_fini(struct ionia_fifo *fifo);

uint16_t ionia_fifo_getreg(struct ionia_fifo *fifo, uint16_t reg);
void ionia_fifo_setreg16(struct ionia_fifo *fifo, uint16_t reg, uint16_t val);
void ionia_fifo_setreg32(struct ionia_fifo *fifo, uint16_t reg, uint32_t val);

uint16_t ionia_fifo_size(struct ionia_fifo *fifo);
uint16_t ionia_fifo_rx_size(struct ionia_fifo *fifo);
uint16_t ionia_fifo_tx_size(struct ionia_fifo *fifo);
int ionia_fifo_connected(struct ionia_fifo *fifo);
int ionia_fifo_set_loopback(struct ionia_fifo *fifo, int enable);
uint16_t ionia_fifo_linestat(struct ionia_fifo *fifo);
int ionia_fifo_get_loopback(struct ionia_fifo *fifo);
int ionia_fifo_putc(struct ionia_fifo *fifo, char c);
int ionia_fifo_getc(struct ionia_fifo *fifo);
int ionia_fifo_num_recv(struct ionia_fifo *fifo);
void ionia_fifo_dump(struct ionia_fifo *fifo);
int ionia_fifo_recv(struct ionia_fifo *fifo, void* buf, size_t sz);

#endif /* __DUAGON_IONIA_FIFO_H */
