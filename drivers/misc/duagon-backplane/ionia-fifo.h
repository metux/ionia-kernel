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

typedef struct {
	int base;
	int slot;
	void * __iomem regs;
	const char *name;
	struct platform_device *pdev;
} ionia_fifo_t;

void ionia_fifo_init(ionia_fifo_t *fifo, int base, int slot, void * __iomem registers, const char *name, struct platform_device *pdev);
void ionia_fifo_fini(ionia_fifo_t *fifo);

uint16_t ionia_fifo_getreg(ionia_fifo_t *fifo, uint16_t reg);
void ionia_fifo_setreg(ionia_fifo_t *fifo, uint16_t reg, uint16_t val);

uint16_t ionia_fifo_size(ionia_fifo_t *fifo);
uint16_t ionia_fifo_rx_size(ionia_fifo_t *fifo);
uint16_t ionia_fifo_tx_size(ionia_fifo_t *fifo);
int ionia_fifo_connected(ionia_fifo_t *fifo);
int ionia_fifo_set_loopback(ionia_fifo_t *fifo, int enable);
uint16_t ionia_fifo_linestat(ionia_fifo_t *fifo);
int ionia_fifo_get_loopback(ionia_fifo_t *fifo);
int ionia_fifo_putc(ionia_fifo_t *fifo, char c);
int ionia_fifo_getc(ionia_fifo_t *fifo);
int ionia_fifo_num_recv(ionia_fifo_t *fifo);
void ionia_fifo_dump(ionia_fifo_t *fifo);

#endif /* __DUAGON_IONIA_FIFO_H */
