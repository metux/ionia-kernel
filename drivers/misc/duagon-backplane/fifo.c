
#include <linux/io.h>
#include <linux/platform_device.h>

#include "ionia-fifo.h"
#include "ionia.h"

#define RSR_MULTIPLIER 4

#define fifo_info(dev, fmt, args...) \
	pdev_info(fifo->pdev, "ionia fifo [%2d@%03X] " fmt "\n", fifo->slot, fifo->base, ##args)

#define fifo_warn(dev, fmt, args...) \
	pdev_warn(fifo->pdev, "ionia fifo [%2d@%03X] " fmt "\n", fifo->slot, fifo->base, ##args)

void ionia_fifo_init(ionia_fifo_t *fifo, int base, int slot, void * __iomem regs, const char *name, struct platform_device *pdev)
{
	fifo->base = base;
	fifo->regs = regs;
	fifo->name = name;
	fifo->pdev = pdev;
	fifo->slot = slot;
}

void ionia_fifo_fini(ionia_fifo_t *fifo)
{
}

uint16_t ionia_fifo_getreg(ionia_fifo_t *fifo, uint16_t reg)
{
	return readw(fifo->regs + reg);
}

void ionia_fifo_setreg(ionia_fifo_t *fifo, uint16_t reg, uint16_t val)
{
	writew(val, fifo->regs + reg);
}

uint16_t ionia_fifo_size(ionia_fifo_t *fifo)
{
	return ionia_fifo_getreg(fifo, IONIA_FIFO_REG_FIFO_SIZE) << 7;
}

uint16_t ionia_fifo_rx_size(ionia_fifo_t *fifo)
{
	return ionia_fifo_getreg(fifo, IONIA_FIFO_REG_RX_SIZE);
}

uint16_t ionia_fifo_tx_size(ionia_fifo_t *fifo)
{
	uint16_t rv = ionia_fifo_getreg(fifo, IONIA_FIFO_REG_TX_SIZE);
	if (rv == 0xFF)
		rv = ionia_fifo_size(fifo);
	return rv;
}

int ionia_fifo_connected(ionia_fifo_t *fifo)
{
	// bits must be 01x000xx
	return ((ionia_fifo_getreg(fifo, IONIA_FIFO_REG_LINESTAT) & 0xdc) == 0x40);
}

int ionia_fifo_set_loopback(ionia_fifo_t *fifo, int enable)
{
	uint16_t rv = ionia_fifo_getreg(fifo, IONIA_FIFO_REG_LINESTAT);
	if (enable)
		rv |= IONIA_BITMASK_LSR_LOOPBACK;
	else
		rv &= ~IONIA_BITMASK_LSR_LOOPBACK;
	ionia_fifo_setreg(fifo, IONIA_FIFO_REG_LINESTAT, rv);
	rv = ionia_fifo_getreg(fifo, IONIA_FIFO_REG_LINESTAT);
	return 0;
}

uint16_t ionia_fifo_linestat(ionia_fifo_t *fifo)
{
	return ionia_fifo_getreg(fifo, IONIA_FIFO_REG_LINESTAT);
}

int ionia_fifo_get_loopback(ionia_fifo_t *fifo)
{
	return ionia_fifo_getreg(fifo, IONIA_FIFO_REG_LINESTAT) & IONIA_BITMASK_LSR_LOOPBACK;
}

int ionia_fifo_putc(ionia_fifo_t *fifo, char c)
{
	fifo_info(fifo, "putc: %X", (int)c);
	ionia_fifo_setreg(fifo, IONIA_FIFO_REG_READWRITE, c);
	ionia_backplane_waitreg();
	ionia_fifo_dump(fifo);
	return 0;
}

int ionia_fifo_getc(ionia_fifo_t *fifo)
{
	uint16_t ret = ionia_fifo_getreg(fifo, IONIA_FIFO_REG_READWRITE);
	uint16_t rxs = ionia_fifo_rx_size(fifo);
	uint16_t txs = ionia_fifo_getreg(fifo, IONIA_FIFO_REG_TX_SIZE);
	fifo_info(fifo, "receive char: %04X -- rxs=%4d txs=%4d", ret, rxs, txs);
	return (char)ret;
}

int ionia_fifo_num_recv(ionia_fifo_t *fifo)
{
	int rxsz;

	// check for data ready flag
	if ((ionia_fifo_getreg(fifo, IONIA_FIFO_REG_LINESTAT) & IONIA_BITMASK_LSR_DR)) {
		fifo_info(fifo, "no rx data ready");
		return 0;
	}

	rxsz = ionia_fifo_rx_size(fifo);

	if (rxsz == 0xFF) {
		fifo_warn(fifo, "rx buffer full");
		rxsz = ionia_fifo_size(fifo);
	}

	return rxsz * RSR_MULTIPLIER;
}

void ionia_fifo_dump(ionia_fifo_t *fifo)
{
	uint16_t lsr = ionia_fifo_linestat(fifo);

	fifo_info(fifo, "sz %d\n lsr 0x%02X %c%c%c%c%c%c %s cf 0x%02X rx  %4d tx %4d avail %4d",
		ionia_fifo_size(fifo),
		lsr,
		(lsr & IONIA_BITMASK_LSR_DR        ? 'R':'_'),
		(lsr & IONIA_BITMASK_LSR_LOOPBACK  ? 'L':'_'),
		(lsr & IONIA_BITMASK_LSR_FLUSHTX   ? 'F':'_'),
		(lsr & IONIA_BITMASK_LSR_THRE      ? 'T':'_'),
		(lsr & IONIA_BITMASK_LSR_DEVREADY  ? 'D':'_'),
		(lsr & IONIA_BITMASK_LSR_HOSTREADY ? 'H':'_'),
		(ionia_fifo_connected(fifo) ? "CON" : "dis"),
		ionia_fifo_getreg(fifo, IONIA_FIFO_REG_CONF),
		ionia_fifo_rx_size(fifo),
		ionia_fifo_tx_size(fifo),
		ionia_fifo_num_recv(fifo)
	);
}
