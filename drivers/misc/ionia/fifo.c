
#include <linux/io.h>
#include <linux/platform_device.h>

#include "ionia-fifo.h"
#include "ionia.h"

#define RSR_MULTIPLIER		4
#define RECV_TIMEOUT_JIFFIES	(3*HZ)

#define fifo_info(dev, fmt, args...) \
	pr_info("ionia fifo [%2d@%03X] " fmt "\n", fifo->slot, fifo->base, ##args)

#define fifo_warn(dev, fmt, args...) \
	pr_warn("ionia fifo [%2d@%03X] " fmt "\n", fifo->slot, fifo->base, ##args)

#define FIFO_SLEEP_MIN	100000
#define FIFO_SLEEP_MAX	200000

void ionia_fifo_init(struct ionia_fifo *fifo, int base, int slot, void * __iomem regs, const char *name, struct platform_device *pdev)
{
	fifo->base = base;
	fifo->regs = regs;
	fifo->name = name;
	fifo->pdev = pdev;
	fifo->slot = slot;
}

void ionia_fifo_fini(struct ionia_fifo *fifo)
{
}

uint16_t ionia_fifo_getreg16(struct ionia_fifo *fifo, uint16_t reg)
{
	return readw(fifo->regs + reg);
}

uint32_t ionia_fifo_getreg32(struct ionia_fifo *fifo, uint16_t reg)
{
	return readl(fifo->regs + reg);
}

void ionia_fifo_setreg16(struct ionia_fifo *fifo, uint16_t reg, uint16_t val)
{
	writew(val, fifo->regs + reg);
}

void ionia_fifo_setreg32(struct ionia_fifo *fifo, uint16_t reg, uint32_t val)
{
	writel(val, fifo->regs + reg);
}

uint16_t ionia_fifo_size(struct ionia_fifo *fifo)
{
	return ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_FIFO_SIZE) << 7;
}

uint16_t ionia_fifo_rx_size(struct ionia_fifo *fifo)
{
	return ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_RX_SIZE);
}

uint16_t ionia_fifo_tx_size(struct ionia_fifo *fifo)
{
	uint16_t rv = ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_TX_SIZE);
	if (rv == 0xFF)
		rv = ionia_fifo_size(fifo);
	return rv;
}

int ionia_fifo_connected(struct ionia_fifo *fifo)
{
	// bits must be 01x000xx
	return ((ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_LINESTAT) & 0xdc) == 0x40);
}

int ionia_fifo_set_loopback(struct ionia_fifo *fifo, int enable)
{
	uint16_t rv = ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_LINESTAT);
	if (enable)
		rv |= IONIA_BITMASK_LSR_LOOPBACK;
	else
		rv &= ~IONIA_BITMASK_LSR_LOOPBACK;
	ionia_fifo_setreg16(fifo, IONIA_FIFO_REG_LINESTAT, rv);
	rv = ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_LINESTAT);
	return 0;
}

uint16_t ionia_fifo_linestat(struct ionia_fifo *fifo)
{
	return ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_LINESTAT);
}

int ionia_fifo_get_loopback(struct ionia_fifo *fifo)
{
	return ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_LINESTAT) & IONIA_BITMASK_LSR_LOOPBACK;
}

int ionia_fifo_putc(struct ionia_fifo *fifo, char c)
{
	fifo->buf = ((fifo->buf << 8) | c);

	if ((++fifo->bufcnt) < 4)
		return 0;

	fifo_info(fifo, "xmit: %08x", fifo->buf);
	ionia_fifo_setreg32(fifo, IONIA_FIFO_REG_READWRITE, fifo->buf);
	fifo->bufcnt = 0;
	fifo->buf = 0;

	ionia_backplane_waitreg();

	return 0;
}

int ionia_fifo_getc(struct ionia_fifo *fifo)
{
	uint16_t ret = ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_READWRITE);
	uint16_t rxs = ionia_fifo_rx_size(fifo);
	uint16_t txs = ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_TX_SIZE);
	fifo_info(fifo, "receive char: %04X -- rxs=%4d txs=%4d", ret, rxs, txs);
	return (char)ret;
}

int ionia_fifo_num_recv(struct ionia_fifo *fifo)
{
	int rxsz;

	// check for data ready flag
	if ((ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_LINESTAT) & IONIA_BITMASK_LSR_DR)) {
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

void ionia_fifo_dump(struct ionia_fifo *fifo)
{
	uint16_t lsr = ionia_fifo_linestat(fifo);

	fifo_info(fifo, "sz %4d %c%c%c%c%c%c %s cf 0x%02X rx %4d tx %4d av %4d",
		ionia_fifo_size(fifo),
		(lsr & IONIA_BITMASK_LSR_DR        ? 'R':'_'),
		(lsr & IONIA_BITMASK_LSR_LOOPBACK  ? 'L':'_'),
		(lsr & IONIA_BITMASK_LSR_FLUSHTX   ? 'F':'_'),
		(lsr & IONIA_BITMASK_LSR_THRE      ? 'T':'_'),
		(lsr & IONIA_BITMASK_LSR_DEVREADY  ? 'D':'_'),
		(lsr & IONIA_BITMASK_LSR_HOSTREADY ? 'H':'_'),
		(ionia_fifo_connected(fifo) ? "C" : " "),
		ionia_fifo_getreg16(fifo, IONIA_FIFO_REG_CONF),
		ionia_fifo_rx_size(fifo),
		ionia_fifo_tx_size(fifo),
		ionia_fifo_num_recv(fifo)
	);
}

int ionia_fifo_recv(struct ionia_fifo *fifo, void* buf, size_t sz)
{
	unsigned long to = jiffies + RECV_TIMEOUT_JIFFIES;

	if ((sz<4) || (sz & 3)) {
		fifo_warn(fifo, "ionia_fifo_recv() invalid size %d", sz);
		return -EINVAL;
	}

	while (time_before(jiffies, to)) {
		uint16_t rx_size = ionia_fifo_rx_size(fifo);
		if (rx_size == 0) {
			usleep_range(FIFO_SLEEP_MIN, FIFO_SLEEP_MAX);
		} else {
			if (rx_size > sz)
				rx_size = sz;

			while (rx_size) {
				((uint32_t*)buf)[0] = ionia_fifo_getreg32(fifo, IONIA_FIFO_REG_READWRITE);
				fifo_info(fifo, "recv: %08x", ((uint32_t*)buf)[0]);
				buf+=4;
				rx_size-=4;
				sz-=4;
			}
		}
		if (!sz)
			return 0;
	}

	fifo_warn(fifo, "ionia_fifo_recv() timed out");
	return -ETIMEDOUT;
}
