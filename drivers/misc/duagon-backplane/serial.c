
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/tty_flip.h>
#include <linux/string.h>

#include "ionia.h"

#define NR_UART			20
#define IONIA_SERIAL_MAJOR	TTY_MAJOR
#define IONIA_SERIAL_MINOR	196

#define UART_POLL_MIN		100000
#define UART_POLL_MAX		200000

#define BITMASK_LSR_DR		0x01
#define BITMASK_LSR_LOOPBACK	0x02
#define BITMASK_LSR_FLUSHTX	0x04

#define BITMASK_LSR_THRE	0x20
#define BITMASK_LSR_DEVREADY	0x40
#define BITMASK_LSR_HOSTREADY	0x80

#define RSR_MULTIPLIER		4

#define HACK_LOOPBACK

struct ionia_port {
	struct uart_port port;
	int card;
	struct platform_device *bp_pdev;
	struct task_struct *kthread;
	int kthread_running;
	const char* name;
};

uint16_t ionia_uart_getreg(struct uart_port *port, uint16_t reg)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	struct ionia_backplane_platform_data *pdata = pp->bp_pdev->dev.platform_data;
	return readw(pdata->registers + i101_cards[pp->card].base + reg);
}

void ionia_uart_setreg(struct uart_port *port, uint16_t reg, uint16_t val)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	struct ionia_backplane_platform_data *pdata = pp->bp_pdev->dev.platform_data;
	writew(val, pdata->registers + i101_cards[pp->card].base + reg);
}

uint16_t ionia_uart_fifo_size(struct uart_port *port)
{
	return ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_FIFO_SIZE) << 7;
}

uint16_t ionia_uart_rx_size(struct uart_port *port)
{
	return ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_RX_SIZE);
}

uint16_t ionia_uart_tx_size(struct uart_port *port)
{
	uint16_t rv = ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_TX_SIZE);
	if (rv == 0xFF)
		rv = ionia_uart_fifo_size(port);
	return rv;
}

int ionia_uart_set_loopback(struct uart_port *port, int enable)
{
	uint16_t rv = ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_LINESTAT);
	if (enable)
		rv |= BITMASK_LSR_LOOPBACK;
	else
		rv &= ~BITMASK_LSR_LOOPBACK;
	ionia_uart_setreg(port, IONIA_BACKPLANE_SERIAL_REG_LINESTAT, rv);
	rv = ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_LINESTAT);
	return 0;
}

static int ionia_uart_connected(struct uart_port *port)
{
	// bits must be 01x000xx
	return ((ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_LINESTAT) & 0xdc) == 0x40);
}

int ionia_uart_num_recv(struct uart_port* port)
{
	int rxsz;
	struct ionia_port *pp = container_of(port, struct ionia_port, port);

	// check for data ready flag
	if ((ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_LINESTAT) & BITMASK_LSR_DR)) {
		dev_info(port->dev, "[%2d] no rx data ready\n", pp->card);
		return 0;
	}

	rxsz = ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_RX_SIZE);

	if (rxsz == 0xFF) {
		dev_warn(port->dev, "[%2d] rx buffer full\n", pp->card);
		rxsz = ionia_uart_fifo_size(port);
	}

	return rxsz * RSR_MULTIPLIER;
}

void ionia_uart_dump(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	uint16_t lsr = ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_LINESTAT);

	dev_info(port->dev,
		"[%2d] @ 0x%03X fifo %d\n lsr 0x%2X %c%c%c%c%c%c %s cf: 0x%02X rx size: %4d tx size: %4d avail: %4d\n",
		pp->card,
		i101_cards[pp->card].base,
		ionia_uart_fifo_size(port),
		lsr,
		(lsr & BITMASK_LSR_DR        ? 'R':'_'),
		(lsr & BITMASK_LSR_LOOPBACK  ? 'L':'_'),
		(lsr & BITMASK_LSR_FLUSHTX   ? 'F':'_'),
		(lsr & BITMASK_LSR_THRE      ? 'T':'_'),
		(lsr & BITMASK_LSR_DEVREADY  ? 'D':'_'),
		(lsr & BITMASK_LSR_HOSTREADY ? 'H':'_'),
		(ionia_uart_connected(port) ? "CON" : "dis"),
		ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_CONF),
		ionia_uart_rx_size(port),
		ionia_uart_tx_size(port),
		ionia_uart_num_recv(port)
	);
}

int ionia_uart_putc(struct uart_port *port, char c)
{
	dev_info(port->dev, "serial_port_putc() c=%X\n", c);
	ionia_uart_setreg(port, IONIA_BACKPLANE_SERIAL_REG_READWRITE, c);
	ionia_backplane_waitreg();
	return 0;
}

int ionia_uart_getc(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	uint16_t ret = ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_READWRITE);
	uint16_t rxs = ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_RX_SIZE);
	uint16_t txs = ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_TX_SIZE);
	dev_info(port->dev, "[%2d] receive char: %04X -- rxs=%4d txs=%4d\n", pp->card, ret, rxs, txs);
	return (char)ret;
}

static unsigned int ionia_uart_tx_empty(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] tx_empty\n", pp->card);
	return 1;	// FIXME
}

static unsigned int ionia_uart_get_mctrl(struct uart_port *port)
{
	char buf[1024];
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	int ret = TIOCM_DSR | TIOCM_CTS;	// FIXME

	if (ionia_uart_connected(port))
		ret |= TIOCM_CAR;
	if (ionia_uart_getreg(port, IONIA_BACKPLANE_SERIAL_REG_LINESTAT) & BITMASK_LSR_LOOPBACK)
		ret |= TIOCM_LOOP;

	sprint_mctrl(buf, sizeof(buf), ret);
	dev_info(port->dev, "[%2d] get_mctrl: %s\n", pp->card, buf);

	return ret;
}

static void ionia_uart_set_mctrl(struct uart_port *port, unsigned int sigs)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	char buf[1024];
	sprint_mctrl(buf, sizeof(buf), sigs);
	dev_info(port->dev, "[%2d] set_mctrl: sigs=%s\n", pp->card, buf);
//	ionia_uart_set_loopback(port, sigs & TIOCM_LOOP);
}

static void ionia_uart_stop_tx(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] stop_tx()\n", pp->card);
}

static void ionia_uart_stop_rx(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] stop_rx\n", pp->card);
}

static void ionia_uart_break_ctl(struct uart_port *port, int break_state)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] break_ctrl\n", pp->card);
}

static void ionia_uart_set_termios(struct uart_port *port,
					struct ktermios *termios,
					struct ktermios *old)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] set_termios\n", pp->card);
	/* Just copy the old termios settings back */
	if (old)
		tty_termios_copy_hw(termios, old);
}

static void ionia_uart_rx_chars(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	uint16_t rx_size = ionia_uart_rx_size(port);

	if (!rx_size)
		return; // nothing to do

	while (rx_size) {
		int ch = ionia_uart_getc(port);
		port->icount.rx++;
		dev_info(&(pp->bp_pdev->dev), " rx char: %02X => \"%c\"\n", ch, ch);
		uart_insert_char(port, 0, 0, ch, TTY_NORMAL);
		rx_size--;
	}

	spin_unlock(&port->lock);
	tty_flip_buffer_push(&port->state->port);
	spin_lock(&port->lock);
}

static void ionia_uart_tx_chars(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);

	struct circ_buf *xmit = &port->state->xmit;
	unsigned int pending, count;

#ifdef HACK_LOOPBACK
	ionia_uart_set_loopback(port, 1);
#endif

	if (port->x_char) {
		/* Send special char - probably flow control */
		ionia_uart_putc(port, port->x_char);
		port->x_char = 0;
		port->icount.tx++;
		return;
	}

	pending = uart_circ_chars_pending(xmit);
	if (pending <= 0)
		return; // nothing to do

	dev_info(&(pp->bp_pdev->dev), "tx_chars() txsz: %d\n", ionia_uart_tx_size(port));

	if (pending > 0) {
		count = ionia_uart_tx_size(port);
		if (count > pending)
			count = pending;
		if (count > 0) {
			pending -= count;
			while (count--) {
				ionia_uart_putc(port, xmit->buf[xmit->tail]);
				xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
				port->icount.tx++;
			}
			if (pending < WAKEUP_CHARS)
				uart_write_wakeup(port);
		}
	}
}

static void ionia_uart_start_tx(struct uart_port *port)
{
//	struct ionia_port *pp = container_of(port, struct ionia_port, port);
//	dev_info(port->dev, "[%2d] start_tx\n", pp->card);
}

static void ionia_uart_config_port(struct uart_port *port, int flags)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] config_port\n", pp->card);
	port->type = PORT_IONIA_BP;
}

int ionia_uart_kthread(void *data)
{
	struct uart_port *port = data;
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	unsigned long flags;
	while (pp->kthread_running) {
		// FIXME: do we need that additional lock ?
		spin_lock_irqsave(&pp->port.lock, flags);
		ionia_uart_rx_chars(&pp->port);
		ionia_uart_tx_chars(&pp->port);
		spin_unlock_irqrestore(&pp->port.lock, flags);
		usleep_range(UART_POLL_MIN, UART_POLL_MAX);
	}
	do_exit(0);
	return 0;
}

static int ionia_uart_startup(struct uart_port *port)
{
	char namebuf[128];
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] startup\n", pp->card);
	snprintf(namebuf, sizeof(namebuf), "ionia-uart-%d", pp->card);
	pp->kthread_running = 1;
	pp->kthread = kthread_run(ionia_uart_kthread, port, namebuf);
	return 0;
}

static void ionia_uart_shutdown(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] shutdown\n", pp->card);
	pp->kthread_running = 0;
}

static const char *ionia_uart_type(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] type\n", pp->card);
	return (port->type == PORT_IONIA_BP) ? "Duagon Ionia Backplane" : NULL;
}

static int ionia_uart_request_port(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] request_port\n", pp->card);
	/* UARTs always present */
	return 0;
}

static void ionia_uart_release_port(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] release_port\n", pp->card);
	/* Nothing to release... */
}

static void ionia_uart_throttle(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] throttle\n", pp->card);
}

static void ionia_uart_unthrottle(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] unthrottle\n", pp->card);
}

static int ionia_uart_verify_port(struct uart_port *port,
				  struct serial_struct *ser)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] verify_port\n", pp->card);
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_IONIA_BP)
		return -EINVAL;
	return 0;
}

struct uart_driver ionia_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "ionia-serial",
	.dev_name	= "ttyIONIA",
	.major		= IONIA_SERIAL_MAJOR,
	.minor		= IONIA_SERIAL_MINOR,
};

static const struct uart_ops ionia_uart_ops = {
	.tx_empty	= ionia_uart_tx_empty,
	.get_mctrl	= ionia_uart_get_mctrl,
	.set_mctrl	= ionia_uart_set_mctrl,
	.start_tx	= ionia_uart_start_tx,
	.stop_tx	= ionia_uart_stop_tx,
	.stop_rx	= ionia_uart_stop_rx,
	.break_ctl	= ionia_uart_break_ctl,
	.startup	= ionia_uart_startup,
	.shutdown	= ionia_uart_shutdown,
	.set_termios	= ionia_uart_set_termios,
	.type		= ionia_uart_type,
	.request_port	= ionia_uart_request_port,
	.release_port	= ionia_uart_release_port,
	.config_port	= ionia_uart_config_port,
	.verify_port	= ionia_uart_verify_port,
	.throttle	= ionia_uart_throttle,
	.unthrottle	= ionia_uart_unthrottle,
};

struct uart_port *ionia_uart_init(struct platform_device *pdev, int card)
{
	struct ionia_port *port;
	int ret;

	dev_info(&pdev->dev, "add uart card %d: %s\n", card, i101_cards[card].name);
	port = kzalloc(sizeof(struct ionia_port), GFP_KERNEL);

	if (!port) {
		dev_err(&pdev->dev, "uart_init: failed to allocate port structure\n");
		return NULL;
	}

	port->card		= card;			// FIXME: isnt port.line enough ?
	port->name		= i101_cards[card].name;
	port->bp_pdev		= pdev;
	port->port.line		= card;
	port->port.type		= PORT_IONIA_BP;
	port->port.iotype	= SERIAL_IO_MEM;	// FIXME: use that for 32bit word transfers ?
	port->port.ops		= &ionia_uart_ops;
	port->port.flags	= UPF_BOOT_AUTOCONF;
	port->port.dev		= &pdev->dev;

	ret = uart_add_one_port(&ionia_uart_driver, &port->port);

	if (ret) {
		dev_info(&pdev->dev, "uart_add_one_port() failed for card %2d \"%s\": %M\n",
			card, i101_cards[card].name, ret);
		kfree(port);
		return NULL;
	}

	return &(port->port);
}

int ionia_serial_init(struct platform_device *pdev)
{
	int card, ret;
	ionia_uart_driver.nr = i101_cards_max;
	ret = uart_register_driver(&ionia_uart_driver);
	for (card=0; card<i101_cards_max; card++) {
		i101_cards[card].port = ionia_uart_init(pdev, card);
		if (i101_cards[card].port) {
#ifdef HACK_LOOPBACK
			ionia_uart_set_loopback(i101_cards[card].port, 1);
#endif
			ionia_uart_dump(i101_cards[card].port);
		}
	}
	return 0;
}

void ionia_serial_dump(struct platform_device *pdev, int card)
{
	ionia_uart_dump(i101_cards[card].port);
}

void ionia_serial_dumpall(struct platform_device *pdev)
{
	int card;
	for (card=0; card<i101_cards_max; card++) {
		ionia_uart_dump(i101_cards[card].port);
	}
}
