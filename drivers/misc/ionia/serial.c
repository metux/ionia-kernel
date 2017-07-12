/*
 * Duagon Ionia backplane core Linux support
 *
 * Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/tty_flip.h>
#include <linux/string.h>

#include "ionia.h"
#include "ionia-fifo.h"
#include "ionia-slots.h"
#include "ionia-serial.h"

#define NR_UART			20
#define IONIA_SERIAL_MAJOR	TTY_MAJOR
#define IONIA_SERIAL_MINOR	196

#define UART_POLL_MIN		100000
#define UART_POLL_MAX		200000

//#define HACK_LOOPBACK
#define HACK_INIT_PING

int driver_registered = 0;

struct ionia_port {
	struct uart_port port;
	struct platform_device *bp_pdev;
	struct task_struct *kthread;
	int kthread_running;
	struct ionia_slot *slot;
};

static unsigned int ionia_uart_tx_empty(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] tx_empty\n", pp->slot->fifo.slot);
	return 1;	// FIXME
}

static unsigned int ionia_uart_get_mctrl(struct uart_port *port)
{
//	char buf[1024];
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	int ret = TIOCM_DSR | TIOCM_CTS;	// FIXME

	if (ionia_fifo_connected(&pp->slot->fifo))
		ret |= TIOCM_CAR;
	if (ionia_fifo_get_loopback(&pp->slot->fifo))
		ret |= TIOCM_LOOP;

//	sprint_mctrl(buf, sizeof(buf), ret);
//	dev_info(port->dev, "[%2d] get_mctrl: %s\n", pp->fifo.slot, buf);

	return ret;
}

static void ionia_uart_set_mctrl(struct uart_port *port, unsigned int sigs)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	char buf[1024];
	sprint_mctrl(buf, sizeof(buf), sigs);
	dev_info(port->dev, "[%2d] set_mctrl: sigs=%s\n", pp->slot->fifo.slot, buf);
//	ionia_fifo_set_loopback(&pp->fifo, sigs & TIOCM_LOOP);
}

static void ionia_uart_stop_tx(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] stop_tx()\n", pp->slot->fifo.slot);
}

static void ionia_uart_stop_rx(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] stop_rx\n", pp->slot->fifo.slot);
}

static void ionia_uart_break_ctl(struct uart_port *port, int break_state)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] break_ctrl\n", pp->slot->fifo.slot);
}

static void ionia_uart_set_termios(struct uart_port *port,
					struct ktermios *termios,
					struct ktermios *old)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] set_termios\n", pp->slot->fifo.slot);
	/* Just copy the old termios settings back */
	if (old)
		tty_termios_copy_hw(termios, old);
}

static void ionia_uart_rx_chars(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	uint16_t rx_size = ionia_fifo_rx_size(&pp->slot->fifo);

	if (!rx_size)
		return; // nothing to do

	while (rx_size) {
		int ch = ionia_fifo_getc(&pp->slot->fifo);
		port->icount.rx++;
		pdev_info(pp->bp_pdev, " rx char: %02X => \"%c\"\n", ch, ch);
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
	ionia_fifo_set_loopback(&pp->slot->fifo, 1);
#endif

	if (port->x_char) {
		/* Send special char - probably flow control */
		ionia_fifo_putc(&pp->slot->fifo, port->x_char);
		port->x_char = 0;
		port->icount.tx++;
		return;
	}

	pending = uart_circ_chars_pending(xmit);
	if (pending <= 0)
		return; // nothing to do

	pdev_info(pp->bp_pdev, "tx_chars() txsz: %d\n", ionia_fifo_tx_size(&pp->slot->fifo));

	if (pending > 0) {
		count = ionia_fifo_tx_size(&pp->slot->fifo);
		count = 128;
		if (count > pending)
			count = pending;
		if (count > 0) {
			pending -= count;
			while (count--) {
				ionia_fifo_putc(&pp->slot->fifo, xmit->buf[xmit->tail]);
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
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] start_tx\n", pp->slot->fifo.slot);
}

static void ionia_uart_config_port(struct uart_port *port, int flags)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] config_port\n", pp->slot->fifo.slot);
	port->type = PORT_IONIA_BP;
}

static int ionia_uart_kthread(void *data)
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
	dev_info(port->dev, "[%2d] startup\n", pp->slot->fifo.slot);
	snprintf(namebuf, sizeof(namebuf), "ionia-uart-%d", pp->slot->fifo.slot);
	pp->kthread_running = 1;
	pp->kthread = kthread_run(ionia_uart_kthread, port, namebuf);
	return 0;
}

static void ionia_uart_shutdown(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] shutdown\n", pp->slot->fifo.slot);
	pp->kthread_running = 0;
}

static const char *ionia_uart_type(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] type\n", pp->slot->fifo.slot);
	return (port->type == PORT_IONIA_BP) ? "Duagon Ionia Backplane" : NULL;
}

static int ionia_uart_request_port(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] request_port\n", pp->slot->fifo.slot);
	/* UARTs always present */
	return 0;
}

static void ionia_uart_release_port(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] release_port\n", pp->slot->fifo.slot);
	/* Nothing to release... */
}

static void ionia_uart_throttle(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] throttle\n", pp->slot->fifo.slot);
}

static void ionia_uart_unthrottle(struct uart_port *port)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] unthrottle\n", pp->slot->fifo.slot);
}

static int ionia_uart_verify_port(struct uart_port *port,
				  struct serial_struct *ser)
{
	struct ionia_port *pp = container_of(port, struct ionia_port, port);
	dev_info(port->dev, "[%2d] verify_port\n", pp->slot->fifo.slot);
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

struct ionia_port *ionia_port_init(struct platform_device *pdev, int slot)
{
	struct ionia_port *port;
	int ret;
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;

	if (slot > pdata->nr_slots) {
		pdev_err(pdev, "uart_init: no such slot -- BUG ?\n");
		return NULL;
	}

	pdev_info(pdev, "add uart slot %d: %s\n", slot, pdata->slots[slot].name);
	port = devm_kzalloc(&pdev->dev, sizeof(struct ionia_port), GFP_KERNEL);

	if (!port) {
		pdev_err(pdev, "uart_init: failed to allocate port structure\n");
		return NULL;
	}

	port->slot		= &pdata->slots[slot];
	port->bp_pdev		= pdev;
	port->port.line		= slot;
	port->port.type		= PORT_IONIA_BP;
	port->port.iotype	= SERIAL_IO_MEM;	// FIXME: use that for 32bit word transfers ?
	port->port.ops		= &ionia_uart_ops;
	port->port.flags	= UPF_BOOT_AUTOCONF;
	port->port.dev		= &pdev->dev;

	ret = uart_add_one_port(&ionia_uart_driver, &port->port);

	// HACK: send some dummy traffic to initialize card
#ifdef HACK_INIT_PING
	pdev_info(pdev, "sending init traffic to slot %d\n", slot);
	ionia_fifo_putc(&(port->slot->fifo), 0x00);
	ionia_fifo_putc(&(port->slot->fifo), 0x00);
	ionia_fifo_putc(&(port->slot->fifo), 0x00);
	ionia_fifo_putc(&(port->slot->fifo), 0x00);
	ionia_fifo_putc(&(port->slot->fifo), 0x00);
	ionia_fifo_putc(&(port->slot->fifo), 0x00);
	ionia_fifo_putc(&(port->slot->fifo), 0x00);
	ionia_fifo_putc(&(port->slot->fifo), 0x00);
#endif

	if (ret) {
		pdev_info(pdev, "uart_add_one_port() failed for slot %2d \"%s\": %M\n",
			slot, pdata->slots[slot].name, ret);
		kfree(port);
		return NULL;
	}

	pdev_info(pdev, "ionia_port_init() done\n");
	return port;
}

static int register_drv(struct platform_device *pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	if (driver_registered) {
		pdev_info(pdev, "uart driver already registered\n");
		return 0;
	}

	ionia_uart_driver.nr = pdata->nr_slots;
	ret = uart_register_driver(&ionia_uart_driver);

	if (ret) {
		pdev_err(pdev, "failed to register uart driver: %M\n", -ret);
		return ret;
	}

	driver_registered = 1;
	pdev_info(pdev, "registered uart driver\n");
	return 0;
}

static int register_port(struct platform_device *pdev, int slot)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	struct ionia_port *p;

	if (pdata->slots[slot].port != NULL) {
		pdev_info(pdev, "uart port for slot %d already registered\n", slot);
		return 0;
	}

	p = ionia_port_init(pdev, slot);
	if (p == NULL) {
		pdev_warn(pdev, "register_port: failed for slot %d\n", slot);
		return -ENOENT;
	}

	pdata->slots[slot].port = p;
	pdev_info(pdev, "register_port: initialized port for slot %d\n", slot);

#ifdef HACK_LOOPBACK
	ionia_fifo_set_loopback(&(pdata->slots[slot].fifo), 1);
#endif
	ionia_fifo_dump(&(pdata->slots[slot].fifo));
	return 0;
}

int ionia_serial_init(struct platform_device *pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	int slot, ret;

	if ((ret = register_drv(pdev)))
		return ret;

	for (slot=0; slot<pdata->nr_slots; slot++)
		register_port(pdev, slot);

	return 0;
}

void ionia_serial_dumpall(struct platform_device *pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	int slot;
	for (slot=0; slot<pdata->nr_slots; slot++)
		ionia_fifo_dump(&(pdata->slots[slot].fifo));
}
