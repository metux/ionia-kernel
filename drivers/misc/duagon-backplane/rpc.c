/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/slab.h>

#include "ionia-serial.h"
#include "ionia-rpc.h"

struct uart_port;

ionia_rpc_t *ionia_rpc_get_uart(struct uart_port *port)
{
	ionia_rpc_t *rpc = kzalloc(GFP_KERNEL, sizeof(ionia_rpc_t));
	rpc->port = port;
	return rpc;
}

void ionia_rpc_put(ionia_rpc_t *rpc)
{
	kfree(rpc);
}

ionia_rpcbuf_t *ionia_rpcbuf_get(ionia_protocol_t proto, u8 cmd)
{
	ionia_rpcbuf_t *rpcbuf = kzalloc(GFP_KERNEL, sizeof(ionia_rpcbuf_t));
	rpcbuf->protocol = proto;
	rpcbuf->command = cmd;
	return rpcbuf;
}

int ionia_rpcbuf_put(ionia_rpcbuf_t *rpcbuf)
{
	kfree(rpcbuf);
	return 0;
}

int ionia_rpcbuf_write_u32(ionia_rpcbuf_t *rpcbuf, u32 val)
{
	u32 v = cpu_to_be32(val);

	if (rpcbuf->write_ptr + sizeof(v) > sizeof(rpcbuf->buf)) {
		pr_err("ionia_rpcbuf_write_u32(): buffer overflow\n");
		return -E2BIG;
	}

	memcpy(&rpcbuf->buf[rpcbuf->write_ptr], &v, sizeof(v));
	rpcbuf->write_ptr += sizeof(v);
	return 0;
}

int ionia_rpcbuf_read_block(ionia_rpcbuf_t *rpcbuf, void *buf, size_t sz)
{
	if (rpcbuf->read_ptr + sz > sizeof(rpcbuf->buf)) {
		pr_err("%s): buffer overflow\n", __func__);
		return -E2BIG;
	}

	memcpy(buf, &rpcbuf->buf[rpcbuf->read_ptr], sz);
	rpcbuf->read_ptr += sz;

	return 0;
}

int ionia_rpcbuf_read_u32(ionia_rpcbuf_t *rpcbuf, u32 *val)
{
	u32 rv;
	int ret = ionia_rpcbuf_read_block(rpcbuf, &rv, sizeof(rv));

	if (ret) {
		pr_err("%s: read error: %M\n", __func__, -ret);
		return ret;
	}

	*val = be32_to_cpu(rv);
	return 0;
}

int ionia_rpc_send(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf)
{
	int x;
	int payload_size = rpcbuf->write_ptr / 4; // 32bit words

	ionia_uart_putc(rpc->port, rpcbuf->protocol);
	ionia_uart_putc(rpc->port, rpcbuf->command);
	ionia_uart_putc(rpc->port, payload_size & 0xFF);
	ionia_uart_putc(rpc->port, payload_size >> 8);

	for (x=0; x<rpcbuf->write_ptr; x++)
		ionia_uart_putc(rpc->port, rpcbuf->buf[x]);

	return 0;
}

int ionia_rpc_recv(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf)
{
	return -EINVAL;
}

int ionia_rpc_call(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf)
{
	int ret;
	if ((ret = ionia_rpc_send(rpc, rpcbuf))) {
		pr_err("ionia_rpc_call: send failed: %M\n", -ret);
		return ret;
	}

	if ((ret = ionia_rpc_recv(rpc, rpcbuf))) {
		pr_err("ionia_rpc_call: recv failed: %M\n", -ret);
		return ret;
	}

	return ret;
}
