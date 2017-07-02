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
#include "ionia-rpc-log.h"

struct ionia_rpc_buf *ionia_rpc_buf_get(ionia_protocol_t proto, u8 cmd)
{
	struct ionia_rpc_buf *rpcbuf = kzalloc(GFP_KERNEL, sizeof(struct ionia_rpc_buf));
	rpcbuf->protocol = proto;
	rpcbuf->command = cmd;
	return rpcbuf;
}

int ionia_rpc_buf_put(struct ionia_rpc_buf *rpcbuf)
{
	kfree(rpcbuf);
	return 0;
}

int ionia_rpc_buf_write_u32(struct ionia_rpc_buf *rpcbuf, u32 val)
{
	u32 bufval = cpu_to_be32(val);

	if (rpcbuf->wptr + sizeof(bufval) > sizeof(rpcbuf->buf)) {
		pr_err("ionia_rpc_buf_write_u32(): buffer overflow\n");
		return -ENOMEM;
	}

	memcpy(&rpcbuf->buf[rpcbuf->wptr], &bufval, sizeof(bufval));
	rpcbuf->wptr += sizeof(bufval);
	return 0;
}

int ionia_rpc_buf_send(struct ionia_rpc_buf *rpcbuf, struct uart_port *port)
{
	int x;
	int payload_size = rpcbuf->wptr / 4; // 32bit words

	ionia_uart_putc(port, rpcbuf->protocol);
	ionia_uart_putc(port, rpcbuf->command);
	ionia_uart_putc(port, payload_size & 0xFF);
	ionia_uart_putc(port, payload_size >> 8);

	for (x=0; x<rpcbuf->wptr; x++)
		ionia_uart_putc(port, rpcbuf->buf[x]);

	return 0;
}
