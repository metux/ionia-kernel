/*
 * Duagon Ionia backplane core Linux support
 *
 * Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
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

#include "ionia-fifo.h"
#include "ionia-rpc.h"

ionia_rpc_t *ionia_rpc_get_fifo(struct ionia_fifo *fifo)
{
	ionia_rpc_t *rpc = kzalloc(sizeof(ionia_rpc_t), GFP_KERNEL);
	if (rpc == NULL) {
		pr_err("rpc_get_fifo(): failed to allocate memory\n");
		return NULL;
	}

	rpc->fifo = fifo;
	return rpc;
}

void ionia_rpc_put(ionia_rpc_t *rpc)
{
	kfree(rpc);
}

ionia_rpcbuf_t *ionia_rpcbuf_get(ionia_protocol_t proto, u8 cmd)
{
	ionia_rpcbuf_t *rpcbuf = kzalloc(sizeof(ionia_rpcbuf_t), GFP_KERNEL);
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
		pr_err("%s: buffer overflow\n", __func__);
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

	ionia_fifo_putc(rpc->fifo, rpcbuf->protocol);
	ionia_fifo_putc(rpc->fifo, rpcbuf->command);
	ionia_fifo_putc(rpc->fifo, payload_size >> 8);
	ionia_fifo_putc(rpc->fifo, payload_size & 0xFF);

	for (x=0; x<rpcbuf->write_ptr; x++)
		ionia_fifo_putc(rpc->fifo, rpcbuf->buf[x]);

	return 0;
}

int ionia_rpc_recv(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf)
{
	char hdrbuf[4];
	int ret;
	size_t payload_size;

	if ((ret = ionia_fifo_recv(rpc->fifo, &hdrbuf, sizeof(hdrbuf)))) {
		pr_err("%s: error reading header: %M\n", __func__, -ret);
		return ret;
	}

	rpcbuf->protocol = hdrbuf[0];
	rpcbuf->command = hdrbuf[1];
	payload_size = (hdrbuf[2] << 8) + hdrbuf[3];

	if (payload_size > sizeof(rpcbuf->buf)) {
		pr_err("%s: recv payload size too big: %d\n", __func__, payload_size);
		return -E2BIG;
	}

	if ((ret = ionia_fifo_recv(rpc->fifo, &(rpcbuf->buf), payload_size))) {
		pr_err("%s: failed to recv payload: %M\n", __func__, -ret);
		return ret;
	}

	rpcbuf->write_ptr = payload_size;
	return 0;
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
