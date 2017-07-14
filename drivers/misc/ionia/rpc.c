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

#define rpc_info(rpc,fmt,args...) \
	pr_info("ionia rpc: %s: " fmt "\n", __func__, ##args)

#define rpc_err(rpc,fmt,args...) \
	pr_info("ionia rpc: %s: " fmt "\n", __func__, ##args)

#define rpcbuf_info(rpcbuf,fmt,args...) \
	pr_info("ionia rpc: %s: " fmt "\n", __func__, ##args)

#define rpcbuf_err(rpcbuf,fmt,args...) \
	pr_info("ionia rpc: %s: " fmt "\n", __func__, ##args)

#define WORD_SIZE	4

ionia_rpc_t *ionia_rpc_get_fifo(struct ionia_fifo *fifo)
{
	ionia_rpc_t *rpc = kzalloc(sizeof(ionia_rpc_t), GFP_KERNEL);
	if (rpc == NULL)
		return NULL;

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
		rpcbuf_err(rpcbuf, "buffer overflow");
		return -E2BIG;
	}

	memcpy(&rpcbuf->buf[rpcbuf->write_ptr], &v, sizeof(v));
	rpcbuf->write_ptr += sizeof(v);
	return 0;
}

int ionia_rpcbuf_read_block(ionia_rpcbuf_t *rpcbuf, void *buf, size_t sz)
{
	if (rpcbuf->read_ptr + sz > rpcbuf->write_ptr) {
		rpcbuf_err(rpcbuf, "buffer overflow");
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
		rpcbuf_err(rpcbuf, "read error: %M", -ret);
		return ret;
	}

	*val = be32_to_cpu(rv);
	return 0;
}

int ionia_rpc_xmit(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf)
{
	int x;
	int payload_size = rpcbuf->write_ptr / WORD_SIZE; // 32bit words

	rpc_info(rpc, "send: proto=%2d cmd=%2d words=%2d",
		rpcbuf->protocol,
		rpcbuf->command,
		payload_size);

	ionia_fifo_putc(rpc->fifo, payload_size >> 8);
	ionia_fifo_putc(rpc->fifo, payload_size & 0xFF);
	ionia_fifo_putc(rpc->fifo, rpcbuf->protocol);
	ionia_fifo_putc(rpc->fifo, rpcbuf->command);

	for (x=0; x<rpcbuf->write_ptr; x++)
		ionia_fifo_putc(rpc->fifo, rpcbuf->buf[x]);

	return 0;
}

int ionia_rpc_recv(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf)
{
	char hdrbuf[WORD_SIZE];
	int ret;
	size_t payload_size;

	if ((ret = ionia_fifo_recv(rpc->fifo, &hdrbuf, sizeof(hdrbuf)))) {
		rpc_err(rpc, "error reading header: %M", -ret);
		return ret;
	}

	payload_size = ((hdrbuf[3] << 8) + hdrbuf[2]) * WORD_SIZE;
	rpcbuf->protocol = hdrbuf[1];
	rpcbuf->command = hdrbuf[0];

	rpc_info(rpc, "recv: proto=%2d cmd=%2d words=%d",
		rpcbuf->protocol, rpcbuf->command,
		payload_size / WORD_SIZE);

	if (payload_size > sizeof(rpcbuf->buf)) {
		rpc_err(rpc, "recv payload size too big: %d", payload_size);
		return -E2BIG;
	}

	if ((ret = ionia_fifo_recv(rpc->fifo, &(rpcbuf->buf), payload_size))) {
		rpc_err(rpc, "failed to recv payload: %M", -ret);
		return ret;
	}

	rpcbuf->write_ptr = payload_size;
	return 0;
}

int ionia_rpc_call(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf)
{
	int ret;
	if ((ret = ionia_rpc_xmit(rpc, rpcbuf))) {
		rpc_err(rpc, "send failed: %M", -ret);
		return ret;
	}

	if ((ret = ionia_rpc_recv(rpc, rpcbuf))) {
		rpc_err(rpc, "recv failed: %M", -ret);
		return ret;
	}

	return ret;
}

int ionia_rpc_call_errno(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf)
{
	u32 *ptr;
	int ret = ionia_rpc_call(rpc, rpcbuf);
	if (ret)
		return ret;

	if (rpcbuf->write_ptr < WORD_SIZE) {
		rpc_err(rpc, "receive frame size too small: %d", rpcbuf->write_ptr);
		return -ENODATA;
	}

	rpc_info(rpc,"received payload size: %d", rpcbuf->write_ptr);
	ptr = (u32*)(&(rpcbuf->buf[rpcbuf->write_ptr-WORD_SIZE]));
	rpcbuf->errno = *ptr;
	rpc_info(rpc, "errno: %u -- %X", rpcbuf->errno, rpcbuf->errno);
	rpcbuf->write_ptr -= WORD_SIZE;
	return 0;
}
