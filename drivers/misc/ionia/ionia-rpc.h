/*
 * Duagon Ionia backplane core Linux support
 *
 * Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_RPC_H
#define __DUAGON_RPC_H

#include <linux/types.h>

#include "ionia-protocol.h"

#define IONIA_RPC_CHANNEL_SERCOM	0x00
#define IONIA_RPC_CHANNEL_IO		0x00
#define IONIA_RPC_CHANNEL_POWER		0x00
#define IONIA_RPC_CHANNEL_SOCKETS	0x00
#define IONIA_RPC_CHANNEL_PFS		get_channel_for_aux(card_index)
#define IONIA_RPC_CHANNEL_IPT_PD	0x00
#define IONIA_RPC_CHANNEL_IPT_MD	get_channel_for_aux(card_index)
#define IONIA_RPC_CHANNEL_SVC		get_channel_for_aux(card_index)
#define IONIA_RPC_CHANNEL_MVB		0x00

typedef struct
{
	ionia_protocol_t protocol;
	u8 command;
	size_t write_ptr;
	size_t read_ptr;
	char buf[1024];
	u32 errno;
} ionia_rpcbuf_t;

typedef struct
{
	struct ionia_fifo *fifo;
} ionia_rpc_t;

ionia_rpc_t *ionia_rpc_get_fifo(struct ionia_fifo *fifo);
void         ionia_rpc_put(ionia_rpc_t *rpc);
int          ionia_rpc_call(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf);
int          ionia_rpc_call_errno(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf);

ionia_rpcbuf_t *ionia_rpcbuf_get(ionia_protocol_t proto, u8 cmd);
int             ionia_rpcbuf_put(ionia_rpcbuf_t *rpcbuf);
int             ionia_rpcbuf_write_u32(ionia_rpcbuf_t *rpcbuf, u32 val);
int             ionia_rpcbuf_read_u32(ionia_rpcbuf_t *rpcbuf, u32 *val);
int             ionia_rpcbuf_read_block(ionia_rpcbuf_t *rpcbuf, void *val, size_t sz);

/* helpers for RPC proto implementation */

#define IONIA_RPC_BEGIN(proto,cmd)				\
	int ret;						\
	u32 rpc_errno;						\
	ionia_rpcbuf_t *rpcbuf = ionia_rpcbuf_get(proto, cmd);

#define IONIA_RPC_END						\
	goto out;						\
err:								\
	pr_err("%s: error: %M\n", __func__, -ret);		\
out:								\
	ionia_rpcbuf_put(rpcbuf);				\
	return ret;

#define IONIA_RPC_END_OKVAL(v)					\
	ret = v;						\
	IONIA_RPC_END

#define IONIA_RPC_PAR_U32(v)					\
	if ((ret = ionia_rpcbuf_write_u32(rpcbuf, v)))		\
		goto out;

#define IONIA_RPC_CALL_ERRNO					\
	if ((ret = ionia_rpc_call_errno(rpc, rpcbuf)))		\
		goto out;					\
	rpc_errno = rpcbuf->errno;				\
	pr_info("%s: errno: %d 0x%x\n", __func__, rpc_errno, rpc_errno);

#define IONIA_RPC_RET_U32(rv)					\
	if ((ret = ionia_rpcbuf_read_u32(rpcbuf, rv)))		\
		goto err;

#define IONIA_RPC_RET_BLOCK(rv,sz)				\
	if ((ret = ionia_rpcbuf_read_block(rpcbuf, rv, sz)))	\
		goto err;

#endif /* __DUAGON_RPC_H */
