/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
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

struct uart_port;

typedef struct
{
	ionia_protocol_t protocol;
	u8 command;
	int write_ptr;
	int read_ptr;
	char buf[1024];
} ionia_rpcbuf_t;

typedef struct
{
	struct uart_port *port;
} ionia_rpc_t;

ionia_rpc_t *ionia_rpc_get_uart(struct uart_port *port);
void         ionia_rpc_put(ionia_rpc_t *rpc);
int          ionia_rpc_send(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf);
int          ionia_rpc_recv(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf);
int          ionia_rpc_call(ionia_rpc_t *rpc, ionia_rpcbuf_t *rpcbuf);

ionia_rpcbuf_t *ionia_rpcbuf_get(ionia_protocol_t proto, u8 cmd);
int             ionia_rpcbuf_put(ionia_rpcbuf_t *rpcbuf);
int             ionia_rpcbuf_write_u32(ionia_rpcbuf_t *rpcbuf, u32 val);
int             ionia_rpcbuf_read_u32(ionia_rpcbuf_t *rpcbuf, u32 *val);
int             ionia_rpcbuf_read_block(ionia_rpcbuf_t *rpcbuf, void *val, size_t sz);

#endif /* __DUAGON_RPC_H */
