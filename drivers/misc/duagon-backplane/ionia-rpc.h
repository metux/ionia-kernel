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

#define IONIA_RPC_PROTO_PFS		0x01
#define IONIA_RPC_PROTO_SOCKET		0x02
#define IONIA_RPC_PROTO_IPTCOM_PD	0x03
#define IONIA_RPC_PROTO_IPTCOM_MD	0x04
#define IONIA_RPC_PROTO_IO		0x06
#define IONIA_RPC_PROTO_AI		0x07
#define IONIA_RPC_PROTO_AO		0x07
#define IONIA_RPC_PROTO_LOG		0x08
#define IONIA_RPC_PROTO_SER_COM		0x09
#define IONIA_RPC_PROTO_SVC		0x0F
#define IONIA_RPC_PROTO_MVB		0x10
#define IONIA_RPC_PROTO_WTB		0x11
#define IONIA_RPC_PROTO_PWR		0x12

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

struct ionia_rpc_buf
{
	u8 command;
	u8 protocol;
	int wptr;
	int rptr;
	char buf[1024];
};

struct ionia_rpc_buf *ionia_rpc_buf_get(u8 proto, u8 cmd);
int ionia_rpc_buf_put(struct ionia_rpc_buf *rpcbuf);
int ionia_rpc_buf_write_u32(struct ionia_rpc_buf *rpcbuf, u32 val);
int ionia_rpc_buf_send(struct ionia_rpc_buf *buf, struct uart_port *port);

#endif /* __DUAGON_RPC_H */
