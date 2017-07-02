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

#include "ionia-rpc.h"
#include "ionia-rpc-log.h"

struct uart_port;

int ionia_log_channel_enable(struct uart_port *port, int mode, int mask)
{
	int ret;
	struct ionia_rpc_buf *rpcbuf = ionia_rpc_buf_get(IONIA_PROTOCOL_LOG, IONIA_LOG_CMD_CHANNEL_ENABLE);

	if ((ret = ionia_rpc_buf_write_u32(rpcbuf, mode + (mask << 16))))
		goto out;

	ret = ionia_rpc_buf_send(rpcbuf, port);
out:
	ionia_rpc_buf_put(rpcbuf);
	return ret;
}
