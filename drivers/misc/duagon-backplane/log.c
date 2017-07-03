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
#include "ionia-log.h"

#define LOG_DRIVER_VERSION 0x00010001  /*!< 0xMMMMmmmm MMMM=major=1 mmmm=minor=1 */

struct uart_port;

int ionia_log_channel_enable(ionia_rpc_t *rpc, int mode, int mask)
{
	int ret;
	ionia_rpcbuf_t *rpcbuf = ionia_rpcbuf_get(IONIA_PROTOCOL_LOG, IONIA_LOG_CMD_CHANNEL_ENABLE);

	if ((ret = ionia_rpcbuf_write_u32(rpcbuf, mode + (mask << 16))))
		goto out;

	if ((ret = ionia_rpc_call(rpc, rpcbuf)))
		goto out;

out:
	ionia_rpcbuf_put(rpcbuf);
	return ret;
}

int ionia_log_init(ionia_rpc_t *rpc)
{
	int ret;
	u32 version;
	u32 log_errno;
	ionia_rpcbuf_t *rpcbuf = ionia_rpcbuf_get(IONIA_PROTOCOL_LOG, IONIA_LOG_CMD_INIT);

	if ((ret = ionia_rpcbuf_write_u32(rpcbuf, LOG_DRIVER_VERSION)))
		goto err;

	if ((ret = ionia_rpc_call(rpc, rpcbuf)))
		goto err;

	if ((ret = ionia_rpcbuf_read_u32(rpcbuf, &version)))
		goto err;

	if ((ret = ionia_rpcbuf_read_u32(rpcbuf, &log_errno)))
		goto err;

	pr_info("ionia_log_init: version=%08X\n", version);
	goto out;

err:
	pr_err("ionia_log_init: err=%M\n", -ret);

out:
	ionia_rpcbuf_put(rpcbuf);
	return ret;
}
