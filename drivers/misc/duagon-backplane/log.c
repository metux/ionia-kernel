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

#define LOG_DRIVER_VERSION		0x00010001	/* 0xMMMMmmmm MMMM=major=1 mmmm=minor=1 */
#define LOG_SENSOR_DATA_WORD_SIZE	120		/* sensor data size */

#define LOGRPC_BEGIN(cmd)							\
	int ret;								\
	u32 log_errno;								\
	ionia_rpcbuf_t *rpcbuf = ionia_rpcbuf_get(IONIA_PROTOCOL_LOG, cmd);	\

#define LOGRPC_END								\
	goto out;								\
err:										\
	pr_err("%s: error: %M\n", __func__, -ret);				\
out:										\
	ionia_rpcbuf_put(rpcbuf);						\
	return ret;								\

#define LOGRPC_END_OKVAL(v)							\
	ret = v;								\
	LOGRPC_END								\

#define LOGRPC_PAR_U32(v)							\
	if ((ret = ionia_rpcbuf_write_u32(rpcbuf, v)))				\
		goto out;

#define LOGRPC_CALL								\
	if ((ret = ionia_rpc_call(rpc, rpcbuf)))				\
		goto out;

#define LOGRPC_RET_ERR								\
	if ((ret = ionia_rpcbuf_read_u32(rpcbuf, &log_errno)))			\
		goto err;

#define LOGRPC_RET_U32(rv)							\
	if ((ret = ionia_rpcbuf_read_u32(rpcbuf, rv)))				\
		goto err;

#define LOGRPC_RET_BLOCK(rv,sz)							\
	if ((ret = ionia_rpcbuf_read_block(rpcbuf, rv, sz)))			\
		goto err;

int ionia_log_init(ionia_rpc_t *rpc)
{
	u32 version;

	LOGRPC_BEGIN(IONIA_LOG_CMD_INIT)
	LOGRPC_PAR_U32(LOG_DRIVER_VERSION)
	LOGRPC_CALL
	LOGRPC_RET_U32(&version)
	LOGRPC_RET_ERR

	pr_info("ionia_log_init: version=%08X\n", version);

	LOGRPC_END
}

int ionia_log_channel_enable(ionia_rpc_t *rpc, int mode, int mask)
{
	LOGRPC_BEGIN(IONIA_LOG_CMD_CHANNEL_ENABLE)
	LOGRPC_PAR_U32(mode + (mask << 16))
	LOGRPC_CALL
	LOGRPC_RET_ERR
	LOGRPC_END
}

int ionia_log_set_samplerate(ionia_rpc_t *rpc, int rate)
{
	LOGRPC_BEGIN(IONIA_LOG_CMD_SAMPLING_RATE)
	LOGRPC_PAR_U32(rate)
	LOGRPC_CALL
	LOGRPC_RET_ERR
	LOGRPC_END
}

int ionia_log_pattern_gen_enable(ionia_rpc_t *rpc, int enable)
{
	LOGRPC_BEGIN(IONIA_LOG_CMD_PATTERN_GEN);
	LOGRPC_PAR_U32(0)
	LOGRPC_CALL
	LOGRPC_RET_ERR
	LOGRPC_END
}

int ionia_log_clear_buffer(ionia_rpc_t *rpc)
{
	LOGRPC_BEGIN(IONIA_LOG_CMD_BUFFER_CLEAR);
	LOGRPC_PAR_U32(0x01);
	LOGRPC_CALL
	LOGRPC_RET_ERR
	LOGRPC_END
}

int ionia_log_get_data(ionia_rpc_t *rpc, u32 *data, size_t sz)
{
	LOGRPC_BEGIN(IONIA_LOG_CMD_SAMPLING_DATA_GET)
	if (sz < LOG_SENSOR_DATA_WORD_SIZE) {
		pr_err("%s: sample buffer %d too small\n", __func__, sz);
		return -EINVAL;
	}
	LOGRPC_RET_BLOCK(data, LOG_SENSOR_DATA_WORD_SIZE)
	LOGRPC_RET_ERR
	LOGRPC_END_OKVAL(LOG_SENSOR_DATA_WORD_SIZE)
}
