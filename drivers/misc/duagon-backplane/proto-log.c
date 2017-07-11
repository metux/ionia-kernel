/*
 * Duagon Ionia logging protocol
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

#include "ionia-rpc.h"
#include "ionia-proto-log.h"

#define LOG_DRIVER_VERSION		0x00010001	/* 0xMMMMmmmm MMMM=major=1 mmmm=minor=1 */
#define LOG_SENSOR_DATA_WORD_SIZE	120		/* sensor data size */

#define IONIA_RPC_BEGIN_LOG(cmd)	IONIA_RPC_BEGIN(IONIA_PROTOCOL_LOG,cmd)

int ionia_log_init(ionia_rpc_t *rpc)
{
	u32 version;

	IONIA_RPC_BEGIN_LOG(IONIA_LOG_CMD_INIT)
	IONIA_RPC_PAR_U32(LOG_DRIVER_VERSION)
	IONIA_RPC_CALL
	IONIA_RPC_RET_U32(&version)
	IONIA_RPC_RET_ERR

	pr_info("ionia_log_init: version=%08X errno=%08X\n", version, rpc_errno);

	IONIA_RPC_END
}

int ionia_log_channel_enable(ionia_rpc_t *rpc, int mode, int mask)
{
	IONIA_RPC_BEGIN_LOG(IONIA_LOG_CMD_CHANNEL_ENABLE)
	IONIA_RPC_PAR_U32(mode + (mask << 16))
	IONIA_RPC_CALL
	IONIA_RPC_RET_ERR
	IONIA_RPC_END
}

int ionia_log_set_samplerate(ionia_rpc_t *rpc, int rate)
{
	IONIA_RPC_BEGIN_LOG(IONIA_LOG_CMD_SAMPLING_RATE)
	IONIA_RPC_PAR_U32(rate)
	IONIA_RPC_CALL
	IONIA_RPC_RET_ERR
	IONIA_RPC_END
}

int ionia_log_pattern_gen_enable(ionia_rpc_t *rpc, int enable)
{
	IONIA_RPC_BEGIN_LOG(IONIA_LOG_CMD_PATTERN_GEN);
	IONIA_RPC_PAR_U32(0)
	IONIA_RPC_CALL
	IONIA_RPC_RET_ERR
	IONIA_RPC_END
}

int ionia_log_clear_buffer(ionia_rpc_t *rpc)
{
	IONIA_RPC_BEGIN_LOG(IONIA_LOG_CMD_BUFFER_CLEAR);
	IONIA_RPC_PAR_U32(0x01);
	IONIA_RPC_CALL
	IONIA_RPC_RET_ERR
	IONIA_RPC_END
}

int ionia_log_get_data(ionia_rpc_t *rpc, u32 *data, size_t sz)
{
	IONIA_RPC_BEGIN_LOG(IONIA_LOG_CMD_SAMPLING_DATA_GET)
	if (sz < LOG_SENSOR_DATA_WORD_SIZE) {
		pr_err("%s: sample buffer %d too small\n", __func__, sz);
		return -EINVAL;
	}
	IONIA_RPC_RET_BLOCK(data, LOG_SENSOR_DATA_WORD_SIZE)
	IONIA_RPC_RET_ERR
	IONIA_RPC_END_OKVAL(LOG_SENSOR_DATA_WORD_SIZE)
}
