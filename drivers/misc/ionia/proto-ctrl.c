/*
 * Duagon Ionia GPIO card protocol support
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
#include "ionia-proto-ctrl.h"

#define IONIA_PROTOCOL_CTRL	0x0F
#define IONIA_CTRL_CMD_VERSION	0x0A

// 0x00 0x00 0x0A 0x0F

#define IONIA_RPC_BEGIN_CTRL(cmd) IONIA_RPC_BEGIN(IONIA_PROTOCOL_CTRL,cmd)

int ionia_ctrl_version(ionia_rpc_t *rpc, u32 *ver)
{
	u32 version;

	IONIA_RPC_BEGIN_CTRL(IONIA_CTRL_CMD_VERSION);
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_RET_U32(&version)

	pr_info("CTRL: %s: version=%08X\n", __func__, version);

	IONIA_RPC_END

	return rpc_errno;
}
