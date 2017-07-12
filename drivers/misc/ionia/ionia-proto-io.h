/*
 * Duagon Ionia IO card protocol
 *
 * Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_IONIA_PROTO_IO_H
#define __DUAGON_IONIA_PROTO_IO_H

#include "ionia-rpc.h"
#include "ionia-proto-io.h"

enum {
	IONIA_IO_CMD_INIT			= 0,
	IONIA_IO_CMD_OUT_SET			= 1,
	IONIA_IO_CMD_IN_GET			= 2,
	IONIA_IO_CMD_PWM_ENABLE			= 3,
	IONIA_IO_CMD_PWM_PERIOD_SET		= 4,
	IONIA_IO_CMD_PWM_PULS_WIDTH_SET		= 5,
	IONIA_IO_CMD_LED_CONFIG			= 6,
	IONIA_IO_CMD_LED_SET			= 7,
	IONIA_IO_CMD_HCI_ENABLE			= 8,
	IONIA_IO_CMD_FREQ_CNT_GATE_SET		= 9,
	IONIA_IO_CMD_FREQ_CNT_GET		= 10,
	IONIA_IO_CMD_TEMP_GET			= 11,
	IONIA_IO_CMD_PERIOD_CNT_GET_ALL		= 14,
	IONIA_IO_CMD_PERIOD_CNT_TIMEOUT_SET	= 15,
};

enum {
	IONIA_IO_MAP_TYPE_NORMAL,
	IONIA_IO_MAP_TYPE_TRANSPARENT
};

enum {
	IONIA_IO_MAP_GATE_8	= 0,	/* i20x cards < rev 5 */
	IONIA_IO_MAP_GATE_4	= 1,	/* i20x cards >= rev 5 */
};

int ionia_io_init(ionia_rpc_t *rpc);

#endif /* __DUAGON_IONIA_PROTO_IO_H */
