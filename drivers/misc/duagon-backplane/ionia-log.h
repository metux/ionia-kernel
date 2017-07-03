/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_LOG_H
#define __DUAGON_LOG_H

#include "ionia-rpc.h"

enum {
	IONIA_LOG_CMD_INIT		= 0,	/* Reset modules? Self test ? tbd */
	IONIA_LOG_CMD_CHANNEL_ENABLE	= 1,	/* Enable 4 sensor input channels individually */
	IONIA_LOG_CMD_CHANNEL_MODE	= 2,	/* Select active/passive sensor tbd */
	IONIA_LOG_CMD_BUFFER_STATUS	= 3,	/* Fill state of the data buffer */
	IONIA_LOG_CMD_PATTERN_GEN	= 4,	/* Pattern generator enable */
	IONIA_LOG_CMD_SAMPLING_RATE	= 5,	/* Sampling rate tbd */
	IONIA_LOG_CMD_LED_CONFIG	= 6,	/* Change LED mode: user defined or input controlled tbd */
	IONIA_LOG_CMD_LED_SET		= 7,	/* Set status for 4 LEDs tbd */
	IONIA_LOG_CMD_BUFFER_CLEAR	= 8,	/* Clears the sampling data buffer */
	IONIA_LOG_CMD_TEMP_GET		= 11,	/* Return temperature */
	IONIA_LOG_CMD_SAMPLING_DATA_GET	= 12	/* Get Sensor Data Array */
};

int ionia_log_init(ionia_rpc_t *rpc);
int ionia_log_channel_enable(ionia_rpc_t *rpc, int mode, int mask);
int ionia_log_set_samplerate(ionia_rpc_t *rpc, int rate);
int ionia_log_get_data(ionia_rpc_t *rpc, u32 *data, size_t sz);
int ionia_log_clear_buffer(ionia_rpc_t *rpc);
int ionia_log_pattern_gen_enable(ionia_rpc_t *rpc, int enable);

#endif /* __DUAGON_RPC_H */
