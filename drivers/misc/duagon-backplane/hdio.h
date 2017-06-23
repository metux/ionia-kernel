/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#ifndef __LINUX_DUAGON_HDIO_H
#define __LINUX_DUAGON_HDIO_H

#include <linux/io.h>
#include "ionia-pdata.h"

struct duagon_hdio_channel;

typedef int (*duagon_hdio_channel_timeout_func)(struct duagon_hdio_channel* ch);

struct duagon_hdio_channel {
	int channel_type;
	int rx_timeout;
	int tx_timeout;
	int blocking;
	int is_polling;
	duagon_hdio_channel_timeout_func op_timeout;
	duagon_hdio_channel_timeout_func op_timeout_default;
};

#endif /* __LINUX_DUAGON_HDIO_H */
