/*
 * Duagon Ionia backplane core Linux support
 *
 * Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#ifndef __IONIA_PROTO_CTRL_H
#define __IONIA_PROTO_CTRL_H

#include "ionia-rpc.h"

int ionia_ctrl_version(ionia_rpc_t *rpc, u32 *ver);

#endif /* __IONIA_PROTO_CTRL_H */
