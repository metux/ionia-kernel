/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_IONIA_CARDS_H
#define __DUAGON_IONIA_CARDS_H

#include <linux/types.h>

#include "ionia-modules.h"

struct ionia_card_type {
	char *name;
	ionia_module_type_t module_type;
	u8 protocol_type;
};

const struct ionia_card_type ionia_card_types[];
const int ionia_card_types_max;

struct ionia_card {
	char *model;
	char *title;
	int id;
	ionia_module_type_t module_type;
	ionia_protocol_t protocol_type;
	int slot_width;
	int width;
	int slot_pos;
	int mod_pos;
};

#endif /* __DUAGON_IONIA_CARDS_H */
