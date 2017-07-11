/*
 * Duagon Ionia backplane core Linux support
 *
 * Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#include <linux/bug.h>
#include <linux/kernel.h>

#include "ionia.h"
#include "ionia-modules.h"
#include "ionia-protocol.h"
#include "ionia-cards.h"


#define CARD_TYPE_DECL(n,mt,pt)		\
	{				\
		.name = n,		\
		.module_type = mt,	\
		.protocol_type = pt,	\
	}

const struct ionia_card_type ionia_card_types[] = {
	// CO cards
	CARD_TYPE_DECL("i305", IONIA_MODULE_TYPE_SER_COM, IONIA_PROTOCOL_SER_COM),
	CARD_TYPE_DECL("i301", IONIA_MODULE_TYPE_MVB_COM, IONIA_PROTOCOL_MVB),
	CARD_TYPE_DECL("i306", IONIA_MODULE_TYPE_IO,      IONIA_PROTOCOL_WTB),
	CARD_TYPE_DECL("i303", IONIA_MODULE_TYPE_SOC_COM, IONIA_PROTOCOL_SOCKETS),

	// IO cards
	CARD_TYPE_DECL("i211", IONIA_MODULE_TYPE_AI,      IONIA_PROTOCOL_AI),
	CARD_TYPE_DECL("i213", IONIA_MODULE_TYPE_AO,      IONIA_PROTOCOL_AO),
	CARD_TYPE_DECL("i701", IONIA_MODULE_TYPE_LOG,     IONIA_PROTOCOL_LOG),

	// PWR cards
	CARD_TYPE_DECL("i012", IONIA_MODULE_TYPE_POWER,   IONIA_PROTOCOL_POWER),

	// CPU cards
	CARD_TYPE_DECL("i101", IONIA_MODULE_TYPE_MVB,     IONIA_PROTOCOL_MVB),
	CARD_TYPE_DECL("i102", IONIA_MODULE_TYPE_MVB,     IONIA_PROTOCOL_MVB),
	CARD_TYPE_DECL("i103", IONIA_MODULE_TYPE_MVB,     IONIA_PROTOCOL_MVB),
};

const int ionia_card_types_max = ARRAY_SIZE(ionia_card_types);

const struct ionia_card ionia_cards[] = {
	/* power supply */
	{
		.id            = 0,
		.model         = "i012",
		.title         = "20W EN50155 compliant Power Supply",
		.module_type   = IONIA_MODULE_TYPE_POWER,
		.protocol_type = IONIA_PROTOCOL_POWER,
		.slot_width    = 4,
		.width         = 4,
		.slot_pos      = 1,
		.mod_pos       = 3,
	},
	/* cpu module */
	{
		.id            = 1,
		.model         = "i103",
		.title         = "ARM Cortex-A8 based CPU module",
		.module_type   = IONIA_MODULE_TYPE_POWER,
		.protocol_type = IONIA_PROTOCOL_POWER,
		.slot_width    = 4,
		.width         = 8,
		.slot_pos      = 1,
		.mod_pos       = 7,
	},
	/* empty space */
	{
		.slot_width    = 3,
	},
	{
		.slot_width    = 3,
	},
	{
		.slot_width    = 3,
	},
	/* ADC card */
	{
		.id            = 2,
		.model         = "i701",
		.title         = "intelligent 4 channel 1bit ADC module for Jaquet",
		.module_type   = IONIA_MODULE_TYPE_LOG,
		.protocol_type = IONIA_PROTOCOL_LOG,
		.slot_width    = 4,
		.width         = 3,
		.slot_pos      = 1,
		.mod_pos       = 21,
	},
	/* DIO Dcard */
	{
		.id            = 3,
		.model         = "i202",
		.title         = "8 digital outputs, 8 digital inputs",
		.module_type   = IONIA_MODULE_TYPE_IO,
		.protocol_type = IONIA_PROTOCOL_IO,
		.slot_width    = 3,
		.width         = 3,
		.slot_pos      = 1,
		.mod_pos       = 24,
	},
};

const int ionia_cards_max = ARRAY_SIZE(ionia_cards);
