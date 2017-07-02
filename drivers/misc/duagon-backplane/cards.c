/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
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

#define CARD_DECL(n,b) 	\
	{			\
		.name = n,	\
		.base = b,	\
	}

#define CARD_DECL_IO(n,c0)	CARD_DECL(n,c0)

struct i101_card i101_cards[] = {
	CARD_DECL_IO("BP channel 0",  0x000),
	CARD_DECL_IO("BP channel 1x", 0x010),
	CARD_DECL_IO("BP channel 1",  0x020),
	CARD_DECL_IO("BP slot 2x",    0x030),
	CARD_DECL_IO("BP slot 2",    0x040),
	CARD_DECL_IO("BP slot 3",    0x050),
	CARD_DECL_IO("BP slot 3",    0x060),
	CARD_DECL_IO("BP slot 4",    0x070),
	CARD_DECL_IO("BP slot 4",    0x080),
	CARD_DECL_IO("BP slot 5",    0x090),
	CARD_DECL_IO("BP slot 5",    0x0a0),
	CARD_DECL_IO("BP slot 6",    0x0b0),
	CARD_DECL_IO("BP slot 6",    0x0c0),
	CARD_DECL_IO("BP slot 7",    0x0d0),
	CARD_DECL_IO("BP slot 7",    0x0e0),
	CARD_DECL_IO("BP slot 8",    0x0f0),
	CARD_DECL_IO("BP slot 8",    0x110),
	CARD_DECL_IO("BP slot 9",    0x120),
	CARD_DECL_IO("BP slot 9",    0x130),
	CARD_DECL_IO("BP slot 10",   0x140),
	CARD_DECL_IO("BP slot 10",   0x150),
	CARD_DECL_IO("BP slot 10",   0x160),
	CARD_DECL_IO("BP slot 10",   0x170),
	CARD_DECL_IO("BP slot 10",   0x180),
};

const int i101_cards_max = ARRAY_SIZE(i101_cards);

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
