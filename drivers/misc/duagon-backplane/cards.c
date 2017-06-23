/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#include <linux/bug.h>
#include <linux/kernel.h>

#include "cards.h"

#define CARD_DECL(t,n,c0,c1) 				\
	{						\
		.type = t,				\
		.name = n,				\
		.host_base_channel = { c0, c1 },	\
	}

#define CARD_DECL_IO(n,c0,c1)	CARD_DECL(I101_CARD_TYPE_IO,n,c0,c1)
#define CARD_DECL_MVB(n,c0,c1)	CARD_DECL(I101_CARD_TYPE_MVB,n,c0,c1)

const struct i101_card i101_cards[] = {
	CARD_DECL_IO("psu",    0x180, 0x188),
	CARD_DECL_MVB("cpu",   0x000, 0x008),
	CARD_DECL_IO("slot2",  0x010, 0x018),
	CARD_DECL_IO("slot3",  0x020, 0x028),
	CARD_DECL_IO("slot4",  0x030, 0x038),
	CARD_DECL_IO("slot5",  0x040, 0x048),
	CARD_DECL_IO("slot6",  0x050, 0x058),
	CARD_DECL_IO("slot7",  0x060, 0x068),
	CARD_DECL_IO("slot8",  0x070, 0x078),
	CARD_DECL_IO("slot9",  0x080, 0x088),
	CARD_DECL_IO("slot10", 0x090, 0x098),
	CARD_DECL_IO("slot11", 0x0A0, 0x0A8),
	CARD_DECL_IO("slot12", 0x0B0, 0x0B8),
	CARD_DECL_IO("slot13", 0x0C0, 0x0C8),
	CARD_DECL_IO("slot14", 0x0D0, 0x0D8),
	CARD_DECL_IO("slot15", 0x0E0, 0x0E8),
	CARD_DECL_IO("slot16", 0x0F0, 0x0F8),
	CARD_DECL_IO("slot17", 0x100, 0x108),
	CARD_DECL_IO("slot18", 0x110, 0x118),
	CARD_DECL_IO("slot19", 0x120, 0x128),
	CARD_DECL_IO("slot20", 0x130, 0x138),
	CARD_DECL_IO("slot21", 0x140, 0x148),
	{{ 0 }}
};

const int i101_cards_max = ARRAY_SIZE(i101_cards);
