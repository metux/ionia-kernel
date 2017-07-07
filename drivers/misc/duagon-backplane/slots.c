/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

//#include <linux/bug.h>
#include <linux/kernel.h>

#include "ionia-slots.h"

#define SLOT_DECL(n,b) 	\
	{			\
		.name = n,	\
		.base = b,	\
	}

#define SLOT_DECL_IO(n,c0)	SLOT_DECL(n,c0)

struct ionia_slot ionia_slots[] = {
	SLOT_DECL("BP channel 0",  0x000),
	SLOT_DECL("BP channel 1x", 0x010),
	SLOT_DECL("BP channel 1",  0x020),
	SLOT_DECL("BP slot 2x",    0x030),
	SLOT_DECL("BP slot 2",    0x040),
	SLOT_DECL("BP slot 3",    0x050),
	SLOT_DECL("BP slot 3",    0x060),
	SLOT_DECL("BP slot 4",    0x070),
	SLOT_DECL("BP slot 4",    0x080),
	SLOT_DECL("BP slot 5",    0x090),
	SLOT_DECL("BP slot 5",    0x0a0),
	SLOT_DECL("BP slot 6",    0x0b0),
	SLOT_DECL("BP slot 6",    0x0c0),
	SLOT_DECL("BP slot 7",    0x0d0),
	SLOT_DECL("BP slot 7",    0x0e0),
	SLOT_DECL("BP slot 8",    0x0f0),
	SLOT_DECL("BP@0x100",     0x100),
	SLOT_DECL("BP slot 8",    0x110),
	SLOT_DECL("BP slot 9",    0x120),
	SLOT_DECL("BP slot 9",    0x130),
	SLOT_DECL("BP slot 10",   0x140),
	SLOT_DECL("BP slot 10",   0x150),
	SLOT_DECL("BP slot 10",   0x160),
	SLOT_DECL("BP slot 10",   0x170),
	SLOT_DECL("BP slot 10",   0x180),
	SLOT_DECL("BP @190",      0x190),
};

const int ionia_slots_max = ARRAY_SIZE(ionia_slots);
