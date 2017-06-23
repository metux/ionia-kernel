/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_CARDS_H
#define __DUAGON_CARDS_H

#define IONIA_HOST_BASE_CHANNEL_MAX	2

enum i101_card_type {
	I101_CARD_TYPE_IO	= 1,
	I101_CARD_TYPE_MVB	= 2
};

struct i101_card
{
	/* register base IDs (not byte offsets) of the channels */
	int host_base_channel[IONIA_HOST_BASE_CHANNEL_MAX];
	int mvb_channel;
	enum i101_card_type type;
	const char* name;
};

const extern struct i101_card i101_cards[];
const extern int i101_cards_max;

struct ionia_backplane_platform_data;

uint16_t ionia_card_getreg(struct ionia_backplane_platform_data *pdata, int card, int reg);
void     ionia_card_setreg(struct ionia_backplane_platform_data *pdata, int card, uint16_t reg, uint16_t val);

#endif /* __DUAGON_CARDS_H */
