#ifndef __DUAGON_I101_CHANNELS_H
#define __DUAGON_I101_CHANNELS_H

#define I101_HOST_BASE_CHANNEL_MAX	2
#define I101_HOST_BASE_CHANNEL_NONE	0x0
#define I101_CARD_MAX			21

enum i101_card_type {
	I101_CARD_TYPE_IO  = 1,
	I101_CARD_TYPE_MVB = 2
};

struct i101_card
{
	/* register base IDs (not byte offsets) of the channels */
	int host_base_channel[I101_HOST_BASE_CHANNEL_MAX];
	int mvb_channel;
	enum i101_card_type type;
	const char* name;
};

const extern struct i101_card i101_cards[];
const extern int i101_cards_max;

#endif /* __DUAGON_I101_CHANNELS_H */
