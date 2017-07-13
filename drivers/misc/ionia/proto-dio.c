/*
 * Duagon Ionia GPIO card protocol support
 *
 * Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>

#include "ionia-rpc.h"
#include "ionia-proto-io.h"

#define IO_DRIVER_VERSION		0x00050001	/* 0xMMMMmmmm MMMM=major=1 mmmm=minor=1 */
#define LOG_SENSOR_DATA_WORD_SIZE	120		/* sensor data size */

#define IO_MIN_PROT_VERSION_MAJOR	1 /* io protocol on the device must have at least this major value */
#define IO_MIN_PROT_VERSION_MINOR	1 /* io protocol on the device must have at least this minor value */

#define IONIA_RPC_BEGIN_IO(cmd)	IONIA_RPC_BEGIN(IONIA_PROTOCOL_IO,cmd)

int ionia_io_init(ionia_rpc_t *rpc)
{
	u32 version;

	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_INIT);
	IONIA_RPC_PAR_U32(IO_DRIVER_VERSION);
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_RET_U32(&version)

	pr_info("%s: version=%08X\n", __func__, version);

	IONIA_RPC_END

	return rpc_errno;
}

int ionia_io_out_set(ionia_rpc_t *rpc, u16 state, u16 mask)
{
	u16 map[] = { 0x0001, 0x0000, 0x0002, 0x0000,
		      0x0004, 0x0000, 0x0008, 0x0000,
		      0x0010, 0x0000, 0x0020, 0x0000,
		      0x0040, 0x0000, 0x0080, 0x0000 };
	int io_chan;
	u16 io_state = 0;
	u16 io_mask = 0;

	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_INIT);

	for (io_chan=0; io_chan<16; io_chan++)
	{
		if (state & BIT(io_chan))
			io_state |= map[io_chan];

		if (mask & BIT(io_chan))
			io_mask |= map[io_chan];
	}

	IONIA_RPC_PAR_U32(io_mask + (io_state << 16));
	IONIA_RPC_CALL_ERRNO

	pr_info("%s: io_state=%08X io_mask=%08X errno=%08X\n", __func__, io_state, io_mask, rpc_errno);

	IONIA_RPC_END
	return rpc_errno;
}

int ionia_io_in_get(ionia_rpc_t *rpc, int io_type, u16* io_state)
{
	u8 map_normal[] =      {2, 4, 6, 8, 10, 12, 14, 16, 1, 3, 5, 7, 9, 11, 13, 15}; /* i202 */
	u8 map_transparent[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
				17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
	u8 *map;
	u8 bitnr;
	u32 state;

	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_IN_GET);

	switch (io_type)
	{
		case IONIA_IO_MAP_TYPE_NORMAL:
			map = map_normal;
			break;
		case IONIA_IO_MAP_TYPE_TRANSPARENT:
		default:
			map = map_transparent;
			break;
	}

	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_RET_U32(&state);

	// FIXME: check rpc_errno

	*io_state = 0;
	for (bitnr=0; bitnr<16; bitnr++ )
	{
		if (state & BIT(bitnr))
			*io_state |= 1 << (map[bitnr]-1);
	}

	IONIA_RPC_END
	return 0;
}

int ionia_io_pwm_enable(ionia_rpc_t *rpc, u16 state, u16 mask)
{
	u32 package = 0;
	int i;

	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_PWM_ENABLE)

	for (i=0; i<8; i++) {
		if (mask & BIT(2*i))
			package += 1 << i;

		if (state & BIT(2*i))
			package += 0x10000 << i;
	}

	IONIA_RPC_PAR_U32(package);
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_END
	return 0;
}

// group in [0..1]
int ionia_io_pwm_freq_set(ionia_rpc_t *rpc, u8 group, u16 freq)
{
	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_PWM_PERIOD_SET);

	if (group > 1) {
		pr_info("%s: group out of range: %d\n", __func__, group);
		return -EINVAL;
	}

	if ((freq < 25) || (freq > 10000)) {
		pr_info("%s: freqency out of range: %d\n", __func__, freq);
		return -EINVAL;
	}

	IONIA_RPC_PAR_U32((group + ((1000 * 1000/freq) << 16)));
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_END
}

int ionia_io_pwm_dutycycle_set(ionia_rpc_t *rpc, u8 chan, u16 duty_cycle, u16 freq)
{
	u8 map[] = {0,255,1,255,2,255,3,255,4,255,5,255,6,255,7,255};

	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_PWM_PULS_WIDTH_SET);

	if (chan > 15) {
		pr_info("%s: channel number too large: %d\n", __func__, chan);
		return -EINVAL;
	}

	if (freq < 25) {
		pr_info("%s: freq too low: %d\n", __func__, freq);
		return -EINVAL;
	}

	if (freq > 10000) {
		pr_info("%s: freq too high: %d\n", __func__, freq);
		return -EINVAL;
	}

	if (duty_cycle > 100) {
		pr_info("%s: duty_cycle too high: %d\n", __func__, duty_cycle);
		return -EINVAL;
	}

	IONIA_RPC_PAR_U32(map[chan] + (((1000 * 1000/freq) * duty_cycle/100) << 16));
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_END

	return 0;
}

static int __pwm_pulse_width_set(ionia_rpc_t *rpc, u8 chan, u16 width_us, u16 freq)
{
	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_PWM_PULS_WIDTH_SET);
	IONIA_RPC_PAR_U32(chan + (width_us << 16));
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_END
	return 0;
}

int ionia_io_pwm_pulse_width_set(ionia_rpc_t *rpc, u8 chan, u16 width_us, u16 freq)
{
	u8 map[] = {0,255,1,255,2,255,3,255,4,255,5,255,6,255,7,255};
	u16 upper_limit = (u16)(1000000/freq) - 10;

	if (chan > 15) {
		pr_info("%s: channel number too large: %d\n", __func__, chan);
		return -EINVAL;
	}

	if (width_us < 10) {
		pr_info("%s: pulse width too short: %d\n", __func__, width_us);
		return -EINVAL;
	}

	if (width_us > upper_limit) {
		pr_info("%s: pulse width too long: %d\n", __func__, width_us);
		return -EINVAL;
	}

	return __pwm_pulse_width_set(rpc, map[chan], width_us, freq);
}

int ionia_io_led_config(ionia_rpc_t *rpc, u8 is_user)
{
	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_LED_CONFIG);
	IONIA_RPC_PAR_U32((is_user ? 1 : 0));
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_END
	return 0;
}

int ionia_io_led_set(ionia_rpc_t *rpc, u16 state, u16 mask)
{
	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_LED_SET);
	IONIA_RPC_PAR_U32(mask + (state << 16));
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_END
	return 0;
}

int ionia_io_hci_enable(ionia_rpc_t *rpc, u16 state, u16 mask)
{
	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_HCI_ENABLE);
	IONIA_RPC_PAR_U32(mask + (state << 16));
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_END
	return 0;
}

static int __freq_counter_gate_set(ionia_rpc_t *rpc, u8 gate, u8 resolution)
{
	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_FREQ_CNT_GATE_SET);
	IONIA_RPC_PAR_U32((gate+1) + (resolution < 16));
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_END
	return 0;
}

// gate num starts w/ 0
int ionia_io_freq_counter_gate_set(ionia_rpc_t *rpc, u8 gate, u8 resolution)
{
	if (resolution > 3) {
		pr_info("%s: resolution too high: %d\n", __func__, resolution);
		return -EINVAL;
	}

	if (gate > 2) {
		pr_info("%s: gate number too large: %d\n", __func__, gate);
		return -EINVAL;
	}

	return __freq_counter_gate_set(rpc, gate, resolution);
}

int ionia_io_freq_get(ionia_rpc_t *rpc, u8 chan, int io_map, u8 resolution, u32 *freq)
{
	u32 input_nr;
	u32 map_gate_8[] = {0,   8,   1, 9,   2, 10,   3, 11,   4, 12,   5, 13,   6, 14,   7, 15}; /* i20x < rev4*/
	u32 map_gate_4[] = {255, 0, 255, 1, 255,  2, 255,  3, 255,  4, 255,  5, 255,  6, 255, 7};  /* i20x >= rev4*/
	u16 multiplier;

	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_FREQ_CNT_GET);

	if (freq == NULL)
		return -EFAULT;

	switch (resolution)
	{
		case 0:
			multiplier = 1;
			break;
		case 1:
			multiplier = 10;
			break;
		case 2:
			multiplier = 100;
			break;
		case 3:
			multiplier = 1000;
			break;
		default:
			pr_info("%s: bad resolution %d\n", __func__, resolution);
			return -EINVAL;
	}

	if (chan > 15) {
		pr_info("%s: chan too high: %d\n", __func__, chan);
		return -EINVAL;
	}

	switch(io_map)
	{
		case IONIA_IO_MAP_GATE_8:
			input_nr = map_gate_8[chan];
		break;

		case IONIA_IO_MAP_GATE_4:
			input_nr = map_gate_4[chan];
		break;

		default:
			pr_info("%s: unsupported map: %d\n", __func__, io_map);
			return -EINVAL;
		break;
	}

	if (input_nr == 255) {
		pr_info("%s: unsupported input_nr: %d\n", __func__, input_nr);
		return -EINVAL;
	}

	IONIA_RPC_PAR_U32(input_nr);
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_RET_U32(freq);
	IONIA_RPC_END
	return 0;
}

int ionia_io_freq_count_get(ionia_rpc_t *rpc, u8 chan, int io_map, u32 *count)
{
	u32 input_nr = 255;
	u32 map_gate_8[] = {0,   8,   1, 9,   2, 10,   3, 11,   4, 12,   5, 13,   6, 14,   7, 15}; /* i20x < rev4*/
	u32 map_gate_4[] = {255, 0, 255, 1, 255,  2, 255,  3, 255,  4, 255,  5, 255,  6, 255,  7}; /* i20x >= rev4*/

	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_FREQ_CNT_GET);

	if (count == NULL) {
		pr_info("%s: count IS NULL\n", __func__);
		return -EFAULT;
	}

	if (chan > 15)
		return -EINVAL;

	switch(io_map)
	{
		case IONIA_IO_MAP_GATE_8:
			input_nr = map_gate_8[chan];
			break;
		case IONIA_IO_MAP_GATE_4:
			input_nr = map_gate_4[chan];
			break;
		default:
			return -EINVAL;
	}

	if (input_nr == 255)
		return -EINVAL;

	IONIA_RPC_PAR_U32(input_nr);
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_RET_U32(count);
	IONIA_RPC_END
	return 0;
}

static int __period_cnt_get_all(ionia_rpc_t *rpc, u32* channels, u8 num_chan)
{
	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_PERIOD_CNT_GET_ALL);
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_RET_U32(&channels[0]);
	IONIA_RPC_RET_U32(&channels[1]);
	IONIA_RPC_RET_U32(&channels[2]);
	IONIA_RPC_RET_U32(&channels[3]);
	IONIA_RPC_RET_U32(&channels[4]);
	IONIA_RPC_RET_U32(&channels[5]);
	IONIA_RPC_RET_U32(&channels[6]);
	IONIA_RPC_RET_U32(&channels[7]);
	IONIA_RPC_END
	return 0;
}

int ionia_io_period_cnt_get_all(ionia_rpc_t *rpc, u32* channels, u8 num_chan)
{
	if (channels == NULL)
		return -EINVAL;

	if (num_chan != 8)
		return -EINVAL;

	return __period_cnt_get_all(rpc, channels, num_chan);
}

int ionia_io_cnt_timeout_set(ionia_rpc_t *rpc, u8 group, u32 timeout)
{
	IONIA_RPC_BEGIN_IO(IONIA_IO_CMD_PERIOD_CNT_TIMEOUT_SET);
	IONIA_RPC_PAR_U32(timeout & 0x0FFFFFF);
	IONIA_RPC_CALL_ERRNO
	IONIA_RPC_END
	return 0;
}
