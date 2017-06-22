/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_IONIA_PROTOCOL_H
#define __DUAGON_IONIA_PROTOCOL_H

typedef enum {
	IONIA_PROTOCOL_PFS	 = 0x01,
	IONIA_PROTOCOL_SOCKETS	 = 0x02,
	IONIA_PROTOCOL_IPTCOM_PD = 0x03,
	IONIA_PROTOCOL_IPTCOM_MD = 0x04,
	IONIA_PROTOCOL_IO	 = 0x06,
	IONIA_PROTOCOL_AI	 = 0x07,
	IONIA_PROTOCOL_AO	 = 0x07,
	IONIA_PROTOCOL_LOG	 = 0x08,
	IONIA_PROTOCOL_SER_COM	 = 0x09,
	IONIA_PROTOCOL_SVC	 = 0x0F,
	IONIA_PROTOCOL_MVB	 = 0x10,
	IONIA_PROTOCOL_WTB	 = 0x11,
	IONIA_PROTOCOL_POWER	 = 0x12,
} ionia_protocol_t;

#endif /* __DUAGON_IONIA_PROTOCOL_H */
