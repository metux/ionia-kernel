/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_IONIA_MODULES_H
#define __DUAGON_IONIA_MODULES_H

typedef enum {
	IONIA_MODULE_TYPE_MVB		= 0,
	IONIA_MODULE_TYPE_UNUSED0	= 1,
	IONIA_MODULE_TYPE_SOCKETS	= 2,
	IONIA_MODULE_TYPE_IPTCOM	= 3,
	IONIA_MODULE_TYPE_CO		= 4,
	IONIA_MODULE_TYPE_IO		= 5,
	IONIA_MODULE_TYPE_AI		= 6,
	IONIA_MODULE_TYPE_SER_COM	= 7,
	IONIA_MODULE_TYPE_AO		= 9,
	IONIA_MODULE_TYPE_MVB_COM	= 10,
	IONIA_MODULE_TYPE_SOC_COM	= 11,
	IONIA_MODULE_TYPE_POWER		= 12,
	IONIA_MODULE_TYPE_LOG		= 13,
	IONIA_MODULE_TYPE_NONE		= 0xFF,
} ionia_module_type_t;

#endif /* __DUAGON_IONIA_MODULES_H */
