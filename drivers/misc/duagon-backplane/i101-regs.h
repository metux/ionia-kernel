/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_I101_REGS_H
#define __DUAGON_I101_REGS_H

/* registers are word-aligned - IOW: byte offset = regid * 2 */
#define IONIA_BACKPLANE_REG_BURNIN              0x04
#define IONIA_BACKPLANE_REG_LOOPBACK            0x16

/** register IDs for G2 backplane -- word-aligned **/

/* read and write register (base address) */
#define IONIA_BACKPLANE_G2_REG_DATA		0x00

/* read current value w/o consumption */
#define IONIA_BACKPLANE_G2_REG_RX_CUR		0x01

/* bytes in rx fifo */
#define IONIA_BACKPLANE_G2_REG_RX_AVAIL		0x02

/* bytes in tx fifo */
#define IONIA_BACKPLANE_G2_REG_TX_AVAIL		0x03

/* channel config register */
#define IONIA_BACKPLANE_G2_REG_CONFIG		0x04

/* line status */
#define IONIA_BACKPLANE_G2_REG_LINESTAT		0x05

/* unused */
#define IONIA_BACKPLANE_G2_REG_UNUSED0		0x06

/* fifo word size */
#define IONIA_BACKPLANE_G2_REG_WORDSIZE		0x07

#endif
