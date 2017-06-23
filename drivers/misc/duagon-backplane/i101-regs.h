#ifndef __DUAGON_I101_REGS_H
#define __DUAGON_I101_REGS_H

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
