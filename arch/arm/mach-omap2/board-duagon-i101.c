/*
 * Code for AM335X EVM.
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_data/at24.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/pwm_backlight.h>
#include <linux/reboot.h>
#include <linux/pwm.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/regulator/machine.h>
#include <linux/usb/musb.h>
#include <linux/clkdev.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-duagon-i101.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/io.h>

#include <plat/irqs-33xx.h>

#include "board-flash.h"
#include "mux.h"
#include "hsmmc.h"
#include "am33xx.h"

#include <linux/sched.h>
#include <linux/signal.h>
#include <asm/irq.h>
#include <asm/atomic.h>

#include <linux/highmem.h>
#include <asm/kmap_types.h>

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

/* TLK PHY IDs */
#define TLK110_PHY_ID		0x2000A201
#define TLK110_PHY_MASK		0xfffffff0

/* BBB PHY IDs */
#define BBB_PHY_ID		0x7c0f1
#define BBB_PHY_MASK		0xfffffffe

/* LCD backlight platform Data */
#define AM335X_PWM_PERIOD_NANO_SECONDS		(5000 * 10)

#include "common.h"

#include <linux/lis3lv02d.h>

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

struct evm_dev_cfg {
	void (*device_init)(int evm_id, int profile);

/*
* If the device is required on both baseboard & daughter board (ex i2c),
* specify DEV_ON_BASEBOARD
*/
#define DEV_ON_BASEBOARD	0
#define DEV_ON_DGHTR_BRD	1
	u32 device_on;

	u32 profile;	/* Profiles (0-7) in which the module is present */
};

/* AM335X - CPLD Register Offsets */
#define CPLD_DEVICE_HDR	0x00 /* CPLD Header */
#define CPLD_DEVICE_ID	0x04 /* CPLD identification */
#define CPLD_DEVICE_REV	0x0C /* Revision of the CPLD code */
#define CPLD_CFG_REG	0x10 /* Configuration Register */

static struct i2c_client *cpld_client;
static u32 am335x_evm_id;

/*
* EVM Config held in On-Board eeprom device.
*
* Header Format
*
*  Name			Size	Contents
*			(Bytes)
*-------------------------------------------------------------
*  Header		4	0xAA, 0x55, 0x33, 0xEE
*
*  Board Name		8	Name for board in ASCII.
*				example "A33515BB" = "AM335X
*				Low Cost EVM board"
*
*  Version		4	Hardware version code for board in
*				in ASCII. "1.0A" = rev.01.0A
*
*  Serial Number	12	Serial number of the board. This is a 12
*				character string which is WWYY4P16nnnn, where
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*
*  Configuration option	32	Codes(TBD) to show the configuration
*				setup on this board.
*
*  Available		32720	Available space for other non-volatile
*				data.
*/
struct am335x_evm_eeprom_config {
	u32	header;
	u8	name[8];
	char	version[4];
	u8	serial[12];
	u8	opt[32];
};

/*
* EVM Config held in daughter board eeprom device.
*
* Header Format
*
*  Name			Size		Contents
*			(Bytes)
*-------------------------------------------------------------
*  Header		4	0xAA, 0x55, 0x33, 0xEE
*
*  Board Name		8	Name for board in ASCII.
*				example "A335GPBD" = "AM335x
*				General Purpose Daughterboard"
*
*  Version		4	Hardware version code for board in
*				in ASCII. "1.0A" = rev.01.0A
*  Serial Number	12	Serial number of the board. This is a 12
*				character string which is: WWYY4P13nnnn, where
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*  Configuration Option	32	Codes to show the configuration
*				setup on this board.
*  CPLD Version	8		CPLD code version for board in ASCII
*				"CPLD1.0A" = rev. 01.0A of the CPLD
*  Available	32700		Available space for other non-volatile
*				codes/data
*/

struct am335x_eeprom_config1 {
	u32	header;
	u8	name[8];
	char	version[4];
	u8	serial[12];
	u8	opt[32];
	u8	cpld_ver[8];
};

static struct am335x_evm_eeprom_config config;
static struct am335x_eeprom_config1 config1;
static bool daughter_brd_detected;

#define GP_EVM_REV_IS_1_0		0x1
#define GP_EVM_REV_IS_1_1A		0x2
#define GP_EVM_REV_IS_UNKNOWN		0xFF

#define CPLD_REV_1_0A			0x1
#define CPLD_REV_1_1A			0x2
#define CPLD_UNKNOWN			0xFF
static unsigned int cpld_version = CPLD_UNKNOWN;

#define EEPROM_MAC_ADDRESS_OFFSET	60 /* 4+8+4+12+32 */
#define EEPROM_NO_OF_MAC_ADDR		3
static char am335x_mac_addr[EEPROM_NO_OF_MAC_ADDR][ETH_ALEN];

#define AM335X_EEPROM_HEADER		0xEE3355AA

/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);
}

/*
* @evm_id - evm id which needs to be configured
* @dev_cfg - single evm structure which includes
*				all module inits, pin-mux defines
* @profile - if present, else PROFILE_NONE
* @dghtr_brd_flg - Whether Daughter board is present or not
*/
static void _configure_device(int evm_id, struct evm_dev_cfg *dev_cfg,
	int profile)
{
	int i;

	/*
	* Only General Purpose & Industrial Auto Motro Control
	* EVM has profiles. So check if this evm has profile.
	* If not, ignore the profile comparison
	*/

	/*
	* If the device is on baseboard, directly configure it. Else (device on
	* Daughter board), check if the daughter card is detected.
	*/
	if (profile == PROFILE_NONE) {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->device_on == DEV_ON_BASEBOARD)
				dev_cfg->device_init(evm_id, profile);
			else if (daughter_brd_detected == true)
				dev_cfg->device_init(evm_id, profile);
		}
	} else {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->profile & profile) {
				if (dev_cfg->device_on == DEV_ON_BASEBOARD)
					dev_cfg->device_init(evm_id, profile);
				else if (daughter_brd_detected == true)
					dev_cfg->device_init(evm_id, profile);
			}
		}
	}
}

/* pinmux for usb0 drvvbus */
static struct pinmux_config usb0_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for usb1 drvvbus */
static struct pinmux_config usb1_pin_mux[] = {
	{"usb1_drvvbus.usb1_drvvbus",		OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

static void usb0_init(int evm_id, int profile)
{
	setup_pin_mux(usb0_pin_mux);
	return;
}

static void usb1_init(int evm_id, int profile)
{
	setup_pin_mux(usb1_pin_mux);
	return;
}

static void d_can_init(int evm_id, int profile)
{
	am33xx_d_can_init(1);
	pr_info("CAN interface can1 init'ed on uart1_rxd/txd\n");
}

/* NAND partition information */
static struct mtd_partition am335x_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name	= "/dev/mtd10 U Boot SPL 1",
		.offset	= 0,			/* Offset = 0x0 */
		.size	= SZ_512K,
	},
	{
		.name	= "/dev/mtd11 U-Boot 1",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x080000 */
		.size	= SZ_512K,
	},
	{
		.name	= "/dev/mtd12 U-Boot Env 1_1",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x100000 */
		.size	= SZ_512K,
	},
	{
		.name	= "/dev/mtd13 U-Boot Env 1_2",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x180000 */
		.size	= SZ_512K,
	},
	{
		.name	= "/dev/mtd14 Kernel 1",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x200000 */
		.size	= 28 * SZ_512K,
	},
	{
		.name	= "/dev/mtd15 File System",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x02000000 */
		.size	= 992 * SZ_512K,	/* size   = 0x40000000, 480 MBytes */
	},
	{
		.name	= "/dev/mtd16 U Boot SPL 2",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x20000000 */
		.size	= SZ_512K,
	},
	{
		.name	= "/dev/mtd17 U-Boot 2",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x20080000 */
		.size	= SZ_512K,
	},
	{
		.name	= "/dev/mtd18 U-Boot Env 2_1",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x20100000 */
		.size	= SZ_512K,
	},
	{
		.name	= "/dev/mtd19 U-Boot Env 2_2",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x20180000 */
		.size	= SZ_512K,
	},
	{
		.name	= "/dev/mtd20 Kernel 2",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x20200000 */
		.size	= 28 * SZ_512K,
	},
	{
		.name	= "/dev/mtd21 File System",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x22000000, 480 MBytes */
		.size	= MTDPART_SIZ_FULL,	/* size   = 0x40000000 1GByte */
	}
	/*Check if Bad Block Table is at the end*/
};

/* SPI 0/1 Platform Data */
/* SPI flash information */
static struct mtd_partition am335x_spi_partitions[] = {
	/* All the partition sizes are listed in terms of erase size */
	{
		.name	= "/dev/mtd0 U Boot SPL 1",
		.offset	= 0,			/* Offset = 0x0 */
		.size	= SZ_128K,
	},
	{
		.name	= "/dev/mtd1 U-Boot 1",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size	= SZ_256K,
	},
	{
		.name	= "/dev/mtd2 U-Boot Env 1",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size	= SZ_64K,
	},
	{
		.name	= "/dev/mtd3 U-Boot Env 2",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x70000 */
		.size	= SZ_64K,
	},
	{
		.name	= "/dev/mtd4 Kernel 1",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size	= 56 * SZ_64K,
	},
	{
		.name	= "/dev/mtd5 U Boot SPL 2",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x400000 */
		.size	= SZ_128K,
	},
	{
		.name	= "/dev/mtd6 U-Boot 2",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x420000 */
		.size	= SZ_256K,
	},
	{
		.name	= "/dev/mtd7 U-Boot Env 3",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x460000 */
		.size	= SZ_64K,
	},
	{
		.name	= "/dev/mtd8 U-Boot Env 4",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x470000 */
		.size	= SZ_64K,
	},
	{
		.name	= "/dev/mtd9 Kernel 2",
		.offset	= MTDPART_OFS_APPEND,	/* Offset = 0x480000 */
		.size	= MTDPART_SIZ_FULL,	/* size ~ = 0x800000 */
	}
};

static const struct flash_platform_data am335x_spi_flash = {
	.type		= "w25q64",
	.name		= "spi_flash",
	.parts		= am335x_spi_partitions,
	.nr_parts	= ARRAY_SIZE(am335x_spi_partitions),
};

/*
 * SPI Flash works at 80Mhz however SPI Controller works at 48MHz.
 * So setup Max speed to be less than that of Controller speed
 */
static struct spi_board_info am335x_spi0_slave_info[] = {
	{
		.modalias	= "m25p80",
		.platform_data	= &am335x_spi_flash,
		.irq		= -1,
		.max_speed_hz	= 24000000,
		.bus_num	= 1,
		.chip_select	= 0,
	},
};

static struct gpmc_timings am335x_nand_timings = {
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 44,
	.cs_wr_off = 44,

	.adv_on = 6,
	.adv_rd_off = 34,
	.adv_wr_off = 44,
	.we_off = 40,
	.oe_off = 54,

	.access = 64,
	.rd_cycle = 82,
	.wr_cycle = 82,

	.wr_access = 40,
	.wr_data_mux_bus = 0,
};

static void duagon_i101_gpmc_init(int evm_id, int profile)
{
	/** gpmc pinmux settings are configured by gpmc/nand driver from DT **/

	pr_info ("board-duagon-101.c: board_nand_init() start\n");

	board_nand_init(am335x_nand_partitions,
		ARRAY_SIZE(am335x_nand_partitions), 0, 0,
		&am335x_nand_timings);

	// FIXME: configure nand to use elm and set ecc_opt to OMAP_ECC_BCH8_CODE_HW
	// elm is initialized via DT
}

#define GPMC_INT_NIOS 0x0001
#define GPMC_INT_48V  0x8000

static void nNMI_interrupt_init(int evm_id, int profile)
{
}

static struct i2c_board_info am335x_i2c_boardinfo2[] = {
};

static void i2c2_init(int evm_id, int profile)
{
	/* pinmux set via DT */
	omap_register_i2c_bus(3, 100, am335x_i2c_boardinfo2,
			ARRAY_SIZE(am335x_i2c_boardinfo2));
	return;
}

/* setup spi0 */
static void spi0_init(int evm_id, int profile)
{
	/* pinmux done in DT */
	spi_register_board_info(am335x_spi0_slave_info,
			ARRAY_SIZE(am335x_spi0_slave_info));
	return;
}

/* Duagon i101 */
static struct evm_dev_cfg duagon_i101_dev_cfg[] = {
	{usb0_init,		DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb1_init,		DEV_ON_BASEBOARD, PROFILE_NONE},
	{i2c2_init,		DEV_ON_BASEBOARD, PROFILE_NONE},
	{duagon_i101_gpmc_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{spi0_init,		DEV_ON_BASEBOARD, PROFILE_NONE},
	{nNMI_interrupt_init,	DEV_ON_BASEBOARD, PROFILE_NONE},
	{d_can_init,		DEV_ON_BASEBOARD, PROFILE_NONE},
	{NULL, 0, 0},
};

/* Duagon i101 setup */
static void setup_duagon_i101(void)
{
	pr_info("The board is a AM335x Duagon i101.\n");

	_configure_device(LOW_COST_EVM, duagon_i101_dev_cfg, PROFILE_NONE);

	/* Fill up global evmid */
	am33xx_evmid_fillup(duagon_i10x_mii);
}

static void am335x_setup_daughter_board(struct memory_accessor *m, void *c)
{
	int ret;

	/*
	 * Read from the EEPROM to see the presence
	 * of daughter board. If present, get daughter board
	 * specific data.
	 */

	ret = m->read(m, (char *)&config1, 0, sizeof(config1));
	if (ret == sizeof(config1)) {
		pr_info("Detected a daughter card on AM335x EVM..");
		daughter_brd_detected = true;
	}
	else {
		pr_info("No daughter card found\n");
		daughter_brd_detected = false;
		return;
	}

	if (!strncmp("CPLD1.0A", config1.cpld_ver, 8))
		cpld_version = CPLD_REV_1_0A;
	else if (!strncmp("CPLD1.1A", config1.cpld_ver, 8))
		cpld_version = CPLD_REV_1_1A;
	else {
		pr_err("Unknown CPLD version found, falling back to 1.0A\n");
		cpld_version = CPLD_REV_1_0A;
	}
}

static void am335x_evm_setup(struct memory_accessor *mem_acc, void *context)
{
	char tmp[10];

	/* 1st get the MAC address from EEPROM */

	am335x_mac_addr[0][0] = 0x01;
	am335x_mac_addr[0][1] = 0x02;
	am335x_mac_addr[0][2] = 0x03;
	am335x_mac_addr[0][3] = 0x04;
	am335x_mac_addr[0][4] = 0x05;
	am335x_mac_addr[0][5] = 0x06;

	am335x_mac_addr[1][0] = 0x01;
	am335x_mac_addr[1][1] = 0x02;
	am335x_mac_addr[1][2] = 0x03;
	am335x_mac_addr[1][3] = 0x04;
	am335x_mac_addr[1][4] = 0x05;
	am335x_mac_addr[1][5] = 0x06;

	am335x_mac_addr[2][0] = 0x01;
	am335x_mac_addr[2][1] = 0x02;
	am335x_mac_addr[2][2] = 0x03;
	am335x_mac_addr[2][3] = 0x04;
	am335x_mac_addr[2][4] = 0x05;
	am335x_mac_addr[2][5] = 0x06;

	/* get board specific data */
	config.header     = AM335X_EEPROM_HEADER;
	config.name[0]    = 'i';
	config.name[1]    = '1';
	config.name[2]    = '0';
	config.name[3]    = 'x';
	config.name[4]    = 'M';
	config.name[5]    = 'I';
	config.name[6]    = 'I';
	config.name[7]    = ' ';

	config.version[0] = '0';
	config.version[1] = '0';
	config.version[2] = '0';
	config.version[3] = 'x';

	config.serial [0] = '0';
	config.serial [1] = '8';
	config.serial [2] = '1';
	config.serial [3] = '2';
	config.serial [4] = '4';
	config.serial [5] = 'P';
	config.serial [6] = '1';
	config.serial [7] = '6';
	config.serial [8] = '0';
	config.serial [9] = '0';
	config.serial[10] = '0';
	config.serial[11] = '1';

	config.opt[ 0]    = 'T';
	config.opt[ 1]    = 'e';
	config.opt[ 2]    = 's';
	config.opt[ 3]    = 't';
	config.opt[ 4]    = '\0';
	config.opt[ 5]    = '\0';
	config.opt[ 6]    = '\0';
	config.opt[ 7]    = '\0';
	config.opt[ 8]    = '\0';
	config.opt[ 9]    = '\0';
	config.opt[10]    = '\0';
	config.opt[11]    = '\0';
	config.opt[12]    = '\0';
	config.opt[13]    = '\0';
	config.opt[14]    = '\0';
	config.opt[15]    = '\0';
	config.opt[16]    = '\0';
	config.opt[17]    = '\0';
	config.opt[18]    = '\0';
	config.opt[19]    = '\0';
	config.opt[20]    = '\0';
	config.opt[21]    = '\0';
	config.opt[22]    = '\0';
	config.opt[23]    = '\0';
	config.opt[24]    = '\0';
	config.opt[25]    = '\0';
	config.opt[26]    = '\0';
	config.opt[27]    = '\0';
	config.opt[28]    = '\0';
	config.opt[29]    = '\0';
	config.opt[30]    = '\0';
	config.opt[31]    = '\0';

	snprintf(tmp, sizeof(config.name) + 1, "%s", config.name);
	pr_info("Board name: %s\n", tmp);
	snprintf(tmp, sizeof(config.version) + 1, "%s", config.version);
	pr_info("Board version: %s\n", tmp);

	daughter_brd_detected = false;
	setup_duagon_i101();

	/* Initialize cpsw after board detection is completed as board
	 * information is required for configuring phy address and hence
	 * should be call only after board detection
	 */
	// FIXME: needs to be done via DT

	return;
}

static struct at24_platform_data am335x_daughter_board_eeprom_info = {
	.byte_len	= (256*1024) / 8,
	.page_size	= 64,
	.flags		= AT24_FLAG_ADDR16,
	.setup		= am335x_setup_daughter_board,
	.context	= (void *)NULL,
};

static struct at24_platform_data am335x_baseboard_eeprom_info = {
	.byte_len	= (256*1024) / 8,
	.page_size	= 64,
	.flags		= AT24_FLAG_ADDR16,
	.setup		= am335x_evm_setup,
	.context	= (void *)NULL,
};

static struct regulator_init_data am335x_dummy = {
	.constraints.always_on	= true,
};

static struct regulator_consumer_supply am335x_vdd1_supply[] = {
	REGULATOR_SUPPLY("vdd_mpu", NULL),
};

static struct regulator_init_data am335x_vdd1 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd1_supply),
	.consumer_supplies	= am335x_vdd1_supply,
};

static struct regulator_consumer_supply am335x_vdd2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_init_data am335x_vdd2 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd2_supply),
	.consumer_supplies	= am335x_vdd2_supply,
};

static struct tps65910_board am335x_tps65910_info = {
	.tps65910_pmic_init_data[TPS65910_REG_VRTC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VIO]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD1]	= &am335x_vdd1,
	.tps65910_pmic_init_data[TPS65910_REG_VDD2]	= &am335x_vdd2,
	.tps65910_pmic_init_data[TPS65910_REG_VDD3]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VPLL]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDAC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX33]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VMMC]	= &am335x_dummy,
};

/*
* Daughter board Detection.
* Every board has a ID memory (EEPROM) on board. We probe these devices at
* machine init, starting from daughter board and ending with baseboard.
* Assumptions :
*	1. probe for i2c devices are called in the order they are included in
*	   the below struct. Daughter boards eeprom are probed 1st. Baseboard
*	   eeprom probe is called last.
*/
static struct i2c_board_info __initdata am335x_i2c_boardinfo[] = {
	{
		/* Daughter Board EEPROM */
		I2C_BOARD_INFO("24c256", DAUG_BOARD_I2C_ADDR),
		.platform_data = &am335x_daughter_board_eeprom_info,
	},
	{
		/* Baseboard board EEPROM */
		I2C_BOARD_INFO("24c256", BASEBOARD_I2C_ADDR),
		.platform_data = &am335x_baseboard_eeprom_info,
	},
	{
		I2C_BOARD_INFO("cpld_reg", 0x35),
	},
	{
		I2C_BOARD_INFO("tlc59108", 0x40),
	},
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1),
		.platform_data = &am335x_tps65910_info,
	},
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * AM335X beta EVM has USB0 in OTG mode and USB1 in host mode.
	 */
	.mode		= MUSB_OTG,
	.power		= 500,
	.instances	= 0,
};

static int cpld_reg_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	cpld_client = client;
	return 0;
}

static int cpld_reg_remove(struct i2c_client *client)
{
	cpld_client = NULL;
	return 0;
}

static const struct i2c_device_id cpld_reg_id[] = {
	{ "cpld_reg", 0 },
	{ }
};

static struct i2c_driver cpld_reg_driver = {
	.driver = {
		.name	= "cpld_reg",
	},
	.probe		= cpld_reg_probe,
	.remove		= cpld_reg_remove,
	.id_table	= cpld_reg_id,
};

static void evm_init_cpld(void)
{
	i2c_add_driver(&cpld_reg_driver);
}

static void __init am335x_evm_i2c_init(void)
{
	/* Initially assume Low Cost EVM Config */
	am335x_evm_id = LOW_COST_EVM;

	evm_init_cpld();

	omap_register_i2c_bus(1, 100, am335x_i2c_boardinfo,
				ARRAY_SIZE(am335x_i2c_boardinfo));
}

static struct resource am335x_rtc_resources[] = {
	{
		.start		= AM33XX_RTC_BASE,
		.end		= AM33XX_RTC_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
	{ /* timer irq */
		.start		= AM33XX_IRQ_RTC_TIMER,
		.end		= AM33XX_IRQ_RTC_TIMER,
		.flags		= IORESOURCE_IRQ,
	},
	{ /* alarm irq */
		.start		= AM33XX_IRQ_RTC_ALARM,
		.end		= AM33XX_IRQ_RTC_ALARM,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device am335x_rtc_device = {
	.name		= "omap_rtc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am335x_rtc_resources),
	.resource	= am335x_rtc_resources,
};

static int am335x_rtc_init(void)
{
	void __iomem *base;
	struct clk *clk;

	clk = clk_get(NULL, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc : Failed to get RTC clock\n");
		return -1;
	}

	if (clk_enable(clk)) {
		pr_err("rtc: Clock Enable Failed\n");
		return -1;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);

	if (WARN_ON(!base))
		return -ENOMEM;

	/* Unlock the rtc's registers */
	writel(0x83e70b13, base + 0x6c);
	writel(0x95a4f1e0, base + 0x70);

	/*
	 * Enable the 32K OSc
	 * TODO: Need a better way to handle this
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 */
	writel(0x48, base + 0x54);

	iounmap(base);

	return platform_device_register(&am335x_rtc_device);
}

/* Enable clkout2 */
static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2",	OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void __init clkout2_enable(void)
{
	struct clk *ck_32;

	ck_32 = clk_get(NULL, "clkout2_ck");
	if (IS_ERR(ck_32)) {
		pr_err("Cannot clk_get ck_32\n");
		return;
	}

	clk_enable(ck_32);

	setup_pin_mux(clkout2_pin_mux);
}

static void __init am335x_evm_init(void)
{
	am33xx_mux_init(board_mux);
	omap_serial_init();
	am335x_rtc_init();
	clkout2_enable();
	am335x_evm_i2c_init();
	omap_sdrc_init(NULL, NULL);
	pr_err("TEST: usb_musb_init()\n");
	usb_musb_init(&musb_board_data);
	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
}

MACHINE_START(AM335XEVM, "am335xevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am33xx_map_io,
	.init_early	= am33xx_init_early,
	.init_time	= omap3_gptimer_timer_init,
	.init_machine	= am335x_evm_init,
MACHINE_END
