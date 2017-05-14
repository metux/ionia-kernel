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
#include <linux/i2c/at24.h>
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
#include <linux/input/ti_tscadc.h>
#include <linux/reboot.h>
#include <linux/pwm.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/regulator/machine.h>
#include <linux/platform_data/mtd-nand-omap2.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-duagon-i101.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/davinci_asp.h>
#include <asm/io.h>

#include <plat/irqs-33xx.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>

#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
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

static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};

/* LCD backlight platform Data */
#define AM335X_BACKLIGHT_MAX_BRIGHTNESS		100
#define AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS	100
#define AM335X_PWM_PERIOD_NANO_SECONDS		(5000 * 10)

#define PWM_DEVICE_ID	"ecap.0"

static struct platform_pwm_backlight_data am335x_backlight_data = {
	.pwm_id		= PWM_DEVICE_ID,
	.ch		= -1,
	.lth_brightness	= 21,
	.max_brightness	= AM335X_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness	= AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns	= AM335X_PWM_PERIOD_NANO_SECONDS,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			= 32,
	.fdd			= 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 0,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 1,
	.invert_frm_clock	= 1,
	.sync_edge		= 0,
	.sync_ctrl		= 1,
	.raster_order		= 0,
};

struct da8xx_lcdc_platform_data TFC_S9700RTWV35TR_01B_pdata = {
	.manu_name		= "ThreeFive",
	.controller_data	= &lcd_cfg,
	.type			= "TFC_S9700RTWV35TR_01B",
};

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

unsigned int gigabit_enable = 1;

#define EEPROM_MAC_ADDRESS_OFFSET	60 /* 4+8+4+12+32 */
#define EEPROM_NO_OF_MAC_ADDR		3
static char am335x_mac_addr[EEPROM_NO_OF_MAC_ADDR][ETH_ALEN];

#define AM335X_EEPROM_HEADER		0xEE3355AA

/* Pin mux for gpmc module duagon i101 */
static struct pinmux_config gpmc_duagon_i101_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad1.gpmc_ad1",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad2.gpmc_ad2",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad3.gpmc_ad3",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad4.gpmc_ad4",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad5.gpmc_ad5",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad6.gpmc_ad6",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad7.gpmc_ad7",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad8.gpmc_ad8",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad9.gpmc_ad9",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad10.gpmc_ad10",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad11.gpmc_ad11",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad12.gpmc_ad12",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad13.gpmc_ad13",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad14.gpmc_ad14",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad15.gpmc_ad15",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_clk.gpmc_clk",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_csn3.gpmc_csn3",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn2.gpmc_csn2",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn1.gpmc_csn1",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wen.gpmc_wen",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_oen_ren.gpmc_oen_ren",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_advn_ale.gpmc_advn_ale",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ben0_cle.gpmc_ben0_cle",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpmc_wpn",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_a0.gpmc_a0",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a1.gpmc_a1",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a2.gpmc_a2",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a3.gpmc_a3",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a4.gpmc_a4",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a5.gpmc_a5",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a6.gpmc_a6",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a7.gpmc_a7",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a8.gpmc_a8",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a9.gpmc_a9",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a10.gpmc_a10",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a11.gpmc_a11",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

/* Module pin mux for SPI flash */
static struct pinmux_config spi0_pin_mux[] = {
	{"spi0_sclk.spi0_sclk",			OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							       | AM33XX_INPUT_EN},
	{"spi0_d0.spi0_d0",			OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							       | AM33XX_PULL_UP
							       | AM33XX_INPUT_EN},
	{"spi0_d1.spi0_d1",			OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							       | AM33XX_INPUT_EN},
	{"spi0_cs0.spi0_cs0",			OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
							       | AM33XX_PULL_UP
							       | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for mii1 */
static struct pinmux_config mii1_pin_mux[] = {
	{"mii1_rxerr.mii1_rxerr",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_rxdv.mii1_rxdv",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txd3.mii1_txd3",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd2.mii1_txd2",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.mii1_txclk",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxclk.mii1_rxclk",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd3.mii1_rxd3",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd2.mii1_rxd2",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd1.mii1_rxd1",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for rmii1 */
static struct pinmux_config rmii1_pin_mux[] = {
	{"mii1_crs.rmii1_crs_dv",		OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxerr.mii1_rxerr",		OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen",			OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1",			OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0",			OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_rxd1.mii1_rxd1",			OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0",			OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"rmii1_refclk.rmii1_refclk",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data",			OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config i2c2_pin_mux[] = {
	{"uart1_ctsn.i2c2_sda",			OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW
							       | AM33XX_PULL_UP
							       | AM33XX_INPUT_EN},
	{"uart1_rtsn.i2c2_scl",			OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW
							       | AM33XX_PULL_UP
							       | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* CAN board has 2 CAN interfaces; d_can1 used as can0, d_can0 may be used as can1 */
static struct pinmux_config d_can_1_pin_mux[] = {
	{"uart1_rxd.d_can1_tx",			OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_txd.d_can1_rx",			OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

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

/* Keys mapping */
static const uint32_t am335x_evm_matrix_keys[] = {
	KEY(0, 0, KEY_MENU),
	KEY(1, 0, KEY_BACK),
	KEY(2, 0, KEY_LEFT),

	KEY(0, 1, KEY_RIGHT),
	KEY(1, 1, KEY_ENTER),
	KEY(2, 1, KEY_DOWN),
};

const struct matrix_keymap_data am335x_evm_keymap_data = {
	.keymap		= am335x_evm_matrix_keys,
	.keymap_size	= ARRAY_SIZE(am335x_evm_matrix_keys),
};

static const unsigned int am335x_evm_keypad_row_gpios[] = {
	GPIO_TO_PIN(1, 25), GPIO_TO_PIN(1, 26), GPIO_TO_PIN(1, 27)
};

static const unsigned int am335x_evm_keypad_col_gpios[] = {
	GPIO_TO_PIN(1, 21), GPIO_TO_PIN(1, 22)
};

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

static int backlight_enable;

/* Setup pwm-backlight */
static struct platform_device am335x_backlight = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &am335x_backlight_data,
	}
};

static int __init ecap0_init(void)
{
	int status = 0;

	if (backlight_enable) {
		am33xx_register_ecap(0);
		platform_device_register(&am335x_backlight);
	}
	return status;
}
late_initcall(ecap0_init);

static void mii1_init(int evm_id, int profile)
{
	printk ("Ethernet 1: MII pin muxing active\n");
	setup_pin_mux(mii1_pin_mux);
	return;
}

static void rmii1_init(int evm_id, int profile)
{
	printk ("Ethernet 1: RMII pin muxing active\n");
	setup_pin_mux(rmii1_pin_mux);
	return;
}

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
	/* CAN board has 2 CAN interfaces; d_can1 used as can0, d_can0 may be used as can1 */
	setup_pin_mux(d_can_1_pin_mux);

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
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[4] = {
		{ NULL, 0 },
		{ NULL, 0 },
		{ NULL, 0 },
		{ NULL, 0 },
	};
	pr_info ("board-duagon-101.c: duagon_i101_gpmc_init() started\n");
	setup_pin_mux(gpmc_duagon_i101_pin_mux);

	pr_info ("board-duagon-101.c: omap_nand_init() start\n");

	pdata = omap_nand_init(am335x_nand_partitions,
		ARRAY_SIZE(am335x_nand_partitions), 0, 0,
		&am335x_nand_timings);
	if (!pdata)
	{
		pr_err ("board-duagon-101.c: duagon_i101_gpmc_init() aborted\n");
		return;
	}
	pdata->ecc_opt =OMAP_ECC_BCH8_CODE_HW;
	pdata->elm_used = true;
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	pr_info ("board-duagon-101.c: omap_init_gpmc() start\n");
	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));

	pr_info ("board-duagon-101.c: omap_init_elm() start\n");
	omap_init_elm();
	pr_info ("board-duagon-101.c: evm_nand_init() finished\n");
}

struct resource *mem_region_gpmc;
void __iomem *base_gpmc_reg;

#define GPMC_INT_NIOS 0x0001
#define GPMC_INT_48V  0x8000

static void nNMI_interrupt_init(int evm_id, int profile)
{
}

static struct i2c_board_info am335x_i2c_boardinfo2[] = {
};

static void i2c2_init(int evm_id, int profile)
{
	setup_pin_mux(i2c2_pin_mux);
	omap_register_i2c_bus(3, 100, am335x_i2c_boardinfo2,
			ARRAY_SIZE(am335x_i2c_boardinfo2));
	return;
}

/* setup spi0 */
static void spi0_init(int evm_id, int profile)
{
	setup_pin_mux(spi0_pin_mux);
	spi_register_board_info(am335x_spi0_slave_info,
			ARRAY_SIZE(am335x_spi0_slave_info));
	return;
}

/* Duagon i101 */
static struct evm_dev_cfg duagon_i101_dev_cfg[] = {
	{mii1_init,		DEV_ON_BASEBOARD, PROFILE_NONE},
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

	/* Fillup global mac id */
	am33xx_cpsw_macidfillup(&am335x_mac_addr[0][0],
				&am335x_mac_addr[1][0]);

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
	am33xx_cpsw_init(gigabit_enable);

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

static int __devexit cpld_reg_remove(struct i2c_client *client)
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

void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{
	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller", __func__);

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am33xx_emif_base;
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");
}

static void __init am335x_evm_init(void)
{
	am33xx_cpuidle_init();
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

static void __init am335x_evm_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(AM335XEVM, "am335xevm")
	/* Maintainer: Texas Instruments */
	.atag_offset	= 0x100,
	.map_io		= am335x_evm_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END
