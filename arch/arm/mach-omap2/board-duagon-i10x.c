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
#include <linux/export.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/pwm_backlight.h>
#include <linux/input/ti_tsc.h>
#include <linux/platform_data/ti_adc.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/rtc/rtc-omap.h>
#include <linux/opp.h>
#include <linux/mm.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-duagon-i10x.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/omap-pm.h>

#include <asm/io.h>

#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>

#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "control.h"
#include "hsmmc.h"

#include <linux/sched.h>
#include <linux/signal.h>
#include <asm/irq.h>
#include <asm/atomic.h>

#include <linux/highmem.h>
#include <asm/kmap_types.h>

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

/* BBB PHY IDs */
#define BBB_PHY_ID		0x7c0f1
#define BBB_PHY_MASK		0xfffffffe

/* AM335X EVM Phy ID and Debug Registers */
#define AM335X_EVM_PHY_ID		0x4dd074
#define AM335X_EVM_PHY_MASK		0xfffffffe
#define AR8051_PHY_DEBUG_ADDR_REG	0x1d
#define AR8051_PHY_DEBUG_DATA_REG	0x1e
#define AR8051_DEBUG_RGMII_CLK_DLY_REG	0x5
#define AR8051_RGMII_TX_CLK_DLY		BIT(8)

static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};

/* LCD backlight platform Data */
#define AM335X_BACKLIGHT_MAX_BRIGHTNESS        100
#define AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS    100
#define AM335X_PWM_PERIOD_NANO_SECONDS        (5000 * 10)

static struct platform_pwm_backlight_data am335x_backlight_data0 = {
	.pwm_id         = "ecap.0",
	.ch             = -1,
	.lth_brightness	= 21,
	.max_brightness = AM335X_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness = AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns  = AM335X_PWM_PERIOD_NANO_SECONDS,
};

static struct platform_pwm_backlight_data am335x_backlight_data2 = {
	.pwm_id         = "ecap.2",
	.ch             = -1,
	.lth_brightness	= 21,
	.max_brightness = AM335X_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness = AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns  = AM335X_PWM_PERIOD_NANO_SECONDS,
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

struct da8xx_lcdc_platform_data  NHD_480272MF_ATXI_pdata = {
	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "NHD-4.3-ATXI#-T-1",
};

#include "common.h"

#include <linux/lis3lv02d.h>

/* TSc controller */
static struct tsc_data am335x_touchscreen_data  = {
	.wires  = 0,
	.x_plate_resistance = 200,
	.steps_to_configure = 5,
};

static struct adc_data am335x_adc_data = {
	.adc_channels = 8,
};

static struct mfd_tscadc_board tscadc = {
	.tsc_init = &am335x_touchscreen_data,
	.adc_init = &am335x_adc_data,
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/*
	 * Setting SYSBOOT[5] should set xdma_event_intr0 pin to mode 3 thereby
	 * allowing clkout1 to be available on xdma_event_intr0.
	 * However, on some boards (like EVM-SK), SYSBOOT[5] isn't properly
	 * latched.
	 * To be extra cautious, setup the pin-mux manually.
	 * If any modules/usecase requries it in different mode, then subsequent
	 * module init call will change the mux accordingly.
	 */
	AM33XX_MUX(XDMA_EVENT_INTR0, OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW | AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW | AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
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
	u32 device_on;

	u32 profile;	/* Profiles (0-7) in which the module is present */
};

/* AM335X - CPLD Register Offsets */
#define	CPLD_DEVICE_HDR	0x00 /* CPLD Header */
#define	CPLD_DEVICE_ID	0x04 /* CPLD identification */
#define	CPLD_DEVICE_REV	0x0C /* Revision of the CPLD code */
#define	CPLD_CFG_REG	0x10 /* Configuration Register */

static struct i2c_client *cpld_client;
static u32 am335x_evm_id;

static bool daughter_brd_detected;

#define EEPROM_MAC_ADDRESS_OFFSET	60 /* 4+8+4+12+32 */
#define EEPROM_NO_OF_MAC_ADDR		3
static char am335x_mac_addr[EEPROM_NO_OF_MAC_ADDR][ETH_ALEN];

static int am33xx_evmid = -EINVAL;

/*
* am335x_evm_set_id - set up board evmid
* @evmid - evm id which needs to be configured
*
* This function is called to configure board evm id.
*/
void am335x_evm_set_id(unsigned int evmid)
{
	am33xx_evmid = evmid;
	return;
}

/*
* am335x_evm_get_id - returns Board Type (EVM/BB/EVM-SK ...)
*
* Note:
*	returns -EINVAL if Board detection hasn't happened yet.
*/
int am335x_evm_get_id(void)
{
	return am33xx_evmid;
}
EXPORT_SYMBOL(am335x_evm_get_id);

/* Pin mux for PHY mode detection */
static struct pinmux_config SYSBOOT_pin_mux[] = {
    {"lcd_data6",                   OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},	/* MII/RMII config pins  */
    {"lcd_data7",                   OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},	/* MII/RMII config pins  */
	{NULL, 0},
};

/* Pin mux for gpmc module duagon i10x */
static struct pinmux_config gpmc_duagon_i10x_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad1.gpmc_ad1",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad2.gpmc_ad2",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad3.gpmc_ad3",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad4.gpmc_ad4",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad5.gpmc_ad5",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad6.gpmc_ad6",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad7.gpmc_ad7",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad8.gpmc_ad8",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad9.gpmc_ad9",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad10.gpmc_ad10",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad11.gpmc_ad11",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad12.gpmc_ad12",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad13.gpmc_ad13",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad14.gpmc_ad14",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_ad15.gpmc_ad15",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_clk.gpmc_clk",           OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_csn3.gpmc_csn3",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn2.gpmc_csn2",      	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn1.gpmc_csn1",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wen.gpmc_wen",           OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_oen_ren.gpmc_oen_ren",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_advn_ale.gpmc_advn_ale", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ben0_cle.gpmc_ben0_cle",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_a1.gpmc_a1",             OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a2.gpmc_a2",             OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a3.gpmc_a3",             OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a6.gpmc_a6",             OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a7.gpmc_a7",             OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a8.gpmc_a8",             OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

/* Module pin mux for no rmii2 */
static struct pinmux_config no_rmii2_pin_mux[] = {
	{"gpmc_wait0.gpmc_wait0",       OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpmc_wpn",	        OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_a0.gpmc_a0",             OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a4.gpmc_a4",             OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a5.gpmc_a5",             OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a9.gpmc_a9",             OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a10.gpmc_a10",           OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a11.gpmc_a11",           OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

/* Module pin mux for SPI0 fash */
static struct pinmux_config spi0_pin_mux[] = {
	{"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"spi0_d0.spi0_d0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{"spi0_d1.spi0_d1", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"spi0_cs0.spi0_cs0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for mii1 */
static struct pinmux_config mii1_pin_mux[] = {
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_rxdv.mii1_rxdv", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txd3.mii1_txd3", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd2.mii1_txd2", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.mii1_txclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxclk.mii1_rxclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd3.mii1_rxd3", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd2.mii1_rxd2", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for rmii1 */
static struct pinmux_config rmii1_pin_mux[] = {
	{"mii1_crs.rmii1_crs_dv", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"rmii1_refclk.rmii1_refclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for rmii2 */
static struct pinmux_config rmii2_pin_mux[] = {
	{"gpmc_wpn.gpmc_wpn",	        OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP  }, /* RMII2_RXER */
	{"gpmc_a0.gpmc_a0",             OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT        }, /* RMII2_TXEN */
	{"gpmc_a4.gpmc_a4",             OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT        }, /* RMII2_TXD1 */
	{"gpmc_a5.gpmc_a5",             OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT        }, /* RMII2_TXD0 */
	{"gpmc_a9.gpmc_a9",             OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN}, /* MMC2DAT7 | RMII2_CRSDV, 2. multiplexer must be set */
	{"gpmc_a10.gpmc_a10",           OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN}, /* RMII2_RXD1 */
	{"gpmc_a11.gpmc_a11",           OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN}, /* RMII2_RXD0 */
	{"mii1_col.mii1_col",           OMAP_MUX_MODE1 | AM33XX_PULL_DISA | AM33XX_INPUT_EN}, /* RMII2_REFCLK */
	{NULL, 0},
};

/* Module pin mux for CPU debug LED */
static struct pinmux_config CPU_debug_LED_pin_mux[] = {
    {"lcd_pclk.lcd_pclk",           OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, /* LCD_PCLK: First boot indicator */
};

/* Module pin mux for rmii2 Rev1 CPU*/
static struct pinmux_config rmii2_Rev1CPU_pin_mux[] = {
	{"gpmc_wpn.gpmc_wpn",	        OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP  }, /* RMII2_RXER */
	{"gpmc_a0.gpmc_a0",             OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT        }, /* RMII2_TXEN */
	{"gpmc_a4.gpmc_a4",             OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT        }, /* RMII2_TXD1 */
	{"gpmc_a5.gpmc_a5",             OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT        }, /* RMII2_TXD0 */
	{"gpmc_wait0.gpmc_wait0",       OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP  }, /* RMII2_CRSDV*/ /* open drain */
	{"gpmc_a10.gpmc_a10",           OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN}, /* RMII2_RXD1 */
	{"gpmc_a11.gpmc_a11",           OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN}, /* RMII2_RXD0 */
	{"mii1_col.mii1_col",           OMAP_MUX_MODE1 | AM33XX_PULL_DISA | AM33XX_INPUT_EN}, /* RMII2_REFCLK */
	{NULL, 0},
};

static struct pinmux_config i2c2_pin_mux[] = {
	{"uart1_ctsn.i2c2_sda", OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW | AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{"uart1_rtsn.i2c2_scl", OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW | AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{NULL, 0},
};


static struct pinmux_config d_can_1_pin_mux[] = {
	{"uart1_rxd.d_can1_tx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_txd.d_can1_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/*
static struct pinmux_config d_can_0_pin_mux[] = {
	{"uart1_ctsn.d_can0_tx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_rtsn.d_can0_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};
*/

/*
static struct pinmux_config gpio_ddr_vtt_enb_pin_mux[] = {
	{"ecap0_in_pwm0_out.gpio0_7", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};
*/

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
	.keymap      = am335x_evm_matrix_keys,
	.keymap_size = ARRAY_SIZE(am335x_evm_matrix_keys),
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

	am335x_evm_set_id(evm_id);

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


/* pinmux for usb0 drvvbus, usb0 over_curr i101 Rev 2, i102 Rev 2, i103 Rev 2 | 3 */
static struct pinmux_config usb0_pin_mux_i101_rev2_i102_rev2_i103_rev3[] = {
	{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"usb1_drvvbus.usb1_drvvbus",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* pinmux for usb0 drvvbus, usb0 over_curr i101 Rev3, i102 Rev3, i103 Rev4 */
static struct pinmux_config usb0_pin_mux_i101_rev3_i102_rev3_i103_rev4[] = {
	{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mcasp0_ahclkr.mcasp0_ahclkr",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* pinmux for usb1 drvvbus, usb1 over_curr i101 Rev3, i102 Rev3, i103 Rev4 */
static struct pinmux_config usb1_pin_mux_i101_rev3_i102_rev3_i103_rev4[] = {
	{"usb1_drvvbus.usb1_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mcasp0_aclkr.mcasp0_aclkr",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

static bool backlight_enable;

/* Setup pwm-backlight */
static struct platform_device am335x_backlight = {
	.name           = "pwm-backlight",
	.id             = -1,
	.dev		= {
		.platform_data = &am335x_backlight_data0,
	},
};

static struct pwmss_platform_data  pwm_pdata[3] = {
	{
		.version = PWM_VERSION_1,
	},
	{
		.version = PWM_VERSION_1,
	},
	{
		.version = PWM_VERSION_1,
	},
};

static int __init backlight_init(void)
{
	int status = 0;

	if (backlight_enable) {
		int ecap_index = 0;

		switch (am335x_evm_get_id()) {
		case GEN_PURP_EVM:
		case GEN_PURP_DDR3_EVM:
			ecap_index = 0;
			break;
		case EVM_SK:
			/*
			 * Invert polarity of PWM wave from ECAP to handle
			 * backlight intensity to pwm brightness
			 */
			ecap_index = 2;
			pwm_pdata[ecap_index].chan_attrib[0].inverse_pol = true;
			am335x_backlight.dev.platform_data =
				&am335x_backlight_data2;
			break;
		default:
			pr_err("%s: Error on attempting to enable backlight,"
				" not supported\n", __func__);
			return -EINVAL;
		}

		am33xx_register_ecap(ecap_index, &pwm_pdata[ecap_index]);
		platform_device_register(&am335x_backlight);
	}
	return status;
}
late_initcall(backlight_init);

static void mfd_tscadc_init(int evm_id, int profile)
{
	int err;

pr_err("*** Start: Register ADC and touchscreen device\n");
	err = am33xx_register_mfd_tscadc(&tscadc);
pr_err("*** End  : Register ADC and touchscreen device\n");
	if (err)
		pr_err("failed to register ADC or touchscreen device\n");
}

static void mii1_init(int evm_id, int profile)
{
	setup_pin_mux(mii1_pin_mux);
	return;
}

static void rmii1_init(int evm_id, int profile)
{
	setup_pin_mux(rmii1_pin_mux);
	return;
}

static void rmii2_init(int evm_id, int profile)
{
    
    #define CTRL_REG_DEVICE_ID 0x600
    int device_id = 0;
    int revision_id = 0;

    device_id = readl(AM33XX_CTRL_REGADDR(CTRL_REG_DEVICE_ID));
    /*printk ("device_id: 0x%08x", device_id);*/
    revision_id = device_id >> 28;
    switch (revision_id)
    {
	case 0:
        printk ("AM335x Silicon Revision 1.0\n");
        break;
	case 1:
        printk ("AM335x Silicon Revision 2.0\n");
        break;
    case 2:
        printk ("AM335x Silicon Revision 2.1\n");
        break;
    default:
        printk ("AM335x Silicon Revision ID: %d\n", revision_id);
        break;
    }
    if (revision_id > 0)
    {
        #define CTRL_REG_SMA2 0x1320
        u32 sma2_val = 1;
        printk ("Ethernet 2: RMII pin muxing active\n");
        setup_pin_mux(rmii2_pin_mux);
        setup_pin_mux(CPU_debug_LED_pin_mux);
        sma2_val = readl(AM33XX_CTRL_REGADDR(CTRL_REG_SMA2));
        /* sets SMA2 register bit 0 to 1 to switch from MMC_DAT7 to RMII2_CRS_DV for Rev 2.0 CPUs onwards */
        sma2_val |= 1;
	    writel(sma2_val, AM33XX_CTRL_REGADDR(CTRL_REG_SMA2));
    }
    else
    {
        printk ("Ethernet 2: RMII pin muxing Rev 1 CPU active\n");
        setup_pin_mux(rmii2_Rev1CPU_pin_mux);
	}
	return;
}

static int fpga_pcbVersionRequest (char* pcbRevisionID, char* pcbTypeID, char* pcbAssemblyID, char* CPURevisionAtLeast2)
{
    short PCBRevision = 0;
    
    /*struct resource *mem_region_gpmc;*/
    void __iomem *base_gpmc_reg;
    
    /*
    if ((mem_region_gpmc = request_mem_region(0x18000800, SZ_4K, "GPMC_reg_area"))==NULL);
    {
        pr_info ("ERROR: fpga_pcbVersionRequest(): request_mem_region() returned NULL.\n");
        return -1;
    }
    */
    
    if (!(base_gpmc_reg = ioremap_nocache(0x18000800, SZ_4K)))
    {
        pr_info ("ERROR: fpga_pcbVersionRequest(): ioremap() returned NULL;\n");
        return -1;
    }
    PCBRevision = ioread16 ( base_gpmc_reg + 0x24);
    iounmap(base_gpmc_reg);
    
    if (PCBRevision == 0x412)
        return -1;
    *CPURevisionAtLeast2 =  (PCBRevision & 0x8000) >> 15;
    *pcbAssemblyID       =  (PCBRevision & 0x7FC0) >> 6;
    *pcbRevisionID       = ((PCBRevision & 0x0038) >> 3) + 1; 
    *pcbTypeID           =  (PCBRevision & 0x0007);
    // i103 has additional offset of 1 in revision
    if (*pcbTypeID == 4)
        *pcbRevisionID=*pcbRevisionID+1;
    return 0;
}

static void print_pcbRevisionInformation (char pcbRevisionID, char pcbTypeID, char pcbAssemblyID, char CPURevisionAtLeast2)
{
    printk ("PCB information: ionia ");
    switch (pcbTypeID)
    {
        case 1:
            printk ("i101");
            break;
        case 2:
            printk ("i102 | i100");
            break;
        case 4:
            printk ("i103");
            break;
        default:
            printk ("unknown: PCB ID: 0x%x", pcbTypeID);
            break;
    }
    printk (", Rev: %i, Assembly ID: 0x%03x, CPU: %s\r\n", pcbRevisionID, pcbAssemblyID, CPURevisionAtLeast2?"at least 2.0":"1");
}

static int isUSB2_available (char pcbRevisionID, char pcbTypeID)
{
    switch (pcbTypeID)
    {
        case 1: /* i101 */
            if (pcbRevisionID >= 3)
                return 1;
            break;
        case 2: /* i102 */
            if (pcbRevisionID >= 3)
                return 1;
            break;
        case 4: /* i103 */
            if (pcbRevisionID >= 4)
                return 1;
            break;
        default:
            break;
    }
    return 0;
}

static void usb0_init(int evm_id, int profile)
{
    int fpga_pcbVersionUnknown = -1;
    char pcbRevisionID = 0;
    char pcbTypeID     = 0;
    char pcbAssemblyID = 0;
    char CPURevisionAtLeast2 = 0;
    int USB2_available = 0;

    
    if ((fpga_pcbVersionUnknown = fpga_pcbVersionRequest(&pcbRevisionID, &pcbTypeID, &pcbAssemblyID, &CPURevisionAtLeast2)))
        printk ("PCB information not available. (GPMC read access from FPGA did not succeed)\r\n");
    if ((USB2_available = isUSB2_available (pcbRevisionID, pcbTypeID)))
    {
        /*todo remove debug output*/
        printk("setup_pin_mux(usb0_pin_mux_i101_rev3_i102_rev3_i103_rev4);\r\n");
        setup_pin_mux(usb0_pin_mux_i101_rev3_i102_rev3_i103_rev4);
    }
    else
    {
        /*todo remove debug output*/
        printk("setup_pin_mux(usb0_pin_mux_i101_rev2_i102_rev2_i103_rev3);\r\n");
        setup_pin_mux(usb0_pin_mux_i101_rev2_i102_rev2_i103_rev3);
    }
	return;
}

static void usb1_init(int evm_id, int profile)
{
    int fpga_pcbVersionUnknown = -1;
    char pcbRevisionID = 0;
    char pcbTypeID     = 0;
    char pcbAssemblyID = 0;
    char CPURevisionAtLeast2 = 0;
    int USB2_available = 0;

    if ((fpga_pcbVersionUnknown = fpga_pcbVersionRequest(&pcbRevisionID, &pcbTypeID, &pcbAssemblyID, &CPURevisionAtLeast2)))
        printk ("PCB information not available. (GPMC read access from FPGA did not succeed)\r\n");
    if ((USB2_available = isUSB2_available (pcbRevisionID, pcbTypeID)))
    {
        /*todo remove debug output*/
        printk ("setup_pin_mux(usb1_pin_mux_i101_rev3_i102_rev3_i103_rev4);\r\n");
        setup_pin_mux(usb1_pin_mux_i101_rev3_i102_rev3_i103_rev4);
    }
	else
        printk ("USB 2 not available on this PCB revision.\r\n");
    return;
}

/*
 * gpio0_7 was driven HIGH in u-boot before DDR configuration
 *
 * setup gpio0_7 for EVM-SK 1.2
 */
/*
static void gpio_ddr_vtt_enb_init(int evm_id, int profile)
{
	setup_pin_mux(gpio_ddr_vtt_enb_pin_mux);
	return;
}
*/

static void d_can_init(int evm_id, int profile)
{
	setup_pin_mux(d_can_1_pin_mux);

	am33xx_d_can_init(1);
	pr_info("CAN interface can1 init'ed on uart1_rxd/txd\n");

/*
	setup_pin_mux(d_can_0_pin_mux);

	am33xx_d_can_init(0);
	pr_info("CAN interface can0 init'ed on uart1_ctsn/rtsn\n");
*/
}

/* NAND partition information */
static struct mtd_partition am335x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "/dev/mtd10 U Boot SPL 1",
		.offset         = 0,			/* Offset = 0x0 */
		.size           = 1 * SZ_512K,
	},
	{
		.name           = "/dev/mtd11 U-Boot 1",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x080000 */
		.size           = 1 * SZ_512K,
	},
	{
		.name           = "/dev/mtd12 U-Boot Env 1_1",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x100000 */
		.size           = 1 * SZ_512K,
	},
	{
		.name           = "/dev/mtd13 U-Boot Env 1_2",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x180000 */
		.size           = 1 * SZ_512K,
	},
	{
		.name           = "/dev/mtd14 Kernel 1",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x200000 */
		.size           = 28 * SZ_512K,
	},
	{
		.name           = "/dev/mtd15 File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x02000000 */
		.size           = 992 * SZ_512K,     /* size   = 0x40000000, 480 MBytes */
	},
	{
		.name           = "/dev/mtd16 U Boot SPL 2",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x20000000 */
		.size           = 1 * SZ_512K,
	},
	{
		.name           = "/dev/mtd17 U-Boot 2",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x20080000 */
		.size           = 1 * SZ_512K,
	},
	{
		.name           = "/dev/mtd18 U-Boot Env 2_1",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x20100000 */
		.size           = 1 * SZ_512K,
	},
	{
		.name           = "/dev/mtd19 U-Boot Env 2_2",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x20180000 */
		.size           = 1 * SZ_512K,
	},
	{
		.name           = "/dev/mtd20 Kernel 2",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x20200000 */
		.size           = 28 * SZ_512K,
	},
	{
		.name           = "/dev/mtd21 File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x22000000, 480 MBytes */
		.size           = MTDPART_SIZ_FULL,     /* size   = 0x40000000 1GByte */
	}
    /*Check if Bad Block Table is at the end*/
};

/* SPI 0/1 Platform Data */
/* SPI flash information */
static struct mtd_partition am335x_spi_partitions[] = {
	/* All the partition sizes are listed in terms of erase size */
	{
		.name       = "/dev/mtd0 U Boot SPL 1",
		.offset     = 0,			/* Offset = 0x0 */
		.size       = 2 * SZ_64K,
	},
	{
		.name       = "/dev/mtd1 U-Boot 1",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size       = 4 * SZ_64K,
	},
	{
		.name       = "/dev/mtd2 U-Boot Env 1",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size       = 1 * SZ_64K,
	},
	{
		.name       = "/dev/mtd3 U-Boot Env 2",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x70000 */
		.size       = 1 * SZ_64K,
	},
	{
		.name       = "/dev/mtd4 Kernel 1",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size       = 56 * SZ_64K,
	},
	{
		.name       = "/dev/mtd5 U Boot SPL 2",
		.offset     = MTDPART_OFS_APPEND,   /* Offset = 0x400000 */
		.size       = 2 * SZ_64K,
	},
	{
		.name       = "/dev/mtd6 U-Boot 2",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x420000 */
		.size       = 4 * SZ_64K,
	},
	{
		.name       = "/dev/mtd7 U-Boot Env 3",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x460000 */
		.size       = 1 * SZ_64K,
	},
	{
		.name       = "/dev/mtd8 U-Boot Env 4",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x470000 */
		.size       = 1 * SZ_64K,
	},
	{
		.name       = "/dev/mtd9 Kernel 2",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x480000 */
		.size       = MTDPART_SIZ_FULL,		/* size ~ = 0x800000 */
	}
};

static const struct flash_platform_data am335x_spi_flash = {
	.type      = "w25q64",
	.name      = "spi_flash",
	.parts     = am335x_spi_partitions,
	.nr_parts  = ARRAY_SIZE(am335x_spi_partitions),
};

/*
 * SPI Flash works at 80Mhz however SPI Controller works at 48MHz.
 * So setup Max speed to be less than that of Controller speed
 */
static struct spi_board_info am335x_spi0_slave_info[] = {
	{
		.modalias      = "m25p80",
		.platform_data = &am335x_spi_flash,
		.irq           = -1,
		.max_speed_hz  = 24000000,
		.bus_num       = 1,
		.chip_select   = 0,
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

static void duagon_i10x_gpmc_init(int evm_id, int profile)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[4] = {
		{ NULL, 0 },
		{ NULL, 0 },
		{ NULL, 0 },
		{ NULL, 0 },
	};
    pr_info ("board-duagon-101.c: duagon_i10x_gpmc_init() started\n");
	setup_pin_mux(gpmc_duagon_i10x_pin_mux);
    
	pr_info ("board-duagon-101.c: omap_nand_init() start\n");
    
	pdata = omap_nand_init(am335x_nand_partitions,
		ARRAY_SIZE(am335x_nand_partitions), 0, 0,
		&am335x_nand_timings);
	if (!pdata)
    {
        pr_err ("board-duagon-101.c: duagon_i10x_gpmc_init() aborted\n");
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
#include <linux/ioport.h>
#include <asm/io.h>

struct resource *mem_region_gpmc;
void __iomem *base_gpmc_reg;

#define GPMC_INT_NIOS 0x0001
#define GPMC_INT_48V  0x8000

/*for future use/debugging */
#if 0
static int nNMIinterruptHandler (int value, void * function, struct pt_regs * pt_regs)
{
	/* what happens if FPGA is not booted at startup of kernel( also with wrong nEXTINT polartity?)
    int resp_time;
	unsigned short int_cause;
    // static short temperature = 0;		// generates "ISO C90 forbids..." warning on purpose to remind the 'tbd'
	int_cause = readw(base_gpmc_reg + 0x0);
    writew (0xFFFF, base_gpmc_reg);

    readw(base_gpmc_reg + 0xc);		// the first read immediately after a write returns always 0; ignore it
    resp_time = readw(base_gpmc_reg + 0xc);
    */
    /*
    pr_info ("GPMC IRQ: status %04x, info %04x, resp.time %d.%dus\n", int_cause, readw(base_gpmc_reg + 0x8), resp_time / 10, resp_time % 10);
    */
    
    /*
    // if status is NIOS, then should write 8-bit board temperature to (base_gpmc_reg + 0x8) for burn-in eeprom_test
    if (int_cause & GPMC_INT_NIOS)
    {
    	// temperature simulator
        static unsigned char temperature_dir = 0;
        if (temperature_dir)
        {
        	temperature--;
        	if (temperature == -40) temperature_dir = 0;
        }
        else
        {
        	temperature++;
        	if (temperature == 85) temperature_dir = 1;
        }
    	writew (temperature & 0x00ff, base_gpmc_reg + 0x8);
    }
    */
    return IRQ_HANDLED;
}


static struct nNMI_interrupt_init_default_type
{
    unsigned long test;
} nNMI_interrupt_init_default =
{
    0
};
#endif

static void nNMI_interrupt_init(int evm_id, int profile)
{
/*    int result=0;

    disable_irq(7);
    pr_info ("board-duagon-101.c: nNMI_interrupt_init() start\n");

    mem_region_gpmc = request_mem_region(0x18000800, SZ_4K, "GPMC_reg_area");
    pr_info ("nNMI_interrupt_init(): mem_region_gpmc %x %x %x %x\n", (unsigned int)mem_region_gpmc, mem_region_gpmc->start, mem_region_gpmc->end, (unsigned int)mem_region_gpmc->flags);

   	if (!(base_gpmc_reg = ioremap_nocache(0x18000800, SZ_4K)))
    {
        pr_info ("ERROR: nNMI_interrupt_init(): ioremap() returned 0;\n");
        return;
    }
    pr_info ("nNMI_interrupt_init(): base_gpmc_reg %x\n", (unsigned int)base_gpmc_reg);

    writew (0xFFFF, base_gpmc_reg);		// clear any spurious int

    pr_info ("FPGA INT enable  (0x18000804): %04x\n", readw(base_gpmc_reg + 0x4));
    pr_info ("FPGA INT enable  (0x18000804): write 0xFFFF\n");
    writew (0xFFFF, base_gpmc_reg + 0x4);
    readw(base_gpmc_reg + 0x4);			// the first read immediately after a write returns always 0; ignore it
    pr_info ("FPGA INT enable  (0x18000804): %04x\n", readw(base_gpmc_reg + 0x4));

    if ((result=(request_irq (7, (irq_handler_t) nNMIinterruptHandler, IRQF_DISABLED, "nNMI interrupt", NULL ))))
    {
        pr_info ("board-duagon-101.c: nNMI_interrupt_init() failed with code=%i\n", result);
    }
    //enable_irq(7);	// not needed; apparently causes "Unbalanced enable for IRQ"

    pr_info ("board-duagon-101.c: nNMI_interrupt_init() end\n");
*/
}


static struct i2c_board_info am335x_i2c2_boardinfo[] = {
};

static void i2c2_init(int evm_id, int profile)
{
	setup_pin_mux(i2c2_pin_mux);
	omap_register_i2c_bus(3, 100, am335x_i2c2_boardinfo, ARRAY_SIZE(am335x_i2c2_boardinfo));
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

static struct omap_rtc_pdata am335x_rtc_info = {
	.pm_off		= false,
	.wakeup_capable	= 0,
};

static void am335x_rtc_init(int evm_id, int profile)
{
	void __iomem *base;
	struct clk *clk;
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char *dev_name = "am33xx-rtc";

	clk = clk_get(NULL, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc : Failed to get RTC clock\n");
		return;
	}

	if (clk_enable(clk)) {
		pr_err("rtc: Clock Enable Failed\n");
		return;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);

	if (WARN_ON(!base))
		return;

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

	switch (evm_id) {
	case BEAGLE_BONE_A3:
	case BEAGLE_BONE_OLD:
		am335x_rtc_info.pm_off = true;
		break;
	default:
		break;
	}

	clk_disable(clk);
	clk_put(clk);

	if (omap_rev() == AM335X_REV_ES2_0)
		am335x_rtc_info.wakeup_capable = 1;

	oh = omap_hwmod_lookup("rtc");
	if (!oh) {
		pr_err("could not look up %s\n", "rtc");
		return;
	}

	pdev = omap_device_build(dev_name, -1, oh, &am335x_rtc_info,
			sizeof(struct omap_rtc_pdata), NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
}

/* Enable clkout2 */
static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void clkout2_enable(int evm_id, int profile)
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

/* Duagon i10x */
static struct evm_dev_cfg duagon_i10x_dev_cfg[] = {
	{am335x_rtc_init,       DEV_ON_BASEBOARD, PROFILE_ALL},
	{clkout2_enable,        DEV_ON_BASEBOARD, PROFILE_ALL},
	{mfd_tscadc_init,	    DEV_ON_BASEBOARD, PROFILE_ALL},
	{i2c2_init,             DEV_ON_BASEBOARD, PROFILE_ALL},
	{duagon_i10x_gpmc_init, DEV_ON_BASEBOARD, PROFILE_ALL},
	{spi0_init,	            DEV_ON_BASEBOARD, PROFILE_ALL},
	{nNMI_interrupt_init,   DEV_ON_BASEBOARD, PROFILE_ALL},
	{d_can_init,			DEV_ON_BASEBOARD, PROFILE_ALL},
	{NULL, 0, 0},
};

/* Duagon i10x MII*/
static struct evm_dev_cfg duagon_i10x_mii_dev_cfg[] = {
	{mii1_init,	            DEV_ON_BASEBOARD, PROFILE_ALL},
	{NULL, 0, 0},
};

/* Duagon i10x MII*/
static struct evm_dev_cfg duagon_i10x_rmii_dev_cfg[] = {
	{rmii1_init,	        DEV_ON_BASEBOARD, PROFILE_ALL},
	{rmii2_init,	        DEV_ON_BASEBOARD, PROFILE_ALL},
	{NULL, 0, 0},
};

/* Duagon i10x */
static struct evm_dev_cfg duagon_i10x_usb_cfg[] = {
	{usb0_init,	            DEV_ON_BASEBOARD, PROFILE_ALL},
	{usb1_init,	            DEV_ON_BASEBOARD, PROFILE_ALL},
	{NULL, 0, 0},
};

int detectMiiPHY (void)
{ // detect MII / RMII Ethernet PHY
    #define GPIO_2_12		(64+12)
    #define GPIO_2_13		(64+13)
    int GPIO_2_12_val = 0;
    int GPIO_2_13_val = 0;
    
    if (gpio_request(GPIO_2_12, "lcd_data6") != 0) 
    {
        printk("%s: gpio_request() of GPIO_2_12 (lcd_data6) failed.\n", __func__);
        return -1;
    }
    if (gpio_request(GPIO_2_13, "lcd_data7") != 0) 
    {
        printk("%s: gpio_request() of GPIO_2_13 (lcd_data7) failed.\n", __func__);
        return -1;
    }
    if (gpio_direction_input(GPIO_2_12))
    {
        printk("%s: gpio_direction_input() of GPIO_2_12 (lcd_data6) failed.\n", __func__);
        return -1;
    }
    if (gpio_direction_input(GPIO_2_13))
    {
        printk("%s: gpio_direction_input() of GPIO_2_13 (lcd_data7) failed.\n", __func__);
        return -1;
    }
//  udelay(1000000);
    if ((GPIO_2_12_val=gpio_get_value(GPIO_2_12)) < 0)
    {
        printk("%s: gpio_direction_input() of GPIO_2_12 (lcd_data7) failed.\n", __func__);
        return -1;
    }
    if ((GPIO_2_13_val=gpio_get_value(GPIO_2_13)) < 0)
    {
        printk("%s: gpio_direction_input() of GPIO_2_13 (lcd_data7) failed.\n", __func__);
        return -1;
    }
    gpio_free(GPIO_2_12);
    gpio_free(GPIO_2_13);
    if (GPIO_2_12_val == 1 && GPIO_2_13_val == 0 )
        return 0;
    else if (GPIO_2_12_val == 0 && GPIO_2_13_val == 0 )
        return 1;
// Workaround. On older PCBs (before i103 Rev 3.0 including, i101 Rev 1.2) 22k resistors are assembled for RMII and MII selector SYSBOOT6 and SYSBOOT7. FPGA is also connected to SYSBOOT6 and SYSBOOT7. If FPGA is not configured the FPGA pullups overwrite SYSBOOT6 and SYSBOOT7. Therefore detection fails.
    printk ("%s: unrealistic PHY settings read. Setting to default MII to be compatible with i103 Rev 3.0, or older (when FPGA not configured)\r\n", __func__);
    printk ("GPIO_2_12_val: %d, GPIO_2_13_val: %d\r\n", GPIO_2_12_val, GPIO_2_13_val);
    return 1;
}

/* Duagon i10x setup */
static void setup_duagon_i10x(void)
{
    int fpga_pcbVersionUnknown = -1;
    char pcbRevisionID = 0;
    char pcbTypeID     = 0;
    char pcbAssemblyID = 0;
    char CPURevisionAtLeast2 = 0;
    int MII_detected = -1;
    setup_pin_mux(SYSBOOT_pin_mux); // enables detection of PHY mode
    MII_detected = detectMiiPHY();

    if (MII_detected == 1)
    {
        printk ("MII PHY detected\n");
        setup_pin_mux(no_rmii2_pin_mux);
        _configure_device(duagon_i10x_mii, duagon_i10x_dev_cfg, PROFILE_NONE);
        _configure_device(duagon_i10x_mii, duagon_i10x_mii_dev_cfg, PROFILE_NONE);
    }
    else if (MII_detected == 0)
    {
        printk ("RMII PHY detected\n");
        _configure_device(duagon_i10x_rmii, duagon_i10x_dev_cfg, PROFILE_NONE);
        _configure_device(duagon_i10x_rmii, duagon_i10x_rmii_dev_cfg, PROFILE_NONE);
    }
    else
        printk ("unknown PHY type detected.\n");
    fpga_pcbVersionUnknown = fpga_pcbVersionRequest(&pcbRevisionID, &pcbTypeID, &pcbAssemblyID, &CPURevisionAtLeast2);
    if (!fpga_pcbVersionUnknown)
        print_pcbRevisionInformation (pcbRevisionID, pcbTypeID, pcbAssemblyID, CPURevisionAtLeast2);
    else
        printk ("PCB information not available. (GPMC read access from FPGA did not succeed)\r\n");
    if (MII_detected == 1)
    {
        _configure_device(duagon_i10x_mii, duagon_i10x_usb_cfg, PROFILE_NONE);
        am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, "0:00", "0:01");
    }
    else if (MII_detected == 0)
    {
        _configure_device(duagon_i10x_rmii, duagon_i10x_usb_cfg, PROFILE_NONE);
        am33xx_cpsw_init(AM33XX_CPSW_MODE_RMII_EXT_CLK, NULL, NULL);
    }
}

static void am335x_evm_setup(struct memory_accessor *mem_acc, void *context)
{
	// char tmp[10];

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
    return;
}

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
	.ignore_check_consumers = 1,
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
	.ignore_check_consumers = 1,
};

#define PMIC_IRQ_GPIO   GPIO_TO_PIN(2, 25)
static struct tps65910_board am335x_tps65910_info = {
    .irq_base = TWL4030_IRQ_BASE,
    .gpio_base = OMAP_MAX_GPIO_LINES,
    .irq = OMAP_GPIO_IRQ(PMIC_IRQ_GPIO),
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

/* Module pin mux for tps65910 irq */
static struct pinmux_config pmic_irq_pin_mux[] = {  
    {"uart0_ctsn.gpio2_25", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
    {NULL, 0},
};

static struct i2c_board_info __initdata am335x_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1),
		.platform_data  = &am335x_tps65910_info,
	},
};


static struct omap_musb_board_data musb_board_data_i101_rev2_i102_rev2_i103_rev3 = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * AM335X beta EVM has USB0 in OTG mode and USB1 in host mode.
	 */

	.mode       = MUSB_OTG,
	.power		= 500,
	.instances	= 0,
};

static struct omap_musb_board_data musb_board_data_i101_rev3_i102_rev3_i103_rev4 = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * AM335X beta EVM has USB0 in OTG mode and USB1 in host mode.
	 */

	.mode       = (MUSB_HOST << 4) | MUSB_OTG, 
	.power		= 500,
	.instances	= 1,
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
	/* Initially assume General Purpose EVM Config */
	am335x_evm_id = GEN_PURP_EVM;

	evm_init_cpld();

    setup_pin_mux(pmic_irq_pin_mux);
	omap_register_i2c_bus(1, 100, am335x_i2c0_boardinfo, ARRAY_SIZE(am335x_i2c0_boardinfo));
}

void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{

	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am33xx_emif_base;
}

void __iomem *am33xx_gpio0_base;

void __iomem *am33xx_get_gpio0_base(void)
{
	am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);

	return am33xx_gpio0_base;
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
    int fpga_pcbVersionUnknown = -1;
    char pcbRevisionID = 0;
    char pcbTypeID     = 0;
    char pcbAssemblyID = 0;
    char CPURevisionAtLeast2 = 0;
    int USB2_available = 0;
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);
	omap_serial_init();
	am335x_evm_i2c_init();
    am335x_evm_setup(NULL, NULL);
    setup_duagon_i10x();
    /*
	// SmartReflex also requires board information.
	    am33xx_sr_init();
    */  

    fpga_pcbVersionUnknown = fpga_pcbVersionRequest(&pcbRevisionID, &pcbTypeID, &pcbAssemblyID, &CPURevisionAtLeast2);
    if (!fpga_pcbVersionUnknown)
        print_pcbRevisionInformation (pcbRevisionID, pcbTypeID, pcbAssemblyID, CPURevisionAtLeast2);
    else
        printk ("PCB information not available. (GPMC read access from FPGA did not succeed)\r\n");

    if ((USB2_available = isUSB2_available (pcbRevisionID, pcbTypeID)))
    {
        /*todo remove debug output*/
        printk("usb_musb_init(&musb_board_data_i101_rev3_i102_rev3_i103_rev4);\r\n");
        usb_musb_init(&musb_board_data_i101_rev3_i102_rev3_i103_rev4);
    }
    else
    {
        /*todo remove debug output*/
        printk("usb_musb_init(&musb_board_data_i101_rev2_i102_rev2_i103_rev3);\r\n");
        usb_musb_init(&musb_board_data_i101_rev2_i102_rev2_i103_rev3);
    }
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
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= am335x_evm_init,
MACHINE_END
