/*
 * linux/arch/arm/mach-omap2/devices.c
 *
 * OMAP2 platform device setup/initialization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/pinctrl/machine.h>
#include <linux/platform_data/mailbox-omap.h>

#include <linux/mfd/ti_am335x_tscadc.h>
#include <linux/platform_data/uio_pruss.h>
#include <linux/dma-mapping.h>

#include <linux/davinci_emac.h>
#include <linux/etherdevice.h>
#include <linux/can/platform/d_can.h>
#include <linux/pwm.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/board-am335xevm.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

#include <linux/omap-dma.h>

#include "gpmc.h"
#include "iomap.h"
#include "omap_hwmod.h"
#include "omap_device.h"

#include <plat/irqs-33xx.h>

/* LCD controller similar DA8xx */
#include <video/da8xx-fb.h>

#include "soc.h"
#include "common.h"
#include "mux.h"
#include "control.h"
#include "display.h"

#define L3_MODULES_MAX_LEN 12
#define L3_MODULES 3

static int __init omap3_l3_init(void)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char oh_name[L3_MODULES_MAX_LEN];

	/*
	 * To avoid code running on other OMAPs in
	 * multi-omap builds
	 */
	if (!(cpu_is_omap34xx()) || of_have_populated_dt() || soc_is_am33xx())
		return -ENODEV;

	snprintf(oh_name, L3_MODULES_MAX_LEN, "l3_main");

	oh = omap_hwmod_lookup(oh_name);

	if (!oh)
		pr_err("could not look up %s\n", oh_name);

	pdev = omap_device_build("omap_l3_smx", 0, oh, NULL, 0);

	WARN(IS_ERR(pdev), "could not build omap_device for %s\n", oh_name);

	return PTR_ERR_OR_ZERO(pdev);
}
omap_postcore_initcall(omap3_l3_init);

#if defined(CONFIG_OMAP2PLUS_MBOX) || defined(CONFIG_OMAP2PLUS_MBOX_MODULE)
static inline void __init omap_init_mbox(void)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	struct omap_mbox_pdata *pdata;

	oh = omap_hwmod_lookup("mailbox");
	if (!oh) {
		pr_err("%s: unable to find hwmod\n", __func__);
		return;
	}
	if (!oh->dev_attr) {
		pr_err("%s: hwmod doesn't have valid attrs\n", __func__);
		return;
	}

	pdata = (struct omap_mbox_pdata *)oh->dev_attr;
	pdev = omap_device_build("omap-mailbox", -1, oh, pdata, sizeof(*pdata));
	WARN(IS_ERR(pdev), "%s: could not build device, err %ld\n",
						__func__, PTR_ERR(pdev));
}
#else
static inline void omap_init_mbox(void) { }
#endif /* CONFIG_OMAP2PLUS_MBOX */

static inline void omap_init_sti(void) {}

#if defined(CONFIG_SND_SOC) || defined(CONFIG_SND_SOC_MODULE)

static struct platform_device omap_pcm = {
	.name	= "omap-pcm-audio",
	.id	= -1,
};

static void omap_init_audio(void)
{
	platform_device_register(&omap_pcm);
}

#else
static inline void omap_init_audio(void) {}
#endif

static int __init omap4_l3_init(void)
{
	int i;
	struct omap_hwmod *oh[3];
	struct platform_device *pdev;
	char oh_name[L3_MODULES_MAX_LEN];

	/* If dtb is there, the devices will be created dynamically */
	if (of_have_populated_dt())
		return -ENODEV;

	/*
	 * To avoid code running on other OMAPs in
	 * multi-omap builds
	 */
	if (!cpu_is_omap44xx() && !soc_is_omap54xx())
		return -ENODEV;

	for (i = 0; i < L3_MODULES; i++) {
		snprintf(oh_name, L3_MODULES_MAX_LEN, "l3_main_%d", i+1);

		oh[i] = omap_hwmod_lookup(oh_name);
		if (!(oh[i]))
			pr_err("could not look up %s\n", oh_name);
	}

	pdev = omap_device_build_ss("omap_l3_noc", 0, oh, 3, NULL, 0);

	WARN(IS_ERR(pdev), "could not build omap_device for %s\n", oh_name);

	return PTR_RET(pdev);
}
omap_postcore_initcall(omap4_l3_init);

#if defined(CONFIG_SPI_OMAP24XX) || defined(CONFIG_SPI_OMAP24XX_MODULE)

#include <linux/platform_data/spi-omap2-mcspi.h>

static int __init omap_mcspi_init(struct omap_hwmod *oh, void *unused)
{
	struct platform_device *pdev;
	char *name = "omap2_mcspi";
	struct omap2_mcspi_platform_config *pdata;
	static int spi_num;
	struct omap2_mcspi_dev_attr *mcspi_attrib = oh->dev_attr;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("Memory allocation for McSPI device failed\n");
		return -ENOMEM;
	}

	pdata->num_cs = mcspi_attrib->num_chipselect;
	switch (oh->class->rev) {
	case OMAP2_MCSPI_REV:
	case OMAP3_MCSPI_REV:
			pdata->regs_offset = 0;
			break;
	case OMAP4_MCSPI_REV:
			pdata->regs_offset = OMAP4_MCSPI_REG_OFFSET;
			break;
	default:
			pr_err("Invalid McSPI Revision value\n");
			kfree(pdata);
			return -EINVAL;
	}

	spi_num++;
	pdev = omap_device_build(name, spi_num, oh, pdata, sizeof(*pdata));
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s\n",
				name, oh->name);
	kfree(pdata);
	return 0;
}

static void omap_init_mcspi(void)
{
	omap_hwmod_for_each_by_class("mcspi", omap_mcspi_init, NULL);
}

#else
static inline void omap_init_mcspi(void) {}
#endif

/**
 * omap_init_rng - bind the RNG hwmod to the RNG omap_device
 *
 * Bind the RNG hwmod to the RNG omap_device.  No return value.
 */
static void omap_init_rng(void)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;

	oh = omap_hwmod_lookup("rng");
	if (!oh)
		return;

	pdev = omap_device_build("omap_rng", -1, oh, NULL, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for omap_rng\n");
}

static void __init omap_init_sham(void)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;

	oh = omap_hwmod_lookup("sham");
	if (!oh)
		return;

	pdev = omap_device_build("omap-sham", -1, oh, NULL, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for omap-sham\n");
}

static void __init omap_init_aes(void)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;

	oh = omap_hwmod_lookup("aes");
	if (!oh)
		return;

	pdev = omap_device_build("omap-aes", -1, oh, NULL, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for omap-aes\n");
}

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_VIDEO_OMAP2_VOUT) || \
	defined(CONFIG_VIDEO_OMAP2_VOUT_MODULE)
#if defined(CONFIG_FB_OMAP2) || defined(CONFIG_FB_OMAP2_MODULE)
static struct resource omap_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};
#else
static struct resource omap_vout_resource[2] = {
};
#endif

static struct platform_device omap_vout_device = {
	.name		= "omap_vout",
	.num_resources	= ARRAY_SIZE(omap_vout_resource),
	.resource 	= &omap_vout_resource[0],
	.id		= -1,
};

int __init omap_init_vout(void)
{
	return platform_device_register(&omap_vout_device);
}
#else
int __init omap_init_vout(void) { return 0; }
#endif

/*-------------------------------------------------------------------------*/

static int __init omap2_init_devices(void)
{
	/* Enable dummy states for those platforms without pinctrl support */
	if (!of_have_populated_dt())
		pinctrl_provide_dummies();

	/*
	 * please keep these calls, and their implementations above,
	 * in alphabetical order so they're easier to sort through.
	 */
	omap_init_audio();
	/* If dtb is there, the devices will be created dynamically */
	if (!of_have_populated_dt()) {
		omap_init_mbox();
		omap_init_mcspi();
		omap_init_sham();
		omap_init_aes();
		omap_init_rng();
	}
	omap_init_sti();

	return 0;
}
omap_arch_initcall(omap2_init_devices);

#define AM33XX_EMAC_MDIO_FREQ		(1000000)

//static u64 am33xx_cpsw_dmamask = DMA_BIT_MASK(32);
/* TODO : Verify the offsets */
//static struct cpsw_slave_data am33xx_cpsw_slaves[] = {
//	{
// pass to cpsw_slave_init()
//		.slave_reg_ofs	= 0x208,
//		.sliver_reg_ofs	= 0xd80,
//		.phy_id		= "0:00",
//	},
//	{
// pass to cpsw_slave_init()
//		.slave_reg_ofs	= 0x308,
//		.sliver_reg_ofs	= 0xdc0,
//		.phy_id		= "0:01",
//	},
//};

/** FIXME: put these into DT **/
//static struct cpsw_platform_data am33xx_cpsw_pdata = {
//	.ss_reg_ofs		= 0x1200,
//	.channels		= 8,
//	.cpts_clock_shift	= 0x800,
//	.slaves			= 2,
//	.slave_data		= am33xx_cpsw_slaves,
//	.channels		= 0xd00,
//	.ale_entries		= 1024,
//removed ?
//	.host_port_reg_ofs      = 0x108,
//	.hw_stats_reg_ofs       = 0x900,
//	.bd_ram_ofs		= 0x2000,
//	.bd_ram_size		= SZ_8K,
//	.rx_descs               = 64,
//	.mac_control            = BIT(5), /* MIIEN */
//	.gigabit_en		= 1,
//	.cpts_clock_mult	= 0,
//	.no_bd_ram		= false,
//	.version		= CPSW_VERSION_2,
//};

//static struct mdio_platform_data am33xx_cpsw_mdiopdata = {
//	.bus_freq	= AM33XX_EMAC_MDIO_FREQ,
//};

//static struct resource am33xx_cpsw_mdioresources[] = {
//	{
//		.start  = AM33XX_CPSW_MDIO_BASE,
//		.end    = AM33XX_CPSW_MDIO_BASE + SZ_256 - 1,
//		.flags  = IORESOURCE_MEM,
//	},
//};

//static struct platform_device am33xx_cpsw_mdiodevice = {
//	.name           = "davinci_mdio",
//	.id             = 0,
//	.num_resources  = ARRAY_SIZE(am33xx_cpsw_mdioresources),
//	.resource       = am33xx_cpsw_mdioresources,
//	.dev.platform_data = &am33xx_cpsw_mdiopdata,
//};

//static struct resource am33xx_cpsw_resources[] = {
//	{
//		.start  = AM33XX_CPSW_BASE,
//		.end    = AM33XX_CPSW_BASE + SZ_2K - 1,
//		.flags  = IORESOURCE_MEM,
//	},
//	{
//		.start  = AM33XX_CPSW_SS_BASE,
//		.end    = AM33XX_CPSW_SS_BASE + SZ_256 - 1,
//		.flags  = IORESOURCE_MEM,
//	},
//	{
//		.start	= AM33XX_IRQ_CPSW_C0_RX,
//		.end	= AM33XX_IRQ_CPSW_C0_RX,
//		.flags	= IORESOURCE_IRQ,
//	},
//	{
//		.start	= AM33XX_IRQ_DMTIMER5,
//		.end	= AM33XX_IRQ_DMTIMER5,
//		.flags	= IORESOURCE_IRQ,
//	},
//	{
//		.start	= AM33XX_IRQ_DMTIMER6,
//		.end	= AM33XX_IRQ_DMTIMER6,
//		.flags	= IORESOURCE_IRQ,
//	},
//	{
//		.start	= AM33XX_IRQ_CPSW_C0,
//		.end	= AM33XX_IRQ_CPSW_C0,
//		.flags	= IORESOURCE_IRQ,
//	},
//};

//static struct platform_device am33xx_cpsw_device = {
//	.name		=	"cpsw",
//	.id		=	0,
//	.num_resources	=	ARRAY_SIZE(am33xx_cpsw_resources),
//	.resource	=	am33xx_cpsw_resources,
//	.dev		=	{
//					.platform_data	= &am33xx_cpsw_pdata,
//					.dma_mask	= &am33xx_cpsw_dmamask,
//					.coherent_dma_mask = DMA_BIT_MASK(32),
//				},
//};

static unsigned int   am33xx_evmid;

/*
* am33xx_evmid_fillup - set up board evmid
* @evmid - evm id which needs to be configured
*
* This function is called to configure board evm id.
* IA Motor Control EVM needs special setting of MAC PHY Id.
* This function is called when IA Motor Control EVM is detected
* during boot-up.
*/
void am33xx_evmid_fillup(unsigned int evmid)
{
	am33xx_evmid = evmid;
	return;
}

#define MII_MODE_ENABLE		0x0
#define RMII_MODE_ENABLE	0x5
#define RMII_MODE_ENABLE_EXT_CLK	0xF5
#define RGMII_MODE_ENABLE	0xA
#define MAC_MII_SEL		0x650

//void am33xx_cpsw_init(unsigned int gigen)
//{
//	u32 mac_lo, mac_hi;
//	u32 i;
//
//	mac_lo = omap_ctrl_readl(TI81XX_CONTROL_MAC_ID0_LO);
//	mac_hi = omap_ctrl_readl(TI81XX_CONTROL_MAC_ID0_HI);
//	am33xx_cpsw_slaves[0].mac_addr[0] = mac_hi & 0xFF;
//	am33xx_cpsw_slaves[0].mac_addr[1] = (mac_hi & 0xFF00) >> 8;
//	am33xx_cpsw_slaves[0].mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
//	am33xx_cpsw_slaves[0].mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
//	am33xx_cpsw_slaves[0].mac_addr[4] = mac_lo & 0xFF;
//	am33xx_cpsw_slaves[0].mac_addr[5] = (mac_lo & 0xFF00) >> 8;
//
//	mac_lo = omap_ctrl_readl(TI81XX_CONTROL_MAC_ID1_LO);
//	mac_hi = omap_ctrl_readl(TI81XX_CONTROL_MAC_ID1_HI);
//	am33xx_cpsw_slaves[1].mac_addr[0] = mac_hi & 0xFF;
//	am33xx_cpsw_slaves[1].mac_addr[1] = (mac_hi & 0xFF00) >> 8;
//	am33xx_cpsw_slaves[1].mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
//	am33xx_cpsw_slaves[1].mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
//	am33xx_cpsw_slaves[1].mac_addr[4] = mac_lo & 0xFF;
//	am33xx_cpsw_slaves[1].mac_addr[5] = (mac_lo & 0xFF00) >> 8;
//
//	if (am33xx_evmid == BEAGLE_BONE_OLD) {
//		printk("Ethernet MAC:RMII mode, internal clock\n");
//		__raw_writel(RMII_MODE_ENABLE,
//				AM33XX_CTRL_REGADDR(MAC_MII_SEL));
//	} else if (am33xx_evmid == BEAGLE_BONE_A3) {
//		printk("Ethernet MAC: MII mode\n");
//		__raw_writel(MII_MODE_ENABLE,
//				AM33XX_CTRL_REGADDR(MAC_MII_SEL));
//	} else if (am33xx_evmid == duagon_i10x_mii) {
//		printk("Ethernet MAC: MII mode\n");
//		__raw_writel(MII_MODE_ENABLE,
//				AM33XX_CTRL_REGADDR(MAC_MII_SEL));
//	} else if (am33xx_evmid == duagon_i10x_rmii) {
//		printk("Ethernet MAC: RMII external clock\n");
//		__raw_writel(RMII_MODE_ENABLE_EXT_CLK,
//				AM33XX_CTRL_REGADDR(MAC_MII_SEL));
//	} else if (am33xx_evmid == IND_AUT_MTR_EVM) {
//		snprintf(am33xx_cpsw_slaves[0].phy_id, sizeof(am33xx_cpsw_slaves[0].phy_id), "%s", "0:1e");
//		snprintf(am33xx_cpsw_slaves[1].phy_id, sizeof(am33xx_cpsw_slaves[1].phy_id), "%s", "0:00");
//	} else {
//		printk("Ethernet MAC: RGMII\n");
//		__raw_writel(RGMII_MODE_ENABLE,
//				AM33XX_CTRL_REGADDR(MAC_MII_SEL));
//	}
//
//	am33xx_cpsw_pdata.gigabit_en = gigen;
//
//	platform_device_register(&am33xx_cpsw_mdiodevice);
//	platform_device_register(&am33xx_cpsw_device);
//	clk_add_alias(NULL, dev_name(&am33xx_cpsw_mdiodevice.dev),
//			NULL, &am33xx_cpsw_device.dev);
//}

#define AM33XX_DCAN_NUM_MSG_OBJS		64
#define AM33XX_DCAN_RAMINIT_OFFSET		0x644
#define AM33XX_DCAN_RAMINIT_START(n)		(0x1 << n)

static void d_can_hw_raminit(unsigned int instance, unsigned int enable)
{
	u32 val;

	/* Read the value */
	val = readl(AM33XX_CTRL_REGADDR(AM33XX_DCAN_RAMINIT_OFFSET));
	if (enable) {
		/* Set to "1" */
		val &= ~AM33XX_DCAN_RAMINIT_START(instance);
		val |= AM33XX_DCAN_RAMINIT_START(instance);
		writel(val, AM33XX_CTRL_REGADDR(AM33XX_DCAN_RAMINIT_OFFSET));
	} else {
		/* Set to "0" */
		val &= ~AM33XX_DCAN_RAMINIT_START(instance);
		writel(val, AM33XX_CTRL_REGADDR(AM33XX_DCAN_RAMINIT_OFFSET));
	}
}

/* dcan dev_attr */
static struct d_can_platform_data am33xx_dcan_info = {
	.num_of_msg_objs	= AM33XX_DCAN_NUM_MSG_OBJS,
	.ram_init		= d_can_hw_raminit,
	.dma_support		= false,
};

void am33xx_d_can_init(unsigned int instance)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char oh_name[L3_MODULES_MAX_LEN];

	/* Copy string name to oh_name buffer */
	snprintf(oh_name, L3_MODULES_MAX_LEN, "d_can%d", instance);

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("could not find %s hwmod data\n", oh_name);
		return;
	}

	pdev = omap_device_build("d_can", instance, oh, &am33xx_dcan_info,
			sizeof(am33xx_dcan_info));
	if (IS_ERR(pdev))
		pr_err("could not build omap_device for %s\n", oh_name);
}

static int __init omap_gpmc_init(void)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char *oh_name = "gpmc";

	/*
	 * if the board boots up with a populated DT, do not
	 * manually add the device from this initcall
	 */
	if (of_have_populated_dt())
		return -ENODEV;

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -ENODEV;
	}

	pdev = omap_device_build("omap-gpmc", -1, oh, NULL, 0);
	WARN(IS_ERR(pdev), "could not build omap_device for %s\n", oh_name);

	return PTR_ERR_OR_ZERO(pdev);
}
omap_postcore_initcall(omap_gpmc_init);
