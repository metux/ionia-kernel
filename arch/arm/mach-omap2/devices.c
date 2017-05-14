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
#include <linux/platform_data/omap4-keypad.h>
#include <linux/platform_data/mailbox-omap.h>

#include <linux/mfd/ti_am335x_tscadc.h>
#include <linux/platform_data/uio_pruss.h>
#include <linux/dma-mapping.h>

#include <linux/platform_data/davinci_asp.h>

#include <linux/davinci_emac.h>
//#include <linux/platform_data/cpsw.h>
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

#include "omap4-keypad.h"
#ifdef	CONFIG_OMAP3_EDMA
#include <mach/edma.h>
#endif

#include <asm/hardware/davinci_asp.h>

/* LCD controller similar DA8xx */
#include <video/da8xx-fb.h>

#include "soc.h"
#include "common.h"
#include "mux.h"
#include "control.h"
#include "devices.h"
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
	if (!(cpu_is_omap34xx()) || (soc_is_am33xx()))
		return -ENODEV;

	snprintf(oh_name, L3_MODULES_MAX_LEN, "l3_main");

	oh = omap_hwmod_lookup(oh_name);

	if (!oh)
		pr_err("could not look up %s\n", oh_name);

	pdev = omap_device_build("omap_l3_smx", 0, oh, NULL, 0);

	WARN(IS_ERR(pdev), "could not build omap_device for %s\n", oh_name);

	return PTR_RET(pdev);
}
omap_postcore_initcall(omap3_l3_init);

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

#if defined(CONFIG_VIDEO_OMAP2) || defined(CONFIG_VIDEO_OMAP2_MODULE)

static struct resource omap2cam_resources[] = {
	{
		.start		= OMAP24XX_CAMERA_BASE,
		.end		= OMAP24XX_CAMERA_BASE + 0xfff,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= 24 + OMAP_INTC_START,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device omap2cam_device = {
	.name		= "omap24xxcam",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap2cam_resources),
	.resource	= omap2cam_resources,
};
#endif /* defined(CONFIG_VIDEO_OMAP2) || defined(CONFIG_VIDEO_OMAP2_MODULE) */

int __init am33xx_register_lcdc(struct da8xx_lcdc_platform_data *pdata)
{
	int id = 0;
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "lcdc";
	char *dev_name = "da8xx_lcdc";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up LCD%d hwmod\n", id);
		return -ENODEV;
	}

	pdev = omap_device_build(dev_name, id, oh, pdata,
			sizeof(struct da8xx_lcdc_platform_data));
	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
		return PTR_ERR(pdev);
	}
	return 0;
}

#if defined(CONFIG_SND_AM335X_SOC_EVM) || \
				defined(CONFIG_SND_AM335X_SOC_EVM_MODULE)
int __init am335x_register_mcasp(struct snd_platform_data *pdata, int ctrl_nr)
{
	int l;
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char oh_name[12];
	char *dev_name = "davinci-mcasp";

	l = snprintf(oh_name, 12, "mcasp%d", ctrl_nr);

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("could not look up %s\n", oh_name);
		return -ENODEV;
	}

	pdev = omap_device_build(dev_name, ctrl_nr, oh, pdata,
			sizeof(struct snd_platform_data), NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
	return IS_ERR(pdev) ? PTR_ERR(pdev) : 0;
}

#else
int __init am335x_register_mcasp(struct snd_platform_data *pdata, int ctrl_nr)
{
	return 0;
}
#endif

#if (defined(CONFIG_SND_AM33XX_SOC) || (defined(CONFIG_SND_AM33XX_SOC_MODULE)))
struct platform_device am33xx_pcm_device = {
	.name		= "davinci-pcm-audio",
	.id		= -1,
};

static void am33xx_init_pcm(void)
{
	platform_device_register(&am33xx_pcm_device);
}

#else
static inline void am33xx_init_pcm(void) {}
#endif

#if defined(CONFIG_IOMMU_API)

#include <linux/platform_data/iommu-omap.h>

static struct resource omap3isp_resources[] = {
	{
		.start		= OMAP3430_ISP_BASE,
		.end		= OMAP3430_ISP_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CCP2_BASE,
		.end		= OMAP3430_ISP_CCP2_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CCDC_BASE,
		.end		= OMAP3430_ISP_CCDC_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_HIST_BASE,
		.end		= OMAP3430_ISP_HIST_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_H3A_BASE,
		.end		= OMAP3430_ISP_H3A_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_PREV_BASE,
		.end		= OMAP3430_ISP_PREV_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_RESZ_BASE,
		.end		= OMAP3430_ISP_RESZ_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_SBL_BASE,
		.end		= OMAP3430_ISP_SBL_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CSI2A_REGS1_BASE,
		.end		= OMAP3430_ISP_CSI2A_REGS1_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CSIPHY2_BASE,
		.end		= OMAP3430_ISP_CSIPHY2_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3630_ISP_CSI2A_REGS2_BASE,
		.end		= OMAP3630_ISP_CSI2A_REGS2_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3630_ISP_CSI2C_REGS1_BASE,
		.end		= OMAP3630_ISP_CSI2C_REGS1_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3630_ISP_CSIPHY1_BASE,
		.end		= OMAP3630_ISP_CSIPHY1_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3630_ISP_CSI2C_REGS2_BASE,
		.end		= OMAP3630_ISP_CSI2C_REGS2_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP343X_CTRL_BASE + OMAP343X_CONTROL_CSIRXFE,
		.end		= OMAP343X_CTRL_BASE + OMAP343X_CONTROL_CSIRXFE + 3,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP343X_CTRL_BASE + OMAP3630_CONTROL_CAMERA_PHY_CTRL,
		.end		= OMAP343X_CTRL_BASE + OMAP3630_CONTROL_CAMERA_PHY_CTRL + 3,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= 24 + OMAP_INTC_START,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device omap3isp_device = {
	.name		= "omap3isp",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3isp_resources),
	.resource	= omap3isp_resources,
};

static struct omap_iommu_arch_data omap3_isp_iommu = {
	.name = "mmu_isp",
};

int omap3_init_camera(struct isp_platform_data *pdata)
{
	omap3isp_device.dev.platform_data = pdata;
	omap3isp_device.dev.archdata.iommu = &omap3_isp_iommu;

	return platform_device_register(&omap3isp_device);
}

#else /* !CONFIG_IOMMU_API */

int omap3_init_camera(struct isp_platform_data *pdata)
{
	return 0;
}

#endif

static inline void omap_init_camera(void)
{
#if defined(CONFIG_VIDEO_OMAP2) || defined(CONFIG_VIDEO_OMAP2_MODULE)
	if (cpu_is_omap24xx())
		platform_device_register(&omap2cam_device);
#endif
}

int __init omap4_keyboard_init(struct omap4_keypad_platform_data
			*sdp4430_keypad_data, struct omap_board_data *bdata)
{
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	struct omap4_keypad_platform_data *keypad_data;
	unsigned int id = -1;
	char *oh_name = "kbd";
	char *name = "omap4-keypad";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -ENODEV;
	}

	keypad_data = sdp4430_keypad_data;

	pdev = omap_device_build(name, id, oh, keypad_data,
				 sizeof(struct omap4_keypad_platform_data));

	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
						name, oh->name);
		return PTR_ERR(pdev);
	}
	oh->mux = omap_hwmod_mux_init(bdata->pads, bdata->pads_cnt);

	return 0;
}

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

#if defined(CONFIG_SND_OMAP_SOC_OMAP_HDMI) || \
		defined(CONFIG_SND_OMAP_SOC_OMAP_HDMI_MODULE)

static struct platform_device omap_hdmi_audio = {
	.name	= "omap-hdmi-audio",
	.id	= -1,
};

static void __init omap_init_hdmi_audio(void)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;

	oh = omap_hwmod_lookup("dss_hdmi");
	if (!oh)
		return;

	pdev = omap_device_build("omap-hdmi-audio-dai", -1, oh, NULL, 0);
	WARN(IS_ERR(pdev),
	     "Can't build omap_device for omap-hdmi-audio-dai.\n");

	platform_device_register(&omap_hdmi_audio);
}
#else
static inline void omap_init_hdmi_audio(void) {}
#endif

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

int __init omap_init_elm(void)
{
	int id = -1;
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "elm";
	char *name = "omap2_elm";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -ENODEV;
	}

	pdev = omap_device_build(name, id, oh, NULL, 0);

	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
						name, oh->name);
		return PTR_ERR(pdev);
	}

	return 0;
}

#ifdef CONFIG_SOC_AM33XX
#define PWM_STR_LEN 10
int __init am33xx_register_ecap(int id)
{
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "ecap";
	char dev_name[PWM_STR_LEN];

	sprintf(dev_name, "ecap.%d", id);

	oh = omap_hwmod_lookup(dev_name);
	if (!oh) {
		pr_err("Could not look up %s hwmod\n", dev_name);
		return -ENODEV;
	}

	pdev = omap_device_build(oh_name, id, oh, NULL, 0);

	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
		return PTR_ERR(pdev);
	}
	return 0;
}

int __init am33xx_register_ehrpwm(int id)
{
	struct platform_device *pdev;
	struct omap_hwmod *oh;
	char *oh_name = "ehrpwm";
	char dev_name[PWM_STR_LEN];

	sprintf(dev_name, "ehrpwm.%d", id);

	oh = omap_hwmod_lookup(dev_name);
	if (!oh) {
		pr_err("Could not look up %s hwmod\n", dev_name);
		return -ENODEV;
	}

	pdev = omap_device_build(oh_name, id, oh, NULL, 0);

	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
		return PTR_ERR(pdev);
	}
	return 0;
}

#else
static int __init am335x_register_ehrpwm(int id) { }
static int __init am335x_register_ecap(int id) { }
#endif

//#if defined(CONFIG_CRYPTO_DEV_OMAP_AES) || defined(CONFIG_CRYPTO_DEV_OMAP_AES_MODULE)

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

//#endif

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

#if 0 /* FIXME: disabled until upstream keeps up */
//#if defined(CONFIG_SOC_AM33XX) && defined(CONFIG_OMAP3_EDMA)

#define AM33XX_SCM_BASE_EDMA		0x00000f90

static const s16 am33xx_dma_rsv_chans[][2] = {
	/* (offset, number) */
	{0, 2},
	{14, 2},
	{26, 6},
	{48, 4},
	{56, 8},
	{-1, -1}
};

static const s16 am33xx_dma_rsv_slots[][2] = {
	/* (offset, number) */
	{0, 2},
	{14, 2},
	{26, 6},
	{48, 4},
	{56, 8},
	{64, 127},
	{-1, -1}
};

/* Three Transfer Controllers on AM33XX */
static const s8 am33xx_queue_tc_mapping[][2] = {
	/* {event queue no, TC no} */
	{0, 0},
	{1, 1},
	{2, 2},
	{-1, -1}
};

static const s8 am33xx_queue_priority_mapping[][2] = {
	/* {event queue no, Priority} */
	{0, 0},
	{1, 1},
	{2, 2},
	{-1, -1}
};

static struct event_to_channel_map am33xx_xbar_event_mapping[] = {
	/* {xbar event no, Channel} */
	{1, 12},	/* SDTXEVT1 -> MMCHS2 */
	{2, 13},	/* SDRXEVT1 -> MMCHS2 */
	{3, -1},
	{4, -1},
	{5, -1},
	{6, -1},
	{7, -1},
	{8, -1},
	{9, -1},
	{10, -1},
	{11, -1},
	{12, -1},
	{13, -1},
	{14, -1},
	{15, -1},
	{16, -1},
	{17, -1},
	{18, -1},
	{19, -1},
	{20, -1},
	{21, -1},
	{22, -1},
	{23, -1},
	{24, -1},
	{25, -1},
	{26, -1},
	{27, -1},
	{28, -1},
	{29, -1},
	{30, -1},
	{31, -1},
	{-1, -1}
};

/**
 * map_xbar_event_to_channel - maps a crossbar event to a DMA channel
 * according to the configuration provided
 * @event: the event number for which mapping is required
 * @channel: channel being activated
 * @xbar_event_mapping: array that has the event to channel map
 *
 * Events that are routed by default are not mapped. Only events that
 * are crossbar mapped are routed to available channels according to
 * the configuration provided
 *
 * Returns zero on success, else negative errno.
 */
int map_xbar_event_to_channel(unsigned int event, unsigned int *channel,
			struct event_to_channel_map *xbar_event_mapping)
{
	unsigned int ctrl = 0;
	unsigned int xbar_evt_no = 0;
	unsigned int val = 0;
	unsigned int offset = 0;
	unsigned int mask = 0;

	ctrl = EDMA_CTLR(event);
	xbar_evt_no = event - (edma_cc[ctrl]->num_channels);

	if (event < edma_cc[ctrl]->num_channels) {
		*channel = event;
	} else if (event < edma_cc[ctrl]->num_events) {
		*channel = xbar_event_mapping[xbar_evt_no].channel_no;
		/* confirm the range */
		if (*channel < EDMA_MAX_DMACH)
			clear_bit(*channel, edma_cc[ctrl]->edma_unused);
		mask = (*channel)%4;
		offset = (*channel)/4;
		offset *= 4;
		offset += mask;
		val = (unsigned int)__raw_readl(AM33XX_CTRL_REGADDR(
					AM33XX_SCM_BASE_EDMA + offset));
		val = val & (~(0xFF));
		val = val | (xbar_event_mapping[xbar_evt_no].xbar_event_no);
		__raw_writel(val,
			AM33XX_CTRL_REGADDR(AM33XX_SCM_BASE_EDMA + offset));
		return 0;
	} else {
		return -EINVAL;
	}

	return 0;
}

static struct edma_soc_info am33xx_edma_info[] = {
	{
		.n_channel		= 64,
		.n_region		= 4,
		.n_slot			= 256,
		.n_tc			= 3,
		.n_cc			= 1,
		.rsv_chans		= am33xx_dma_rsv_chans,
		.rsv_slots		= am33xx_dma_rsv_slots,
		.queue_tc_mapping	= am33xx_queue_tc_mapping,
		.queue_priority_mapping	= am33xx_queue_priority_mapping,
		.is_xbar		= 1,
		.n_events		= 95,
		.xbar_event_mapping	= am33xx_xbar_event_mapping,
		.map_xbar_channel	= map_xbar_event_to_channel,
	},
};

static int __init am33xx_register_edma(void)
{
	int i, l;
	struct omap_hwmod *oh[4];
	struct platform_device *pdev;
	struct edma_soc_info *pdata = am33xx_edma_info;
	char oh_name[8];

	if (!soc_is_am33xx())
		return -ENODEV;

	oh[0] = omap_hwmod_lookup("tpcc");
	if (!oh[0]) {
		pr_err("could not look up %s\n", "tpcc");
		return -ENODEV;
	}

	for (i = 0; i < 3; i++) {
		l = snprintf(oh_name, 8, "tptc%d", i);

		oh[i+1] = omap_hwmod_lookup(oh_name);
		if (!oh[i+1]) {
			pr_err("could not look up %s\n", oh_name);
			return -ENODEV;
		}
	}

	pdev = omap_device_build_ss("edma", 0, oh, 4, pdata, sizeof(*pdata));

	WARN(IS_ERR(pdev), "could not build omap_device for edma\n");

	return IS_ERR(pdev) ? PTR_ERR(pdev) : 0;

}

#else
static inline void am33xx_register_edma(void) {}
#endif

#if defined (CONFIG_SOC_AM33XX)
struct uio_pruss_pdata am335x_pruss_uio_pdata = {
	.pintc_base	= 0x20000,
};

static struct resource am335x_pruss_resources[] = {
	{
		.start	= AM33XX_ICSS_BASE,
		.end	= AM33XX_ICSS_BASE + AM33XX_ICSS_LEN,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_0,
		.end	= AM33XX_IRQ_ICSS0_0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_1,
		.end	= AM33XX_IRQ_ICSS0_1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_2,
		.end	= AM33XX_IRQ_ICSS0_2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_3,
		.end	= AM33XX_IRQ_ICSS0_3,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_4,
		.end	= AM33XX_IRQ_ICSS0_4,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_5,
		.end	= AM33XX_IRQ_ICSS0_5,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_6,
		.end	= AM33XX_IRQ_ICSS0_6,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= AM33XX_IRQ_ICSS0_7,
		.end	= AM33XX_IRQ_ICSS0_7,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device am335x_pruss_uio_dev = {
	.name		= "pruss_uio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am335x_pruss_resources),
	.resource	= am335x_pruss_resources,
	.dev	 =	{
		.coherent_dma_mask = 0xffffffff,
	}
};

int __init am335x_register_pruss_uio(struct uio_pruss_pdata *config)
{
	am335x_pruss_uio_dev.dev.platform_data = config;
	return platform_device_register(&am335x_pruss_uio_dev);
}

static struct platform_device am335x_sgx = {
	.name	= "sgx",
	.id	= -1,
};

#endif /* CONFIG_SOC_AM33XX */

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
	omap_init_camera();
	omap_init_hdmi_audio();
	omap_init_mbox();
	/* If dtb is there, the devices will be created dynamically */
	if (!of_have_populated_dt()) {
		omap_init_mcspi();
		omap_init_sham();
		omap_init_aes();
		omap_init_rng();
	}
	omap_init_sti();

	am33xx_register_edma();
	am33xx_init_pcm();
#if defined (CONFIG_SOC_AM33XX)
	am335x_register_pruss_uio(&am335x_pruss_uio_pdata);
	if (omap3_has_sgx())
		platform_device_register(&am335x_sgx);
#endif
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

static unsigned char  am33xx_macid0[ETH_ALEN];
static unsigned char  am33xx_macid1[ETH_ALEN];
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

/*
* am33xx_cpsw_macidfillup - setup mac adrresses
* @eeprommacid0 - mac id 0 which needs to be configured
* @eeprommacid1 - mac id 1 which needs to be configured
*
* This function is called to configure mac addresses.
* Mac addresses are read from eeprom and this function is called
* to store those mac adresses in am33xx_macid0 and am33xx_macid1.
* In case, mac address read from eFuse are invalid, mac addresses
* stored in these variable are used.
*/
void am33xx_cpsw_macidfillup(char *eeprommacid0, char *eeprommacid1)
{
	u32 i;

	/* Fillup these mac addresses with the mac adresses from eeprom */
	for (i = 0; i < ETH_ALEN; i++) {
		am33xx_macid0[i] = eeprommacid0[i];
		am33xx_macid1[i] = eeprommacid1[i];
	}

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
//	/* Read MACID0 from eeprom if eFuse MACID is invalid */
//	if (!is_valid_ether_addr(am33xx_cpsw_slaves[0].mac_addr)) {
//		for (i = 0; i < ETH_ALEN; i++)
//			am33xx_cpsw_slaves[0].mac_addr[i] = am33xx_macid0[i];
//	}
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
//	/* Read MACID1 from eeprom if eFuse MACID is invalid */
//	if (!is_valid_ether_addr(am33xx_cpsw_slaves[1].mac_addr)) {
//		for (i = 0; i < ETH_ALEN; i++)
//			am33xx_cpsw_slaves[1].mac_addr[i] = am33xx_macid1[i];
//	}
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

int __init omap_init_gpmc(struct gpmc_devices_info *pdata, int pdata_len)
{
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char *name = "omap-gpmc";
	char *oh_name = "gpmc";

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		pr_err("Could not look up %s\n", oh_name);
		return -ENODEV;
	}

	pdev = omap_device_build(name, -1, oh, pdata, pdata_len);
	if (IS_ERR(pdev)) {
		WARN(1, "Can't build omap_device for %s:%s.\n",
						name, oh->name);
		return PTR_ERR(pdev);
	}

	return 0;
}
