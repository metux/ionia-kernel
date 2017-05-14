/*
 * Texas Instruments TI81XX "usb platform glue layer"
 *
 * Copyright (c) 2008, MontaVista Software, Inc. <source@mvista.com>
 *
 * Based on the DaVinci "glue layer" code.
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/usb/otg.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/platform_data/usb-omap.h>

#include "ti81xx.h"

#include "musb_core.h"

// FIXME!
#define cpu_is_ti816x()	(0)

#ifdef CONFIG_PM
struct ti81xx_usbss_regs {
	u32	sysconfig;

	u32	irq_en_set;
};

struct ti81xx_usb_regs {
	u32	control;

	u32	irq_en_set[2];

	u32	srp_fix;
	u32	phy_utmi;
	u32	mgc_utmi_loopback;
	u32	mode;
};
#endif

struct ti81xx_glue {
	struct device *dev;
	struct resource *mem_pa;	/* usbss memory resource */
	void *mem_va;			/* ioremapped virtual address */
	struct platform_device *musb[2];/* child musb pdevs */
	struct platform_device *phy[2];
	u8	irq;			/* usbss irq */
	u8	first;			/* ignore first call of resume */

#ifdef CONFIG_PM
	struct ti81xx_usbss_regs usbss_regs;
	struct ti81xx_usb_regs usb_regs[2];
#endif
};

static u64 musb_dmamask = DMA_BIT_MASK(32);
static void *usbss_virt_base;
static u8 usbss_init_done;
struct musb *gmusb[2];

u8 usbid_sw_ctrl;
#undef USB_TI81XX_DEBUG

#ifdef USB_TI81XX_DEBUG
#define	dprintk(x, ...) printk(x, ## __VA_ARGS__)
#else
#define dprintk(x, ...)
#endif

extern void omap_ctrl_writel(u32 val, u16 offset);
extern u32 omap_ctrl_readl(u16 offset);

static inline u32 usbss_read(u32 offset)
{
	if (!usbss_init_done)
		return 0;
	return readl(usbss_virt_base + offset);
}

static inline void usbss_write(u32 offset, u32 data)
{
	if (!usbss_init_done)
		return ;
	writel(data, usbss_virt_base + offset);
}

static void usbotg_ss_init(void)
{
	if (!usbss_init_done) {
		usbss_init_done = 1;

		/* clear any USBSS interrupts */
		usbss_write(USBSS_IRQ_EOI, 0);
		usbss_write(USBSS_IRQ_STATUS, usbss_read(USBSS_IRQ_STATUS));
	}
}
static void usbotg_ss_uninit(void)
{
	if (usbss_init_done) {
		usbss_init_done = 0;
		usbss_virt_base = 0;
	}
}
void set_frame_threshold(struct musb *musb, u8 is_tx, u8 epnum, u8 value, u8 en_intr)
{
	u32     base, reg_val, frame_intr = 0, frame_base = 0;
	u32     offs = epnum/4*4;
	u8      indx = (epnum % 4) * 8;

	if (is_tx)
		base = musb->id ? USBSS_IRQ_FRAME_THRESHOLD_TX1 :
				USBSS_IRQ_FRAME_THRESHOLD_TX0;
	else
		base = musb->id ? USBSS_IRQ_FRAME_THRESHOLD_RX1 :
				USBSS_IRQ_FRAME_THRESHOLD_RX0;

	reg_val = usbss_read(base + offs);
	reg_val &= ~(0xFF << indx);
	reg_val |= (value << indx);
	usbss_write(base + offs, reg_val);

	if (en_intr) {
		frame_base = musb->id ? USBSS_IRQ_FRAME_ENABLE_1 :
			USBSS_IRQ_FRAME_ENABLE_0;
		frame_intr = musb->id ? usbss_read(USBSS_IRQ_FRAME_ENABLE_0) :
			usbss_read(USBSS_IRQ_FRAME_ENABLE_1);
		frame_intr |= is_tx ? (1 << epnum) : (1 << (16 + epnum));
		usbss_write(frame_base, frame_intr);
		dev_dbg(musb->controller, "%s: framebase=%x, frame_intr=%x\n",
			is_tx ? "tx" : "rx", frame_base, frame_intr);
	}
}

void set_dma_threshold(struct musb *musb, u8 is_tx, u8 epnum, u8 value)
{
	u32     base, reg_val;
	u32     offs = epnum/4*4;
	u8      indx = (epnum % 4) * 8;

	if (musb->id == 0)
		base = is_tx ? USBSS_IRQ_DMA_THRESHOLD_TX0 :
				USBSS_IRQ_DMA_THRESHOLD_RX0;
	else
		base = is_tx ? USBSS_IRQ_DMA_THRESHOLD_TX1 :
				USBSS_IRQ_DMA_THRESHOLD_RX1;

	reg_val = usbss_read(base + offs);
	reg_val &= ~(0xFF << indx);
	reg_val |= (value << indx);
	dev_dbg(musb->controller, "base=%x, offs=%x, indx=%d, reg_val = (%x)%x\n",
		base, offs, indx, reg_val, usbss_read(base + offs));
	usbss_write(base + offs, reg_val);
}

/* ti81xx specific read/write functions */
u16 ti81xx_musb_readw(const void __iomem *addr, unsigned offset)
{
	u32 tmp;
	u16 val;

	tmp = readl(addr + (offset & ~3));

	switch (offset & 0x3) {
	case 0:
		val = (tmp & 0xffff);
		break;
	case 1:
		val = (tmp >> 8) & 0xffff;
		break;
	case 2:
	case 3:
	default:
		val = (tmp >> 16) & 0xffff;
		break;
	}
	return val;
}

void ti81xx_musb_writew(void __iomem *addr, unsigned offset, u16 data)
{
	__raw_writew(data, addr + offset);
}

u8 ti81xx_musb_readb(const void __iomem *addr, unsigned offset)
{
	u32 tmp;
	u8 val;

	tmp = readl(addr + (offset & ~3));

	switch (offset & 0x3) {
	case 0:
		val = tmp & 0xff;
		break;
	case 1:
		val = (tmp >> 8);
		break;
	case 2:
		val = (tmp >> 16);
		break;
	case 3:
	default:
		val = (tmp >> 24);
		break;
	}
	return val;
}
void ti81xx_musb_writeb(void __iomem *addr, unsigned offset, u8 data)
{
	__raw_writeb(data, addr + offset);
}

/*
 * Because we don't set CTRL.UINT, it's "important" to:
 *	- not read/write INTRUSB/INTRUSBE (except during
 *	  initial setup, as a workaround);
 *	- use INTSET/INTCLR instead.
 */

/**
 * ti81xx_musb_enable - enable interrupts
 */
void ti81xx_musb_enable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	u32 epmask, coremask;

	/* Workaround: setup IRQs through both register sets. */
	epmask = ((musb->epmask & USB_TX_EP_MASK) << USB_INTR_TX_SHIFT) |
	       ((musb->epmask & USB_RX_EP_MASK) << USB_INTR_RX_SHIFT);
	coremask = (0x01ff << USB_INTR_USB_SHIFT);

	coremask &= ~MUSB_INTR_SOF;

	musb_writel(reg_base, USB_EP_INTR_SET_REG, epmask);
	musb_writel(reg_base, USB_CORE_INTR_SET_REG, coremask);
	/* Force the DRVVBUS IRQ so we can start polling for ID change. */
	musb_writel(reg_base, USB_CORE_INTR_SET_REG,
		    USB_INTR_DRVVBUS << USB_INTR_USB_SHIFT);
}

/**
 * ti81xx_musb_disable - disable HDRC and flush interrupts
 */
void ti81xx_musb_disable(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;

	musb_writel(reg_base, USB_CORE_INTR_CLEAR_REG, USB_INTR_USB_MASK);
	musb_writel(reg_base, USB_EP_INTR_CLEAR_REG,
			 USB_TX_INTR_MASK | USB_RX_INTR_MASK);
	musb_writeb(musb->mregs, MUSB_DEVCTL, 0);
	musb_writel(reg_base, USB_IRQ_EOI, 0);
}

#define	POLL_SECONDS	2

static void otg_timer(unsigned long _musb)
{
	struct musb		*musb = (void *)_musb;
	void __iomem		*mregs = musb->mregs;
	u8			devctl;
	unsigned long		flags;

	/* We poll because DaVinci's won't expose several OTG-critical
	* status change events (from the transceiver) otherwise.
	 */
	devctl = musb_readb(mregs, MUSB_DEVCTL);
	dev_dbg(musb->controller, "Poll devctl %02x (%s)\n", devctl,
			usb_otg_state_string(musb->xceiv->otg->state));

	spin_lock_irqsave(&musb->lock, flags);
	switch (musb->xceiv->otg->state) {
	case OTG_STATE_A_WAIT_BCON:
		devctl &= ~MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_HM) {
			musb->xceiv->otg->state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		} else {
			musb->xceiv->otg->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
			mod_timer(&musb->otg_workaround,
					jiffies + POLL_SECONDS * HZ);
		}
		break;
	case OTG_STATE_A_WAIT_VFALL:
		/*
		 * Wait till VBUS falls below SessionEnd (~0.2 V); the 1.3
		 * RTL seems to mis-handle session "start" otherwise (or in
		 * our case "recover"), in routine "VBUS was valid by the time
		 * VBUSERR got reported during enumeration" cases.
		 */
		if (devctl & MUSB_DEVCTL_VBUS) {
			mod_timer(&musb->otg_workaround,
					jiffies + POLL_SECONDS * HZ);
			break;
		}
		musb->xceiv->otg->state = OTG_STATE_A_WAIT_VRISE;
		musb_writel(musb->ctrl_base, USB_CORE_INTR_SET_REG,
			    MUSB_INTR_VBUSERROR << USB_INTR_USB_SHIFT);
		break;
	case OTG_STATE_B_IDLE:
		/*
		 * There's no ID-changed IRQ, so we have no good way to tell
		 * when to switch to the A-Default state machine (by setting
		 * the DEVCTL.SESSION flag).
		 *
		 * Workaround:  whenever we're in B_IDLE, try setting the
		 * session flag every few seconds.  If it works, ID was
		 * grounded and we're now in the A-Default state machine.
		 *
		 * NOTE: setting the session flag is _supposed_ to trigger
		 * SRP but clearly it doesn't.
		 */
		devctl = musb_readb(mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_HM) {
			musb->xceiv->otg->state = OTG_STATE_A_IDLE;
		} else {
			mod_timer(&musb->otg_workaround,
					jiffies + POLL_SECONDS * HZ);
			musb_writeb(musb->mregs, MUSB_DEVCTL, devctl |
				MUSB_DEVCTL_SESSION);
		}
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}

void ti81xx_musb_try_idle(struct musb *musb, unsigned long timeout)
{
	if (timeout == 0)
		timeout = jiffies + msecs_to_jiffies(3);

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || (musb->a_wait_bcon == 0 &&
				musb->xceiv->otg->state == OTG_STATE_A_WAIT_BCON)) {
		dev_dbg(musb->controller, "%s active, deleting timer\n",
			usb_otg_state_string(musb->xceiv->otg->state));
		del_timer(&musb->otg_workaround);
		musb->last_timer = jiffies;
		return;
	}

	if (time_after(musb->last_timer, timeout) &&
					timer_pending(&musb->otg_workaround)) {
		dev_dbg(musb->controller, "Longer idle timer already pending, ignoring...\n");
		return;
	}
	musb->last_timer = timeout;

	dev_dbg(musb->controller, "%s inactive, starting idle timer for %u ms\n",
	    usb_otg_state_string(musb->xceiv->otg->state),
		jiffies_to_msecs(timeout - jiffies));
	mod_timer(&musb->otg_workaround, timeout);
}

void musb_babble_workaround(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;

	/* Reset the controller */
	musb_writel(reg_base, USB_CTRL_REG, USB_SOFT_RESET_MASK);
	udelay(100);

	/* Shutdown the on-chip PHY and its PLL. */
	if (data->set_phy_power)
		data->set_phy_power(0);
	udelay(100);

	musb_platform_set_mode(musb, MUSB_HOST);
	udelay(100);

	/* enable the usbphy */
	if (data->set_phy_power)
		data->set_phy_power(1);
	mdelay(100);

	/* save the usbotgss register contents */
	musb_platform_enable(musb);

	musb_start(musb);
}

static void evm_deferred_musb_restart(struct work_struct *work)
{
	struct musb *musb =
		container_of(work, struct musb, work);

	ERR("deferred musb restart musbid(%d)\n", musb->id);
	musb_babble_workaround(musb);
}

static irqreturn_t ti81xx_interrupt(int irq, void *hci)
{
	struct musb  *musb = hci;
	void __iomem *reg_base = musb->ctrl_base;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	u32 pend1 = 0, pend2 = 0;
	u32 epintr, usbintr;
	u8  is_babble = 0;

	spin_lock_irqsave(&musb->lock, flags);

	/* Acknowledge and handle non-CPPI interrupts */
	/* Get endpoint interrupts */
	epintr = musb_readl(reg_base, USB_EP_INTR_STATUS_REG);
	musb->int_rx = (epintr & USB_RX_INTR_MASK) >> USB_INTR_RX_SHIFT;
	musb->int_tx = (epintr & USB_TX_INTR_MASK) >> USB_INTR_TX_SHIFT;
	if (epintr)
		musb_writel(reg_base, USB_EP_INTR_STATUS_REG, epintr);

	/* Get usb core interrupts */
	usbintr = musb_readl(reg_base, USB_CORE_INTR_STATUS_REG);
	if (!usbintr && !epintr) {
		dev_dbg(musb->controller, "sprious interrupt\n");
		goto eoi;
	}

	if (usbintr)
		musb_writel(reg_base, USB_CORE_INTR_STATUS_REG, usbintr);
	musb->int_usb =	(usbintr & USB_INTR_USB_MASK) >> USB_INTR_USB_SHIFT;

	dev_dbg(musb->controller, "usbintr (%x) epintr(%x)\n", usbintr, epintr);
	/*
	 * DRVVBUS IRQs are the only proxy we have (a very poor one!) for
	 * AM3517's missing ID change IRQ.  We need an ID change IRQ to
	 * switch appropriately between halves of the OTG state machine.
	 * Managing DEVCTL.SESSION per Mentor docs requires that we know its
	 * value but DEVCTL.BDEVICE is invalid without DEVCTL.SESSION set.
	 * Also, DRVVBUS pulses for SRP (but not at 5V) ...
	 */
	if ((usbintr & MUSB_INTR_BABBLE)
		&& (musb->xceiv->otg->state == OTG_STATE_A_HOST))
		is_babble = 1;

	if (is_babble) {
		if (musb->enable_babble_work)
			musb->int_usb |= MUSB_INTR_DISCONNECT;

		ERR("CAUTION: musb%d: Babble Interrupt Occured\n", musb->id);
		ERR("Please issue long reset to make usb functional !!\n");
	}

	if (usbintr & (USB_INTR_DRVVBUS << USB_INTR_USB_SHIFT)) {
		int drvvbus = musb_readl(reg_base, USB_STAT_REG);
		void __iomem *mregs = musb->mregs;
		u8 devctl = musb_readb(mregs, MUSB_DEVCTL);
		int err;

		err = (musb->int_usb & MUSB_INTR_VBUSERROR);
		if (err) {
			/*
			 * The Mentor core doesn't debounce VBUS as needed
			 * to cope with device connect current spikes. This
			 * means it's not uncommon for bus-powered devices
			 * to get VBUS errors during enumeration.
			 *
			 * This is a workaround, but newer RTL from Mentor
			 * seems to allow a better one: "re"-starting sessions
			 * without waiting for VBUS to stop registering in
			 * devctl.
			 */
			musb->int_usb &= ~MUSB_INTR_VBUSERROR;
			musb->xceiv->otg->state = OTG_STATE_A_WAIT_VFALL;
			mod_timer(&musb->otg_workaround,
						jiffies + POLL_SECONDS * HZ);
			WARNING("VBUS error workaround (delay coming)\n");
		} else if (drvvbus) {
			musb->is_active = 1;
			MUSB_HST_MODE(musb);
			musb->xceiv->otg->default_a = 1;
			musb->xceiv->otg->state = OTG_STATE_A_WAIT_VRISE;
			del_timer(&musb->otg_workaround);
		} else {
			musb->is_active = 0;
			MUSB_DEV_MODE(musb);
			musb->xceiv->otg->default_a = 0;
			musb->xceiv->otg->state = OTG_STATE_B_IDLE;
		}

		/* NOTE: this must complete power-on within 100 ms. */
		dev_dbg(musb->controller, "VBUS %s (%s)%s, devctl %02x\n",
				drvvbus ? "on" : "off",
				usb_otg_state_string(musb->xceiv->otg->state),
				err ? " ERROR" : "",
				devctl);
		ret = IRQ_HANDLED;
	}

	if (musb->int_tx || musb->int_rx || musb->int_usb)
		ret |= musb_interrupt(musb);

 eoi:
	/* EOI needs to be written for the IRQ to be re-asserted. */
	if (ret == IRQ_HANDLED || epintr || usbintr) {
		/* write EOI */
		musb_writel(reg_base, USB_IRQ_EOI, 1);
	}

	/* Poll for ID change */
	if (musb->xceiv->otg->state == OTG_STATE_B_IDLE)
		mod_timer(&musb->otg_workaround, jiffies + POLL_SECONDS * HZ);

	spin_unlock_irqrestore(&musb->lock, flags);

	if (ret != IRQ_HANDLED) {
		if (epintr || usbintr)
			/*
			 * We sometimes get unhandled IRQs in the peripheral
			 * mode from EP0 and SOF...
			 */
			dev_dbg(musb->controller, "Unhandled USB IRQ %08x-%08x\n",
					 epintr, usbintr);
		else if (printk_ratelimit())
			/*
			 * We've seen series of spurious interrupts in the
			 * peripheral mode after USB reset and then after some
			 * time a real interrupt storm starting...
			 */
			dev_dbg(musb->controller, "Spurious IRQ, CPPI 4.1 status %08x, %08x\n",
					 pend1, pend2);
	}

	if (is_babble) {
		if (!musb->enable_babble_work) {
			musb_writeb(musb->mregs, MUSB_DEVCTL,
				musb_readb(musb->mregs, MUSB_DEVCTL) |
				MUSB_DEVCTL_SESSION);
		} else {
			ERR("Babble: devtcl(%x)Restarting musb....\n",
				 musb_readb(musb->mregs, MUSB_DEVCTL));
			schedule_work(&musb->work);
		}
	}
	return ret;
}
int ti81xx_musb_set_mode(struct musb *musb, u8 musb_mode)
{
	void __iomem *reg_base = musb->ctrl_base;
	u32 regval;

	/* TODO: implement this using CONF0 */
	if (musb_mode == MUSB_HOST) {
		regval = musb_readl(reg_base, USB_MODE_REG);

		regval &= ~USBMODE_USBID_HIGH;
		if (usbid_sw_ctrl && cpu_is_ti816x())
			regval |= USBMODE_USBID_MUXSEL;

		musb_writel(reg_base, USB_MODE_REG, regval);
		musb_writel(musb->ctrl_base, USB_PHY_UTMI_REG, 0x02);
		dev_dbg(musb->controller, "host: value of mode reg=%x regval(%x)\n",
			musb_readl(reg_base, USB_MODE_REG), regval);
	} else if (musb_mode == MUSB_PERIPHERAL) {
		/* TODO commmented writing 8 to USB_MODE_REG device
			mode is not working */
		regval = musb_readl(reg_base, USB_MODE_REG);

		regval |= USBMODE_USBID_HIGH;
		if (usbid_sw_ctrl && cpu_is_ti816x())
			regval |= USBMODE_USBID_MUXSEL;

		musb_writel(reg_base, USB_MODE_REG, regval);
		dev_dbg(musb->controller, "device: value of mode reg=%x regval(%x)\n",
			musb_readl(reg_base, USB_MODE_REG), regval);
	} else if (musb_mode == MUSB_OTG) {
		musb_writel(musb->ctrl_base, USB_PHY_UTMI_REG, 0x02);
	} else
		return -EIO;

	return 0;
}

int ti81xx_musb_init(struct musb *musb)
{
	void __iomem *reg_base = musb->ctrl_base;
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	u32 rev;
	u8 mode;

	if (musb->id < 2)
		gmusb[musb->id] = musb;

	musb->xceiv = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!musb->xceiv)
		return -ENODEV;

	/* mentor is at offset of 0x400 in am3517/ti81xx */
	musb->mregs += USB_MENTOR_CORE_OFFSET;

	/* Returns zero if e.g. not clocked */
	rev = musb_readl(reg_base, USB_REVISION_REG);
	if (!rev)
		return -ENODEV;

	setup_timer(&musb->otg_workaround, otg_timer, (unsigned long) musb);

	/* Reset the controller */
	musb_writel(reg_base, USB_CTRL_REG, USB_SOFT_RESET_MASK);

	/* wait till reset bit clears */
	while ((musb_readl(reg_base, USB_CTRL_REG) & 0x1))
		cpu_relax();

	/* Start the on-chip PHY and its PLL. */
	if (data->set_phy_power)
		data->set_phy_power(1);

	musb->a_wait_bcon = A_WAIT_BCON_TIMEOUT;
	musb->isr = ti81xx_interrupt;

	if (cpu_is_ti816x())
		usbid_sw_ctrl = 1;

	/* if usb-id contolled through software for ti816x then
	 * configure the usb0 in peripheral mode and usb1 in
	 * host mode
	*/
	if (usbid_sw_ctrl && cpu_is_ti816x())
		mode = musb->id ? MUSB_HOST : MUSB_PERIPHERAL;
	else
		mode = MUSB_OTG;

	/* set musb controller to host mode */
	musb_platform_set_mode(musb, mode);

	/* enable babble workaround */
	INIT_WORK(&musb->work, evm_deferred_musb_restart);
	musb->enable_babble_work = 0;

	musb_writel(reg_base, USB_IRQ_EOI, 0);

	return 0;
}

/* TI81xx supports only 32bit read operation */
void ti81xx_musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
	void __iomem *fifo = hw_ep->fifo;
	u32		val;
	int		i;

	/* Read for 32bit-aligned destination address */
	if (likely((0x03 & (unsigned long) dst) == 0) && len >= 4) {
		readsl(fifo, dst, len >> 2);
		dst += len & ~0x03;
		len &= 0x03;
	}
	/*
	 * Now read the remaining 1 to 3 byte or complete length if
	 * unaligned address.
	 */
	if (len > 4) {
		for (i = 0; i < (len >> 2); i++) {
			*(u32 *) dst = musb_readl(fifo, 0);
			dst += 4;
		}
		len &= 0x03;
	}
	if (len > 0) {
		val = musb_readl(fifo, 0);
		memcpy(dst, &val, len);
	}
}

int ti81xx_musb_exit(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;

	del_timer_sync(&musb->otg_workaround);

	/* Shutdown the on-chip PHY and its PLL. */
	if (data->set_phy_power)
		data->set_phy_power(0);

	usb_put_phy(musb->xceiv);

	return 0;
}

static struct musb_platform_ops ti81xx_ops = {
	.fifo_mode	= 4,
	.init		= ti81xx_musb_init,
	.exit		= ti81xx_musb_exit,

	.enable		= ti81xx_musb_enable,
	.disable	= ti81xx_musb_disable,

	.try_idle	= ti81xx_musb_try_idle,
	.set_mode	= ti81xx_musb_set_mode,

	.read_fifo      = ti81xx_musb_read_fifo,
	.write_fifo     = musb_write_fifo,
};

static void ti81xx_delete_musb_pdev(struct ti81xx_glue *glue, u8 id)
{
	platform_device_del(glue->musb[id]);
	platform_device_put(glue->musb[id]);
	usb_phy_generic_unregister(glue->phy[id]);
}

static int ti81xx_create_musb_pdev(struct ti81xx_glue *glue, u8 id)
{
	struct device *dev = glue->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct musb_hdrc_platform_data  *pdata = dev->platform_data;
	struct omap_musb_board_data *bdata = pdata->board_data;
	struct platform_device	*musb;
	struct resource *res;
	struct resource	resources[2];
	char res_name[10];
	int ret = 0;

	/* get memory resource */
	sprintf(res_name, "musb%d", id);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, res_name);
	if (!res) {
		dev_err(dev, "%s get mem resource failed\n", res_name);
		ret = -ENODEV;
		goto err0;
	}
	res->parent = NULL;
	resources[0] = *res;

	/* get irq resource */
	sprintf(res_name, "musb%d-irq", id);
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, res_name);
	if (!res) {
		dev_err(dev, "%s get irq resource failed\n", res_name);
		ret = -ENODEV;
		goto err0;
	}
	res->parent = NULL;
	resources[1] = *res;

	/* allocate the child platform device */
	musb = platform_device_alloc("musb-hdrc", id);
	if (!musb) {
		dev_err(dev, "failed to allocate musb device\n");
		goto err0;
	}

	musb->id			= id;
	musb->dev.parent		= dev;
	musb->dev.dma_mask		= &musb_dmamask;
	musb->dev.coherent_dma_mask	= musb_dmamask;

	glue->musb[id]			= musb;
	glue->phy[id]			= usb_phy_generic_register();
	if (IS_ERR(glue->phy))
		goto err2;

	pdata->platform_ops		= &ti81xx_ops;

	ret = platform_device_add_resources(musb, resources, 2);
	if (ret) {
		dev_err(dev, "failed to add resources\n");
		goto err1;
	}

	if (id == 0)
		pdata->mode = bdata->mode & USB0PORT_MODEMASK;
	else
		pdata->mode = (bdata->mode & USB1PORT_MODEMASK)
					>> USB1PORT_MODESHIFT;

	dev_info(dev, "musb%d, board_mode=0x%x, plat_mode=0x%x\n",
					id, bdata->mode, pdata->mode);

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(dev, "failed to add platform_data\n");
		goto err1;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(dev, "failed to register musb device\n");
		goto err1;
	}

	return 0;

err2:
	usb_phy_generic_unregister(glue->phy[id]);
err1:
	platform_device_put(musb);
err0:
	return ret;
}

static int __init ti81xx_probe(struct platform_device *pdev)
{
	struct ti81xx_glue *glue;
	struct device *dev = &pdev->dev;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	int ret = 0, i;
	struct resource *res;

	/* allocate glue */
	glue = kzalloc(sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&pdev->dev, "unable to allocate glue memory\n");
		ret = -ENOMEM;
		goto err0;
	}

	/* get memory resource */
	glue->mem_pa = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!glue->mem_pa) {
		dev_err(&pdev->dev, "failed to get usbss mem resourse\n");
		ret = -ENODEV;
		goto err1;
	}

	/* get memory resource */
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "usbss-irq");
	if (!res) {
		dev_err(&pdev->dev, "failed to get usbss irq resourse\n");
		ret = -ENODEV;
		goto err1;
	}
	glue->irq = res->start;

	/* iomap for usbss mem space */
	glue->mem_va =
		ioremap(glue->mem_pa->start, resource_size(glue->mem_pa));
	if (!glue->mem_va) {
		dev_err(&pdev->dev, "usbss ioremap failed\n");
		ret = -ENOMEM;
		goto err1;
	}
	usbss_virt_base = glue->mem_va;

	glue->first = 1;
	glue->dev = &pdev->dev;
	platform_set_drvdata(pdev, glue);

	/* enable clocks */
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync FAILED");
		goto err2;
	}

	/* usb subsystem init */
	usbotg_ss_init();

	/* clear any USBSS interrupts */
	writel(0, glue->mem_va + USBSS_IRQ_EOI);
	writel(readl(glue->mem_va + USBSS_IRQ_STATUS),
					glue->mem_va + USBSS_IRQ_STATUS);

	/* create the child platform device for mulitple instances of musb */
	for (i = 0; i <= data->instances; ++i) {
		ret = ti81xx_create_musb_pdev(glue, i);
		if (ret != 0)
			goto err3;
	}

	return 0;

err3:
	pm_runtime_put_sync(&pdev->dev);
err2:
	pm_runtime_disable(&pdev->dev);
	iounmap(glue->mem_va);
err1:
	kfree(glue);
err0:
	return ret;
}

static int __exit ti81xx_remove(struct platform_device *pdev)
{
	struct ti81xx_glue *glue = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	int i;

	/* delete the child platform device for mulitple instances of musb */
	for (i = 0; i <= data->instances; ++i)
		ti81xx_delete_musb_pdev(glue, i);

	/* iounmap */
	iounmap(glue->mem_va);
	usbotg_ss_uninit();

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	kfree(glue);

	return 0;
}

#ifdef CONFIG_PM
static void ti81xx_save_context(struct ti81xx_glue *glue)
{
	struct ti81xx_usbss_regs *usbss = &glue->usbss_regs;
	u8 i, j;

	/* save USBSS register */
	usbss->irq_en_set = usbss_read(USBSS_IRQ_ENABLE_SET);

	/* save usbX register */
	for (i = 0 ; i < 2 ; i++) {
		struct ti81xx_usb_regs *usb = &glue->usb_regs[i];
		struct musb *musb = platform_get_drvdata(glue->musb[i]);
		void __iomem *cbase = musb->ctrl_base;

		/* disable the timers */
		if (timer_pending(&musb->otg_workaround)) {
			del_timer_sync(&musb->otg_workaround);
			musb->en_otgw_timer = 1;
		}

		if (timer_pending(&musb->otg_workaround)) {
			del_timer_sync(&musb->otg_timer);
			musb->en_otg_timer = 1;
		}

		musb_save_context(musb);
		usb->control = musb_readl(cbase, USB_CTRL_REG);

		for (j = 0 ; j < 2 ; j++)
			usb->irq_en_set[j] = musb_readl(cbase,
					USB_IRQ_ENABLE_SET_0 + (0x4 * j));
		usb->srp_fix = musb_readl(cbase, USB_SRP_FIX_TIME_REG);
		usb->phy_utmi = musb_readl(cbase, USB_PHY_UTMI_REG);
		usb->mgc_utmi_loopback = musb_readl(cbase, USB_PHY_UTMI_LB_REG);
		usb->mode = musb_readl(cbase, USB_MODE_REG);
	}
}

static void ti81xx_restore_context(struct ti81xx_glue *glue)
{
	struct ti81xx_usbss_regs *usbss = &glue->usbss_regs;
	u8 i, j;

	/* restore USBSS register */
	usbss_write(USBSS_IRQ_ENABLE_SET, usbss->irq_en_set);

	/* restore usbX register */
	for (i = 0 ; i < 2 ; i++) {
		struct ti81xx_usb_regs *usb = &glue->usb_regs[i];
		struct musb *musb = platform_get_drvdata(glue->musb[i]);
		void __iomem *cbase = musb->ctrl_base;

		musb_restore_context(musb);
		musb_writel(cbase, USB_CTRL_REG, usb->control);

		for (j = 0 ; j < 2 ; j++)
			musb_writel(cbase, USB_IRQ_ENABLE_SET_0 + (0x4 * j),
					usb->irq_en_set[j]);

		musb_writel(cbase, USB_SRP_FIX_TIME_REG, usb->srp_fix);
		musb_writel(cbase, USB_PHY_UTMI_REG, usb->phy_utmi);
		musb_writel(cbase, USB_PHY_UTMI_LB_REG, usb->mgc_utmi_loopback);
		musb_writel(cbase, USB_MODE_REG, usb->mode);

		/* reenable the timers */
		if (musb->en_otgw_timer) {
			mod_timer(&musb->otg_workaround,
					jiffies + POLL_SECONDS * HZ);
			musb->en_otgw_timer = 0;
		}
		if (musb->en_otg_timer) {
			mod_timer(&musb->otg_timer,
					jiffies + POLL_SECONDS * HZ);
			musb->en_otg_timer = 0;
		}
	}
	/* controller needs delay for successful resume */
	msleep(200);
}

static int ti81xx_runtime_suspend(struct device *dev)
{
	struct ti81xx_glue *glue = dev_get_drvdata(dev);
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	int i;

	/* save wrappers and cppi4.1 dma register */
	ti81xx_save_context(glue);

	/* Shutdown the on-chip PHY and its PLL. */
	for (i = 0; i <= data->instances; ++i) {
		if (data->set_phy_power)
			data->set_phy_power(0);
	}

	return 0;
}

static int ti81xx_runtime_resume(struct device *dev)
{
	struct ti81xx_glue *glue = dev_get_drvdata(dev);
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct omap_musb_board_data *data = plat->board_data;
	int i;

	/*
	 * ignore first call of resume as all registers are not yet
	 * initialized
	 */
	if (glue->first) {
		glue->first = 0;
		return 0;
	}

	/* Start the on-chip PHY and its PLL. */
	for (i = 0; i <= data->instances; ++i) {
		if (data->set_phy_power)
			data->set_phy_power(1);
	}

	/* restore wrappers and cppi4.1 dma register */
	ti81xx_restore_context(glue);

	return 0;
}

static const struct dev_pm_ops ti81xx_pm_ops = {
	.runtime_suspend = ti81xx_runtime_suspend,
	.runtime_resume	= ti81xx_runtime_resume,
};

#define DEV_PM_OPS	(&ti81xx_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static struct platform_driver ti81xx_musb_driver = {
	.remove         = __exit_p(ti81xx_remove),
	.driver         = {
		.name   = "musb-ti81xx",
		.pm	= DEV_PM_OPS,
	},
};

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_DESCRIPTION("AM35x MUSB Glue Layer");
MODULE_LICENSE("GPL v2");

static int __init ti81xx_glue_init(void)
{
	return platform_driver_probe(&ti81xx_musb_driver, ti81xx_probe);
}
subsys_initcall(ti81xx_glue_init);

static void __exit ti81xx_glue_exit(void)
{
	/* disable the interrupts */
	usbss_write(USBSS_IRQ_EOI, 0);
	usbss_write(USBSS_IRQ_ENABLE_SET, 0);
	usbss_write(USBSS_IRQ_DMA_ENABLE_0, 0);

	/* unregister platform driver */
	platform_driver_unregister(&ti81xx_musb_driver);
}
module_exit(ti81xx_glue_exit);
