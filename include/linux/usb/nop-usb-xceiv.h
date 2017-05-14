#ifndef __LINUX_USB_NOP_XCEIV_H
#define __LINUX_USB_NOP_XCEIV_H

#include <linux/usb/otg.h>

struct nop_usb_xceiv_platform_data {
	enum usb_phy_type type;
};

#if defined(CONFIG_NOP_USB_XCEIV) || (defined(CONFIG_NOP_USB_XCEIV_MODULE) && defined(MODULE))
/* sometimes transceivers are accessed only through e.g. ULPI */
extern void usb_nop_xceiv_register(int id);
extern void usb_nop_xceiv_unregister(int id);
#else
static inline void usb_nop_xceiv_register(int id)
{
}

static inline void usb_nop_xceiv_unregister(int id)
{
}
#endif

#endif /* __LINUX_USB_NOP_XCEIV_H */
