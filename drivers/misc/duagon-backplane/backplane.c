
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "ionia.h"
#include "ionia-pdata.h"
#include "ionia-slots.h"
#include "ionia-serial.h"
#include "ionia-rpc.h"

#define SLOT_DEVNAME	"ionia-card.%d"

int ionia_backplane_devname_for_slot(char *buf, size_t sz, int slot)
{
	return snprintf(buf, sz, SLOT_DEVNAME, slot);
}

int ionia_backplane_slot_for_devname(const char* name)
{
	int slot = -1;
	sscanf(name, SLOT_DEVNAME, &slot);
	return slot;
}

struct ionia_slot *ionia_backplane_slot_for_device(struct platform_device *pdev)
{
	struct ionia_backplane_platform_data *bp_pdata;
	int slot_id;

	if (pdev->dev.parent == NULL) {
		pdev_info(pdev, "device has no parent\n");
		return NULL;
	}

	bp_pdata = pdev->dev.parent->platform_data;

	slot_id = ionia_backplane_slot_for_devname(pdev->dev.kobj.name);

	if (bp_pdata->nr_slots <= slot_id) {
		pdev_err(pdev, "slot id %d out of range (%d)\n", slot_id, bp_pdata->nr_slots);
		return NULL;
	}

	return &(bp_pdata->slots[slot_id]);
}

ionia_rpc_t *ionia_backplane_rpc_for_device(struct platform_device *pdev)
{
	struct ionia_slot *slot = ionia_backplane_slot_for_device(pdev);
	if (slot == NULL)
		return NULL;

	return ionia_rpc_get_fifo(&slot->fifo);
}

int ionia_backplane_probe_card(struct platform_device *pdev, struct ionia_slot *slot)
{
	char busid[64];
	ionia_backplane_devname_for_slot(busid, sizeof(busid), slot->id);

	if (slot->pdev != NULL) {
		pdev_info(pdev, "card for slot %d already probed\n", slot->id);
		return 0;
	}

	if (!(slot->pdev = of_platform_device_create(slot->devnode, busid, &pdev->dev))) {
		pdev_info(pdev, "failed to create child for slot %d\n", slot->id);
		return -EINVAL;
	}

	pdev_info(pdev, "created card for slot %d\n", slot->id);
	return 0;
}

int ionia_backplane_probe_cards(struct platform_device *pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	int x;
	pdev_info(pdev, "probing cards: %d slots\n", pdata->nr_slots);

	for (x=0; x<pdata->nr_slots; x++)
		ionia_backplane_probe_card(pdev, &(pdata->slots[x]));
	return 0;
}

int ionia_backplane_probe_slots(struct platform_device *pdev)
{
	struct ionia_backplane_platform_data *pdata = pdev->dev.platform_data;
	struct device_node *backplane_node = pdev->dev.of_node;
	struct device_node *slots_node;
	struct device_node *child;
	int nr_slots = 0;
	int cnt_slots = 0;

	if (pdata->nr_slots) {
		pdev_info(pdev, "slots already probed\n");
		return 0;
	}

	if (!(backplane_node = pdev->dev.of_node)) {
		pdev_err(pdev, "failed to get of node\n");
		return -ENOENT;
	}

	if (!(slots_node = of_get_child_by_name(backplane_node, "slots"))) {
		pdev_err(pdev, "no slots node\n");
		return -ENOENT;
	}

	for_each_child_of_node(slots_node, child) {
		nr_slots++;
	}

	if (!nr_slots) {
		pdev_err(pdev, "no slots");
		return -ENOENT;
	}

	if (!(pdata->slots = devm_kzalloc(&pdev->dev, sizeof(struct ionia_slot)*nr_slots, GFP_KERNEL))) {
		pdev_err(pdev, "kzalloc() failed\n");
		return -ENOMEM;
	}

	cnt_slots = 0;
	//for_each_available_child_of_node() ?
	//kick out the slots subnode ?
	for_each_child_of_node(slots_node, child) {
		int a_cells = of_n_addr_cells(child);
		int s_cells = of_n_addr_cells(child);
		int len;
		const __be32 *reg = of_get_property(child, "reg", &len);
		struct ionia_slot *slot = &(pdata->slots[cnt_slots]);

		slot->name = of_get_property(child, "label", NULL);
		slot->devnode = child;
		slot->id = cnt_slots;

		if (!reg) {
			pdev_warn(pdev, "   missing reg property in slot %d\n", cnt_slots);
			nr_slots--;
			continue;
		}

		if (len / 4 != a_cells + s_cells) {
			pdev_warn(pdev, "   reg property size mismatch in slot %d\n", cnt_slots);
			nr_slots--;
			continue;
		}

		slot->base = of_read_number(reg, a_cells);
		slot->sz = of_read_number(reg + a_cells, s_cells);

		ionia_fifo_init(&slot->fifo,
				slot->base,
				cnt_slots,
				pdata->registers + slot->base,
				slot->name,
				pdev);

		pdev_info(pdev, "slot %2d at 0x%04X:%02X: %s\n",
			 cnt_slots,
			 slot->base,
			 slot->sz,
			 slot->name);

		cnt_slots++;
	}

	pdata->nr_slots = nr_slots;

	return 0;
}
