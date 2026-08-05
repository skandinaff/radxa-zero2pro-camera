// SPDX-License-Identifier: GPL-2.0
/*
 * dtbo_loader: applies a compiled devicetree overlay blob at insmod time via
 * of_overlay_fdt_apply(), and removes it via of_overlay_remove() at rmmod
 * time. Exists because this kernel build has CONFIG_OF_OVERLAY=y but not
 * CONFIG_OF_CONFIGFS, so /sys/kernel/config/device-tree/overlays doesn't
 * exist. Reads the .dtbo from an arbitrary path (default /lib/firmware) via
 * kernel_read_file_from_path() rather than request_firmware(), since the
 * latter wants a struct device this standalone module doesn't have.
 *
 * Deliberately fails insmod (returns an error, nothing stays loaded) if the
 * overlay can't be read or doesn't apply — no partial/dangling state to
 * remember to clean up by hand. rmmod always removes exactly the overlay
 * this instance applied.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/kernel_read_file.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

static char *dtbo_path = "/lib/firmware/camera-overlay.dtbo";
module_param(dtbo_path, charp, 0444);
MODULE_PARM_DESC(dtbo_path, "Path to the compiled .dtbo to apply on load");

static int ovcs_id = -1;

static int __init dtbo_loader_init(void)
{
	void *buf = NULL;
	size_t size = 0;
	int ret;

	ret = kernel_read_file_from_path(dtbo_path, 0, &buf, INT_MAX, NULL,
					  READING_FIRMWARE);
	if (ret < 0) {
		pr_err("dtbo_loader: failed to read %s: %d\n", dtbo_path, ret);
		return ret;
	}
	size = ret;

	ret = of_overlay_fdt_apply(buf, size, &ovcs_id);
	vfree(buf);

	if (ret) {
		pr_err("dtbo_loader: of_overlay_fdt_apply(%s) failed: %d\n",
		       dtbo_path, ret);
		return ret;
	}

	pr_info("dtbo_loader: applied %s as overlay changeset id %d\n",
		dtbo_path, ovcs_id);
	return 0;
}

static void __exit dtbo_loader_exit(void)
{
	int ret;

	if (ovcs_id < 0)
		return;

	ret = of_overlay_remove(&ovcs_id);
	if (ret)
		pr_err("dtbo_loader: of_overlay_remove failed: %d (overlay may still be active)\n",
		       ret);
	else
		pr_info("dtbo_loader: overlay removed\n");
}

module_init(dtbo_loader_init);
module_exit(dtbo_loader_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Applies a devicetree overlay blob at runtime via of_overlay_fdt_apply");
