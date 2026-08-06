#!/bin/sh
# Full rollback: unload every module and remove the devicetree overlay, in
# reverse dependency order. Safe to run at any point, including after a partial
# or failed load -- each step is skipped if that module isn't present.
#
# After this the kernel is back to exactly its pre-load state; nothing on disk
# outside this repo was ever modified.
set -e

[ "$(id -u)" -eq 0 ] || { echo "error: must run as root" >&2; exit 1; }

# iv009_isp before imx415, not after. iv009_isp's v4l2_async notifier holds a
# reference on the bound sensor subdev, so while it is loaded "rmmod imx415"
# fails with EBUSY -- which left imx415 behind on every unload and made the
# next load.sh a partial reload rather than a clean one. Everything else is in
# reverse dependency order as before.
for m in iv009_isp imx415 ao_mclk dtbo_loader isp_clkc; do
	if lsmod | grep -q "^$m "; then
		printf 'rmmod %s\n' "$m"
		rmmod "$m" || printf '  WARNING: rmmod %s failed\n' "$m"
	fi
done

printf '\nRemaining camera modules (should be empty):\n'
lsmod | grep -E "^(imx415|iv009_isp|ao_mclk|dtbo_loader|isp_clkc) " || printf '  none\n'
