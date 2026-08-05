#!/bin/sh
# Full rollback: unload every module and remove the devicetree overlay, in
# reverse dependency order. Safe to run at any point, including after a partial
# or failed load -- each step is skipped if that module isn't present.
#
# After this the kernel is back to exactly its pre-load state; nothing on disk
# outside this repo was ever modified.
set -e

[ "$(id -u)" -eq 0 ] || { echo "error: must run as root" >&2; exit 1; }

for m in imx415 iv009_isp dtbo_loader isp_clkc; do
	if lsmod | grep -q "^$m "; then
		printf 'rmmod %s\n' "$m"
		rmmod "$m" || printf '  WARNING: rmmod %s failed\n' "$m"
	fi
done

printf '\nRemaining camera modules (should be empty):\n'
lsmod | grep -E '^(imx415|iv009_isp|dtbo_loader|isp_clkc) ' || printf '  none\n'
