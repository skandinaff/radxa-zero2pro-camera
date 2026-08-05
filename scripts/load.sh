#!/bin/sh
# Load the camera stack onto the running kernel. Nothing here touches /boot,
# the bootloader, or the shipped devicetree -- everything is undone by
# scripts/unload.sh, or by a power cycle if the board ever stops responding.
#
# Usage:
#   sudo ./scripts/load.sh          # full stack: clocks + ISP + sensor
#   sudo ./scripts/load.sh clk      # clock provider only (stage 1 bring-up)
set -e

cd "$(dirname "$0")/.."
ROOT=$(pwd)
STAGE=${1:-full}

[ "$(id -u)" -eq 0 ] || { echo "error: must run as root" >&2; exit 1; }

case "$STAGE" in
clk)   DTBO="$ROOT/overlays/aux-clk-overlay.dtbo" ;;
full)  DTBO="$ROOT/overlays/camera-overlay.dtbo" ;;
*)     echo "usage: $0 [clk|full]" >&2; exit 1 ;;
esac

[ -f "$DTBO" ] || { echo "error: $DTBO not built -- run scripts/build.sh" >&2; exit 1; }

# The clock provider must be loaded *before* the overlay is applied: applying
# the overlay creates the isp-clkc platform device, and the isp/imx415 nodes in
# the same overlay reference its clocks. If the driver isn't registered yet,
# those consumers get -EPROBE_DEFER and retry, which works, but loading in this
# order keeps dmesg readable.
printf '=== insmod isp_clkc ===\n'
insmod "$ROOT/isp-clkc/isp_clkc.ko"

printf '=== apply overlay (%s) ===\n' "$(basename "$DTBO")"
insmod "$ROOT/dtbo-loader/dtbo_loader.ko" dtbo_path="$DTBO"

if [ "$STAGE" = clk ]; then
	printf '\nStage 1 loaded. Check clocks with:\n'
	printf '  sudo grep -E "mipi_isp|csi|gen_clk" /sys/kernel/debug/clk/clk_summary\n'
	exit 0
fi

# iv009_isp and imx415 link against V4L2 helpers that nothing on a stock
# board has pulled in yet: the only in-tree V4L2 user here is meson-vdec (the
# hardware video decoder), which is m2m and needs neither the async-subdev
# notifier nor vmalloc vb2 buffers. Without these, insmod fails with
# "Unknown symbol in module" (v4l2_async_nf_*, vb2_vmalloc_memops).
printf '=== modprobe v4l2 dependencies ===\n'
for m in v4l2-async v4l2-fwnode videobuf2-vmalloc; do
	modprobe "$m" && printf '  %s\n' "$m"
done

# Route CLK12_24 to GPIOAO_10 so the sensor actually gets its MCLK. Must come
# before imx415: with no clock the IMX415 does not respond on i2c at all.
printf '=== insmod ao_mclk (camera MCLK pinmux) ===\n'
insmod "$ROOT/ao-mclk/ao_mclk.ko"

printf '=== insmod iv009_isp ===\n'
insmod "$ROOT/isp-module/iv009_isp.ko"

printf '=== insmod imx415 ===\n'
insmod "$ROOT/imx415/imx415.ko"

printf '\nLoaded. Check:\n'
printf '  dmesg | tail -40\n'
printf '  ls /dev/video*\n'
printf '  media-ctl -p\n'
