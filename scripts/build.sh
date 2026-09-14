#!/bin/sh
# Build the VIM3 G12B camera runtime modules and VIM3-only overlays.
# KDIR may point at an out-of-tree prepared kernel build.
set -e

cd "$(dirname "$0")/.."
ROOT=$(pwd)

KDIR=${KDIR:-/usr/src/linux-headers-$(uname -r)}
if [ ! -d "$KDIR" ]; then
	echo "error: kernel headers not found at $KDIR" >&2
	echo "       install with: sudo apt install linux-headers-\$(uname -r)" >&2
	exit 1
fi

# Keep the default native build path for the VIM3, while allowing an ARM64
# out-of-tree kernel build to supply the target architecture and toolchain.
#
# These go into the positional parameters rather than a single string: CC is
# routinely a compiler *plus* arguments (e.g. "aarch64-linux-gnu-gcc-14
# -B/path/to/binutils", needed because an extracted cross-GCC ships no
# assembler of its own). Word-splitting a flat string would hand make the
# "-B" as one of its own options and it would print its usage and do nothing.
set -- "KDIR=$KDIR"
[ -n "${ARCH:-}" ] && set -- "$@" "ARCH=$ARCH"
[ -n "${CROSS_COMPILE:-}" ] && set -- "$@" "CROSS_COMPILE=$CROSS_COMPILE"
[ -n "${CC:-}" ] && set -- "$@" "CC=$CC"

# The camera stack is exactly three modules:
#   isp-clkc   ISP/CSI clocks + the sensor MCLK (gen_clk) and its output pad
#   imx415     upstream sensor driver, unmodified
#   isp-module Amlogic G12B ISP + CSI-2 receiver
# ao-mclk and dtbo-loader are gone: the first duplicated gen_clk, which
# isp-clkc already owns, and the second could never work because this
# kernel has CONFIG_OF_OVERLAY unset (of_overlay_fdt_apply() is a stub
# returning -ENOTSUPP). The overlay is applied by U-Boot at boot instead.
for d in isp-clkc imx415 isp-module; do
	printf '=== %s ===\n' "$d"
	make -C "$ROOT/$d" "$@"
done

printf '=== overlays ===\n'
for f in "$ROOT"/overlays/vim3-camera-overlay.dts; do
	out="${f%.dts}.dtbo"
	# -@ is required: the overlays reference labels (&isp_clkc, &imx415_ep,
	# ...) and need __symbols__/__local_fixups__ to resolve at apply time.
	dtc -@ -I dts -O dtb -o "$out" "$f" 2>&1 | grep -v '^$' || true
	printf '  %s (%s bytes)\n' "$(basename "$out")" "$(stat -c %s "$out")"
done

printf '\nBuilt:\n'
find "$ROOT" -name '*.ko' | sed "s|$ROOT|.|"
