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

make_args="KDIR=$KDIR"
# Keep the default native build path for the VIM3, while allowing an ARM64
# out-of-tree kernel build to supply the target architecture and toolchain.
[ -n "${ARCH:-}" ] && make_args="$make_args ARCH=$ARCH"
[ -n "${CROSS_COMPILE:-}" ] && make_args="$make_args CROSS_COMPILE=$CROSS_COMPILE"
[ -n "${CC:-}" ] && make_args="$make_args CC=$CC"

for d in dtbo-loader isp-clkc ao-mclk imx415 isp-module; do
	printf '=== %s ===\n' "$d"
	# Deliberate word splitting: make_args contains only assignment words
	# constructed above, never user-supplied shell syntax.
	# shellcheck disable=SC2086
	make -C "$ROOT/$d" $make_args
done

printf '=== overlays ===\n'
for f in "$ROOT"/overlays/vim3-aux-clk-overlay.dts \
         "$ROOT"/overlays/vim3-camera-overlay.dts; do
	out="${f%.dts}.dtbo"
	# -@ is required: the overlays reference labels (&isp_clkc, &imx415_ep,
	# ...) and need __symbols__/__local_fixups__ to resolve at apply time.
	dtc -@ -I dts -O dtb -o "$out" "$f" 2>&1 | grep -v '^$' || true
	printf '  %s (%s bytes)\n' "$(basename "$out")" "$(stat -c %s "$out")"
done

printf '\nBuilt:\n'
find "$ROOT" -name '*.ko' | sed "s|$ROOT|.|"
