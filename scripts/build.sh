#!/bin/sh
# Build all four kernel modules and compile the devicetree overlays.
# Run this ON the Radxa Zero 2 Pro (needs linux-headers for the running kernel).
set -e

cd "$(dirname "$0")/.."
ROOT=$(pwd)

KDIR=${KDIR:-/usr/src/linux-headers-$(uname -r)}
if [ ! -d "$KDIR" ]; then
	echo "error: kernel headers not found at $KDIR" >&2
	echo "       install with: sudo apt install linux-headers-\$(uname -r)" >&2
	exit 1
fi

for d in dtbo-loader isp-clkc imx415 isp-module; do
	printf '=== %s ===\n' "$d"
	make -C "$ROOT/$d" KDIR="$KDIR"
done

printf '=== overlays ===\n'
for f in "$ROOT"/overlays/*.dts; do
	out="${f%.dts}.dtbo"
	# -@ is required: the overlays reference labels (&isp_clkc, &imx415_ep,
	# ...) and need __symbols__/__local_fixups__ to resolve at apply time.
	dtc -@ -I dts -O dtb -o "$out" "$f" 2>&1 | grep -v '^$' || true
	printf '  %s (%s bytes)\n' "$(basename "$out")" "$(stat -c %s "$out")"
done

printf '\nBuilt:\n'
find "$ROOT" -name '*.ko' | sed "s|$ROOT|.|"
