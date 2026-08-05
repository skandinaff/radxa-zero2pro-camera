#!/bin/sh
# Determine, empirically, which GPIOAO_10 mux value selects the SoC's CLK12_24
# output -- the camera's MCLK. Mainline's pinctrl-meson-g12a.c doesn't model
# that function, so the value isn't documented in-tree and has to be found.
#
# For each candidate value we program the pin, deassert the sensor's reset, and
# look for the IMX415 ACKing at 0x1a on i2c-3. A hit means that mux value is
# feeding the sensor a usable clock.
#
# Every step is reversible: ao_mclk restores the pin field on rmmod, and the
# reset GPIO is released when gpioset exits.
set -e

cd "$(dirname "$0")/.."
ROOT=$(pwd)

[ "$(id -u)" -eq 0 ] || { echo "error: must run as root" >&2; exit 1; }
[ -f "$ROOT/ao-mclk/ao_mclk.ko" ] || { echo "error: run scripts/build.sh first" >&2; exit 1; }

# The sensor must not be held in reset while we probe. CM_RST_L is GPIOA_11 =
# gpiochip0 line 60 (schematic page 6, ball G22); active-low, so drive it high.
RESET_LINE=60

rmmod imx415 2>/dev/null || true
rmmod ao_mclk 2>/dev/null || true

gpioset -m signal gpiochip0 "$RESET_LINE"=1 &
GPIO_PID=$!
trap 'kill $GPIO_PID 2>/dev/null; rmmod ao_mclk 2>/dev/null || true' EXIT
sleep 1

for m in 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
	insmod "$ROOT/ao-mclk/ao_mclk.ko" mux="$m" 2>/dev/null || {
		printf 'mux %-2s : insmod failed\n' "$m"; continue; }
	sleep 1
	# Column for address 0x1a on the "10:" row of i2cdetect output.
	hit=$(i2cdetect -y -r 3 2>/dev/null | awk '/^10:/{print $12}')
	rmmod ao_mclk 2>/dev/null || true
	if [ "$hit" = "1a" ] || [ "$hit" = "UU" ]; then
		printf 'mux %-2s : *** SENSOR FOUND at 0x1a ***\n' "$m"
	else
		printf 'mux %-2s : 0x1a = %s\n' "$m" "${hit:-??}"
	fi
done
