#!/bin/sh
# Read-only inspection of what actually came up. Run after scripts/load.sh.

printf '=== loaded modules ===\n'
lsmod | grep -E '^(imx415|iv009_isp|ao_mclk|dtbo_loader|isp_clkc) ' || printf '  none\n'

printf '\n=== isp_clkc clocks ===\n'
if [ -r /sys/kernel/debug/clk/clk_summary ]; then
	head -1 /sys/kernel/debug/clk/clk_summary
	grep -E 'mipi_isp|mipi_csi|csi_dig|csi2_phy|gen_clk' \
		/sys/kernel/debug/clk/clk_summary || printf '  none registered\n'
else
	printf '  (need root to read clk_summary)\n'
fi

printf '\n=== overlay nodes in live devicetree ===\n'
for n in /proc/device-tree/soc/bus@ff600000/bus@3c000/system-controller@0/isp-clkc \
         /proc/device-tree/isp@ff140000 \
         /proc/device-tree/soc/bus@ff800000/i2c@5000/camera-sensor@1a; do
	[ -d "$n" ] && printf '  present: %s\n' "$n"
done

printf '\n=== sensor on VIM3 Camera0 I2C_AO / i2c-0 (candidate 0x1a) ===\n'
if command -v i2cdetect >/dev/null; then
	i2cdetect -y 0 2>&1 | sed 's/^/  /'
else
	printf '  i2cdetect not installed (sudo apt install i2c-tools)\n'
fi

printf '\n=== v4l2 devices ===\n'
# Note: /dev/video0 is meson-vdec (the hardware video decoder) on a stock
# board -- it is NOT the camera. The ISP node is the one named isp_v4l2.
found=0
for d in /dev/video* /dev/media*; do
	[ -e "$d" ] || continue
	found=1
	drv=$(v4l2-ctl -d "$d" --info 2>/dev/null | awk -F': ' '/Driver name/{print $2}')
	printf '  %-14s %s\n' "$d" "${drv:-?}"
done
[ "$found" = 1 ] || printf '  none\n'

printf '\n=== driver bind status ===\n'
for d in /sys/bus/platform/drivers/*isp*/ /sys/bus/i2c/drivers/imx415/; do
	[ -d "$d" ] || continue
	printf '  %s\n' "$d"
	ls -l "$d" 2>/dev/null | awk '/->/ {print "    bound: " $9}'
done

printf '\n=== recent dmesg ===\n'
dmesg 2>/dev/null | grep -iE 'isp|imx415|csi|clkc|dtbo' | tail -30 | sed 's/^/  /'
