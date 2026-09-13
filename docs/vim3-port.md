# Khadas VIM3 Linux 6.18 port

This branch adapts the G12B runtime camera stack for the VIM3. It does not
modify boot files or the read-only Khadas 5.15 reference checkout.

## Authority and scope

- VIM3-specific CSI, ISP, clock, GPIO and Device Tree values come from
  Khadas common_drivers branch khadas-vims-5.15.y at
  3a11a86a02e759fc57fc79410215f7c0c3a0d8e0.
- This fork supplies the 6.1-to-6.18 out-of-tree API port and runtime-load
  pattern.
- imx415/imx415.c is exactly Linux v6.18.44 upstream, not the older Radxa
  sensor copy.

## IMX415 comparison

Linux 6.18 already accepts a 24 MHz input and 1440 Mbps lane rate, derives
four-lane timing from the endpoint, writes LANEMODE=4, and uses the 533 HMAX
minimum for this configuration. Therefore the Radxa 4-lane sensor patch is
not carried forward.

The Radxa 50 ms post-reset delay and crop/ROI changes are also intentionally
not copied. They were board-specific or unverified; a VIM3 measurement is
needed before introducing either workaround.

## VIM3 board layer

The VIM3 overlays are vim3-aux-clk-overlay.dts and vim3-camera-overlay.dts.
They use the VIM3 Camera0 AO I2C controller (i2c-0), vendor G12B CSI/ISP
resources, GPIOAO_10 CLK12_24 for the 24 MHz clock, and the vendor reset
line gpio_expander 3.

The vendor 5.15 IMX415 path also sequences gpio_expander 2 as PWDN. Mainline
IMX415 has no PWDN binding, so the runtime load is intentionally held until
that VIM3 control sequence and the sensor I2C address are confirmed.
