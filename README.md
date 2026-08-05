# Radxa Zero 2 Pro — MIPI CSI camera support for Linux

An out-of-tree attempt at getting the **Radxa 4K camera (Sony IMX415)** working
under mainline-ish Linux on the **Radxa Zero 2 Pro** (Amlogic A311D, G12B).

Everything here loads and unloads at runtime. **Nothing touches `/boot`, the
bootloader, or the shipped devicetree.** See [Rollback](#rollback).

> **Status: builds and loads cleanly; image capture not yet confirmed.**
> See [Current status](#current-status) for exactly what is and isn't proven.
> If you are looking for a working camera today, this is not that yet.

## Why this is needed

The Zero 2 Pro has a CSI connector and the camera works under Radxa's Android
build, but Linux has no path to it:

| | Radxa Zero 3W | Radxa Zero 2 Pro |
|---|---|---|
| SoC | Rockchip RK3566 | **Amlogic A311D (G12B)** |
| Camera under Linux | works (mainline `rkisp1`) | **no driver** |

These are entirely different camera subsystems — nothing from the Zero 3W
carries over. Under mainline Linux on A311D specifically:

1. **No ISP driver.** Amlogic's ISP ("ACamera", ARM lineage) has never been
   mainlined. The only source is a vendor tree written for **Linux 4.9**.
2. **No MIPI-CSI clocks.** `drivers/clk/meson/g12a.c` mainlined the MIPI-**DSI**
   (display output) clocks but not MIPI-**CSI** (camera input). The clocks the
   ISP needs do not exist anywhere in the kernel.
3. **No devicetree nodes** for the ISP, CSI adapter, or CSI PHY.
4. **No sensor node**, though the IMX415 driver itself has been mainline since
   kernel 6.3.

So this repo supplies all four pieces as loadable modules plus a devicetree
overlay applied at runtime.

## What's in here

| Path | What it is |
|---|---|
| `isp-clkc/` | **New.** Auxiliary clock provider registering the 7 missing ISP/CSI/`gen_clk` clocks. Written from scratch against vendor register definitions. |
| `isp-module/` | Amlogic's vendor ACamera ISP driver (192 files), **forward-ported 4.9 → 6.1**. Builds `iv009_isp.ko`. |
| `imx415/` | Mainline `drivers/media/i2c/imx415.c` from kernel v6.3, **unmodified**, with an out-of-tree Makefile. |
| `dtbo-loader/` | **New.** Applies a `.dtbo` at `insmod` time, removes it at `rmmod` time. Needed because this kernel lacks `CONFIG_OF_CONFIGFS`. |
| `overlays/` | The devicetree overlays (`camera-overlay.dts` is the real one). |
| `scripts/` | `build.sh`, `load.sh`, `unload.sh`, `status.sh`, `capture.sh`. |
| `docs/` | Full file-by-file porting changelogs. |

## Target

- **Board:** Radxa Zero 2 Pro (v1.2), Amlogic A311D
- **Camera:** Radxa Camera 4K (Sony IMX415), 4-lane MIPI CSI-2
- **Kernel:** `6.1.68-3-stable` (Radxa Debian). Needs matching
  `linux-headers-$(uname -r)`.

Other 6.1.x Amlogic G12B kernels will probably work; the devicetree
`target-path`s in the overlays are the thing most likely to need adjusting.

## Build & run

All on the board itself:

```sh
sudo apt install linux-headers-$(uname -r) device-tree-compiler v4l-utils i2c-tools

git clone <this repo> && cd radxa-zero2pro-camera
./scripts/build.sh

sudo ./scripts/load.sh clk     # stage 1: clock provider only
sudo ./scripts/status.sh       # expect 7 new clocks in clk_summary
sudo ./scripts/unload.sh

sudo ./scripts/load.sh         # full stack
sudo ./scripts/status.sh
./scripts/capture.sh
```

Load in stages the first time. If stage 1's clocks don't appear, nothing
downstream will work and you've isolated the problem cheaply.

## Rollback

This was built under a hard constraint: **the board boots from eMMC and may
only be reachable over SSH**, so a bad devicetree written to `/boot` could brick
it with no recovery path. Consequently:

- The overlay is applied via `of_overlay_fdt_apply()` **into the running
  kernel** and removed via `of_overlay_remove()` on `rmmod`. Nothing is
  persisted.
- No file outside this repo is written. `/boot` is never opened for writing.
- `scripts/unload.sh` returns the kernel to its exact pre-load state.
- Worst case — a probe hangs or panics the kernel — a **power cycle** boots the
  board back into its untouched stock configuration, because nothing on disk
  ever changed.

Nothing here is installed to `/lib/modules` or added to `/etc/modules`. It will
not survive a reboot, by design.

## Current status

**Proven:**
- All four modules compile clean against the board's real 6.1.68 headers.
- All overlays compile clean with `dtc -@`, with phandles/fixups resolving.
- `dtbo_loader` works: a test overlay was applied to the live kernel, verified
  in `/proc/device-tree`, then cleanly removed. This is the rollback mechanism,
  and it is the one runtime thing that has been end-to-end validated.
- `/boot` verified byte-identical (SHA-256, 70 files) after all work.

**Not yet proven:**
- The ISP/clock/sensor modules have **not been loaded onto a live kernel yet**.
- No image has been captured.

### Known open issues

These are real and unresolved — listed so nobody wastes time rediscovering them.

1. **Sensor reset/power-down GPIOs are unknown.** The schematic (pages 9, 13)
   documents the CSI connector's `CM_PWRDN_1` / `CM_RST_L` / `CM_FS` pins but
   not which SoC ball they land on. `imx415.c` uses
   `devm_gpiod_get_optional()` for `reset-gpios`, so omitting it is safe for
   probe — but if the sensor never responds at `0x1a` on `i2c-3`, a missing
   reset toggle is suspect #1.
2. **DMA coherency is unverified.** The ported ISP code assumes
   `dma_alloc_coherent()` returns a vaddr in the kernel's linear map. That held
   on 4.9 on this SoC; it has not been re-verified for 6.1. Affects capture
   correctness, not probe.
3. **No IQ/calibration data for the IMX415.** The vendor ISP expects per-sensor
   tuning tables. Even a successful capture would likely look bad (wrong colour,
   exposure, black level) until calibration data exists.
4. **`gen_clk` (sensor MCLK) routing to `GPIOAO_10` is inferred**, not measured.
   The schematic shows `CM_MCLK` on ball BF16 (`GPIOAO_10`), whose alt-function
   is the SoC `gen_clk` output. If the sensor gets no clock, it will not ACK on
   I²C.

## Hardware facts (from the official schematic)

From `radxa_zero_2_pro_v1.2_schematic.pdf`, pages 9 ("CPU I/O") and 13
("Panel/CAM/40PIN") — these are verified, not guessed:

- CSI connector **J5002**, 31-pin 0.3 mm FPC. `MIPI_CSI_D0..D3` + `CLKA` wire
  directly to the A311D's CSI PHY pins — silicon-fixed, nothing to configure.
- `CM_MCLK` → ball **BF16 = GPIOAO_10**, whose `CLK12_24` alt-function is the
  SoC's `gen_clk` output.
- `CM_I2C_SCL/SDA` (connector pins 24/25) are the **same net** as the 40-pin
  header's I²C pins 3/5 → `i2c3` (`/dev/i2c-3`), which **already ships
  enabled**. No I²C overlay needed.
- The connector gets fixed VCC3.3V/VCC5V with no software power-gating GPIO, so
  the overlay's three regulators are always-on stand-ins purely to satisfy
  `devm_regulator_bulk_get()` (this kernel has no `CONFIG_REGULATOR_DUMMY`).

## Porting notes

- [`docs/port-notes-isp.md`](docs/port-notes-isp.md) — the 4.9 → 6.1 ISP port,
  file by file. Highlights: the `v4l2_async` notifier API rewrite
  (`V4L2_ASYNC_MATCH_CUSTOM` is gone, replaced by OF-graph fwnode matching),
  `vb2_mem_ops`/`dma_buf_ops` signature changes, `refcount_t`, `iosys_map`,
  y2038 `struct timeval` removal, `get_fs()`/`set_fs()` removal,
  `ioremap_nocache()`, `kzfree()` → `kfree_sensitive()`.
- [`docs/port-notes-clk.md`](docs/port-notes-clk.md) — the clock provider.
  Highlights: it shares the live HHI syscon's regmap via
  `device_node_to_regmap()` rather than a second private `ioremap()` of the same
  registers, and uses `.prepare`/`.unprepare` (mutex context) rather than
  `.enable`/`.disable` (spinlock/atomic context) for the gates, because regmap
  access here can sleep.

## Licence

GPL-2.0. `isp-module/` derives from Amlogic/Khadas vendor GPL sources
(`github.com/khadas/linux`, branch `khadas-vims-4.9.y`); `imx415/imx415.c` is
unmodified mainline. Original work in `isp-clkc/`, `dtbo-loader/`, and
`overlays/` is likewise GPL-2.0.

## Credits & references

- [`github.com/khadas/linux`](https://github.com/khadas/linux) `khadas-vims-4.9.y` — vendor ISP driver
- [`github.com/radxa/manifests`](https://github.com/radxa/manifests), [`radxa-pkg/radxa-overlays`](https://github.com/radxa-pkg/radxa-overlays)
- [Radxa Zero 2 Pro schematic](https://dl.radxa.com/zero2pro/docs/hw/v1.2/radxa_zero_2_pro_v1.2_schematic.pdf)
