# Radxa Zero 2 Pro — MIPI CSI camera support for Linux

An out-of-tree attempt at getting the **Radxa 4K camera (Sony IMX415)** working
under mainline-ish Linux on the **Radxa Zero 2 Pro** (Amlogic A311D, G12B).

Everything here loads and unloads at runtime. **Nothing touches `/boot`, the
bootloader, or the shipped devicetree.** See [Rollback](#rollback).

> **Status: it captures.** The full path — IMX415 → MIPI CSI-2 → PHY → Amlogic
> adapter → ISP → V4L2 — delivers 3864×2192 GREY frames at a sustained
> **60.00 fps on 4 lanes** through `/dev/video1`. See
> [captures/4lane-60fps-2026-08-06.png](captures/4lane-60fps-2026-08-06.png)
> and [Current status](#current-status) for what is and is not done.
>
> Image *tuning* is a separate matter: there is no real IQ calibration for this
> sensor yet, so frames are correctly formed but flat and untuned.

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

5. **No pinctrl for the camera's MCLK.** The clock pin's `CLK12_24` function
   isn't modelled by `pinctrl-meson-g12a.c` at all, so it can't be selected
   from devicetree.

So this repo supplies all of that as loadable modules plus a devicetree
overlay applied at runtime.

## What's in here

| Path | What it is |
|---|---|
| `isp-clkc/` | **New.** Auxiliary clock provider registering the 7 missing ISP/CSI/`gen_clk` clocks. Written from scratch against vendor register definitions. |
| `ao-mclk/` | **New.** Routes CLK12_24 to GPIOAO_10 (the camera's MCLK pin) by programming the AO pinmux field mainline has no group for. Restores it on unload. |
| `isp-module/` | Amlogic's vendor ACamera ISP driver (192 files), **forward-ported 4.9 → 6.1**. Builds `iv009_isp.ko`. |
| `imx415/` | Mainline `drivers/media/i2c/imx415.c` from kernel v6.3 plus an out-of-tree Makefile. One change only: a longer post-reset delay (10 lines, commented in place). |
| `dtbo-loader/` | **New.** Applies a `.dtbo` at `insmod` time, removes it at `rmmod` time. Needed because this kernel lacks `CONFIG_OF_CONFIGFS`. |
| `overlays/` | The devicetree overlays (`camera-overlay.dts` is the real one). |
| `scripts/` | `build.sh`, `load.sh`, `unload.sh`, `status.sh`, `capture.sh`, `find-mclk-mux.sh`. |
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

**Working on hardware:**
- All five modules build clean against the board's real 6.1.68 headers, and
  the whole stack loads and unloads without incident.
- `dtbo_loader` works: overlays apply to the live kernel and are cleanly
  removed on `rmmod`. This is the rollback mechanism, and it is validated
  end-to-end.
- **`isp_clkc` works.** All 7 clocks register and appear in `clk_summary`.
- **The ISP itself comes up.** `iv009_isp` probes: it maps its registers, takes
  its interrupt, parses the scaler via `link-device`, registers a v4l2 device
  and 3 subdevs, and reports
  `isp init clock is 666 MHZ` / `mipi init clock is 199 MHZ` — the rates it
  asks for, so the clock chain is genuinely driving the hardware.
- **The camera's MCLK route was found.** `ao_mclk` selects CLK12_24 on
  GPIOAO_10 and this demonstrably changes the bus: with it, a device ACKs at
  `0x1a` on `i2c-3`; without it, nothing does, for any GPIO combination.
- Camera control GPIOs identified from the schematic (see the overlay header).
- `/boot` verified byte-identical (SHA-256, 70 files) throughout.
- 896 MB CMA pool, ~880 MB free — no DMA-allocation concern.

- **`imx415` probes and streams.** The MCLK question below was resolved; the
  sensor does register-level I²C and enters its 4-lane 1440 Mbps mode.
- **Capture works end to end.** `v4l2-ctl --stream-mmap` on the ISP node
  delivers 120 frames at **60.00 fps**, 3864×2192 GREY, 3968-byte stride.
  Frames are real images — the stride autocorrelation peaks exactly at 3968
  (0.976), adjacent-row correlation is 0.977, and consecutive frames differ
  (mean abs difference ~9.3/255), so it is live video, not one buffer repeated.
  Note `v4l2-ctl` must be given `--set-fmt-video` in the *same* invocation:
  this driver rejects `STREAMON` without an `S_FMT` on the same file handle.
- **Rolling-shutter skew halved, which is why 4 lanes matters.** Skew is
  (active rows) × (line period), *not* the frame period, and the line period is
  the sensor's HMAX register / 72 MHz:

  | | HMAX | line period | skew over 2192 rows | fps |
  |---|---|---|---|---|
  | 2 lanes | 1066 | 14.806 µs | **32.45 ms** | 30.02 |
  | 4 lanes | 533 | 7.403 µs | **16.23 ms** | 60.04 |

  The measured 60.00 fps is itself the proof the line period halved, since
  fps = 72 MHz / (VMAX × HMAX) and VMAX is unchanged at 2250.

  **16.23 ms is the floor for a full-height readout on this board.** HMAX
  cannot go below 483 (3864 px × 10 bit / 5760 Mbps = 6.708 µs of payload),
  and the 24 MHz INCK caps the lane rate at 1440 Mbps, so the remaining 9 % is
  not worth the loss of link margin. Going meaningfully below 16 ms requires
  reading fewer **rows** — 2×2 binning or a vertical window — not a faster link.

  HMAX sets skew; VMAX sets frame rate. `V4L2_CID_VBLANK` is writable and
  programs VMAX at runtime, so the frame rate can be dropped back toward 30 fps
  *without* giving back the skew improvement.

- **STREAMOFF is clean, and streaming sessions now repeat.** Previously every
  STREAMOFF ended with the ISP error routine firing on `BROKEN_FRAME` and left
  the block unusable until the board was rebooted. The cause was ordering:
  `fw_intf_stream_stop()` tore the sensor and MIPI down first, cutting CSI-2
  off mid-frame, so the ISP saw a truncated frame and `fr_pipeline_busy`
  latched at 1 with no way to ever clear (the register's own documentation says
  `global_fsm_reset` may only be asserted while that bit is low). The error
  routine then correctly refused to restart and left interrupts masked — and
  nothing on the STREAMON path ever unmasked them again, because
  `acamera_fw_init()` runs only at module load. The hardware was never wedged;
  the state was.

  Fixed by draining the ISP *before* the sensor stops
  (`acamera_fw_stream_quiesce()`: SAFE_STOP, then wait for `fr_pipeline_busy`
  to clear while pixels are still flowing) and re-arming interrupts, the input
  port and the error budget on every STREAMON (`acamera_fw_stream_rearm()`).
  Verified 2026-08-06: five consecutive 60-frame captures in one module load,
  three more across a full `unload.sh`/`load.sh` cycle, every one passing the
  stride-autocorrelation check (peak at lag 3968, r = 0.93–0.96), and zero ISP
  error-routine entries in dmesg across the lot.

  `scripts/unload.sh` also had its order corrected — `iv009_isp` must come off
  before `imx415`, since the ISP's `v4l2_async` notifier holds a reference on
  the sensor subdev and `rmmod imx415` returned `EBUSY` otherwise, silently
  leaving the sensor module loaded across every "full" unload.

  The `[Stream#0] Error: wait_event return < 0` that accompanied every
  STREAMOFF was a separate 4.9→6.1 artifact, now suppressed: 6.1's
  `kthread_stop()` sets `TIF_NOTIFY_SIGNAL` before waking the target, so every
  `wait_event_interruptible*` in the stream-copy thread returns `-ERESTARTSYS`
  from that moment. On 4.9 it did not, so the vendor's error log was
  unreachable. It now only fires for a signal that is *not* a stop request.

**Not working / not done:**
- **No IQ calibration.** Frames are correctly formed but untuned.
- **The last 96 bytes of each line's stride padding are never written.** The FR
  DMA writer emits 121 aligned 32-byte bursts per line = 3872 bytes, so columns
  3864–3871 are zero-filled by the final burst and columns 3872–3967 keep
  whatever was in the buffer before. Harmless — every consumer trims rows to
  the visible 3864 — and it is *not* a kernel-memory leak, since
  `dma_alloc_coherent()` hands back zeroed pages and only ISP writes ever land
  there. But it does mean **"padding is all zero" is not a valid check that a
  capture is correct**; use the stride autocorrelation instead.
- **23 buffers dropped at stream start**, then none: all three of `v4l2-ctl`'s
  progress lines across the 120-frame run reported the same cumulative 23, so
  it is a startup transient (the ISP settling over roughly the first 0.4 s),
  not an inability to sustain 60 fps. Discard the opening frames.

### Historical: the sensor once half-answered on I²C

Kept because the measurements are still useful and the dead ends are worth not
re-running. This was resolved — the sensor now talks — but the reasoning below
is what narrowed it down.

This is the open problem, described precisely so nobody re-runs the dead ends.

With `ao_mclk` loaded and reset (GPIOA_11) deasserted, something at `0x1a`
**acknowledges its address** — `i2cdetect` reports it in both quick-write and
read-byte modes, persistently, for as long as you care to poll. But it will not
do register-level traffic:

```
i2cget -y 3 0x1a                     -> 0xff          (address ACKs, data is all-ones)
i2ctransfer -y 3 w2@0x1a 0x3f 0x12 r2 -> No such device or address
i2cset -y 3 0x1a 0x30 0x00           -> Write failed
```

So `imx415` always fails the same way: `8-bit write to 0x3000 failed: -6`
(`-ENXIO`) → `failed to read sensor information`.

Things that were ruled out by direct measurement, not assumption:
- **Not reset polarity or wiring.** Instrumented `imx415_power_on()` prints
  `logical=0 raw=1`, i.e. the DT `GPIO_ACTIVE_LOW` flag is applied and XCLR is
  genuinely released at the time of the failing transfer.
- **Not settle time.** Tried 100 µs → 10 ms → 50 ms → 500 ms. No change.
- **Not the reset pulse width.** Pulsing XCLR low for 1 ms / 5 ms / 20 ms /
  100 ms / 300 ms from userspace all leave the device equally responsive.
- **Not the driver touching reset at all.** With the GPIO requested `GPIOD_ASIS`
  and every `gpiod_set_value()` skipped — the device verified ACKing at `0x1a`
  immediately beforehand — the very next register write from the driver still
  NAKs.
- **Not a bus/address conflict.** `/dev/i2c-3` is confirmed to be DT `i2c3`
  (`ffd1c000.i2c`); its only other occupant is `fusb302` at `0x22`.

**The leading hypothesis is the MCLK frequency.** The pin function is literally
named `CLK12_24` — 12 *or* 24 MHz — and nothing here has verified which one it
currently emits. The IMX415 only supports 24 or 27 MHz INCK; fed 12 MHz its
I²C front end could plausibly ACK its address while failing real register
access. The overlay currently *asserts* 24 MHz via a `fixed-clock`, but that is
a declaration to `imx415.c`, not a measurement. Finding and setting the 12/24
select bit (or scoping the pin) is the next concrete step. A second possibility
worth eliminating first is simply that `0x1a` is a bus artifact rather than the
sensor — reading `0xff` is what a floating bus looks like.

### Other known open issues

1. **DMA coherency — this was real, and it bit twice.** The ported ISP code
   assumed `dma_alloc_coherent()` returns a vaddr in the kernel's linear map, so
   `virt_to_phys()` on it is meaningful. On 6.1 with this device that is false,
   and `virt_to_phys()` fails silently rather than loudly. Both sides are now
   fixed: the address handed to the ISP comes from `vb2_plane_cookie()`
   (`isp-vb2.c`), and the userspace mapping comes from `dma_mmap_coherent()`
   (`isp-vb2-cmalloc.c`). The second one is why capture appeared to work at
   30 fps while every buffer userspace read back was unrelated kernel memory.
   Worth re-checking if the DT node's coherency ever changes.
2. **No IQ/calibration data for the IMX415.** The vendor ISP expects per-sensor
   tuning tables. Capture works; it just looks flat and untuned until real
   calibration exists.
3. **Four lanes are configured and working**, via `imx415_mode_4_1440[]` which
   this port adds. The claim that 4-lane needed a 27 MHz INCK was wrong: it
   confused two independent lookups. `imx415_check_inck()` keys on
   (lane_rate, inck) — lane count is not part of that key — and only then is
   `supported_modes[]` matched on (lanes, lane_rate). A 4-lane mode at an
   already-supported lane rate therefore needs no new INCK settings, just a
   register list. See the endpoint comment in the overlay.
4. **Sub-16 ms skew needs fewer rows, not more bandwidth.** 2×2 binning
   (1932×1096) would cut skew to roughly 4 ms while keeping the full field of
   view, and would still leave ~1.7 px/mm at 10 m with a 50 mm lens — above the
   1.45 px/mm that `shot-vision`'s detector is designed around. It needs a
   binning register list (not in mainline `imx415.c`), matching geometry in
   `V4L2_drv.c`, and an update to `shot-vision`'s `SENSOR_FRAME_BYTES` DMA
   guard.
5. **`ao_mclk` pokes a pinmux register directly**, because mainline has no
   pinctrl group for this function. It saves and restores the field, but it is a
   register poke, not a driver. The clean fix is a `clk12_24` group in
   `pinctrl-meson-g12a.c`.

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
