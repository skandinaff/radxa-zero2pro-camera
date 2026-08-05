# isp_clkc PORT_NOTES

Out-of-tree clk provider that resurrects `cts_mipi_isp_clk_composite` and
`cts_mipi_csi_phy_clk0_composite` (plus 4 related MPEG-bus gates), which do
not exist in this kernel's built-in `amlogic,g12b-clkc` driver. See the
header comment in `isp_clkc.c` for the full design rationale; this file is
the "what I verified vs. assumed" record.

## Build status: clean compile on the live board, not loaded

Built with the real `make`/`Makefile` pattern (`obj-m += isp_clkc.o`,
`KDIR = /usr/src/linux-headers-6.1.68-3-stable`) directly on the board, in
`~/camera-port/scratch/clk-build/` (a fresh dir, not
`~/camera-port/scratch/isp_module/`, which is the parallel effort's — did
not touch it).

Result: `isp_clkc.ko` built with **zero warnings and zero errors** from our
code (only the standard, harmless "compiler differs from the one used to
build the kernel" gcc-12-vs-gcc-10.2.1 notice that any out-of-tree module on
this system gets). `modinfo` on the resulting `.ko`:

```
license:        GPL
depends:                       <- empty: every symbol resolved against vmlinux, nothing missing
vermagic:       6.1.68-3-stable SMP mod_unload aarch64   <- matches running kernel exactly
alias:          of:N*T*Cradxa,zero2pro-isp-clkcC*
alias:          of:N*T*Cradxa,zero2pro-isp-clkc
```

`aux-clk-overlay.dts` also compiled cleanly with `dtc -@ -I dts -O dtb`
(symbols enabled) into a `.dtbo`, and decompiling it back confirms the
`__symbols__` node correctly captures
`isp_clkc = "/fragment@0/__overlay__/isp-clkc"` — i.e. the label survives
compilation and is available for another overlay's `__fixups__` to resolve
against, once this overlay has been applied to the live tree (see
"Overlay ordering / symbols" below).

**Not** insmod'd, **not** applied to the live tree — per the constraints,
that's for a human to do separately. `~/camera-port/scratch/clk-build/` on
the board still has the build outputs (`isp_clkc.ko`, `aux-clk-overlay.dtbo`)
if you want to inspect them without rebuilding.

## What was verified directly on the live board vs. assumed

**Verified (commands run, output checked):**

- `dt-bindings/clock/g12a-clkc.h` and `clk_summary` genuinely have no
  ISP/CSI-input clock IDs (this was the task's own starting premise, and I
  re-confirmed the same two greps came back MIPI-DSI-only).
- All 8 mux parent names for both composites are present **verbatim** in
  `/sys/kernel/debug/clk/clk_summary` (had to `sudo -n cat`, plain user read
  is permission-denied but passwordless sudo works): `xtal`, `gp0_pll`,
  `hifi_pll`, `mpll1`, `mpll2`, `fclk_div2p5`, `fclk_div3`, `fclk_div4`,
  `fclk_div5`, `fclk_div7` — all present, all exactly these names.
- `clk81` (the parent the vendor's `MESON_GATE()` macro hardcodes for all
  four simple gates) is present in `clk_summary` too.
- Exact live devicetree path to the syscon node, via
  `sudo find /sys/firmware/devicetree/base -iname '*system-controller*'`:
  `/soc/bus@ff600000/bus@3c000/system-controller@0`. Cross-checked its
  `compatible`, `reg`, and `phandle` (0xed) against
  `reference/zero2pro.dts` — byte-for-byte match. This is the exact
  `target-path` used in `aux-clk-overlay.dts`. (There's a second,
  *unrelated* "clock-controller" at `/soc/bus@ff800000/sys-ctrl@0/` — the
  AO/always-on domain — easy to grab by mistake; the one we want is the
  child of `bus@3c000/system-controller@0`.)
- `system-controller@0`'s `reg` is `<0x0 0x0 0x0 0x400>` (1024 bytes) — all
  four register offsets we touch (0x144, 0x148, 0x1c0, 0x340, each read as
  a 4-byte word) fall inside that range, so they're in-bounds for the
  regmap the syscon core creates from this `reg` property.
- **Exported-symbol / GPL status** for everything load-bearing, checked
  against the real `Module.symvers` on the board (`grep -w <sym>
  /usr/src/linux-headers-6.1.68-3-stable/Module.symvers`), not assumed from
  memory:

  | symbol | exported | GPL? |
  |---|---|---|
  | `device_node_to_regmap` | yes | `EXPORT_SYMBOL_GPL` |
  | `syscon_node_to_regmap` / `syscon_regmap_lookup_by_phandle` | yes | GPL (not used in the final driver — see below) |
  | `regmap_read`, `regmap_write` | yes | GPL |
  | `regmap_update_bits_base` (backs the `regmap_update_bits()` inline) | yes | GPL |
  | `__clk_hw_register_mux` / `_divider` / `_gate` (back the `devm_clk_hw_register_*()` macros) | yes | GPL |
  | `devm_clk_hw_register` (generic, what this driver actually calls) | yes | GPL |
  | `clk_hw_register`, `clk_hw_unregister` | yes | GPL |
  | `of_clk_add_hw_provider`, `devm_of_clk_add_hw_provider`, `of_clk_del_provider` | yes | GPL |
  | `of_clk_hw_onecell_get` | yes | GPL |
  | `divider_recalc_rate`, `divider_round_rate_parent`, `divider_get_val` | yes | GPL |
  | `dev_err_probe` | yes | GPL |
  | `clk_divider_ops`, `clk_mux_ops`, `clk_gate_ops` (exported, but **not used** — see design notes) | yes | GPL |
  | `devm_clk_hw_register_mux/_divider/_gate` **as bare symbols** | **not present** in `Module.symvers` — they're `#define`s in `clk-provider.h` wrapping the `__`-prefixed functions above, not separate exports | n/a |

  Everything this driver calls is `EXPORT_SYMBOL_GPL`, so `isp_clkc.c` is
  `MODULE_LICENSE("GPL")` (matches `dtbo_loader`).
- `CONFIG_MFD_SYSCON=y`, `CONFIG_REGMAP_MMIO=y`, `CONFIG_OF_OVERLAY=y`,
  `CONFIG_COMMON_CLK_G12A=y`, `CONFIG_COMMON_CLK_MESON_REGMAP=y`,
  `CONFIG_COMMON_CLK_MESON_EE_CLKC=y` all confirmed `=y` in
  `/boot/config-6.1.68-3-stable` (also cross-checked against
  `/proc/config.gz`). The last three are the real smoking gun for the next
  point.
- **The built-in `amlogic,g12b-clkc` driver on this exact kernel is the
  mainline `drivers/clk/meson/g12a.c`, and it is regmap-based, not a
  private `ioremap()`.** I did not just assume this from memory of
  mainline source — I checked two independent pieces of live evidence:
  1. `CONFIG_COMMON_CLK_MESON_EE_CLKC=y` is set, which in mainline is what
     provides `meson_eeclkc_probe()`, whose entire implementation is
     `regmap = device_node_to_regmap(dev->parent->of_node)` — i.e. exactly
     the syscon-shared-regmap pattern, no ioremap.
  2. `sudo cat /proc/iomem` shows **no** reserved region for the
     `system-controller@0` physical range (`0xff63c000`), meaning nobody
     called `devm_ioremap_resource()`/`request_mem_region()` against it —
     if the base clk driver did its own raw ioremap the way the vendor
     driver does, this range would very likely show up reserved (compare:
     the *unrelated* AO clock-controller at `0xff642000` **does** show up
     in `/proc/iomem`, since that one apparently *is* separately
     resource-mapped — so the absence for the HHI block isn't just
     "nothing shows up in this file ever," it's specific to this node).
  3. `sudo ls /sys/kernel/debug/regmap/` lists exactly **one** regmap
     entry for that physical address:
     `dummy-system-controller@0x00000000ff63c000` (the `dummy-` prefix is
     how regmap-core names a regmap that isn't tied to a fully-populated
     `struct device` at registration time — consistent with the very-early
     syscon registration path). One shared regmap, used by both the base
     driver and (once loaded) this module.

**Assumed / not independently re-derived (lower confidence, flagged
explicitly):**

- I trust register offsets/bitfields/mux-parent-ordering exactly as given
  in the task brief and cross-checked in `reference/g12a.h` and
  `reference/g12b_clk.c` — I did not have a datasheet to verify these
  against, only the vendor source. This is the same trust level the task
  brief itself operates at ("ground truth" vendor file).
- I did not verify on-device that `clk_prepare_enable()` (rather than a
  bare `clk_enable()`) is what the parallel ISP/CSI2 driver actually calls
  — I'm relying on this being near-universal modern-driver practice. If
  that driver calls raw `clk_enable()`/`clk_disable()` without a preceding
  `clk_prepare()`, our gates (implemented via `.prepare`/`.unprepare`, see
  below) will not actually gate the clock. Worth a one-line check against
  that driver's source before final integration.
- I did not verify whether anything **else** in the live system ever
  writes to `HHI_MIPI_ISP_CLK_CNTL` (0x1c0) or `HHI_MIPI_CSI_PHY_CLK_CNTL`
  (0x340) — these looked, from `clk_summary`, to be entirely unused/absent
  today (consistent with the premise that these clocks don't exist yet in
  this kernel), so I'm treating those two registers as exclusively ours.
  `HHI_GCLK_MPEG1`/`HHI_GCLK_MPEG2`, by contrast, are **not** exclusively
  ours (see locking section) — the base driver has other gate bits in
  those same words.

## Locking: what I concluded, and why regmap's own locking is (just barely) not enough on its own

The task asked me to think about this rather than assume it's fine. Short
version: **the mux/div/gate register-word sharing is safe as long as every
field write goes through a single `regmap_update_bits()` call (never a
separate read then write), but the *gate* stage specifically cannot be
implemented as `.enable`/`.disable` — it has to be `.prepare`/`.unprepare`
— because the shared regmap's locking is a sleeping mutex and clk core
calls `.enable`/`.disable` with a spinlock held.**

Longer version, in three parts:

**1. Does regmap's own internal lock cover the "read-modify-write of a
shared register" hazard?** Yes, and this fully replaces the vendor driver's
private `clk_lock` spinlock, for two separate hazards at once:

- *Intra-driver*: our own mux (bits 11:9), div (bits 6:0), and gate (bit 8)
  for the same composite all live in the same 32-bit register
  (`HHI_MIPI_ISP_CLK_CNTL` / `HHI_MIPI_CSI_PHY_CLK_CNTL`). A plain
  read-then-write from two of these racing would corrupt each other's
  field.
- *Inter-driver*: `HHI_GCLK_MPEG1`/`HHI_GCLK_MPEG2` are **not** exclusive
  to this driver — the base `amlogic,g12b-clkc` driver has its own gate
  bits in those same two 32-bit words (ethernet, USB, SD/eMMC, etc). A
  private spinlock inside *this* module would do nothing to protect against
  the base driver's concurrent access to the same word.

`regmap_update_bits()` (→ `regmap_update_bits_base()`) does its own
locked read-modify-write *inside the regmap core*, keyed on the `struct
regmap *` instance, not on whichever driver happens to be calling it. Since
`device_node_to_regmap()` on the syscon node hands back the *same* regmap
object the base driver uses (see verified evidence above — one
`dummy-system-controller@...` regmap, not two), every `regmap_update_bits()`
call from *any* caller against that instance is mutually exclusive with
every other one, for both hazards above, for free. This is the entire
point of the simple-mfd/syscon pattern and is why I didn't add a private
spinlock like the vendor driver's `clk_lock`. The one rule this depends on:
**never split a field update into a separate `regmap_read()` +
`regmap_write()`** — always `regmap_update_bits()`, which this driver does
consistently (mux `set_parent`, div `set_rate`, gate `prepare`/`unprepare`
all use it).

**2. Is that lock safe to take from every context the clk core might call
us in?** No — and this is the subtlety that would have silently broken
things if copied naively from the vendor driver's `clk_gate_ops`. Checked
directly in `linux/clk-provider.h`'s `struct clk_ops` kerneldoc on the
board:

- `.enable`/`.disable`: *"Called with enable_lock held. This function must
  not sleep."* `enable_lock` is a spinlock.
- `.is_enabled`: *"This function must not sleep."*
- `.prepare`/`.unprepare`: *"Called with prepare_lock held"* (a mutex —
  confirmed by the surrounding doc block describing prepare/enable as the
  sleepable/atomic split).
- `.is_prepared`: *"This function is allowed to sleep."*

`drivers/mfd/syscon.c`'s `syscon_regmap_config` does not set `.fast_io`, so
the regmap we get is the default mutex-locked kind — sleep-capable, and
therefore **illegal to touch from `.enable`/`.disable`/`.is_enabled`**
(would be "sleeping while atomic" the instant it actually contends the
mutex). This is presumably exactly why the vendor driver uses raw
`readl`/`writel` + its own spinlock instead of anything regmap-based — that
combination genuinely is atomic-safe, but only because it forgoes sharing
the register range safely with anyone else.

**Resolution**: `isp_clkc_gate_ops` in `isp_clkc.c` implements
`.prepare`/`.unprepare`/`.is_prepared` and deliberately leaves
`.enable`/`.disable`/`.is_enabled` unset. This is the officially-supported
split documented in the same header ("Clock enable code that will never be
called in a sleepable context may be implemented in clk_enable" — the
converse, doing it all in `.prepare`, is equally valid and is what several
in-tree "slow" gates, e.g. regulator- or I2C-backed ones, do for the same
reason). With no `.enable`/`.disable`, clk core treats "prepared" as
"running," which is correct here since there's no genuinely-atomic part of
flipping this bit. **Consequence for the consumer** (noted in the driver
header too): it must call `clk_prepare_enable()`/`clk_prepare()` before
`clk_enable()`, not a bare `clk_enable()` — see the "assumed" section
above, this wasn't verified against the actual ISP/CSI2 driver source.

**3. Mux and divider** (`.get_parent`/`.set_parent`,
`.recalc_rate`/`.round_rate`/`.set_rate`) all run under `prepare_lock`
(a mutex) per the same doc, so they're fine to use the mutex-backed regmap
directly — no `.prepare`-style indirection needed for those.

## Design choices worth flagging

- **No `clk_register_composite()`.** The vendor driver (and the mainline
  clk core) has a helper that bundles a mux+div+gate triplet behind one
  `clk_hw`/one clock-tree name. I didn't use it — its `clk_ops` require
  `void __iomem *`-shaped registers via the standard `clk_mux_ops`/
  `clk_divider_ops`/`clk_gate_ops`, which is exactly the raw-ioremap
  pattern the task said not to copy. Instead this driver registers three
  separate `clk_hw`s per composite (`..._mux` → `..._div` → `..._gate`,
  parented by name, same topology, same final register-bit layout as the
  vendor's composite), and exposes only the terminal gate-stage `clk_hw`
  through the `#clock-cells` onecell array under indices 0/1. The two
  intermediate stages are still fully registered/visible in
  `clk_summary` under their own names (`cts_mipi_isp_clk_mux`,
  `cts_mipi_isp_clk_div`, etc.) — they're just not directly reachable by
  DT phandle, which nothing needs them to be.
- **Why a child of the syscon node instead of a `syscon = <&phandle>`
  property**: the task allowed either. I mirrored the existing, already-
  working `clock-controller`/`power-controller` siblings exactly (literal
  child of `system-controller@0`, look up the regmap via
  `device_node_to_regmap(dev->parent->of_node)` in `probe()`) rather than
  inventing a new attachment convention, on the theory that copying a
  pattern already proven to probe correctly on this exact board/kernel is
  lower-risk than a novel one. If for some reason the isp/camera overlay
  effort would rather reference `isp_clkc` from somewhere else in the
  tree, `syscon_regmap_lookup_by_phandle(np, "syscon-name-here")` is a
  drop-in replacement for the `device_node_to_regmap()` call in `probe()`
  and both are already confirmed exported/GPL (see table above).
- **Only clk0 of the CSI-PHY composite, not clk1 or the clk0/clk1 select
  mux** — per the task brief, since the isp_module driver's DT binding
  only references clk0.
- **`CLK_IGNORE_UNUSED`** is set on all gate registrations (matches the
  vendor `MESON_GATE()` macro's flags), so generic `clk_disable_unused`
  housekeeping won't turn these back off before a consumer driver has
  attached. In practice this module loads long after boot (`insmod`, not
  built-in), so `clk_disable_unused`'s one-shot late-boot pass will
  already be long done by the time these clocks are even registered — this
  flag is defense-in-depth, not load-bearing.

## Overlay ordering / symbols

`aux-clk-overlay.dts` only adds the `isp_clkc` provider node — not the full
camera overlay (per the task brief, that's being assembled separately).
For the eventual camera overlay to say `<&isp_clkc 0>` and have it resolve:

1. Both `.dts` files need to be compiled with `dtc -@ ...` (confirmed this
   overlay's compiled `.dtbo` correctly emits a `__symbols__` node —
   `isp_clkc = "/fragment@0/__overlay__/isp-clkc"` — checked by
   decompiling the built `.dtbo` back with `dtc -I dtb -O dts`).
2. This overlay must be `of_overlay_fdt_apply()`'d (e.g. via
   `dtbo_loader`, pointed at this `.dtbo`, loaded before/independently of
   the camera module) **before** the overlay that references
   `&isp_clkc`, so the label exists in the live tree's symbol table by the
   time the second overlay's fixups resolve it.
3. `isp_clkc.ko` needs to be `insmod`'d for the node to actually bind to a
   driver and register clocks — inserting the overlay alone only adds an
   (unbound, unprobed) devicetree node.

None of this was exercised end-to-end (no insmod, no overlay apply, per
the read-only/scratch-only constraint) — steps 1–2 were verified by
compiling+decompiling the `.dtbo` as described above; step 3 was verified
by the clean out-of-tree `make` build. The human doing on-device
loading should insmod `isp_clkc.ko` and confirm via
`cat /sys/kernel/debug/clk/clk_summary | grep cts_mipi` that both
composites (and the four gates) show up before layering the camera
overlay on top.

## Addendum (hand-edit after agent handoff, not agent-authored)

Added a 7th clock, `gen_clk` (index `CLKID_GEN_CLK_COMP` = 6): the sensor
MCLK source (GPIOAO_10's "CLK12_24" alt-function per the Zero 2 Pro
schematic; the vendor's `kvim3_linux.dts` sensor node references
`<&clkc CLKID_GEN_CLK>` / `pinctrl-0=<&gen_clk_ee_ao>` for exactly this).
Same missing-from-mainline story as clocks 0/1, re-confirmed the same way
(`grep GEN dt-bindings/clock/g12a-clkc.h` and `clk_summary` both empty on
the live board). Register layout ported from vendor
`drivers/amlogic/clk/g12a/g12a.c`'s `g12a_gen_clk_{sel,div,gate}`
(HHI_GEN_CLK_CNTL @ 0x228) -- this is a *base* G12A clock, not a G12B-only
addition like 0-5, consistent with the vendor dts referencing it from the
base `&clkc` rather than anywhere isp/adapter/phycsi-specific.

Unlike clocks 0/1, `gen_clk`'s 5-bit mux field is not a dense parent index
- it uses a real value table (`{0,5,6,7,20,21,22,23,24,25,26,27,28}`).
Extended `isp_clkc_mux` with an optional `.table` (NULL for 0/1, set for
gen_clk) and a reverse-lookup `.get_parent`, matching how the core's own
`clk_mux_ops` handles a `.table`-mapped mux. One of gen_clk's 13 listed
parents, `gp1_pll`, does not exist on this board (checked
`clk_summary` -- absent, the only one of the 13 that is); left in the
parent-name list to match the vendor table exactly since the clk core
tolerates an unresolvable parent name as long as nothing selects that
index, and nothing here does (we only ever select index 0, `xtal`).

Also corrected gate `init.flags` to match the vendor source exactly
per-clock rather than one blanket value: the two composite terminal gates
(`cts_mipi_isp_clk_composite`/`cts_mipi_csi_phy_clk0_composite`) get
`CLK_GET_RATE_NOCACHE` only (matching vendor, no `CLK_IGNORE_UNUSED`); the
four simple MPEG-bus gates get `CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED`
(matching the `MESON_GATE()` macro exactly, which the original pass had
half of -- `CLK_IGNORE_UNUSED` but not `CLK_SET_RATE_PARENT`); `gen_clk`'s
own gate gets `CLK_SET_RATE_PARENT` only (matching vendor
`g12a_gen_clk`'s explicit flags, no `CLK_IGNORE_UNUSED` since the vendor
doesn't set it there either -- this one predates/isn't generated by the
`MESON_GATE()` macro). `isp_clkc_register_div()`/`_register_gate()` both
gained a `flags` parameter to make this per-call-site instead of hardcoded.

Not yet re-verified by a real on-device `make` after this edit at the time
of writing -- doing that next as part of final integration, alongside the
isp_module build.
