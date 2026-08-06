# IMX415 ISP calibration

What the ACamera ISP's calibration surface is, what this port now puts in it,
where every number came from, and what still has to be measured on real
hardware before the image is trustworthy.

Scope: `isp-module/src/calibration/`. Two complete calibration sets now exist —
ARM's neutral reference (`*_dummy.c`, untouched) and an IMX415 set
(`*_imx415.c`, new) — with `acamera_get_calibrations_v4l2.c` selecting between
them.

> **Status.** The IMX415 set is written and compiles, but **none of it has been
> validated on hardware**, because the sensor still does not complete I²C
> register access (see README, "Where it stands"). Everything below that is
> labelled *derived* is arithmetic from a datasheet or from this tree's own
> source; everything labelled *placeholder* is explicitly not a measurement.
> Nothing here has been confirmed against a real frame.

---

## 1. Why this matters more than usual here

The consumer is not a viewfinder. It is a Rust computer-vision pipeline that

1. Otsu-thresholds a grayscale frame, runs connected components, and circle-fits
   a paper target; and
2. frame-differences a pre-shot frame against a post-shot frame to locate a new
   bullet hole.

Both stages make assumptions that a photographic ISP tuning quietly violates:

| The pipeline assumes | An ISP tuned for human eyes does |
|---|---|
| output DN ∝ scene radiance | applies a ~1/2.2 gamma to lift shadows |
| one radiance→DN mapping for the whole frame | applies Iridix local tone mapping |
| the same mapping in both frames | re-runs AE/AWB/auto-level per frame |
| small dark blobs are signal | denoises them away (sinter), and blends them across frames (temper) |
| an edge is where the region ends | adds overshoot halos on both sides (sharpening) |
| a global threshold works across the target | leaves lens vignetting in |

So this tuning is not "the dummy tables but better". It is a deliberate move in
the opposite direction from every published IMX415 profile: **linear, flat,
inert, and boring**, with the one genuinely image-quality-shaped requirement
being lens shading correction.

---

## 2. The calibration surface

### 2.1 Mechanics

`get_calibrations_v4l2()` fills an `ACameraCalibrations` — an array of
`CALIBRATION_TOTAL_SIZE` (123) `LookupTable *`. Supplying them is not optional:
`_GET_LUT_PTR()` in `acamera_calibrations.c` responds to a NULL table by
entering a deliberate infinite loop with a 3 s sleep per iteration, which wedges
`insmod` in uninterruptible sleep and makes the module unloadable.

```c
typedef struct LookupTable {
    void *ptr;
    uint16_t rows, cols, width;   /* width = sizeof(element) */
} LookupTable;
```

The set is split across two files by lifetime, not by function:

* **static** (51 tables) — mode-invariant; loaded once.
* **dynamic** (60 tables) — mostly `{x, y}` modulation curves the firmware
  interpolates against a running variable.

111 of the 123 slots are filled; the remainder belong to FSMs this build does
not compile.

**Two modulation axes, and they are not the same.** Getting these confused
silently produces a curve that is off by a factor of 8:

* Noise reduction / sharpening / saturation / AE tables use
  `log2(total_gain) * 256` (`noise_reduction_func.c`:
  `log2_gain = total_gain >> (LOG2_GAIN_SHIFT - 8)`, `LOG2_GAIN_SHIFT` = 18).
  So a row written `{3 * 256, v}` applies at 8× gain.
* The black-level tables use `log2(analog_gain) * 32` (`sensor_func.c`:
  `again_log2 = exp.info.again_log2 >> (LOG2_GAIN_SHIFT - 5)`). Their nine rows
  at 0…256 span 1×…256×.

**Not everything in the set is read by the kernel.** This build compiles the
*manual* variants of the 3A FSMs (`ISP_HAS_AE_MANUAL_FSM`,
`ISP_HAS_AWB_MANUAL_FSM`, `ISP_HAS_IRIDIX8_MANUAL_FSM`), meaning the actual AE /
AWB / Iridix algorithms live in a **userspace 3A library** which receives the
whole calibration set over the sbuf channel (`sbuf_func.c` ships all
`CALIBRATION_TOTAL_SIZE` entries into `cali_data[]`). That library is not part
of this port. `CALIBRATION_AE_CONTROL` and `CALIBRATION_AUTO_LEVEL_CONTROL`
have **no reader anywhere in the kernel module** — grep confirms it. Until a 3A
daemon exists, 3A does not adapt at all and those tables are inert. They are set
correctly anyway so they stay correct when it appears.

### 2.2 Inventory

Grouped by what they actually drive, and by whether they matter for a grayscale
CV consumer. "Changed" means changed from the ARM reference by this work.

#### Directly load-bearing for this application

| Table | Type / size | ISP block | Changed | Source |
|---|---|---|---|---|
| `BLACK_LEVEL_{R,GR,GB,B}` | u16 9×2 | `sensor_offset_pre_shading` | ✅ | **datasheet** |
| `GAMMA` | u16 ×129 | `fr_gamma_rgb_mem`, `ds1_gamma_rgb_mem` | ✅ | **neutral-CV** (exact linear) |
| `SHADING_LS_{A,TL84,D65}_{R,G,B}` | u8 ×1024 (32×32) | `mesh_shading_mem` | — | ⚠️ **PLACEHOLDER** |
| `MESH_SHADING_STRENGTH` | u16 1×2 | `mesh_shading_mesh_strength` | — | ARM ref (full strength) |
| `SINTER_STRENGTH`, `SINTER_STRENGTH1` | u16 8×2 | `sinter_strength{,_1}` | ✅ | ARM ref shape × 0.3 / 0.4 |
| `TEMPER_STRENGTH` | u16 8×2 | temper (TNR) | ✅ | **neutral-CV** (zeroed) |
| `SHARP_ALT_{D,UD,DU}`, `SHARPEN_FR`, `SHARPEN_DS1` | u16 ~8×2 | sharpening | ✅ | **neutral-CV** (zeroed) |
| `IRIDIX_STRENGTH_MAXIMUM`, `IRIDIX8_STRENGTH_DK_ENH_CONTROL` | u8 ×1, u32 ×15 | Iridix | ✅ | **neutral-CV** (off) |
| `CMOS_CONTROL` | u32 ×18 | AE/exposure policy | ✅ | anti-flicker **on**, 60 Hz |
| `AE_CONTROL`, `AE_CORRECTION` | u32 ×9, u8 ×12 | userspace AE | ✅ | derived + ⚠️ placeholder target |
| `AUTO_LEVEL_CONTROL` | u32 ×7 | userspace auto-level | ✅ | **neutral-CV** (off) |
| `NOISE_PROFILE` | u8 ×128 | `sinter_/temper_noise_profile_lut` | ✅ | shot-noise model; ⚠️ scale placeholder |
| `RGB2YUV_CONVERSION` | u16 ×12 | `matrix_yuv` | — | ARM ref — **defines the grayscale** |
| `DP_SLOPE`, `DP_THRESHOLD` | u16 7×2 | defect pixel | — | ARM ref (kept **on**, deliberately) |

#### Matter only weakly (colour path; grayscale consumer discards it)

| Table | Type / size | Changed | Source |
|---|---|---|---|
| `STATIC_WB` | u16 ×4 | ✅ | **datasheet** sensitivity ratios |
| `MT_ABSOLUTE_LS_{A,D40,D50}_CCM` | u16 ×9 | ✅ | **neutral-CV** (identity) |
| `AWB_WARMING_LS_{A,D50,D75}` | u16 ×3 | — | already neutral |
| `SATURATION_STRENGTH` | u16 8×2 | ✅ | flattened |
| `DEMOSAIC` | u8 ×128 | ✅ | shot-noise model |
| `CNR_UV_DELTA12_SLOPE` | u16 8×2 | — | ARM ref |
| `CA_FILTER_MEM`, `CA_CORRECTION{,_MEM}`, `LUT3D_MEM` | u32/u16 | — | ARM ref |
| `PF_RADIAL_LUT` | u8 ×33 | — | ARM ref |

#### AWB colour-temperature machinery — inert (white balance is pinned)

`LIGHT_SRC`, `RG_POS`, `BG_POS`, `MESH_RGBG_WEIGHT`, `MESH_LS_WEIGHT`,
`MESH_COLOR_TEMPERATURE`, `WB_STRENGTH`, `SKY_LUX_TH`, `CT_RG_POS_CALC`,
`CT_BG_POS_CALC`, `COLOR_TEMP`, `CT{65,40,30}POS`, `AWB_SCENE_PRESETS`,
`AWB_BG_MAX_GAIN`, `AWB_COLOUR_PREFERENCE`, `AWB_MIX_LIGHT_PARAMETERS`,
`AWB_AVG_COEF`, `CCM_ONE_GAIN_THRESHOLD`, `EVTOLUX_{EV,LUX}_LUT`,
`EVTOLUX_PROBABILITY_ENABLE` — all carried from the ARM reference.

#### Dead in this configuration

* **WDR only** (build is linear-only): `WDR_NP_LUT`, `DECOMPANDER{0,1}_MEM`,
  `STITCHING_*`, `EXPOSURE_RATIO_ADJUSTMENT`, `FS_MC_OFF`,
  `AE_CONTROL_HDR_TARGET`.
* **Iridix-dependent** (Iridix pinned off): `IRIDIX_ASYMMETRY`,
  `IRIDIX_AVG_COEF`, `IRIDIX_EV_LIM_{FULL,NO}_STR`, `IRIDIX_MIN_MAX_STR`,
  `SINTER_STRENGTH_MC_CONTRAST`.
* **No AF motor** (fixed lens, no lens driver linked): `AF_LMS`,
  `AF_ZONE_WGHT_{HOR,VER}`.
* **Sensor-independent**: `SCALER_{H,V}_FILTER` (polyphase resampler taps).
* **Radial shading**: `SHADING_RADIAL_{R,G,B}` are loaded into
  `radial_shading_mem`, but nothing in this build ever clears
  `top_bypass_radial_shading` — contrast `color_matrix_func.c`, which explicitly
  enables mesh shading. Set to unity here; see §5.

---

## 3. The OpenIPC sensor-profiles investigation — **dead end, with evidence**

**Verdict: not usable, and not even sensor-specific.** Two independent lines of
evidence, either of which alone would settle it.

Files examined (all five in
`github.com/OpenIPC/sensor-profiles/tree/master/files`, downloaded 2026-08-05):

```
imx335_greg15.bin            88738 bytes  md5 426c62b83cea5e34990a10cb986648e5
imx335_milos6.bin            88738 bytes  md5 63df0c9f57794bb8ac286cd6dcfda2e0
imx335_ssc338q_20240628.bin  88738 bytes  md5 8fb07710f42ec8c364c3c684a627b710
imx415_milos12.bin           88738 bytes  md5 095b53ce58d5489e070d52b94dccfd16
imx415_ssc338q_20240613.bin  88738 bytes  md5 d7174c3e0e2eb208b78650b65dad973f
```

### 3.1 It is a different ISP's format

The container is not ACamera. Every file begins `21 03 65 6d` (0x6d650321),
followed by a fully-packed TLV stream — 81 records of
`{u32 magic 0x4d2, u32 type, u32 len, u8 payload[len]}` with a 16-byte header
and no gaps (verified: the record chain walks the file exactly). Each payload
opens with a u16 module id from a space including 0x1005–0x1036, 0x1403–0x1416,
0x1802–0x180b, 0x1c01–0x1c2f, 0x2803, 0x2c03, 0x3000–0x3003.

ACamera has no such container. Its calibration set is C source compiled into the
driver: 123 named `LookupTable` slots with fixed semantics, each a specific
shape (129-entry gamma, 32×32×3 mesh, 9-row modulation curves). There is no
id-to-slot correspondence to discover, because these are two different ISPs'
register maps with different block decompositions.

Supporting evidence for the vendor: three of the five filenames literally
contain `ssc338q`, a Sigmastar SoC part number, and the repository README
directs users to copy the `.bin` to `/etc/sensors` and point
`/etc/majestic.yaml` at it — Majestic being the OpenIPC video daemon for those
SoCs. Nothing about ARM ACamera.

### 3.2 The files are not sensor characterisations anyway

This is the more interesting finding, and it kills the idea even for someone
willing to write a converter.

Whole-file byte difference between every pair:

|  | 415_ssc338q | 415_milos12 | 335_ssc338q | 335_greg15 | 335_milos6 |
|---|---|---|---|---|---|
| **415_ssc338q** | 0.00% | 27.69% | **0.89%** | 0.89% | 28.06% |
| **415_milos12** | 27.69% | 0.00% | 27.05% | 27.04% | **0.73%** |
| **335_ssc338q** | 0.89% | 27.05% | 0.00% | 0.03% | 27.64% |
| **335_greg15** | 0.89% | 27.04% | 0.03% | 0.00% | 27.63% |
| **335_milos6** | 28.06% | 0.73% | 27.64% | 27.63% | 0.00% |

Byte-identical records, out of 81:

|  | 415_ssc338q | 415_milos12 | 335_ssc338q | 335_greg15 | 335_milos6 |
|---|---|---|---|---|---|
| **415_ssc338q** | 81 | 27 | 50 | 49 | 18 |
| **415_milos12** | 27 | 81 | 34 | 36 | 57 |
| **335_ssc338q** | 50 | 34 | 81 | 72 | 24 |

**The files cluster by author, not by sensor.** `imx415_ssc338q` differs from
`imx335_ssc338q` by 0.89% of bytes — but from `imx415_milos12`, the *same
sensor*, by 27.69%. Likewise `imx415_milos12` vs `imx335_milos6`: 0.73%. Whoever
produced each profile started from a shared template and adjusted a handful of
knobs; the sensor name in the filename records what the tuner pointed the camera
at, not a characterisation of the part.

The three largest tables in the file — 26 128 B (id 0x1008), 17 424 B (0x1009),
8 720 B (0x1035), together 58% of the file — are **byte-identical** between
`imx415_ssc338q` and `imx335_ssc338q`. Whatever they are (their size makes LSC
mesh or a per-gain curve family likely), they carry zero information that
distinguishes an IMX415 from an IMX335.

### 3.3 What *is* recognisable, and why it still doesn't help

For completeness, the extraction is not impossible — recognisable structures do
decode. Record `0x1036` (1036 B) contains a clean 256-entry monotonic curve
rising 0 → 1023:

```
i:      0    1    4    8   16   32   64  128  192  255
val:    0    8   31   71  169  365  593  807  928 1023
```

That is unmistakably a gamma / tone curve, and an aggressive one — it lifts
input 16/255 (6.3%) to 169/1023 (16.5%), i.e. shadow-boosting well beyond a
plain 1/2.2.

It is also **byte-identical across all five profiles**, including across both
sensors and both tuners. It is a stock Sigmastar default, not IMX415 data.

And even if it were IMX415-specific, this application wants the *opposite* of
it. §4 explains why linear gamma is required; importing a shadow-lifting
photographic curve would be a regression, not a win.

**Conclusion.** Nothing was taken from these files. No numbers from OpenIPC
appear anywhere in `*_imx415.c`. The analysis scripts are not committed — they
were throwaway — but every figure above is reproducible in a few lines of Python
against the published `.bin` files.

---

## 4. What was filled in, and from where

Every table in both `*_imx415.c` files carries a `SOURCE:` comment. The four
categories used:

* **datasheet** — arithmetic from `IMX415-AAQR-C_Datasheet_E19504.pdf`
  (published by Radxa for this exact camera at
  `dl.radxa.com/accessories/camera-4k/`) or from this tree's own source.
* **carried from a similar Sony sensor** — **not used.** See §4.5.
* **neutral-CV** — a principled neutral/identity choice, with the reason stated.
* **PLACEHOLDER** — needs a hardware measurement. Listed in §5.

### 4.1 Black level — datasheet, and the units cross-check

The datasheet's "Black Level Adjustment Function" recommends `BLKLEVEL` = 032h
and defines the unit as **1 digit/1h in 10-bit readout, 4 digit/1h in 12-bit**,
i.e. **50 LSB at 10 bits, 200 LSB at 12 bits**. `imx415/imx415.c` programs
exactly that (`IMX415_BLKLEVEL_DEFAULT` = 50) and this board runs 10-bit
(`IMX415_BITS` in `V4L2_drv.c`).

The calibration table's unit is a 12-bit domain, which two independent routes
agree on:

* `sensor_func.c` writes `sensor_offset_pre_shading_offset_XX` as
  `value << BLACK_LEVEL_SHIFT_WB`, `BLACK_LEVEL_SHIFT_WB` = 8
  (`acamera_firmware_config.h`), into a 20-bit register field
  (`acamera_isp1_config.h`: mask `0xfffff`). 20 − 8 = 12.
* `ISP_INPUT_BITS` is 20 and the input formatter MSB-aligns the sensor's 10 bits
  (`sensor_update_bayer_bits()` selects `input_bitwidth_select` = 1 for 10-bit).
  A 10-bit pedestal of 50 therefore sits at 50 << 10 = 51 200 in the 20-bit
  domain, i.e. 51 200 >> 8 = **200** in this table's units.

Both give 200. Written flat across all nine gain rows: the datasheet applies
`BLKLEVEL` *after* gain modulation ("added relative to the data in which the
digital gain modulation was performed"), so it is a constant in output DN, and
dark signal is specified at ≤ 0.89 digit (12-bit, 1/30 s, Tj = 60 °C) — under
half an LSB — so dark current does not move it either.

Still worth one confirming capture; the procedure is §5.1. If it disagrees, the
MSB-alignment assumption is what is wrong.

### 4.2 Gamma — neutral-CV, exact linear

`gamma[i] = round(i * 4095 / 128)`, 129 entries.

Otsu's method maximises between-class variance on the intensity histogram. It is
a *thresholding* rule, not a ranking rule, so a monotone-but-nonlinear remap does
not commute with it: a shadow-lifting curve compresses the highlight end where
white paper sits and expands the dark end where holes sit, dragging the optimal
split point around as scene brightness changes.

Frame differencing is worse. Under a nonlinear gamma, the output delta produced
by a fixed reflectance change depends on the absolute level it happens at — so
the same new hole yields a different magnitude depending on how brightly the
target was lit. Linear gamma makes output DN proportional to scene radiance,
which is what both stages are built on.

### 4.3 Static white balance — datasheet, with a stated caveat

The datasheet's Image Sensor Characteristics gives **sensitivity ratios**, not
white-balance gains: R/G = 0.42…0.58, B/G = 0.26…0.44. White balance is the
inverse, so at the midpoints: R gain = 256/0.50 = **512**, B gain = 256/0.35 =
**731**, Gr = Gb = 256.

Caveat, stated plainly: those ratios are measured under Sony's "standard imaging
condition II" — a 3200 K source behind an IR-cut filter — so this is a
**tungsten-referenced** white balance. Under daylight or white LED the true blue
gain is lower and the red gain higher.

That is tolerable here because white balance reaches a grayscale consumer only
through the RGB→Y matrix, where a channel gain error becomes a small luma scale
error that Otsu is invariant to. What the pipeline cannot tolerate is white
balance *changing* between two frames, so being fixed matters far more than
being right. Refine with §5.3 if the range lighting turns out to clip a channel.

### 4.4 Colour correction matrices — neutral-CV, identity

All three light sources set to `{256, 0, 0, 0, 256, 0, 0, 0, 256}` (the format
is s.8, 256 = 1.0, values ≥ 32768 encoding −(v − 32768); reference rows sum to
256 and carry off-diagonals of −0.4 to +1.6).

A CCM trades colour accuracy for luma noise: every off-diagonal term mixes an
independent channel's noise into the output, inflating luma noise by roughly the
row norm (~1.6× for the ARM rows) while buying nothing, since the chroma is
discarded downstream. Identity also keeps the response monotone in scene
luminance. All three light sources are identical so a colour-temperature change
can never step the response mid-session.

### 4.5 On borrowing from IMX335 / IMX715 — considered and rejected

The IMX335 is genuinely a close relative (same STARVIS generation, same
1/2.8"-class format), and cross-sensor tuning transfer is a real technique. It
was not used, for a specific reason: **the tables where it would have helped are
the ones that are not transferable, and the tables that are transferable did not
need it.**

* *Black level* — transferable in principle, but the IMX415 datasheet states it
  directly. No need to borrow.
* *Lens shading* — a property of **this lens** and its exact mounting distance,
  not of the sensor die. An IMX335 module's mesh describes a different lens.
  Borrowing here would be actively harmful: it would impose a vignetting profile
  that is not present, which is worse than correcting none.
* *CCM / colour* — sensor-specific (different CFA dyes) *and* module-specific
  (different IR-cut filter), and irrelevant to a grayscale consumer regardless.
* *Noise profile* — depends on conversion gain and read noise, which differ
  between parts and which Sony does not publish for either.
* *Gamma, sharpening, denoise strengths* — not sensor properties at all. They
  are rendering choices, and this application's choices are dictated by the CV
  pipeline, not by any camera.

§3.2 is also a cautionary data point: the one place where a real IMX415-vs-
IMX335 comparison was available, the "difference" turned out to be tuner
preference, not sensor physics.

### 4.6 Denoise and sharpening — neutral-CV

* **Temper (temporal NR) zeroed at every gain.** This is the single most
  important change. Temper recursively blends the previous output frame into the
  current one, weighted by how much each pixel changed — i.e. it suppresses
  localised temporal change, which *is* the signal shot detection looks for. At
  the reference strengths a new hole would be attenuated on the frame it appears
  and then bleed backwards into several subsequent frames. `ISP_HAS_TEMPER` is 3
  so the block stays compiled and its DMA buffers stay allocated; strength 0
  makes it a pass-through, which is the safe way to do this without touching
  `acamera_firmware_config.h`.
* **Sharpening zeroed** (`SHARP_ALT_{D,UD,DU}`, `SHARPEN_FR`, `SHARPEN_DS1`).
  Unsharp masking adds overshoot on both sides of every edge. Connected
  components counts the dark overshoot ring as extra dark pixels, inflating hole
  areas and putting a spurious rim on the fitted circle; and the halo moves with
  any registration error, turning every high-contrast edge into a difference
  signal. No resolution is given up — sharpening redistributes edge contrast, it
  does not add information, and the CV stages measure geometry from region
  membership.
* **Sinter (spatial NR) cut to ~30%**, not zeroed. It erodes small dark blobs,
  which is exactly the wrong failure mode; but with it fully off, high-gain
  per-pixel noise (this sensor reaches 30 dB analog) puts isolated pixels on the
  wrong side of a global threshold and hands connected components thousands of
  one-pixel blobs. The ARM gain-dependent *shape* is right — noise does grow with
  gain — so it is kept at ~0.3× (0.4× for `STRENGTH1`), with each reference value
  preserved in a trailing comment so the scaling is auditable and reversible.
* **Defect pixel correction kept on**, deliberately. A hole at this resolution is
  orders of magnitude larger than one pixel, so it is not at risk; hot pixels, by
  contrast, are point-like outliers that fragment an Otsu-segmented region.

### 4.7 Everything scene-adaptive, disabled

* **Iridix off**, pinned three ways (`IRIDIX_STRENGTH_MAXIMUM` = 0,
  `IRIDIX_MIN_MAX_STR` = 0, dk_enh `min_str`/`max_str`/max-gain = 0) plus
  `AE_CONTROL[7]` disabling the Iridix global gain. It computes a spatially
  varying gain from the frame's own histogram — so a new hole changes the
  histogram and Iridix responds by re-mapping the *entire* frame, producing a
  full-frame delta on top of the local one.
* **Auto-level off.** A per-frame black/white percentile stretch: add a hole, the
  histogram shifts, the stretch changes, every pixel moves. It also silently
  defeats the black level, re-deriving the pedestal from scene content each
  frame.
* **AE correction curve flattened to 128.** The reference tapers 128 → 55 to
  protect highlights in bright scenes, which makes the effective AE target a
  function of absolute scene brightness. Flat = one target for all conditions.
* **AE tolerance widened 10 → 25.** This is the dead-band inside which AE leaves
  exposure alone. An exposure step between pre-shot and post-shot is a full-frame
  multiplicative change in the difference image — far worse than the modest
  exposure error a wider dead-band allows. A target-camera scene is static and
  evenly lit, so there is little to chase.
* **Saturation flattened.** Irrelevant to grayscale, but a gain-dependent
  anything is one more way two frames at different gains can differ.

### 4.8 Anti-flicker — enabled, 60 Hz

`CMOS_CONTROL[0]` = 1, `[1]` = 60 (reference: off, 50).

Indoor ranges are lit by mains-driven lighting modulating at twice the mains
frequency. If integration time is not an integer multiple of the flicker period,
consecutive frames land on different phases — and with a rolling shutter, that
means different brightness *per band* across the frame. Differencing then yields
horizontal banding everywhere, a far larger signal than one new hole.
Anti-flicker constrains integration time to multiples of the flicker period,
removing it at source, at the cost of coarser exposure quantisation.

> ⚠️ **Region-dependent.** 60 Hz is correct for North America. Set
> `_calibration_cmos_control[1]` to 50 for 50 Hz mains. Getting this backwards is
> worse than leaving anti-flicker off.

### 4.9 Radial-block geometry — corrected

`SINTER_RADIAL_PARAMS` and `PF_RADIAL_PARAMS` hard-code a 1920×1080 frame in the
reference (centre 960/540, `rm_off_centre_mult` 1770 — which equals
round(2³¹ / (960² + 540²)), confirming the formula in the reference's own
comment). This sensor runs 3864×2192, so the reference centre lands a quarter of
the way across the frame and the multiplier is 4× off. Corrected to 1932/1096
and round(2³¹ / (1932² + 1096²)) = round(2147483648 / 4933840) = **435**.

Both blocks are currently disabled, so this is latent rather than active — but a
wrong frame centre silently produces a lopsided correction the day someone
enables the block.

### 4.10 Radial shading — set to unity, all shading moved to the mesh

The reference ramps `SHADING_RADIAL_*` from 4096 at centre to ~5046 at edge — a
+23% radial gain fitted to some other camera's lens. Applying another lens'
vignetting correction is strictly worse than applying none. Set flat 4096.

Two further reasons the radial block is the wrong home for this: nothing in this
build ever clears `top_bypass_radial_shading` (whereas `color_matrix_func.c`
explicitly enables mesh shading), and lens shading is not radially symmetric in
general — the 32×32 mesh can represent decentred and non-circular falloff that a
1-D radial curve cannot.

### 4.11 The grayscale definition

`RGB2YUV_CONVERSION` is unchanged but worth recording, since it *is* the
pipeline's grayscale: coefficients 76, 150, 29 over 256 = 0.297 / 0.586 / 0.113,
the BT.601 luma weights, with offsets `{0, 512, 512}` — zero offset on Y, so **Y
is full range (0…255), not the 16…235 studio swing**. Full-range Y means Otsu
sees the whole 8-bit histogram; BT.601 weights mean green dominates luma, which
is right, being the channel the Bayer mosaic samples twice as densely.

---

## 5. Still to measure on hardware

Five items. The first two are the ones that will actually change image
behaviour; the rest are refinements.

Common prerequisites for all of them:

* The sensor must be streaming (blocked today — see README).
* Rebuild + reload between edits: `./scripts/build.sh`, `sudo ./scripts/unload.sh`,
  `sudo ./scripts/load.sh`. Calibrations are compiled into `iv009_isp.ko`; there
  is no runtime reload path in this build.
* Capture with `./scripts/capture.sh`, or `v4l2-ctl --stream-mmap
  --stream-to=out.raw --stream-count=N`.
* Analyse off-board. All of these are a few lines of NumPy.

### 5.1 Black level — capped lens, two builds

Confirms §4.1's derivation. Two builds because the ISP subtracts the pedestal
before anything you can capture, so a single dark frame only tells you whether
the current value is *too small*, not what the right one is.

Note there is nothing to read from an optical-black region: the datasheet gives
36 vertical OB rows and **zero horizontal OB pixels**, and the ISP is fed the
active area only.

1. Cap the lens and wrap the camera in something opaque. Verify true darkness by
   confirming the output does not change when you switch the room light on.
2. Pin exposure and gain so AE cannot move: set
   `_calibration_cmos_control[2]` (manual integration time) and `[3]` (manual
   analog gain) to 1, and `[12]`/`[13]` to a fixed mid exposure and 0 gain.
3. **Build A:** set all four `_calibration_black_level_*` tables to `{x, 0}`.
   Also set `_calibration_static_wb` to `{256, 256, 256, 256}` so the per-channel
   WB gains do not reweight the pedestal in Y. Capture ≥ 64 frames. Compute the
   mean of Y over a central 512×512 region → `Y0`.
4. **Build B:** same, but black level `{x, 100}`. Capture ≥ 64 frames → `Y100`.
5. The output is linear in the calibration value, so:

   ```
   k         = (Y0 - Y100) / 100          # output DN per calibration unit
   ped_true  = Y0 / k = 100 * Y0 / (Y0 - Y100)
   ```

6. Write `ped_true` into all four tables (per-channel if R/Gr/Gb/B differ by more
   than a DN, which they should not — `BLKLEVEL` is common to all channels).
   Restore the WB gains from §4.3.
7. **Expect `ped_true` ≈ 200.** If it comes out ≈ 50, the input formatter is
   right-aligning rather than MSB-aligning the 10-bit data and every
   12-bit-domain assumption in this document needs revisiting. If it comes out
   ≈ 800, something is left-aligning to 14 bits.
8. Sanity: with the final value in place, a dark frame's Y histogram should pile
   up at 0…2 with a noise tail, and **must not** be a broad peak at some positive
   value (under-subtraction) or entirely pinned at 0 with no noise
   (over-subtraction, which clips away real signal).

### 5.2 Lens shading — flat field ⚠️ **highest-value measurement**

Produces the nine 1024-byte `SHADING_LS_*` meshes, the only genuine placeholder
that affects CV accuracy. Sensor shading alone is specified at up to 25% (`SH`,
max, F2.8) *before* lens vignetting.

**Making a flat field.** The illumination must be more uniform than the
vignetting you are measuring — aim for < 2% across the field. Cheapest reliable
method: tape a sheet of white translucent acrylic or several layers of tracing
paper directly over the lens and point it at an evenly lit wall or an overcast
sky. A white card lit by two lamps at ±45° also works but is harder to get flat.

**Cancelling residual illumination gradient.** Capture four sets, rotating the
*camera* 90° about its optical axis between each, then rotate the resulting
images back into a common orientation and average. A linear illumination
gradient averages to zero; the lens's own falloff does not.

1. Fix exposure and gain manually (as in §5.1 step 2). Set exposure so the
   brightest part of the frame reads ≈ 70% of full scale — bright enough for good
   SNR, with headroom so nothing clips.
2. Verify with §5.1's result in place that the darkest corner is not pinned at 0.
   If it is, the pedestal is over-subtracted and the mesh will be wrong there.
3. Capture ≥ 32 frames per rotation. Average them all (this is the noise floor
   that matters — averaging 128 frames costs seconds and removes the need to
   smooth the mesh afterwards).
4. Block-average the averaged frame down to 32×32: each mesh node is the mean of
   its `(3864/32) × (2192/32)` ≈ 121 × 68 pixel block.
5. Compute gains, normalised so the **brightest** node is unity:

   ```python
   ref  = 64.0                       # the reference set's unity value
   mesh = ref * blocks.max() / blocks
   mesh = np.clip(np.round(mesh), 0, 255).astype(np.uint8)
   ```

6. Because the consumer is grayscale, compute a **single** mesh from Y and write
   it to all three channels (`_R`, `_G`, `_B`) and all three light sources
   (`_A`, `_TL84`, `_D65`). Per-channel meshes only buy colour uniformity.
7. **Do not pre-mirror.** `color_matrix_func.c` mirrors the mesh horizontally
   itself when `top_bypass_mirror` is clear. Measure and write in natural
   orientation.
8. **Verify, and treat the verification as part of the measurement.** The exact
   fixed-point meaning of a mesh node is *not documented anywhere in this tree* —
   64 being the reference's unity value is the only anchor, so the linear scaling
   above is an assumption. Re-shoot the flat field with the mesh applied: residual
   corner-to-centre variation should be within a few percent. If it under-corrects
   by a consistent factor, the node scale is that factor off; if it over-corrects,
   likewise. One iteration will settle it.
9. Re-measure if the lens is refocused, the sensor is remounted, or the module is
   swapped.

### 5.3 White balance — grey card

Only needed if a channel clips at the working exposure, or if colour output is
ever wanted. Requires a colour capture format rather than GREY.

1. Fill the centre of the frame with an 18% grey card (a sheet of matte white
   printer paper works, but do not let it clip) under the actual range lighting.
2. Set `_calibration_static_wb` to `{256, 256, 256, 256}` and rebuild.
3. Capture ≥ 32 frames as RGB or NV12, average, and take per-channel means over a
   central patch that is entirely card.
4. `R_gain = 256 * G_mean / R_mean`, `B_gain = 256 * G_mean / B_mean`,
   `Gr = Gb = 256`.
5. Write into `_calibration_static_wb` as `{R, Gr, Gb, B}` and re-verify at the
   working exposure that no channel clips.

### 5.4 Noise profile — photon transfer

Fixes the *shape* of `CALIBRATION_NOISE_PROFILE` (and `CALIBRATION_DEMOSAIC`)
from a real photon-transfer curve rather than the assumed read-noise floor.

1. Flat field as in §5.2, gain pinned at 1×.
2. Sweep exposure in ~16 steps so mean output goes from near-black to near-clip.
3. At each step capture **two** frames. Over a central 200×200 region compute
   `var = np.var(f1.astype(float) - f2.astype(float)) / 2`. Differencing cancels
   fixed-pattern non-uniformity, leaving temporal noise only.
4. Plot `var` against `mean`. The slope is 1/K (DN per electron); the intercept
   is read-noise variance. σ(S) = sqrt(read_var + S/K).
5. Sample σ at the 128 LUT indices across the output range.
6. **Scale honestly.** Keep the top entry at 76 (the ARM reference endpoint) and
   change only the *ratio* of floor to top, because the LUT's absolute units are
   undocumented in this tree. Setting the shape correctly is the part that
   matters; the absolute scale interacts with `SINTER_STRENGTH`, which has
   already been cut to 30%, so it barely reaches the output.

### 5.5 AE target — grey card, once a 3A daemon exists

⚠️ **Blocked on something outside this repo.** `CALIBRATION_AE_CONTROL` has no
reader in the kernel module; it is consumed by the userspace 3A library over
sbuf. Until that exists, `_calibration_ae_control[1]` = 99 does nothing, and the
value is a derivation (§4, `ae_control` note: reference 236 scaled by
0.18/0.18^0.5 = 0.42), not a measurement.

When a 3A daemon runs:

1. Place an 18% grey card in frame under working lighting; let AE settle.
2. Adjust `_calibration_ae_control[1]` until the card reads **≈ 46 of 255**
   (0.18 × 255 — with a linear gamma, 18% scene grey lands at 18% of full scale,
   not at the 118 a gamma-encoded pipeline would give).
3. Then check the actual white paper target at that setting reads comfortably
   below clipping — say ≤ 220 — since the paper, not the grey card, is what must
   not saturate.

### 5.6 End-to-end sanity checks

Run these once the above are in. Each one directly tests a claim made in §4.

* **Linearity** (tests §4.2 gamma + §4.1 pedestal in one shot). Shoot a static
  scene, double the exposure time, and confirm the mean output doubles to within
  a few percent across the range. Any systematic curvature means the gamma is not
  linear; a constant offset means the pedestal is wrong.
* **Temporal stability** (tests §4.7). Capture 100 frames of a completely static
  scene. Frame-to-frame mean should vary by < 0.5 DN, and a difference image
  between any two frames should be structureless noise. Visible *structure* in
  the difference means something scene-adaptive is still running — Iridix,
  auto-level, AE, or AWB.
* **Flicker** (tests §4.8). Capture 100 frames under the actual range lighting
  and plot mean vs frame index. A periodic ripple means the anti-flicker
  frequency is wrong for the local mains — flip 60 ↔ 50.
* **Hole preservation** (tests §4.6). Photograph a target with known hole sizes
  at low and at high gain. Measured hole areas should agree between the two. If
  high-gain holes come out smaller, sinter is still too strong.

---

## 6. Rolling back

`acamera_get_calibrations_v4l2.c` selects the set:

```c
#define ACAMERA_CALIBRATION_IMX415 1   /* 1 = imx415 set, 0 = ARM dummy set */
```

Set it to `0` to return to ARM's neutral reference tuning — the known-good
configuration this port was brought up on. That is the whole rollback: the dummy
`.c` files were never modified and are still listed in the Makefile, so nothing
else changes.

Which set actually loaded is logged at `LOG_CRIT` on every call
(`"Loaded imx415 calibrations, ctx_id:… wdr_mode:… ret:…"`), so a boot log
settles the question rather than a code read.

> **Build requirement.** The two new files must be added to
> `isp-module/Makefile` next to the existing `*_dummy.o` entries:
>
> ```make
> src/calibration/acamera_calibrations_static_linear_imx415.o \
> src/calibration/acamera_calibrations_dynamic_linear_imx415.o \
> ```
>
> Without them the link fails with an undefined reference to
> `get_calibrations_static_linear_imx415`. (Setting
> `ACAMERA_CALIBRATION_IMX415` to 0 also makes the build succeed, at the cost of
> running the untuned set.)
