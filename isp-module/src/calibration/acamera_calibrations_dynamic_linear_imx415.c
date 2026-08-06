/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#include "acamera_command_api.h"
#include "acamera_firmware_settings.h"

/*
 * IMX415 dynamic (gain/scene-modulated) calibration set for the Radxa Camera 4K
 * on a Radxa Zero 2 Pro. Derived from the ARM/Amlogic neutral reference set
 * (acamera_calibrations_dynamic_linear_dummy.c), retuned for a marksmanship
 * target camera feeding a grayscale computer-vision pipeline.
 *
 * See acamera_calibrations_static_linear_imx415.c for the full statement of what
 * the consumer is and why these choices differ from a photographic tuning. In
 * one line: the downstream pipeline Otsu-thresholds a paper target and
 * frame-differences pre-shot against post-shot frames, so it needs a linear,
 * *unchanging* response far more than a pretty one.
 *
 * The four substantive changes here, in order of how much they matter:
 *   1. TEMPER (temporal denoise) zeroed -- it directly suppresses the signal
 *      frame differencing exists to detect.
 *   2. IRIDIX (local tone mapping) pinned off -- scene-adaptive per-pixel gain.
 *   3. SHARPENING zeroed -- overshoot halos read as edges and as motion.
 *   4. SINTER (spatial denoise) cut to ~30% -- it erodes small dark blobs.
 * Plus: anti-flicker enabled, auto-level disabled, AE target rescaled for the
 * linear gamma and its dead-band widened, and the radial-block frame geometry
 * corrected from 1920x1080 to this sensor's 3864x2192.
 *
 * Most gain-modulated tables are {x, y} pairs where x is log2(gain) * 256, so
 * row N corresponds to 2^N total gain (noise_reduction_func.c). The black-level
 * tables in the static file use a different axis -- log2(analog gain) * 32.
 *
 * EVERY TABLE CARRIES A "SOURCE:" COMMENT. Where a value was scaled from the ARM
 * reference, the reference value is kept in a trailing comment so the change is
 * auditable. Tables that still need hardware measurement are called out in
 * docs/calibration-imx415.md.
 *
 * NOTE ON WHAT IS ACTUALLY LIVE TODAY: the AE, AWB, Iridix and auto-level tables
 * are not read by the kernel firmware at all. This build compiles the "manual"
 * variants of those FSMs (ISP_HAS_AE_MANUAL_FSM, ISP_HAS_AWB_MANUAL_FSM,
 * ISP_HAS_IRIDIX8_MANUAL_FSM), meaning the algorithms live in a userspace 3A
 * library that receives the whole calibration set over the sbuf channel
 * (sbuf_func.c ships all CALIBRATION_TOTAL_SIZE entries). That library is not
 * part of this port. Until it exists, 3A does not adapt and those tables are
 * inert -- which is stated here so nobody spends an afternoon wondering why
 * changing the AE target had no effect.
 */
// ------------ 3A & iridix
/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Iridix time-filtering and EV limits. Inert -- Iridix strength is pinned to 0
 * (see CALIBRATION_IRIDIX_STRENGTH_MAXIMUM above).
 */
static uint8_t _calibration_evtolux_probability_enable[] = {1};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * AWB tuning. Left at the reference deliberately rather than retuned: white
 * balance in this build is pinned by CALIBRATION_STATIC_WB and an identity CCM,
 * so the adaptive AWB path should not be moving anything. Retuning tables that
 * are meant to be inert would only obscure that intent.
 *
 * (awb_avg_coef is AWB's temporal smoothing coefficient, and slowing AWB down
 * would help stability -- but the sign convention is not documented in this tree
 * and guessing wrong would speed it up. Left alone.)
 *
 * ccm_one_gain_threshold forces the CCM to identity above the given gain, to stop
 * a CCM amplifying noise in dark scenes. Already moot: the CCM is identity at
 * every gain.
 */
static uint8_t _calibration_awb_avg_coef[] = {7};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Iridix time-filtering and EV limits. Inert -- Iridix strength is pinned to 0
 * (see CALIBRATION_IRIDIX_STRENGTH_MAXIMUM above).
 */
static uint8_t _calibration_iridix_avg_coef[] = {30};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * AWB tuning. Left at the reference deliberately rather than retuned: white
 * balance in this build is pinned by CALIBRATION_STATIC_WB and an identity CCM,
 * so the adaptive AWB path should not be moving anything. Retuning tables that
 * are meant to be inert would only obscure that intent.
 *
 * (awb_avg_coef is AWB's temporal smoothing coefficient, and slowing AWB down
 * would help stability -- but the sign convention is not documented in this tree
 * and guessing wrong would speed it up. Left alone.)
 *
 * ccm_one_gain_threshold forces the CCM to identity above the given gain, to stop
 * a CCM amplifying noise in dark scenes. Already moot: the CCM is identity at
 * every gain.
 */
static uint16_t _calibration_ccm_one_gain_threshold[] = {1408};

/* SOURCE: principled neutral choice for this CV application -- IRIDIX OFF.
 * Iridix is ARM's local tone-mapping / dynamic-range-compression block. It
 * computes a spatially varying gain from the frame's own histogram, so the same
 * scene radiance maps to different output DN depending on where in the frame it
 * is and on what else is in the frame at that instant.
 *
 * Every one of this application's assumptions breaks under that. A global Otsu
 * threshold assumes one radiance-to-DN mapping across the frame. Frame
 * differencing assumes that mapping is the same in both frames -- but a new
 * bullet hole changes the histogram, so Iridix would respond to the shot by
 * re-mapping the *entire* frame, producing a full-frame delta on top of the
 * local one.
 *
 * Pinned off three ways so no single path can re-enable it:
 *   - CALIBRATION_IRIDIX_STRENGTH_MAXIMUM = 0 (the hard cap; ARM ref 255)
 *   - CALIBRATION_IRIDIX_MIN_MAX_STR = 0
 *   - dk_enh_control min_str / max_str / max-gain = 0 (ARM ref 0 / 40 / 20)
 * and CALIBRATION_AE_CONTROL disables the Iridix global gain as well.
 */
static uint8_t _calibration_iridix_strength_maximum[] = {0}; /* ARM ref 255 */

/* SOURCE: principled neutral choice for this CV application -- IRIDIX OFF.
 * Iridix is ARM's local tone-mapping / dynamic-range-compression block. It
 * computes a spatially varying gain from the frame's own histogram, so the same
 * scene radiance maps to different output DN depending on where in the frame it
 * is and on what else is in the frame at that instant.
 *
 * Every one of this application's assumptions breaks under that. A global Otsu
 * threshold assumes one radiance-to-DN mapping across the frame. Frame
 * differencing assumes that mapping is the same in both frames -- but a new
 * bullet hole changes the histogram, so Iridix would respond to the shot by
 * re-mapping the *entire* frame, producing a full-frame delta on top of the
 * local one.
 *
 * Pinned off three ways so no single path can re-enable it:
 *   - CALIBRATION_IRIDIX_STRENGTH_MAXIMUM = 0 (the hard cap; ARM ref 255)
 *   - CALIBRATION_IRIDIX_MIN_MAX_STR = 0
 *   - dk_enh_control min_str / max_str / max-gain = 0 (ARM ref 0 / 40 / 20)
 * and CALIBRATION_AE_CONTROL disables the Iridix global gain as well.
 */
static uint16_t _calibration_iridix_min_max_str[] = {0};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Iridix time-filtering and EV limits. Inert -- Iridix strength is pinned to 0
 * (see CALIBRATION_IRIDIX_STRENGTH_MAXIMUM above).
 */
static uint32_t _calibration_iridix_ev_lim_full_str[] = {2557570};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Iridix time-filtering and EV limits. Inert -- Iridix strength is pinned to 0
 * (see CALIBRATION_IRIDIX_STRENGTH_MAXIMUM above).
 */
static uint32_t _calibration_iridix_ev_lim_no_str[] = {3750000, 3574729}; //3574729

/* SOURCE: principled neutral choice for this CV application -- flattened to 128.
 * This is a per-exposure-bracket correction applied to the AE target, indexed by
 * CALIBRATION_AE_EXPOSURE_CORRECTION. The ARM reference tapers it 128 -> 55,
 * deliberately under-exposing bright scenes to protect highlights.
 *
 * That taper makes the effective AE target a function of absolute scene
 * brightness. Turn the range lights up and the target moves; the whole image
 * scales; the pre/post frame difference picks up the scale change. Flat 128
 * (= no correction) keeps one target for all conditions, which is what a
 * frame-differencing consumer needs.
 */
static uint8_t _calibration_ae_correction[] = {128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128}; /* ARM ref: 128 128 128 128 128 108 98 88 78 78 78 55 */

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * The exposure axis for CALIBRATION_AE_CORRECTION. Harmless now that the
 * correction curve above is flat -- every bucket maps to 128.
 */
static uint32_t _calibration_ae_exposure_correction[] = {6710, 15739, 15778, 23282, 56186, 500325, 632161, 1190074, 1406400, 2382765, 3295034, 5491142}; //500,157778,500325,632161,1406400,6046465 //23282 - Max Lab Exposure

// ------------Noise reduction ----------------------//
/* SOURCE: ARM reference shape, scaled down for this CV application.
 * x is log2(total gain) * 256, so row N is 2^N gain (noise_reduction_func.c:
 * log2_gain = total_gain >> (LOG2_GAIN_SHIFT - 8)).
 *
 * Sinter is the spatial denoiser. Its cost here is specific: it is an edge-aware
 * smoother, and a bullet hole in paper is a *small* dark blob -- exactly the
 * feature class a spatial denoiser treats as noise and erodes. Losing a few
 * pixels off the rim of every hole shifts the connected-component centroid and
 * shrinks the area, which is the measurement being taken.
 *
 * Not set to zero, though, and that is deliberate. With sinter fully off, the
 * per-pixel noise at high analog gain (this sensor goes to 30 dB) puts isolated
 * pixels on the wrong side of a global Otsu threshold, which fragments the paper
 * region into speckle and gives connected-components thousands of one-pixel
 * blobs to sift. A little smoothing is cheaper than that cleanup.
 *
 * The compromise is ARM's gain-dependent shape (which is the right shape -- noise
 * does grow with gain) at roughly 30% of its strength, and 40% for strength1.
 * The per-row ARM reference values are kept in trailing comments so the scaling
 * is auditable and reversible. If holes come out undersized, drop these further;
 * if the threshold speckles, raise them.
 */
static uint16_t _calibration_sinter_strength[][2] = {
    {0 * 256, 11},  /* ARM ref 35 */
    {1 * 256, 13},  /* ARM ref 43 */
    {2 * 256, 16},  /* ARM ref 53 */
    {3 * 256, 20},  /* ARM ref 65 */
    {4 * 256, 20},  /* ARM ref 65 */
    {5 * 256, 21},  /* ARM ref 70 */
    {6 * 256, 23},  /* ARM ref 78 */
    {7 * 256, 25}}; /* ARM ref 82 */
// ------------Noise reduction ----------------------//
/* SOURCE: ARM reference (carried verbatim from the dummy set), value 0.
 * Modulates sinter strength by Iridix's local contrast output. Inert here
 * because Iridix strength is pinned to 0.
 */
static uint16_t _calibration_sinter_strength_MC_contrast[][2] = {
    {0 * 256, 0}};

/* SOURCE: ARM reference shape, scaled down for this CV application.
 * x is log2(total gain) * 256, so row N is 2^N gain (noise_reduction_func.c:
 * log2_gain = total_gain >> (LOG2_GAIN_SHIFT - 8)).
 *
 * Sinter is the spatial denoiser. Its cost here is specific: it is an edge-aware
 * smoother, and a bullet hole in paper is a *small* dark blob -- exactly the
 * feature class a spatial denoiser treats as noise and erodes. Losing a few
 * pixels off the rim of every hole shifts the connected-component centroid and
 * shrinks the area, which is the measurement being taken.
 *
 * Not set to zero, though, and that is deliberate. With sinter fully off, the
 * per-pixel noise at high analog gain (this sensor goes to 30 dB) puts isolated
 * pixels on the wrong side of a global Otsu threshold, which fragments the paper
 * region into speckle and gives connected-components thousands of one-pixel
 * blobs to sift. A little smoothing is cheaper than that cleanup.
 *
 * The compromise is ARM's gain-dependent shape (which is the right shape -- noise
 * does grow with gain) at roughly 30% of its strength, and 40% for strength1.
 * The per-row ARM reference values are kept in trailing comments so the scaling
 * is auditable and reversible. If holes come out undersized, drop these further;
 * if the threshold speckles, raise them.
 */
static uint16_t _calibration_sinter_strength1[][2] = {
    {0 * 256, 62},  /* ARM ref 155 */
    {1 * 256, 62},  /* ARM ref 155 */
    {2 * 256, 50},  /* ARM ref 125 */
    {3 * 256, 46},  /* ARM ref 115 */
    {4 * 256, 46},  /* ARM ref 115 */
    {5 * 256, 46},  /* ARM ref 115 */
    {6 * 256, 34},  /* ARM ref  85 */
    {7 * 256, 34}}; /* ARM ref  85 */ //255 4 int

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * These are sinter's internal thresholds and filter configuration rather than
 * an overall strength; the strength reduction above is applied through
 * CALIBRATION_SINTER_STRENGTH / _STRENGTH1, which is the documented knob. The
 * reference values are already small (thresh1 tops out at 5).
 */
static uint16_t _calibration_sinter_thresh1[][2] = {
    {0 * 256, 0},
    {1 * 256, 0},
    {2 * 256, 2},
    {3 * 256, 3},
    {4 * 256, 4},
    {5 * 256, 4},
    {6 * 256, 5}};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * These are sinter's internal thresholds and filter configuration rather than
 * an overall strength; the strength reduction above is applied through
 * CALIBRATION_SINTER_STRENGTH / _STRENGTH1, which is the documented knob. The
 * reference values are already small (thresh1 tops out at 5).
 */
static uint16_t _calibration_sinter_thresh4[][2] = {
    {0 * 256, 0},
    {1 * 256, 0},
    {2 * 256, 0},
    {3 * 256, 5},
    {4 * 256, 64},
    {5 * 256, 64},
    {6 * 256, 128}};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * These are sinter's internal thresholds and filter configuration rather than
 * an overall strength; the strength reduction above is applied through
 * CALIBRATION_SINTER_STRENGTH / _STRENGTH1, which is the documented knob. The
 * reference values are already small (thresh1 tops out at 5).
 */
static uint16_t _calibration_sinter_intConfig[][2] = {
    {0 * 256, 10},
    {1 * 256, 10},
    {2 * 256, 8},
    {3 * 256, 8},
    {4 * 256, 7},
    {5 * 256, 5},
    {6 * 256, 4}};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Inert: rm_enable is 0 in CALIBRATION_SINTER_RADIAL_PARAMS below.
 */
static uint8_t _calibration_sinter_radial_lut[] = {0, 0, 0, 0, 0, 0, 1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 18, 19, 21, 22, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24};

/* SOURCE: corrected geometry, derived from this camera's actual mode.
 * The ARM reference hard-codes a 1920x1080 frame (rm_centre 960/540, and
 * rm_off_centre_mult 1770 = round(2^31 / (960^2 + 540^2)), which confirms the
 * formula in the reference's own comment). This sensor runs 3864x2192
 * (IMX415_WIDTH/IMX415_HEIGHT in V4L2_drv.c), so the reference centre lands at
 * roughly a quarter of the way across the frame and the multiplier is off by 4x.
 *
 * Both blocks are currently disabled (sinter radial rm_enable = 0; purple-fringe
 * radial LUT is flat), so this is latent rather than active -- but a wrong frame
 * centre is the kind of thing that silently produces a lopsided correction the
 * day somebody enables the block, so it is fixed now.
 */
static uint16_t _calibration_sinter_radial_params[] = {
    0,        // rm_enable -- left off; radial NR modulation is lens-specific
    3864 / 2, // rm_centre_x  (ARM ref had 1920/2 -- wrong sensor geometry)
    2192 / 2, // rm_centre_y  (ARM ref had 1080/2 -- wrong sensor geometry)
    435       // rm_off_centre_mult: round((2^31)/((rm_centre_x^2)+(rm_centre_y^2)))
              //   = round(2147483648 / (1932^2 + 1096^2)) = round(2147483648/4933840)
};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * These are sinter's internal thresholds and filter configuration rather than
 * an overall strength; the strength reduction above is applied through
 * CALIBRATION_SINTER_STRENGTH / _STRENGTH1, which is the documented knob. The
 * reference values are already small (thresh1 tops out at 5).
 */
static uint16_t _calibration_sinter_sad[][2] = {
    {0, 8},
    {1 * 256, 8},
    {2 * 256, 5},
    {3 * 256, 5},
    {4 * 256, 9},
    {5 * 256, 11},
    {6 * 256, 13}};
// ------------ Sharpening and demosaic
/* SOURCE: principled neutral choice for this CV application -- sharpening OFF.
 * All strengths zeroed across the whole gain range. The ARM reference values are
 * in the trailing comment on each table.
 *
 * Unsharp-mask sharpening works by adding a scaled high-pass of the image back
 * onto itself. At a step edge that produces overshoot on both sides -- a bright
 * halo outside and a dark halo inside. For a human that reads as "crisp". For
 * this pipeline it is poison twice over:
 *
 *   - Otsu + connected components sees the dark overshoot ring around the edge
 *     of the paper target, and around each hole, as additional dark pixels. Hole
 *     areas inflate and the target's fitted circle gains a spurious rim.
 *   - Frame differencing sees the halo move whenever anything moves, so a small
 *     registration error between the pre-shot and post-shot frame turns every
 *     high-contrast edge in the scene into a difference signal.
 *
 * There is no resolution being given up. Sharpening does not add information; it
 * only redistributes contrast at edges, and the CV stages measure geometry from
 * region membership, not from apparent edge acutance.
 */
static uint16_t _calibration_sharp_alt_d[][2] = {
    {0 * 256, 0},
    {1 * 256, 0},
    {2 * 256, 0},
    {3 * 256, 0},
    {4 * 256, 0},
    {5 * 256, 0},
    {6 * 256, 0},
    {7 * 256, 0}}; /* ARM ref: 28 28 26 26 29 15 5 0 */

/* SOURCE: principled neutral choice for this CV application -- sharpening OFF.
 * All strengths zeroed across the whole gain range. The ARM reference values are
 * in the trailing comment on each table.
 *
 * Unsharp-mask sharpening works by adding a scaled high-pass of the image back
 * onto itself. At a step edge that produces overshoot on both sides -- a bright
 * halo outside and a dark halo inside. For a human that reads as "crisp". For
 * this pipeline it is poison twice over:
 *
 *   - Otsu + connected components sees the dark overshoot ring around the edge
 *     of the paper target, and around each hole, as additional dark pixels. Hole
 *     areas inflate and the target's fitted circle gains a spurious rim.
 *   - Frame differencing sees the halo move whenever anything moves, so a small
 *     registration error between the pre-shot and post-shot frame turns every
 *     high-contrast edge in the scene into a difference signal.
 *
 * There is no resolution being given up. Sharpening does not add information; it
 * only redistributes contrast at edges, and the CV stages measure geometry from
 * region membership, not from apparent edge acutance.
 */
static uint16_t _calibration_sharp_alt_ud[][2] = {
    {0 * 256, 0},
    {1 * 256, 0},
    {2 * 256, 0},
    {3 * 256, 0},
    {4 * 256, 0},
    {5 * 256, 0},
    {6 * 256, 0},
    {7 * 256, 0}}; /* ARM ref: 40 25 10 8 8 2 0 0 */


/* SOURCE: principled neutral choice for this CV application -- sharpening OFF.
 * All strengths zeroed across the whole gain range. The ARM reference values are
 * in the trailing comment on each table.
 *
 * Unsharp-mask sharpening works by adding a scaled high-pass of the image back
 * onto itself. At a step edge that produces overshoot on both sides -- a bright
 * halo outside and a dark halo inside. For a human that reads as "crisp". For
 * this pipeline it is poison twice over:
 *
 *   - Otsu + connected components sees the dark overshoot ring around the edge
 *     of the paper target, and around each hole, as additional dark pixels. Hole
 *     areas inflate and the target's fitted circle gains a spurious rim.
 *   - Frame differencing sees the halo move whenever anything moves, so a small
 *     registration error between the pre-shot and post-shot frame turns every
 *     high-contrast edge in the scene into a difference signal.
 *
 * There is no resolution being given up. Sharpening does not add information; it
 * only redistributes contrast at edges, and the CV stages measure geometry from
 * region membership, not from apparent edge acutance.
 */
static uint16_t _calibration_sharp_alt_du[][2] = {
    {0 * 256, 0},
    {1 * 256, 0},
    {2 * 256, 0},
    {3 * 256, 0},
    {4 * 256, 0},
    {5 * 256, 0},
    {6 * 256, 0},
    {7 * 256, 0}}; /* ARM ref: 70 70 65 55 29 17 5 0 */

/* SOURCE: principled neutral choice for this CV application -- sharpening OFF.
 * All strengths zeroed across the whole gain range. The ARM reference values are
 * in the trailing comment on each table.
 *
 * Unsharp-mask sharpening works by adding a scaled high-pass of the image back
 * onto itself. At a step edge that produces overshoot on both sides -- a bright
 * halo outside and a dark halo inside. For a human that reads as "crisp". For
 * this pipeline it is poison twice over:
 *
 *   - Otsu + connected components sees the dark overshoot ring around the edge
 *     of the paper target, and around each hole, as additional dark pixels. Hole
 *     areas inflate and the target's fitted circle gains a spurious rim.
 *   - Frame differencing sees the halo move whenever anything moves, so a small
 *     registration error between the pre-shot and post-shot frame turns every
 *     high-contrast edge in the scene into a difference signal.
 *
 * There is no resolution being given up. Sharpening does not add information; it
 * only redistributes contrast at edges, and the CV stages measure geometry from
 * region membership, not from apparent edge acutance.
 */
static uint16_t _calibration_sharpen_fr[][2] = {
    {0 * 256, 0},
    {1 * 256, 0},
    {2 * 256, 0},
    {3 * 256, 0},
    {4 * 256, 0},
    {5 * 256, 0},
    {6 * 256, 0}}; /* ARM ref: 42 27 20 10 8 2 1 */

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Offsets the demosaic noise-profile lookup with gain. Reference values are
 * already low (1..18) and demosaic quality reaches luma only weakly.
 */
static uint16_t _calibration_demosaic_np_offset[][2] = {
    {0 * 256, 1},
    {1 * 256, 1},
    {2 * 256, 1},
    {3 * 256, 3},
    {4 * 256, 18},
    {5 * 256, 18},
    {6 * 256, 15}};


/* SOURCE: ARM reference (carried verbatim from the dummy set), value 4096.
 * Full-strength mesh shading correction, applied uniformly across gain. Correct
 * to leave at full: once CALIBRATION_SHADING_LS_* holds a measured flat-field
 * mesh, we want all of it -- a partial correction leaves a residual gradient
 * under the global threshold. Until then the mesh is unity and this is moot.
 */
static uint16_t _calibration_mesh_shading_strength[][2] = {
    {0 * 256, 4096}};

/* SOURCE: principled neutral choice for this CV application -- flat.
 * The ARM reference tapers saturation 128 -> 90 with gain, desaturating noisy
 * high-gain frames. Saturation scales chroma, which the grayscale consumer
 * discards, so the taper buys nothing -- but a gain-dependent anything is one
 * more way for two frames shot at different gains to differ. Flat 128 (unity).
 */
static uint16_t _calibration_saturation_strength[][2] = {
    {0 * 256, 128},
    {1 * 256, 128},
    {2 * 256, 128},
    {3 * 256, 128},
    {4 * 256, 128},
    {5 * 256, 128},
    {6 * 256, 128},
    {7 * 256, 128}}; /* ARM ref tapered 128..90; flattened -- see note */

// ----------- Frame stitching motion
/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * WDR frame-stitching. This build is linear-only, so none of it runs.
 */
static uint16_t _calibration_stitching_lm_np[][2] = {
    {0, 540},
    {3 * 256, 1458},
    {4 * 256, 1458},
    {5 * 256, 3000}};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * WDR frame-stitching. This build is linear-only, so none of it runs.
 */
static uint16_t _calibration_stitching_lm_mov_mult[][2] = {
    {0, 128},
    {2 * 256 - 128, 20},
    {5 * 256, 8},
};
/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * WDR frame-stitching. This build is linear-only, so none of it runs.
 */
static uint16_t _calibration_stitching_lm_med_noise_intensity_thresh[][2] = {
    {0, 32},
    {6 * 256, 32},
    {8 * 256, 4095},
};
/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * WDR frame-stitching. This build is linear-only, so none of it runs.
 */
static uint16_t _calibration_stitching_ms_np[][2] = {
    {0, 3680},
    {1 * 256, 3680},
    {2 * 256, 2680}};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * WDR frame-stitching. This build is linear-only, so none of it runs.
 */
static uint16_t _calibration_stitching_ms_mov_mult[][2] = {
    {0, 128},
    {1 * 256, 128},
    {2 * 256, 100}};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Defect-pixel correction, kept ON. Worth a note because "minimal processing"
 * might suggest disabling it: DPC replaces single pixels that differ wildly from
 * their same-colour neighbours. A bullet hole at this resolution is orders of
 * magnitude larger than one pixel, so holes are not at risk. Hot pixels, by
 * contrast, are point-like dark/bright outliers that fragment an Otsu-segmented
 * region and give connected-components junk to filter. The reference curve is
 * already conservative at low gain (threshold 4095 = effectively off at unity
 * gain) and tightens as gain rises, which is the right behaviour.
 */
static uint16_t _calibration_dp_slope[][2] = {
    {0 * 256, 170},
    {1 * 256, 170},
    {2 * 256, 170},
    {3 * 256, 1800},
    {4 * 256, 1911},
    {5 * 256, 2200},
    {6 * 256, 2400},
};


/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Defect-pixel correction, kept ON. Worth a note because "minimal processing"
 * might suggest disabling it: DPC replaces single pixels that differ wildly from
 * their same-colour neighbours. A bullet hole at this resolution is orders of
 * magnitude larger than one pixel, so holes are not at risk. Hot pixels, by
 * contrast, are point-like dark/bright outliers that fragment an Otsu-segmented
 * region and give connected-components junk to filter. The reference curve is
 * already conservative at low gain (threshold 4095 = effectively off at unity
 * gain) and tightens as gain rises, which is the right behaviour.
 */
static uint16_t _calibration_dp_threshold[][2] = {
    {0 * 256, 4095},
    {1 * 256, 312},
    {2 * 256, 302},
    {3 * 256, 110},
    {4 * 256, 95},
    {5 * 256, 85},
    {6 * 256, 70},
};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * AWB tuning. Left at the reference deliberately rather than retuned: white
 * balance in this build is pinned by CALIBRATION_STATIC_WB and an identity CCM,
 * so the adaptive AWB path should not be moving anything. Retuning tables that
 * are meant to be inert would only obscure that intent.
 *
 * (awb_avg_coef is AWB's temporal smoothing coefficient, and slowing AWB down
 * would help stability -- but the sign convention is not documented in this tree
 * and guessing wrong would speed it up. Left alone.)
 *
 * ccm_one_gain_threshold forces the CCM to identity above the given gain, to stop
 * a CCM amplifying noise in dark scenes. Already moot: the CCM is identity at
 * every gain.
 */
static uint16_t _calibration_AWB_bg_max_gain[][2] = {
    {0 * 256, 100},
    {1 * 256, 100},
    {7 * 256, 200},
};

/* SOURCE: mixed -- reference values, with anti-flicker deliberately enabled.
 *
 * [0]/[1] Anti-flicker: turned ON at 60 Hz (ARM ref: off, 50 Hz). This is a
 * CV-motivated change, not a photographic one. Indoor shooting ranges are lit by
 * mains-driven lighting, which modulates at twice the mains frequency. If the
 * exposure time is not an integer multiple of the flicker period, consecutive
 * frames land on different phases of the light and come out at different
 * brightness -- with a rolling shutter, different brightness *per band* across
 * the frame. Differencing a pre-shot against a post-shot frame then yields
 * horizontal banding everywhere, which is a far larger signal than one new hole.
 *
 * Anti-flicker constrains integration time to multiples of the flicker period,
 * which removes the effect at source. The cost is coarser exposure quantisation,
 * which matters much less here than frame-to-frame stability.
 *
 * *** REGION-DEPENDENT: 60 Hz is correct for North America. Set field [1] to 50
 * for 50 Hz mains. Getting this backwards is worse than leaving it off. ***
 *
 * [8] max sensor AG 150: in log2(gain)*32 units that is 2^(150/32) = 25.6x
 * = 28.2 dB, just inside the IMX415's 30 dB analog gain range (datasheet: 0 to
 * 30 dB analog in 0.3 dB steps; beyond that it is digital gain). The bridge caps
 * again_log2_max at 30 dB (IMX415_AGAIN_MAX_DB), so this is consistent.
 * Remaining fields are the ARM reference.
 */
static uint32_t _calibration_cmos_control[] = {
    1,   // enable antiflicker -- ENABLED (ARM ref 0). See note above.
    60,  // antiflicker frequency [Hz] -- 60 for North American mains.
         // *** SET THIS TO 50 FOR 50 Hz MAINS (EU/UK/AU/most of Asia). ***
    0,   // manual integration time
    0,   // manual sensor analog gain
    0,   // manual sensor digital gain
    0,   // manual isp digital gain
    0,   // manual max integration time
    0,   // max integration time
    150, // max sensor AG
    0,   // max sensor DG
    112, // max isp DG
    255, // max exposure ratio
    0,   // integration time.
    0,   // sensor analog gain. log2 fixed - 5 bits
    0,   // sensor digital gain. log2 fixed - 5 bits
    0,   // isp digital gain. log2 fixed - 5 bits
    0,   // analog_gain_last_priority
    4    // analog_gain_reserve
};


/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Status reporting slots, not tuning.
 */
static uint32_t _calibration_status_info[] = {
    0xFFFFFFFF, // sys.total_gain_log2
    0xFFFFFFFF, // sys.expsoure_log2
    0xFFFFFFFF, // awb.mix_light_contrast
    0xFFFFFFFF, // af.cur_lens_pos
    0xFFFFFFFF  // af.cur_focus_value
};

/* SOURCE: principled neutral choice for this CV application -- IRIDIX OFF.
 * Iridix is ARM's local tone-mapping / dynamic-range-compression block. It
 * computes a spatially varying gain from the frame's own histogram, so the same
 * scene radiance maps to different output DN depending on where in the frame it
 * is and on what else is in the frame at that instant.
 *
 * Every one of this application's assumptions breaks under that. A global Otsu
 * threshold assumes one radiance-to-DN mapping across the frame. Frame
 * differencing assumes that mapping is the same in both frames -- but a new
 * bullet hole changes the histogram, so Iridix would respond to the shot by
 * re-mapping the *entire* frame, producing a full-frame delta on top of the
 * local one.
 *
 * Pinned off three ways so no single path can re-enable it:
 *   - CALIBRATION_IRIDIX_STRENGTH_MAXIMUM = 0 (the hard cap; ARM ref 255)
 *   - CALIBRATION_IRIDIX_MIN_MAX_STR = 0
 *   - dk_enh_control min_str / max_str / max-gain = 0 (ARM ref 0 / 40 / 20)
 * and CALIBRATION_AE_CONTROL disables the Iridix global gain as well.
 */
static uint32_t _calibration_iridix8_strength_dk_enh_control[] = {
    20,      // dark_prc
    95,      // bright_prc
    800,     // min_dk: minimum dark enhancement
    2000,    // max_dk: maximum dark enhancement
    8,       // pD_cut_min
    20,      // pD_cut_max
    30 << 8, // dark contrast min
    50 << 8, // dark contrast max
    0,       // min_str: iridix strength in percentage
    0,       // max_str: FORCED TO 0 (ARM ref 40). See the note above.
    40,      // dark_prc_gain_target
    30 << 8, // contrast_min
    40 << 8, // contrast_max
    0,       // max iridix gain: FORCED TO 0 (ARM ref 20)
    0        // print debug
};

/* SOURCE: mixed -- see per-field notes. Two fields are genuinely derived, one is
 * a stability choice, and the AE target is the weakest number in this file.
 *
 * [1] LDR AE target. The ARM reference is 236, and its own comment says the value
 *     "should match the 18% grey of the output gamma" -- i.e. it is expressed in
 *     the *gamma-encoded* output domain, so it has to move when the gamma does.
 *     Under the reference's ~1/2.0 curve, 18% scene grey emerges at about
 *     0.18^(1/2.0) = 0.42 of full scale. Under this file's exact linear gamma it
 *     emerges at 0.18. The target therefore scales by 0.18/0.42 = 0.42:
 *         236 * 0.42 = 99
 *
 *     *** PLACEHOLDER -- THIS IS A DERIVATION, NOT A MEASUREMENT. *** It assumes
 *     units are proportional to output level, which the reference comment
 *     implies but this build cannot confirm -- CALIBRATION_AE_CONTROL is not read
 *     anywhere in the kernel firmware. It is consumed by the userspace 3A
 *     library over the sbuf calibration channel (sbuf_func.c ships all
 *     CALIBRATION_TOTAL_SIZE entries), and that library is not part of this port.
 *     Until a 3A daemon runs, this field does nothing at all. Verify it against a
 *     grey card when one does: docs/calibration-imx415.md ("AE target").
 *
 *     If this is left at 236 with a linear gamma, AE will drive the sensor until
 *     the paper target clips -- and a clipped target has no texture for Otsu.
 *
 * [7] Iridix global gain: disabled (ARM ref 1), consistent with Iridix being
 *     pinned off above.
 *
 * [8] AE tolerance: widened from 10 to 25. This is the dead-band inside which AE
 *     leaves exposure alone. Every exposure step between the pre-shot and the
 *     post-shot frame is a full-frame multiplicative change in the difference
 *     image, which is far more damaging than the modest exposure error a wider
 *     dead-band allows. A target-camera scene is static and evenly lit, so there
 *     is little for AE to chase anyway.
 *
 * [0] AE convergence left at the reference 30: the field's sign convention is not
 *     documented in this tree and guessing wrong would make AE *faster*, which is
 *     the opposite of what is wanted.
 */
static uint32_t _calibration_ae_control[] = {
    30,  // AE convergance (ARM ref 30, unchanged -- see note)
    99,  // LDR AE target. DERIVED from the linear-gamma change; see note above.
    0,   // AE tail weight
    0,   // WDR only: max percentage of clipped pixels for long exposure
    0,   // WDR only: time filter for exposure ratio
    100, // control for clipping: bright percentage below hi_target_prc
    99,  // control for clipping: highlights percentage (hi_target_prc)
    0,   // 1:0 enable | disable iridix global gain -- DISABLED (ARM ref 1)
    25,  // AE tolerance. WIDENED from 10; see note above.
};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * WDR/HDR AE targets. This build is linear-only, so unused.
 */
static uint16_t _calibration_ae_control_HDR_target[][2] = {
    {0 * 256, 139}, // HDR AE target should not be higher than LDR target
    {1 * 256, 139},
    {2 * 256, 139},
    {3 * 256, 187},
    {4 * 256, 236},
    {5 * 256, 236},
    {6 * 256, 236},
    {7 * 256, 236},
};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Purple-fringe radial LUT, flat 255. Chroma-only.
 */
static uint8_t _calibration_pf_radial_lut[] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};

/* SOURCE: corrected geometry, derived from this camera's actual mode.
 * The ARM reference hard-codes a 1920x1080 frame (rm_centre 960/540, and
 * rm_off_centre_mult 1770 = round(2^31 / (960^2 + 540^2)), which confirms the
 * formula in the reference's own comment). This sensor runs 3864x2192
 * (IMX415_WIDTH/IMX415_HEIGHT in V4L2_drv.c), so the reference centre lands at
 * roughly a quarter of the way across the frame and the multiplier is off by 4x.
 *
 * Both blocks are currently disabled (sinter radial rm_enable = 0; purple-fringe
 * radial LUT is flat), so this is latent rather than active -- but a wrong frame
 * centre is the kind of thing that silently produces a lopsided correction the
 * day somebody enables the block, so it is fixed now.
 */
static uint16_t _calibration_pf_radial_params[] = {
    3864 / 2, // rm_centre_x  (ARM ref had 1920/2 -- wrong sensor geometry)
    2192 / 2, // rm_centre_y  (ARM ref had 1080/2 -- wrong sensor geometry)
    435       // rm_off_centre_mult, same derivation as sinter_radial_params
};

/* SOURCE: principled neutral choice for this CV application -- auto-level OFF.
 * Auto-level is a per-frame histogram stretch: it finds the black and white
 * percentiles of the current frame and remaps them to the output endpoints. That
 * is a scene-adaptive, frame-varying affine transform on intensity, which is the
 * one thing frame differencing cannot survive -- add a hole, the histogram
 * shifts, the stretch changes, and every pixel in the frame moves.
 *
 * It also silently defeats the carefully-set black level: the pedestal would be
 * re-derived from scene content every frame instead of being the sensor's actual
 * dark floor.
 *
 * Note for whoever reads this next: CALIBRATION_AUTO_LEVEL_CONTROL is not
 * referenced anywhere in this build's kernel firmware -- like the AE tables it is
 * shipped to the userspace 3A library over sbuf. So today this flag does nothing;
 * it is set correctly so it stays correct when that library appears.
 */
static uint32_t _calibration_auto_level_control[] = {
    1,  // black_percentage
    99, // white_percentage
    0,  // auto_black_min
    50, // auto_black_max
    75, // auto_white_prc
    15, // avg_coeff
    0   // enable_auto_level -- DISABLED (ARM ref 1). See note above.
};


/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * WDR frame-stitching. This build is linear-only, so none of it runs.
 */
static uint16_t _calibration_exposure_ratio_adjustment[][2] = {
    //contrast, adjustment
    {1 * 256, 256},
    {16 * 256, 256},
    {32 * 256, 256},
    {64 * 256, 256}};


/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Chroma noise reduction. Operates on U/V only; invisible to a grayscale
 * consumer. Left alone rather than zeroed, since it costs nothing.
 */
static uint16_t _calibration_cnr_uv_delta12_slope[][2] = {
    {0 * 256, 1500}, //3800
    {1 * 256, 2000},
    {2 * 256, 2100},
    {3 * 256, 2100},
    {4 * 256, 3500},
    {5 * 256, 4100},
    {6 * 256, 5900},
    {7 * 256, 6100},
};


/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * WDR frame-stitching. This build is linear-only, so none of it runs.
 */
static uint16_t _calibration_fs_mc_off[] = {
    8 * 256, // gain_log2 threshold. if gain is higher than the current gain_log2. mc off mode will be enabed.
};
/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * AWB tuning. Left at the reference deliberately rather than retuned: white
 * balance in this build is pinned by CALIBRATION_STATIC_WB and an identity CCM,
 * so the adaptive AWB path should not be moving anything. Retuning tables that
 * are meant to be inert would only obscure that intent.
 *
 * (awb_avg_coef is AWB's temporal smoothing coefficient, and slowing AWB down
 * would help stability -- but the sign convention is not documented in this tree
 * and guessing wrong would speed it up. Left alone.)
 *
 * ccm_one_gain_threshold forces the CCM to identity above the given gain, to stop
 * a CCM amplifying noise in dark scenes. Already moot: the CCM is identity at
 * every gain.
 */
static int16_t _AWB_colour_preference[] = {7500, 6000, 4700, 2800};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * AWB tuning. Left at the reference deliberately rather than retuned: white
 * balance in this build is pinned by CALIBRATION_STATIC_WB and an identity CCM,
 * so the adaptive AWB path should not be moving anything. Retuning tables that
 * are meant to be inert would only obscure that intent.
 *
 * (awb_avg_coef is AWB's temporal smoothing coefficient, and slowing AWB down
 * would help stability -- but the sign convention is not documented in this tree
 * and guessing wrong would speed it up. Left alone.)
 *
 * ccm_one_gain_threshold forces the CCM to identity above the given gain, to stop
 * a CCM amplifying noise in dark scenes. Already moot: the CCM is identity at
 * every gain.
 */
static uint32_t _calibration_awb_mix_light_parameters[] = {
    1,    // 1 = enable, 0 = disable
    500,  //lux low boundary for mix light lux range : range = {500: inf}
    3000, // lux high boundary for mix light range : range = {500: inf}
    2000, // contrast threshold for mix light: range = {200:2000}
    330,  //BG threshold {255:400}
    5,    // BG weight
    260,  // rgHigh_LUT_max
    252,  // rgHigh_LUT_min
    0     // print debug
};

/* SOURCE: ARM reference (carried verbatim from the dummy set) -- and this is the
 * table that literally defines the pipeline's grayscale.
 *
 * The first three coefficients are 76, 150, 29 over 256 = 0.297, 0.586, 0.113:
 * the BT.601 luma weights (0.299 / 0.587 / 0.114). The trailing offsets are
 * {0, 512, 512}, i.e. zero offset on Y and mid-scale on the two chroma channels,
 * so Y is FULL RANGE (0..255), not the 16..235 studio swing.
 *
 * Both facts matter downstream: full-range Y means Otsu sees the whole 8-bit
 * histogram, and BT.601 weights mean green dominates luma -- which is the right
 * choice, being the channel the Bayer mosaic samples twice as densely.
 * Unchanged, but documented rather than left implicit.
 */
static uint16_t _calibration_rgb2yuv_conversion[] = {76, 150, 29, 0x8025, 0x8049, 111, 157, 0x8083, 0x8019, 0, 512, 512};


/* SOURCE: ARM reference (carried verbatim from the dummy set) -- uniform weights.
 * Uniform 3A metering is the right choice here and worth stating: a paper target
 * typically fills most of the frame, and centre-weighted metering would make the
 * exposure depend on how the camera happens to be aimed. Uniform weighting makes
 * exposure a function of the scene, not of framing.
 */
static uint16_t _calibration_ae_zone_wght_hor[] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
/* SOURCE: ARM reference (carried verbatim from the dummy set) -- uniform weights.
 * Uniform 3A metering is the right choice here and worth stating: a paper target
 * typically fills most of the frame, and centre-weighted metering would make the
 * exposure depend on how the camera happens to be aimed. Uniform weighting makes
 * exposure a function of the scene, not of framing.
 */
static uint16_t _calibration_ae_zone_wght_ver[] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};

/* SOURCE: ARM reference (carried verbatim from the dummy set) -- uniform weights.
 * Uniform 3A metering is the right choice here and worth stating: a paper target
 * typically fills most of the frame, and centre-weighted metering would make the
 * exposure depend on how the camera happens to be aimed. Uniform weighting makes
 * exposure a function of the scene, not of framing.
 */
static uint16_t _calibration_awb_zone_wght_hor[] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
/* SOURCE: ARM reference (carried verbatim from the dummy set) -- uniform weights.
 * Uniform 3A metering is the right choice here and worth stating: a paper target
 * typically fills most of the frame, and centre-weighted metering would make the
 * exposure depend on how the camera happens to be aimed. Uniform weighting makes
 * exposure a function of the scene, not of framing.
 */
static uint16_t _calibration_awb_zone_wght_ver[] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Polyphase resampling coefficients for the DS1/DS2 scalers. Generic filter
 * design, entirely independent of the sensor -- there is nothing to tune.
 */
static uint32_t _scaler_h_filter[] = {
    0x27f70200, 0x0002f727, 0x29f70200, 0x0002f824, 0x2cf70200, 0x0002f821, 0x2ef70200, 0x0002f91e, 0x30f70200, 0x0002f91c, 0x33f70200, 0x0001fa19, 0x35f70200, 0x0001fa17, 0x37f70200, 0x0001fb14, 0x39f70200, 0x0001fc11, 0x3af80200, 0x0001fc0f, 0x3bf90200, 0x0001fd0c, 0x3cfa0200, 0x0001fd0a, 0x3efb0100, 0x0000fe08, 0x3efc0100, 0x0000ff06, 0x3ffd0100, 0x0000ff04, 0x40fe0000, 0x00000002, 0x40000000, 0x00000000, 0x40020000, 0x000000fe, 0x3f04ff00, 0x000001fd, 0x3e06ff00, 0x000001fc, 0x3e08fe00, 0x000001fb, 0x3c0afd01, 0x000002fa, 0x3b0cfd01, 0x000002f9, 0x3a0ffc01, 0x000002f8, 0x3911fc01, 0x000002f7, 0x3714fb01, 0x000002f7, 0x3517fa01, 0x000002f7, 0x3319fa01, 0x000002f7, 0x301cf902, 0x000002f7, 0x2e1ef902, 0x000002f7, 0x2c21f802, 0x000002f7, 0x2924f802, 0x000002f7,
    0x25fbfd05, 0x00fdfb26, 0x27fcfc05, 0x00fefa24, 0x28fdfc05, 0x00fef923, 0x29fefb05, 0x00fff921, 0x29fffb06, 0x00fff820, 0x2a00fa06, 0x0000f81e, 0x2b01fa06, 0x0000f71d, 0x2b02f906, 0x0001f71c, 0x2c03f906, 0x0001f71a, 0x2d04f806, 0x0002f619, 0x2d06f806, 0x0002f617, 0x2d07f806, 0x0003f615, 0x2f08f705, 0x0003f614, 0x2f09f705, 0x0004f612, 0x2f0bf705, 0x0004f610, 0x2f0df605, 0x0004f60f, 0x2f0df605, 0x0005f60e, 0x2f0ff604, 0x0005f60d, 0x2f10f604, 0x0005f70b, 0x2f12f604, 0x0005f709, 0x2f14f603, 0x0005f708, 0x2d15f603, 0x0006f807, 0x2d17f602, 0x0006f806, 0x2d19f602, 0x0006f804, 0x2c1af701, 0x0006f903, 0x2b1cf701, 0x0006f902, 0x2b1df700, 0x0006fa01, 0x2a1ef800, 0x0006fa00, 0x2920f8ff, 0x0006fbff, 0x2921f9ff, 0x0005fbfe, 0x2823f9fe, 0x0005fcfd, 0x2724fafe, 0x0005fcfc,
    0x1e0afafc, 0x00fa0a1e, 0x1e0bfafc, 0x00fa091e, 0x1e0cfbfb, 0x00fa091d, 0x1f0cfbfb, 0x00fa081d, 0x200dfbfb, 0x00f9071d, 0x200efbfb, 0x00f9071c, 0x200efcfb, 0x00f9061c, 0x210ffcfa, 0x00f9051c, 0x2110fcfa, 0x00f9051b, 0x2111fdfa, 0x00f9041a, 0x2211fdfa, 0x00f9031a, 0x2212fefa, 0x00f90318, 0x2213fefa, 0x00f90218, 0x2213fff9, 0x00f90218, 0x2215fff9, 0x00f90117, 0x2215fff9, 0x00f90117, 0x221600f9, 0x00f90016, 0x221701f9, 0x00f9ff15, 0x221701f9, 0x00f9ff15, 0x221802f9, 0x00f9ff13, 0x221802f9, 0x00fafe13, 0x221803f9, 0x00fafe12, 0x221a03f9, 0x00fafd11, 0x211a04f9, 0x00fafd11, 0x211b05f9, 0x00fafc10, 0x211c05f9, 0x00fafc0f, 0x201c06f9, 0x00fbfc0e, 0x201c07f9, 0x00fbfb0e, 0x201d07f9, 0x00fbfb0d, 0x1f1d08fa, 0x00fbfb0c, 0x1e1d09fa, 0x00fbfb0c, 0x1e1e09fa, 0x00fcfa0b,
    0x0e0b0602, 0x00060b0e, 0x0e0b0702, 0x00060b0d, 0x0e0b0702, 0x00060b0d, 0x0e0b0702, 0x00060b0d, 0x0e0c0702, 0x00060a0d, 0x0e0c0702, 0x00060a0d, 0x0e0c0703, 0x00050a0d, 0x0e0c0703, 0x00050a0d, 0x0e0c0703, 0x00050a0d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0804, 0x0004090d, 0x0e0c0804, 0x0004090d, 0x0e0c0904, 0x0004090c, 0x0e0c0904, 0x0004090c, 0x0e0c0904, 0x0004090c, 0x0e0d0904, 0x0004080c, 0x0e0d0904, 0x0004080c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0a05, 0x0003070c, 0x0e0d0a05, 0x0003070c, 0x0e0d0a05, 0x0003070c, 0x0e0d0a06, 0x0002070c, 0x0e0d0a06, 0x0002070c, 0x0e0d0b06, 0x0002070b, 0x0e0d0b06, 0x0002070b, 0x0e0d0b06, 0x0002070b};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Polyphase resampling coefficients for the DS1/DS2 scalers. Generic filter
 * design, entirely independent of the sensor -- there is nothing to tune.
 */
static uint32_t _scaler_v_filter[] = {
    0x00400000, 0x00000000, 0x0240fe00, 0x00000000, 0x0340fd01, 0x000000ff, 0x053ffc01, 0x000000ff, 0x073ffb01, 0x000000fe, 0x093efa01, 0x000000fe, 0x0c3cf901, 0x000000fe, 0x0e3bf901, 0x000000fd, 0x1138f801, 0x000001fd, 0x1337f801, 0x000001fc, 0x1635f801, 0x000001fb, 0x1932f801, 0x000001fb, 0x1b31f801, 0x000001fa, 0x1e2ef801, 0x000001fa, 0x212cf801, 0x000001f9, 0x2429f801, 0x000001f9, 0x2626f901, 0x000001f9, 0x2924f901, 0x000001f8, 0x2c21f901, 0x000001f8, 0x2e1efa01, 0x000001f8, 0x311bfa01, 0x000001f8, 0x3219fb01, 0x000001f8, 0x3516fb01, 0x000001f8, 0x3713fc01, 0x000001f8, 0x3811fd01, 0x000001f8, 0x3b0efd00, 0x000001f9, 0x3c0cfe00, 0x000001f9, 0x3e09fe00, 0x000001fa, 0x3f07fe00, 0x000001fb, 0x3f05ff00, 0x000001fc, 0x4003ff00, 0x000001fd, 0x40020000, 0x000000fe,
    0x2526fbfd, 0x0005fdfb, 0x2724fafe, 0x0005fcfc, 0x2823f9fe, 0x0005fcfd, 0x2921f9ff, 0x0005fbfe, 0x2920f8ff, 0x0006fbff, 0x2a1ef800, 0x0006fa00, 0x2b1df700, 0x0006fa01, 0x2b1cf701, 0x0006f902, 0x2c1af701, 0x0006f903, 0x2d19f602, 0x0006f804, 0x2d17f602, 0x0006f806, 0x2d15f603, 0x0006f807, 0x2f14f603, 0x0005f708, 0x2f12f604, 0x0005f709, 0x2f10f604, 0x0005f70b, 0x2f0ff604, 0x0005f60d, 0x2f0ef605, 0x0005f60d, 0x2f0df605, 0x0004f60f, 0x2f0bf705, 0x0004f610, 0x2f09f705, 0x0004f612, 0x2f08f705, 0x0003f614, 0x2d07f806, 0x0003f615, 0x2d06f806, 0x0002f617, 0x2d04f806, 0x0002f619, 0x2c03f906, 0x0001f71a, 0x2b02f906, 0x0001f71c, 0x2b01fa06, 0x0000f71d, 0x2a00fa06, 0x0000f81e, 0x29fffb06, 0x00fff820, 0x29fefb05, 0x00fff921, 0x28fdfc05, 0x00fef923, 0x27fcfc05, 0x00fefa24,
    0x1e1e0afa, 0x00fcfa0a, 0x1e1e09fa, 0x00fcfa0b, 0x1e1d09fa, 0x00fbfb0c, 0x1f1d08fa, 0x00fbfb0c, 0x201d07f9, 0x00fbfb0d, 0x201c07f9, 0x00fbfb0e, 0x201c06f9, 0x00fbfc0e, 0x211c05f9, 0x00fafc0f, 0x211b05f9, 0x00fafc10, 0x211a04f9, 0x00fafd11, 0x221a03f9, 0x00fafd11, 0x221803f9, 0x00fafe12, 0x221802f9, 0x00fafe13, 0x221802f9, 0x00f9ff13, 0x221701f9, 0x00f9ff15, 0x221701f9, 0x00f9ff15, 0x221600f9, 0x00f90016, 0x2215fff9, 0x00f90117, 0x2215fff9, 0x00f90117, 0x2213fff9, 0x00f90218, 0x2213fefa, 0x00f90218, 0x2212fefa, 0x00f90318, 0x2211fdfa, 0x00f9031a, 0x2111fdfa, 0x00f9041a, 0x2110fcfa, 0x00f9051b, 0x210ffcfa, 0x00f9051c, 0x200efcfb, 0x00f9061c, 0x200efbfb, 0x00f9071c, 0x200dfbfb, 0x00f9071d, 0x1f0cfbfb, 0x00fa081d, 0x1e0cfbfb, 0x00fa091d, 0x1e0bfafc, 0x00fa091e,
    0x0e0e0b06, 0x0002060b, 0x0e0d0b06, 0x0002070b, 0x0e0d0b06, 0x0002070b, 0x0e0d0b06, 0x0002070b, 0x0e0d0a06, 0x0002070c, 0x0e0d0a06, 0x0002070c, 0x0e0d0a05, 0x0003070c, 0x0e0d0a05, 0x0003070c, 0x0e0d0a05, 0x0003070c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0905, 0x0003080c, 0x0e0d0904, 0x0004080c, 0x0e0d0904, 0x0004080c, 0x0e0c0904, 0x0004090c, 0x0e0c0904, 0x0004090c, 0x0e0c0904, 0x0004090c, 0x0e0c0804, 0x0004090d, 0x0e0c0804, 0x0004090d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0803, 0x0005090d, 0x0e0c0703, 0x00050a0d, 0x0e0c0703, 0x00050a0d, 0x0e0c0703, 0x00050a0d, 0x0e0c0702, 0x00060a0d, 0x0e0c0702, 0x00060a0d, 0x0e0b0702, 0x00060b0d, 0x0e0b0702, 0x00060b0d, 0x0e0b0702, 0x00060b0d};

/* SOURCE: principled neutral choice for this CV application -- sharpening OFF.
 * All strengths zeroed across the whole gain range. The ARM reference values are
 * in the trailing comment on each table.
 *
 * Unsharp-mask sharpening works by adding a scaled high-pass of the image back
 * onto itself. At a step edge that produces overshoot on both sides -- a bright
 * halo outside and a dark halo inside. For a human that reads as "crisp". For
 * this pipeline it is poison twice over:
 *
 *   - Otsu + connected components sees the dark overshoot ring around the edge
 *     of the paper target, and around each hole, as additional dark pixels. Hole
 *     areas inflate and the target's fitted circle gains a spurious rim.
 *   - Frame differencing sees the halo move whenever anything moves, so a small
 *     registration error between the pre-shot and post-shot frame turns every
 *     high-contrast edge in the scene into a difference signal.
 *
 * There is no resolution being given up. Sharpening does not add information; it
 * only redistributes contrast at edges, and the CV stages measure geometry from
 * region membership, not from apparent edge acutance.
 */
static uint16_t _calibration_sharpen_ds1[][2] = {
    {0 * 256, 0},
    {1 * 256, 0},
    {2 * 256, 0},
    {3 * 256, 0},
    {4 * 256, 0},
    {5 * 256, 0},
    {6 * 256, 0},
    {7 * 256, 0},
    {8 * 256, 0}}; /* ARM ref: 70 70 70 70 70 50 40 25 10 */
/* SOURCE: principled neutral choice for this CV application -- TEMPER OFF.
 * *** THIS IS THE MOST IMPORTANT SINGLE CHANGE IN THIS FILE. ***
 *
 * Temper is the temporal denoiser: it recursively blends the previous output
 * frame into the current one, weighted by how much each pixel changed. That is a
 * direct, head-on conflict with the application. The whole shot-detection method
 * is "difference a pre-shot frame against a post-shot frame and find the new
 * dark blob" -- i.e. the signal being detected is precisely a localised temporal
 * change, which is precisely what temper is designed to suppress.
 *
 * With the ARM reference strengths (120 at unity gain, 195 at high gain), a
 * newly-appeared hole would be attenuated on the frame it appears, and would
 * then bleed backwards into several subsequent frames as the recursion settles,
 * smearing both the detection instant and the hole's shape.
 *
 * Zeroed across all gains. ISP_HAS_TEMPER is 3 so the block is still compiled and
 * its DMA buffers are still allocated (sensor_func.c wires them up); strength 0
 * makes it a pass-through rather than removing it, which is the safe way to do
 * this without touching the firmware config.
 *
 * The noise this leaves behind is handled spatially by sinter, and -- much more
 * effectively -- by the fact that frame differencing over a static paper target
 * can average several frames on each side of the shot.
 */
static uint16_t _calibration_temper_strength[][2] = {
    {0 * 256, 0},
    {1 * 256, 0},
    {2 * 256, 0},
    {3 * 256, 0},
    {4 * 256, 0},
    {5 * 256, 0},
    {6 * 256, 0},
    {7 * 256, 0}}; /* ARM ref: 120 120 120 120 140 145 195 195 */

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Autofocus lens-servo tuning. The Radxa Camera 4K has a fixed lens and no focus
 * motor, and no lens driver is built into this module, so AF never actuates.
 */
static uint32_t _calibration_af_lms[] = {
    70 << 6,  // Down_FarEnd
    70 << 6,  // Hor_FarEnd
    70 << 6,  // Up_FarEnd
    112 << 6, // Down_Infinity
    112 << 6, // Hor_Infinity
    112 << 6, // Up_Infinity
    // 50<<6, // Down_FarEnd
    // 50<<6, // Hor_FarEnd
    // 50<<6, // Up_FarEnd
    // 167<<6, // Down_Infinity
    // 167<<6, // Hor_Infinity
    // 167<<6, // Up_Infinity
    832 << 6,                      // Down_Macro
    832 << 6,                      // Hor_Macro
    832 << 6,                      // Up_Macro
    915 << 6,                      // Down_NearEnd
    915 << 6,                      // Hor_NearEnd
    915 << 6,                      // Up_NearEnd
    11,                            // step_num
    6,                             // skip_frames_init
    2,                             // skip_frames_move
    30,                            // dynamic_range_th
    2 << ( LOG2_GAIN_SHIFT - 2 ),  // spot_tolerance
    1 << ( LOG2_GAIN_SHIFT - 1 ),  // exit_th
    16 << ( LOG2_GAIN_SHIFT - 4 ), // caf_trigger_th
    4 << ( LOG2_GAIN_SHIFT - 4 ),  // caf_stable_th
    0,                             // print_debug
};

/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Autofocus lens-servo tuning. The Radxa Camera 4K has a fixed lens and no focus
 * motor, and no lens driver is built into this module, so AF never actuates.
 */
static uint16_t _calibration_af_zone_wght_hor[] = {0, 0, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0};
/* SOURCE: ARM reference (carried verbatim from the dummy set).
 * Autofocus lens-servo tuning. The Radxa Camera 4K has a fixed lens and no focus
 * motor, and no lens driver is built into this module, so AF never actuates.
 */
static uint16_t _calibration_af_zone_wght_ver[] = {0, 0, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0};

static LookupTable calibration_fs_mc_off = {.ptr = _calibration_fs_mc_off, .rows = 1, .cols = sizeof( _calibration_fs_mc_off ) / sizeof( _calibration_fs_mc_off[0] ), .width = sizeof( _calibration_fs_mc_off[0] )};
static LookupTable calibration_exposure_ratio_adjustment = {.ptr = _calibration_exposure_ratio_adjustment, .rows = sizeof( _calibration_exposure_ratio_adjustment ) / sizeof( _calibration_exposure_ratio_adjustment[0] ), .cols = 2, .width = sizeof( _calibration_exposure_ratio_adjustment[0][0] )};
static LookupTable AWB_colour_preference = {.ptr = _AWB_colour_preference, .rows = 1, .cols = sizeof( _AWB_colour_preference ) / sizeof( _AWB_colour_preference[0] ), .width = sizeof( _AWB_colour_preference[0] )};
static LookupTable calibration_awb_mix_light_parameters = {.ptr = _calibration_awb_mix_light_parameters, .rows = 1, .cols = sizeof( _calibration_awb_mix_light_parameters ) / sizeof( _calibration_awb_mix_light_parameters[0] ), .width = sizeof( _calibration_awb_mix_light_parameters[0] )};
static LookupTable calibration_sinter_strength_MC_contrast = {.ptr = _calibration_sinter_strength_MC_contrast, .rows = sizeof( _calibration_sinter_strength_MC_contrast ) / sizeof( _calibration_sinter_strength_MC_contrast[0] ), .cols = 2, .width = sizeof( _calibration_sinter_strength_MC_contrast[0][0] )};
static LookupTable calibration_pf_radial_lut = {.ptr = _calibration_pf_radial_lut, .rows = 1, .cols = sizeof( _calibration_pf_radial_lut ) / sizeof( _calibration_pf_radial_lut[0] ), .width = sizeof( _calibration_pf_radial_lut[0] )};
static LookupTable calibration_pf_radial_params = {.ptr = _calibration_pf_radial_params, .rows = 1, .cols = sizeof( _calibration_pf_radial_params ) / sizeof( _calibration_pf_radial_params[0] ), .width = sizeof( _calibration_pf_radial_params[0] )};
static LookupTable calibration_sinter_radial_lut = {.ptr = _calibration_sinter_radial_lut, .rows = 1, .cols = sizeof( _calibration_sinter_radial_lut ) / sizeof( _calibration_sinter_radial_lut[0] ), .width = sizeof( _calibration_sinter_radial_lut[0] )};
static LookupTable calibration_sinter_radial_params = {.ptr = _calibration_sinter_radial_params, .rows = 1, .cols = sizeof( _calibration_sinter_radial_params ) / sizeof( _calibration_sinter_radial_params[0] ), .width = sizeof( _calibration_sinter_radial_params[0] )};
static LookupTable calibration_AWB_bg_max_gain = {.ptr = _calibration_AWB_bg_max_gain, .rows = sizeof( _calibration_AWB_bg_max_gain ) / sizeof( _calibration_AWB_bg_max_gain[0] ), .cols = 2, .width = sizeof( _calibration_AWB_bg_max_gain[0][0] )};
static LookupTable calibration_iridix8_strength_dk_enh_control = {.ptr = _calibration_iridix8_strength_dk_enh_control, .rows = 1, .cols = sizeof( _calibration_iridix8_strength_dk_enh_control ) / sizeof( _calibration_iridix8_strength_dk_enh_control[0] ), .width = sizeof( _calibration_iridix8_strength_dk_enh_control[0] )};
static LookupTable calibration_auto_level_control = {.ptr = _calibration_auto_level_control, .rows = 1, .cols = sizeof( _calibration_auto_level_control ) / sizeof( _calibration_auto_level_control[0] ), .width = sizeof( _calibration_auto_level_control[0] )};
static LookupTable calibration_dp_threshold = {.ptr = _calibration_dp_threshold, .rows = sizeof( _calibration_dp_threshold ) / sizeof( _calibration_dp_threshold[0] ), .cols = 2, .width = sizeof( _calibration_dp_threshold[0][0] )};
static LookupTable calibration_stitching_lm_np = {.ptr = _calibration_stitching_lm_np, .rows = sizeof( _calibration_stitching_lm_np ) / sizeof( _calibration_stitching_lm_np[0] ), .cols = 2, .width = sizeof( _calibration_stitching_lm_np[0][0] )};
static LookupTable calibration_stitching_lm_med_noise_intensity_thresh = {.ptr = _calibration_stitching_lm_med_noise_intensity_thresh, .rows = sizeof( _calibration_stitching_lm_med_noise_intensity_thresh ) / sizeof( _calibration_stitching_lm_med_noise_intensity_thresh[0] ), .cols = 2, .width = sizeof( _calibration_stitching_lm_med_noise_intensity_thresh[0][0] )};
static LookupTable calibration_stitching_lm_mov_mult = {.ptr = _calibration_stitching_lm_mov_mult, .rows = sizeof( _calibration_stitching_lm_mov_mult ) / sizeof( _calibration_stitching_lm_mov_mult[0] ), .cols = 2, .width = sizeof( _calibration_stitching_lm_mov_mult[0][0] )};
static LookupTable calibration_stitching_ms_np = {.ptr = _calibration_stitching_ms_np, .rows = sizeof( _calibration_stitching_ms_np ) / sizeof( _calibration_stitching_ms_np[0] ), .cols = 2, .width = sizeof( _calibration_stitching_ms_np[0][0] )};
static LookupTable calibration_stitching_ms_mov_mult = {.ptr = _calibration_stitching_ms_mov_mult, .rows = sizeof( _calibration_stitching_ms_mov_mult ) / sizeof( _calibration_stitching_ms_mov_mult[0] ), .cols = 2, .width = sizeof( _calibration_stitching_ms_mov_mult[0][0] )};
static LookupTable calibration_evtolux_probability_enable = {.ptr = _calibration_evtolux_probability_enable, .rows = 1, .cols = sizeof( _calibration_evtolux_probability_enable ) / sizeof( _calibration_evtolux_probability_enable[0] ), .width = sizeof( _calibration_evtolux_probability_enable[0] )};
static LookupTable calibration_awb_avg_coef = {.ptr = _calibration_awb_avg_coef, .rows = 1, .cols = sizeof( _calibration_awb_avg_coef ) / sizeof( _calibration_awb_avg_coef[0] ), .width = sizeof( _calibration_awb_avg_coef[0] )};
static LookupTable calibration_iridix_avg_coef = {.ptr = _calibration_iridix_avg_coef, .rows = 1, .cols = sizeof( _calibration_iridix_avg_coef ) / sizeof( _calibration_iridix_avg_coef[0] ), .width = sizeof( _calibration_iridix_avg_coef[0] )};
static LookupTable calibration_iridix_strength_maximum = {.ptr = _calibration_iridix_strength_maximum, .rows = 1, .cols = sizeof( _calibration_iridix_strength_maximum ) / sizeof( _calibration_iridix_strength_maximum[0] ), .width = sizeof( _calibration_iridix_strength_maximum[0] )};
static LookupTable calibration_iridix_min_max_str = {.ptr = _calibration_iridix_min_max_str, .rows = 1, .cols = sizeof( _calibration_iridix_min_max_str ) / sizeof( _calibration_iridix_min_max_str[0] ), .width = sizeof( _calibration_iridix_min_max_str[0] )};
static LookupTable calibration_iridix_ev_lim_full_str = {.ptr = _calibration_iridix_ev_lim_full_str, .rows = 1, .cols = sizeof( _calibration_iridix_ev_lim_full_str ) / sizeof( _calibration_iridix_ev_lim_full_str[0] ), .width = sizeof( _calibration_iridix_ev_lim_full_str[0] )};
static LookupTable calibration_iridix_ev_lim_no_str = {.ptr = _calibration_iridix_ev_lim_no_str, .rows = 1, .cols = sizeof( _calibration_iridix_ev_lim_no_str ) / sizeof( _calibration_iridix_ev_lim_no_str[0] ), .width = sizeof( _calibration_iridix_ev_lim_no_str[0] )};
static LookupTable calibration_ae_correction = {.ptr = _calibration_ae_correction, .rows = 1, .cols = sizeof( _calibration_ae_correction ) / sizeof( _calibration_ae_correction[0] ), .width = sizeof( _calibration_ae_correction[0] )};
static LookupTable calibration_ae_exposure_correction = {.ptr = _calibration_ae_exposure_correction, .rows = 1, .cols = sizeof( _calibration_ae_exposure_correction ) / sizeof( _calibration_ae_exposure_correction[0] ), .width = sizeof( _calibration_ae_exposure_correction[0] )};
static LookupTable calibration_sinter_strength = {.ptr = _calibration_sinter_strength, .rows = sizeof( _calibration_sinter_strength ) / sizeof( _calibration_sinter_strength[0] ), .cols = 2, .width = sizeof( _calibration_sinter_strength[0][0] )};
static LookupTable calibration_sinter_strength1 = {.ptr = _calibration_sinter_strength1, .rows = sizeof( _calibration_sinter_strength1 ) / sizeof( _calibration_sinter_strength1[0] ), .cols = 2, .width = sizeof( _calibration_sinter_strength1[0][0] )};
static LookupTable calibration_sinter_thresh1 = {.ptr = _calibration_sinter_thresh1, .rows = sizeof( _calibration_sinter_thresh1 ) / sizeof( _calibration_sinter_thresh1[0] ), .cols = 2, .width = sizeof( _calibration_sinter_thresh1[0][0] )};
static LookupTable calibration_sinter_thresh4 = {.ptr = _calibration_sinter_thresh4, .rows = sizeof( _calibration_sinter_thresh4 ) / sizeof( _calibration_sinter_thresh4[0] ), .cols = 2, .width = sizeof( _calibration_sinter_thresh4[0][0] )};
static LookupTable calibration_sinter_intConfig = {.ptr = _calibration_sinter_intConfig, .rows = sizeof( _calibration_sinter_intConfig ) / sizeof( _calibration_sinter_intConfig[0] ), .cols = 2, .width = sizeof( _calibration_sinter_intConfig[0][0] )};
static LookupTable calibration_sharp_alt_d = {.ptr = _calibration_sharp_alt_d, .rows = sizeof( _calibration_sharp_alt_d ) / sizeof( _calibration_sharp_alt_d[0] ), .cols = 2, .width = sizeof( _calibration_sharp_alt_d[0][0] )};
static LookupTable calibration_sharp_alt_ud = {.ptr = _calibration_sharp_alt_ud, .rows = sizeof( _calibration_sharp_alt_ud ) / sizeof( _calibration_sharp_alt_ud[0] ), .cols = 2, .width = sizeof( _calibration_sharp_alt_ud[0][0] )};
static LookupTable calibration_sharp_alt_du = {.ptr = _calibration_sharp_alt_du, .rows = sizeof( _calibration_sharp_alt_du ) / sizeof( _calibration_sharp_alt_du[0] ), .cols = 2, .width = sizeof( _calibration_sharp_alt_du[0][0] )};
static LookupTable calibration_sharpen_fr = {.ptr = _calibration_sharpen_fr, .rows = sizeof( _calibration_sharpen_fr ) / sizeof( _calibration_sharpen_fr[0] ), .cols = 2, .width = sizeof( _calibration_sharpen_fr[0][0] )};
static LookupTable calibration_demosaic_np_offset = {.ptr = _calibration_demosaic_np_offset, .rows = sizeof( _calibration_demosaic_np_offset ) / sizeof( _calibration_demosaic_np_offset[0] ), .cols = 2, .width = sizeof( _calibration_demosaic_np_offset[0][0] )};
static LookupTable calibration_mesh_shading_strength = {.ptr = _calibration_mesh_shading_strength, .rows = sizeof( _calibration_mesh_shading_strength ) / sizeof( _calibration_mesh_shading_strength[0] ), .cols = 2, .width = sizeof( _calibration_mesh_shading_strength[0][0] )};
static LookupTable calibration_saturation_strength = {.ptr = _calibration_saturation_strength, .rows = sizeof( _calibration_saturation_strength ) / sizeof( _calibration_saturation_strength[0] ), .cols = 2, .width = sizeof( _calibration_saturation_strength[0][0] )};
static LookupTable calibration_ccm_one_gain_threshold = {.ptr = _calibration_ccm_one_gain_threshold, .cols = sizeof( _calibration_ccm_one_gain_threshold ) / sizeof( _calibration_ccm_one_gain_threshold[0] ), .rows = 1, .width = sizeof( _calibration_ccm_one_gain_threshold[0] )};
static LookupTable calibration_cmos_control = {.ptr = _calibration_cmos_control, .rows = 1, .cols = sizeof( _calibration_cmos_control ) / sizeof( _calibration_cmos_control[0] ), .width = sizeof( _calibration_cmos_control[0] )};
static LookupTable calibration_status_info = {.ptr = _calibration_status_info, .rows = 1, .cols = sizeof( _calibration_status_info ) / sizeof( _calibration_status_info[0] ), .width = sizeof( _calibration_status_info[0] )};
static LookupTable calibration_ae_control = {.ptr = _calibration_ae_control, .rows = 1, .cols = sizeof( _calibration_ae_control ) / sizeof( _calibration_ae_control[0] ), .width = sizeof( _calibration_ae_control[0] )};
static LookupTable calibration_ae_control_HDR_target = {.ptr = _calibration_ae_control_HDR_target, .rows = sizeof( _calibration_ae_control_HDR_target ) / sizeof( _calibration_ae_control_HDR_target[0] ), .cols = 2, .width = sizeof( _calibration_ae_control_HDR_target[0][0] )};
static LookupTable calibration_rgb2yuv_conversion = {.ptr = _calibration_rgb2yuv_conversion, .rows = 1, .cols = sizeof( _calibration_rgb2yuv_conversion ) / sizeof( _calibration_rgb2yuv_conversion[0] ), .width = sizeof( _calibration_rgb2yuv_conversion[0] )};
static LookupTable calibration_calibration_af_lms = {.ptr = _calibration_af_lms, .rows = 1, .cols = sizeof( _calibration_af_lms ) / sizeof( _calibration_af_lms[0] ), .width = sizeof( _calibration_af_lms[0] )};
static LookupTable calibration_calibration_af_zone_wght_hor = {.ptr = _calibration_af_zone_wght_hor, .rows = 1, .cols = sizeof( _calibration_af_zone_wght_hor ) / sizeof( _calibration_af_zone_wght_hor[0] ), .width = sizeof( _calibration_af_zone_wght_hor[0] )};
static LookupTable calibration_calibration_af_zone_wght_ver = {.ptr = _calibration_af_zone_wght_ver, .rows = 1, .cols = sizeof( _calibration_af_zone_wght_ver ) / sizeof( _calibration_af_zone_wght_ver[0] ), .width = sizeof( _calibration_af_zone_wght_ver[0] )};
static LookupTable calibration_calibration_ae_zone_wght_hor = {.ptr = _calibration_ae_zone_wght_hor, .rows = 1, .cols = sizeof( _calibration_ae_zone_wght_hor ) / sizeof( _calibration_ae_zone_wght_hor[0] ), .width = sizeof( _calibration_ae_zone_wght_hor[0] )};
static LookupTable calibration_calibration_ae_zone_wght_ver = {.ptr = _calibration_ae_zone_wght_ver, .rows = 1, .cols = sizeof( _calibration_ae_zone_wght_ver ) / sizeof( _calibration_ae_zone_wght_ver[0] ), .width = sizeof( _calibration_ae_zone_wght_ver[0] )};
static LookupTable calibration_calibration_awb_zone_wght_hor = {.ptr = _calibration_awb_zone_wght_hor, .rows = 1, .cols = sizeof( _calibration_awb_zone_wght_hor ) / sizeof( _calibration_awb_zone_wght_hor[0] ), .width = sizeof( _calibration_awb_zone_wght_hor[0] )};
static LookupTable calibration_calibration_awb_zone_wght_ver = {.ptr = _calibration_awb_zone_wght_ver, .rows = 1, .cols = sizeof( _calibration_awb_zone_wght_ver ) / sizeof( _calibration_awb_zone_wght_ver[0] ), .width = sizeof( _calibration_awb_zone_wght_ver[0] )};
static LookupTable calibration_dp_slope = {.ptr = _calibration_dp_slope, .rows = sizeof( _calibration_dp_slope ) / sizeof( _calibration_dp_slope[0] ), .cols = 2, .width = sizeof( _calibration_dp_slope[0][0] )};
static LookupTable calibration_cnr_uv_delta12_slope = {.ptr = _calibration_cnr_uv_delta12_slope, .rows = sizeof( _calibration_cnr_uv_delta12_slope ) / sizeof( _calibration_cnr_uv_delta12_slope[0] ), .cols = 2, .width = sizeof( _calibration_cnr_uv_delta12_slope[0][0] )};
static LookupTable calibration_sinter_sad = {.ptr = _calibration_sinter_sad, .rows = sizeof( _calibration_sinter_sad ) / sizeof( _calibration_sinter_sad[0] ), .cols = 2, .width = sizeof( _calibration_sinter_sad[0][0] )};
static LookupTable calibration_scaler_h_filter = {.ptr = _scaler_h_filter, .rows = 1, .cols = sizeof( _scaler_h_filter ) / sizeof( _scaler_h_filter[0] ), .width = sizeof( _scaler_h_filter[0] )};
static LookupTable calibration_scaler_v_filter = {.ptr = _scaler_v_filter, .rows = 1, .cols = sizeof( _scaler_v_filter ) / sizeof( _scaler_v_filter[0] ), .width = sizeof( _scaler_v_filter[0] )};
static LookupTable calibration_sharpen_ds1 = {.ptr = _calibration_sharpen_ds1, .rows = sizeof( _calibration_sharpen_ds1 ) / sizeof( _calibration_sharpen_ds1[0] ), .cols = 2, .width = sizeof( _calibration_sharpen_ds1[0][0] )};
static LookupTable calibration_temper_strength = {.ptr = _calibration_temper_strength, .rows = sizeof( _calibration_temper_strength ) / sizeof( _calibration_temper_strength[0] ), .cols = 2, .width = sizeof( _calibration_temper_strength[0][0] )};

uint32_t get_calibrations_dynamic_linear_imx415( ACameraCalibrations *c )
{
    uint32_t result = 0;
    if ( c != 0 ) {
        c->calibrations[CALIBRATION_STITCHING_LM_MED_NOISE_INTENSITY] = &calibration_stitching_lm_med_noise_intensity_thresh;
        c->calibrations[CALIBRATION_EXPOSURE_RATIO_ADJUSTMENT] = &calibration_exposure_ratio_adjustment;
        c->calibrations[CALIBRATION_SINTER_STRENGTH_MC_CONTRAST] = &calibration_sinter_strength_MC_contrast;
        c->calibrations[AWB_COLOUR_PREFERENCE] = &AWB_colour_preference;
        c->calibrations[CALIBRATION_AWB_MIX_LIGHT_PARAMETERS] = &calibration_awb_mix_light_parameters;
        c->calibrations[CALIBRATION_PF_RADIAL_LUT] = &calibration_pf_radial_lut;
        c->calibrations[CALIBRATION_PF_RADIAL_PARAMS] = &calibration_pf_radial_params;
        c->calibrations[CALIBRATION_SINTER_RADIAL_LUT] = &calibration_sinter_radial_lut;
        c->calibrations[CALIBRATION_SINTER_RADIAL_PARAMS] = &calibration_sinter_radial_params;
        c->calibrations[CALIBRATION_AWB_BG_MAX_GAIN] = &calibration_AWB_bg_max_gain;
        c->calibrations[CALIBRATION_IRIDIX8_STRENGTH_DK_ENH_CONTROL] = &calibration_iridix8_strength_dk_enh_control;
        c->calibrations[CALIBRATION_CMOS_CONTROL] = &calibration_cmos_control;
        c->calibrations[CALIBRATION_STATUS_INFO] = &calibration_status_info;
        c->calibrations[CALIBRATION_AUTO_LEVEL_CONTROL] = &calibration_auto_level_control;
        c->calibrations[CALIBRATION_DP_SLOPE] = &calibration_dp_slope;
        c->calibrations[CALIBRATION_DP_THRESHOLD] = &calibration_dp_threshold;
        c->calibrations[CALIBRATION_STITCHING_LM_MOV_MULT] = &calibration_stitching_lm_mov_mult;
        c->calibrations[CALIBRATION_STITCHING_LM_NP] = &calibration_stitching_lm_np;
        c->calibrations[CALIBRATION_STITCHING_MS_MOV_MULT] = &calibration_stitching_ms_mov_mult;
        c->calibrations[CALIBRATION_STITCHING_MS_NP] = &calibration_stitching_ms_np;
        c->calibrations[CALIBRATION_EVTOLUX_PROBABILITY_ENABLE] = &calibration_evtolux_probability_enable;
        c->calibrations[CALIBRATION_AWB_AVG_COEF] = &calibration_awb_avg_coef;
        c->calibrations[CALIBRATION_IRIDIX_AVG_COEF] = &calibration_iridix_avg_coef;
        c->calibrations[CALIBRATION_IRIDIX_STRENGTH_MAXIMUM] = &calibration_iridix_strength_maximum;
        c->calibrations[CALIBRATION_IRIDIX_MIN_MAX_STR] = &calibration_iridix_min_max_str;
        c->calibrations[CALIBRATION_IRIDIX_EV_LIM_FULL_STR] = &calibration_iridix_ev_lim_full_str;
        c->calibrations[CALIBRATION_IRIDIX_EV_LIM_NO_STR] = &calibration_iridix_ev_lim_no_str;
        c->calibrations[CALIBRATION_AE_CORRECTION] = &calibration_ae_correction;
        c->calibrations[CALIBRATION_AE_EXPOSURE_CORRECTION] = &calibration_ae_exposure_correction;
        c->calibrations[CALIBRATION_SINTER_STRENGTH] = &calibration_sinter_strength;
        c->calibrations[CALIBRATION_SINTER_STRENGTH1] = &calibration_sinter_strength1;
        c->calibrations[CALIBRATION_SINTER_THRESH1] = &calibration_sinter_thresh1;
        c->calibrations[CALIBRATION_SINTER_THRESH4] = &calibration_sinter_thresh4;
        c->calibrations[CALIBRATION_SINTER_INTCONFIG] = &calibration_sinter_intConfig;
        c->calibrations[CALIBRATION_SHARP_ALT_D] = &calibration_sharp_alt_d;
        c->calibrations[CALIBRATION_SHARP_ALT_UD] = &calibration_sharp_alt_ud;
        c->calibrations[CALIBRATION_SHARP_ALT_DU] = &calibration_sharp_alt_du;
        c->calibrations[CALIBRATION_SHARPEN_FR] = &calibration_sharpen_fr;
        c->calibrations[CALIBRATION_DEMOSAIC_NP_OFFSET] = &calibration_demosaic_np_offset;
        c->calibrations[CALIBRATION_MESH_SHADING_STRENGTH] = &calibration_mesh_shading_strength;
        c->calibrations[CALIBRATION_SATURATION_STRENGTH] = &calibration_saturation_strength;
        c->calibrations[CALIBRATION_CCM_ONE_GAIN_THRESHOLD] = &calibration_ccm_one_gain_threshold;
        c->calibrations[CALIBRATION_AE_CONTROL] = &calibration_ae_control;
        c->calibrations[CALIBRATION_AE_CONTROL_HDR_TARGET] = &calibration_ae_control_HDR_target;
        c->calibrations[CALIBRATION_RGB2YUV_CONVERSION] = &calibration_rgb2yuv_conversion;
        c->calibrations[CALIBRATION_AF_LMS] = &calibration_calibration_af_lms;
        c->calibrations[CALIBRATION_AF_ZONE_WGHT_HOR] = &calibration_calibration_af_zone_wght_hor;
        c->calibrations[CALIBRATION_AF_ZONE_WGHT_VER] = &calibration_calibration_af_zone_wght_ver;
        c->calibrations[CALIBRATION_AE_ZONE_WGHT_HOR] = &calibration_calibration_ae_zone_wght_hor;
        c->calibrations[CALIBRATION_AE_ZONE_WGHT_VER] = &calibration_calibration_ae_zone_wght_ver;
        c->calibrations[CALIBRATION_AWB_ZONE_WGHT_HOR] = &calibration_calibration_awb_zone_wght_hor;
        c->calibrations[CALIBRATION_AWB_ZONE_WGHT_VER] = &calibration_calibration_awb_zone_wght_ver;
        c->calibrations[CALIBRATION_CNR_UV_DELTA12_SLOPE] = &calibration_cnr_uv_delta12_slope;
        c->calibrations[CALIBRATION_FS_MC_OFF] = &calibration_fs_mc_off;
        c->calibrations[CALIBRATION_SINTER_SAD] = &calibration_sinter_sad;
        c->calibrations[CALIBRATION_SCALER_H_FILTER] = &calibration_scaler_h_filter;
        c->calibrations[CALIBRATION_SCALER_V_FILTER] = &calibration_scaler_v_filter;
        c->calibrations[CALIBRATION_SHARPEN_DS1] = &calibration_sharpen_ds1;
        c->calibrations[CALIBRATION_TEMPER_STRENGTH] = &calibration_temper_strength;
    } else {
        result = -1;
    }
    return result;
}
