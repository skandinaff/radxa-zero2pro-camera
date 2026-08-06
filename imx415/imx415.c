// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the Sony IMX415 CMOS Image Sensor.
 *
 * Copyright (C) 2023 WolfVision GmbH.
 */

/*
 * radxa-zero2pro-camera: vendored from mainline v6.3, with the deviations
 * listed here. Every one of them is also commented at the point of change;
 * this is the index, not the explanation.
 *
 *  1. imx415_power_on(): 50 ms settle after XCLR release instead of 100 us.
 *     Pre-existing, hardware-measured.
 *
 *  2. NEW all-pixel 4-lane 1440 Mbps 60.038 fps mode (imx415_mode_4_1440[] +
 *     a supported_modes[] entry). Halves the rolling-shutter skew and doubles
 *     the frame rate versus the 2-lane 1440 mode this board runs today, with
 *     no new INCK settings -- imx415_clk_params[] is keyed on lane RATE, not
 *     lane count. Requires overlays/camera-overlay.dts to say
 *     data-lanes = <1 2 3 4>.
 *
 *  3. V4L2_CID_VBLANK is a writable range instead of being pinned READ_ONLY
 *     at 58, and the driver now actually programs VMAX from it. VBLANK only
 *     makes the sensor slower; it is a debugging and long-exposure lever.
 *
 *  4. NEW window cropping / ROI: .set_selection, a real .get_selection, and a
 *     set_fmt that means "centred window of this size". This is the large
 *     skew win -- a 512-line strip reads out in 3.79 ms instead of 32.45 ms.
 *
 *  5. Consequences of 3 and 4: HBLANK/VBLANK/EXPOSURE ranges are re-published
 *     when the window changes; EXPOSURE clamps to VMAX - 8 rather than VMAX
 *     (mainline allowed SHR0 == 0, which is out of range).
 *
 * The motivation is a marksmanship shot trainer: frame rate, rolling-shutter
 * skew and short exposure are what matter; image quality does not, since the
 * consumer is a grayscale blob detector.
 *
 * WHAT IS UNVERIFIED. None of 2-5 has run on hardware -- there is one board
 * and this work was done without access to it, and without the IMX415
 * datasheet. The specific things to check first are flagged inline:
 * the PIX_HST/PIX_HWIDTH/PIX_VST/PIX_VWIDTH register addresses (see their
 * defines), the crop alignment granularities, and whether 58 lines of vertical
 * blanking is still enough in cropping mode (see imx415_set_window()).
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX415_PIXEL_ARRAY_TOP	  0
#define IMX415_PIXEL_ARRAY_LEFT	  0
#define IMX415_PIXEL_ARRAY_WIDTH  3864
#define IMX415_PIXEL_ARRAY_HEIGHT 2192
#define IMX415_PIXEL_ARRAY_VBLANK 58

/*
 * radxa-zero2pro-camera: window-cropping limits.
 *
 * ============================ ALIGNMENT ==================================
 * The IMX415 datasheet is not available to this port, so the alignment
 * constraints below are DERIVED/CONSERVATIVE, not quoted:
 *
 *   - left/top MUST be even, or the Bayer phase of the first output pixel
 *     flips and the advertised MEDIA_BUS_FMT_SGBRG10_1X10 becomes a lie.
 *     That much is a property of any Bayer sensor and is certain.
 *   - Sony's readout is pixel-parallel in the horizontal direction (the ADC
 *     block converts several columns at once), which normally forces the H
 *     start/width to a multiple of 4 or 8. 8 is used here because it is a
 *     superset of 1/2/4/8 and costs at most 7 columns.
 *   - 4 is used vertically for the same "superset of 1/2/4" reason.
 *
 * If the datasheet turns out to demand a coarser step (16 columns, say), the
 * numbers below are the single place to change. Every geometry this port
 * cares about survives the chosen alignment exactly: 3864 = 8*483,
 * 2192 = 4*548, 1920 = 8*240, 1080 = 4*270, 512 = 4*128.
 *
 * ========================== MINIMUM WINDOW ===============================
 * Genuinely unknown. 256x64 is a guess chosen to be far away from anything
 * this application needs (the smallest planned ROI is 1920x512), so it
 * should never be exercised. Do not treat it as validated.
 */
#define IMX415_CROP_LEFT_ALIGN	  8
#define IMX415_CROP_WIDTH_ALIGN	  8
#define IMX415_CROP_TOP_ALIGN	  4
#define IMX415_CROP_HEIGHT_ALIGN  4
#define IMX415_CROP_MIN_WIDTH	  256
#define IMX415_CROP_MIN_HEIGHT	  64

/*
 * SHR0 holds "VMAX minus the exposure in lines". The sensor needs a few
 * lines of margin at the end of the frame, so the exposure can never reach
 * VMAX: exposure_max = VMAX - IMX415_SHR0_MIN. 8 is the value mainline
 * already assumed (it computed exposure_max as height + vblank - 8) and the
 * same number the ISP bridge uses (IMX415_INTEGRATION_MIN in V4L2_drv.c).
 */
#define IMX415_SHR0_MIN		  8
#define IMX415_EXPOSURE_MIN	  4

/*
 * VMAX is written as 3 bytes (0x3024..0x3026) but the field is believed to be
 * 20 bits. ASSUMED, not verified against the datasheet. It only bounds the
 * V4L2_CID_VBLANK control, and overshooting it can at worst give a wrong
 * frame rate, never damage; the useful range is nowhere near the top.
 */
#define IMX415_VMAX_MAX		  0xFFFFF

/*
 * The clock HMAX and VMAX are counted in. NOT the pixel rate, and not the
 * lane rate -- it is the sensor's internal system clock, and it is a function
 * of INCK alone:
 *
 *   HMAX clock = INCK * INCKSEL4 / 84
 *      24 MHz * 252 / 84 = 72.00  MHz   (INCKSEL4 = 0x0FC, both 24 MHz entries)
 *      27 MHz * 231 / 84 = 74.25  MHz   (INCKSEL4 = 0x0E7, all 27 MHz entries)
 *
 * Derived, not quoted: every imx415_clk_params[] entry with inck == 24 MHz
 * carries INCKSEL3 = 0x0B4 and INCKSEL4 = 0x0FC, and every 27 MHz entry
 * carries 0x0A5..0x0C6 / 0x0E7 -- i.e. INCKSEL4 tracks INCK and nothing else.
 * Confirmed against all three pre-existing modes, which reproduce their
 * documented frame rates exactly as VMAX * HMAX / (this clock):
 *
 *   2-lane 720 : 2250 * 0x07F0(2032) / 72.00e6 = 63.500 ms = 15.748 fps
 *   2-lane 1440: 2250 * 0x042A(1066) / 72.00e6 = 33.313 ms = 30.019 fps
 *   4-lane 891 : 2250 * 0x044C(1100) / 74.25e6 = 33.333 ms = 30.000 fps
 *
 * This board runs a 24 MHz INCK (see overlays/camera-overlay.dts), so every
 * timing figure quoted in this file is against 72 MHz.
 */
#define IMX415_HMAX_CLK_24MHZ	  72000000UL

#define IMX415_NUM_CLK_PARAM_REGS 11

#define IMX415_REG_8BIT(n)	  ((1 << 16) | (n))
#define IMX415_REG_16BIT(n)	  ((2 << 16) | (n))
#define IMX415_REG_24BIT(n)	  ((3 << 16) | (n))
#define IMX415_REG_SIZE_SHIFT	  16
#define IMX415_REG_ADDR_MASK	  0xffff

#define IMX415_MODE		  IMX415_REG_8BIT(0x3000)
#define IMX415_MODE_OPERATING	  (0)
#define IMX415_MODE_STANDBY	  BIT(0)
#define IMX415_REGHOLD		  IMX415_REG_8BIT(0x3001)
#define IMX415_REGHOLD_INVALID	  (0)
#define IMX415_REGHOLD_VALID	  BIT(0)
#define IMX415_XMSTA		  IMX415_REG_8BIT(0x3002)
#define IMX415_XMSTA_START	  (0)
#define IMX415_XMSTA_STOP	  BIT(0)
#define IMX415_BCWAIT_TIME	  IMX415_REG_16BIT(0x3008)
#define IMX415_CPWAIT_TIME	  IMX415_REG_16BIT(0x300A)
#define IMX415_WINMODE		  IMX415_REG_8BIT(0x301C)
#define IMX415_WINMODE_ALL_PIXEL  0x00
#define IMX415_WINMODE_CROP	  0x04
#define IMX415_ADDMODE		  IMX415_REG_8BIT(0x3022)
#define IMX415_REVERSE		  IMX415_REG_8BIT(0x3030)
#define IMX415_HREVERSE_SHIFT	  (0)
#define IMX415_VREVERSE_SHIFT	  BIT(0)
#define IMX415_ADBIT		  IMX415_REG_8BIT(0x3031)
#define IMX415_MDBIT		  IMX415_REG_8BIT(0x3032)
#define IMX415_SYS_MODE		  IMX415_REG_8BIT(0x3033)
#define IMX415_OUTSEL		  IMX415_REG_8BIT(0x30C0)
#define IMX415_DRV		  IMX415_REG_8BIT(0x30C1)
#define IMX415_VMAX		  IMX415_REG_24BIT(0x3024)
#define IMX415_HMAX		  IMX415_REG_16BIT(0x3028)
/*
 * radxa-zero2pro-camera: window-cropping window registers.
 *
 * !!! UNVERIFIED ADDRESSES !!!  These four 16-bit pairs are the IMX415's
 * cropping window (PIX_HST / PIX_HWIDTH / PIX_VST / PIX_VWIDTH). They are NOT
 * used by mainline and could not be checked against the datasheet, which this
 * port does not have. They are taken from the Sony IMX415 register map as it
 * appears in vendor/BSP driver sources, which agree on:
 *
 *   0x3040/0x3041  PIX_HST[12:0]     horizontal window start, pixels
 *   0x3042/0x3043  PIX_HWIDTH[12:0]  horizontal window width, pixels
 *   0x3044/0x3045  PIX_VST[10:0]     vertical window start, lines
 *   0x3046/0x3047  PIX_VWIDTH[10:0]  vertical window width, lines
 *
 * All are little-endian byte pairs, which is what IMX415_REG_16BIT() emits
 * (imx415_write() puts the low byte at the named address). The high bytes
 * carry only the top few bits; the values programmed here are always in range
 * for the pixel array, so no masking is done.
 *
 * VALIDATE THESE ON HARDWARE BEFORE TRUSTING CROPPED OUTPUT. The failure mode
 * if an address is wrong is a garbled or black frame, not damage: 0x3040-0x3047
 * is inside the sensor's normal mode-setting register block, and every register
 * this driver writes is a mode register written while the sensor is in standby.
 * The all-pixel path is unaffected either way -- it writes WINMODE = 0 and
 * touches none of these.
 */
#define IMX415_PIX_HST		  IMX415_REG_16BIT(0x3040)
#define IMX415_PIX_HWIDTH	  IMX415_REG_16BIT(0x3042)
#define IMX415_PIX_VST		  IMX415_REG_16BIT(0x3044)
#define IMX415_PIX_VWIDTH	  IMX415_REG_16BIT(0x3046)
#define IMX415_SHR0		  IMX415_REG_24BIT(0x3050)
#define IMX415_GAIN_PCG_0	  IMX415_REG_16BIT(0x3090)
#define IMX415_AGAIN_MIN	  0
#define IMX415_AGAIN_MAX	  100
#define IMX415_AGAIN_STEP	  1
#define IMX415_BLKLEVEL		  IMX415_REG_16BIT(0x30E2)
#define IMX415_BLKLEVEL_DEFAULT	  50
#define IMX415_TPG_EN_DUOUT	  IMX415_REG_8BIT(0x30E4)
#define IMX415_TPG_PATSEL_DUOUT	  IMX415_REG_8BIT(0x30E6)
#define IMX415_TPG_COLORWIDTH	  IMX415_REG_8BIT(0x30E8)
#define IMX415_TESTCLKEN_MIPI	  IMX415_REG_8BIT(0x3110)
#define IMX415_INCKSEL1		  IMX415_REG_8BIT(0x3115)
#define IMX415_INCKSEL2		  IMX415_REG_8BIT(0x3116)
#define IMX415_INCKSEL3		  IMX415_REG_16BIT(0x3118)
#define IMX415_INCKSEL4		  IMX415_REG_16BIT(0x311A)
#define IMX415_INCKSEL5		  IMX415_REG_8BIT(0x311E)
#define IMX415_DIG_CLP_MODE	  IMX415_REG_8BIT(0x32C8)
#define IMX415_WRJ_OPEN		  IMX415_REG_8BIT(0x3390)
#define IMX415_SENSOR_INFO	  IMX415_REG_16BIT(0x3F12)
#define IMX415_SENSOR_INFO_MASK	  0xFFF
#define IMX415_CHIP_ID		  0x514
#define IMX415_LANEMODE		  IMX415_REG_16BIT(0x4001)
#define IMX415_LANEMODE_2	  1
#define IMX415_LANEMODE_4	  3
#define IMX415_TXCLKESC_FREQ	  IMX415_REG_16BIT(0x4004)
#define IMX415_INCKSEL6		  IMX415_REG_8BIT(0x400C)
#define IMX415_TCLKPOST		  IMX415_REG_16BIT(0x4018)
#define IMX415_TCLKPREPARE	  IMX415_REG_16BIT(0x401A)
#define IMX415_TCLKTRAIL	  IMX415_REG_16BIT(0x401C)
#define IMX415_TCLKZERO		  IMX415_REG_16BIT(0x401E)
#define IMX415_THSPREPARE	  IMX415_REG_16BIT(0x4020)
#define IMX415_THSZERO		  IMX415_REG_16BIT(0x4022)
#define IMX415_THSTRAIL		  IMX415_REG_16BIT(0x4024)
#define IMX415_THSEXIT		  IMX415_REG_16BIT(0x4026)
#define IMX415_TLPX		  IMX415_REG_16BIT(0x4028)
#define IMX415_INCKSEL7		  IMX415_REG_8BIT(0x4074)

struct imx415_reg {
	u32 address;
	u32 val;
};

static const char *const imx415_supply_names[] = {
	"dvdd",
	"ovdd",
	"avdd",
};

/*
 * The IMX415 data sheet uses lane rates but v4l2 uses link frequency to
 * describe MIPI CSI-2 speed. This driver uses lane rates wherever possible
 * and converts them to link frequencies by a factor of two when needed.
 */
static const s64 link_freq_menu_items[] = {
	594000000 / 2,	720000000 / 2,	891000000 / 2,
	1440000000 / 2, 1485000000 / 2,
};

struct imx415_clk_params {
	u64 lane_rate;
	u64 inck;
	struct imx415_reg regs[IMX415_NUM_CLK_PARAM_REGS];
};

/* INCK Settings - includes all lane rate and INCK dependent registers */
static const struct imx415_clk_params imx415_clk_params[] = {
	{
		.lane_rate = 594000000,
		.inck = 27000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x05D },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x042 },
		.regs[2] = { IMX415_SYS_MODE, 0x7 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x084 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E7 },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x06C0 },
	},
	{
		.lane_rate = 720000000,
		.inck = 24000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x054 },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x03B },
		.regs[2] = { IMX415_SYS_MODE, 0x9 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x0B4 },
		.regs[6] = { IMX415_INCKSEL4, 0x0FC },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x0600 },
	},
	{
		.lane_rate = 891000000,
		.inck = 27000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x05D },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x042 },
		.regs[2] = { IMX415_SYS_MODE, 0x5 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x0C6 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E7 },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x0 },
		.regs[9] = { IMX415_INCKSEL7, 0x1 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x06C0 },
	},
	{
		.lane_rate = 1440000000,
		.inck = 24000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x054 },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x03B },
		.regs[2] = { IMX415_SYS_MODE, 0x8 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x0B4 },
		.regs[6] = { IMX415_INCKSEL4, 0x0FC },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x0600 },
	},
	{
		.lane_rate = 1485000000,
		.inck = 27000000,
		.regs[0] = { IMX415_BCWAIT_TIME, 0x05D },
		.regs[1] = { IMX415_CPWAIT_TIME, 0x042 },
		.regs[2] = { IMX415_SYS_MODE, 0x8 },
		.regs[3] = { IMX415_INCKSEL1, 0x00 },
		.regs[4] = { IMX415_INCKSEL2, 0x23 },
		.regs[5] = { IMX415_INCKSEL3, 0x0A5 },
		.regs[6] = { IMX415_INCKSEL4, 0x0E7 },
		.regs[7] = { IMX415_INCKSEL5, 0x23 },
		.regs[8] = { IMX415_INCKSEL6, 0x1 },
		.regs[9] = { IMX415_INCKSEL7, 0x0 },
		.regs[10] = { IMX415_TXCLKESC_FREQ, 0x06C0 },
	},
};

/* all-pixel 2-lane 720 Mbps 15.74 Hz mode */
static const struct imx415_reg imx415_mode_2_720[] = {
	{ IMX415_VMAX, 0x08CA },
	{ IMX415_HMAX, 0x07F0 },
	{ IMX415_LANEMODE, IMX415_LANEMODE_2 },
	{ IMX415_TCLKPOST, 0x006F },
	{ IMX415_TCLKPREPARE, 0x002F },
	{ IMX415_TCLKTRAIL, 0x002F },
	{ IMX415_TCLKZERO, 0x00BF },
	{ IMX415_THSPREPARE, 0x002F },
	{ IMX415_THSZERO, 0x0057 },
	{ IMX415_THSTRAIL, 0x002F },
	{ IMX415_THSEXIT, 0x004F },
	{ IMX415_TLPX, 0x0027 },
};

/* all-pixel 2-lane 1440 Mbps 30.01 Hz mode */
static const struct imx415_reg imx415_mode_2_1440[] = {
	{ IMX415_VMAX, 0x08CA },
	{ IMX415_HMAX, 0x042A },
	{ IMX415_LANEMODE, IMX415_LANEMODE_2 },
	{ IMX415_TCLKPOST, 0x009F },
	{ IMX415_TCLKPREPARE, 0x0057 },
	{ IMX415_TCLKTRAIL, 0x0057 },
	{ IMX415_TCLKZERO, 0x0187 },
	{ IMX415_THSPREPARE, 0x005F },
	{ IMX415_THSZERO, 0x00A7 },
	{ IMX415_THSTRAIL, 0x005F },
	{ IMX415_THSEXIT, 0x0097 },
	{ IMX415_TLPX, 0x004F },
};

/* all-pixel 4-lane 891 Mbps 30 Hz mode */
static const struct imx415_reg imx415_mode_4_891[] = {
	{ IMX415_VMAX, 0x08CA },
	{ IMX415_HMAX, 0x044C },
	{ IMX415_LANEMODE, IMX415_LANEMODE_4 },
	{ IMX415_TCLKPOST, 0x007F },
	{ IMX415_TCLKPREPARE, 0x0037 },
	{ IMX415_TCLKTRAIL, 0x0037 },
	{ IMX415_TCLKZERO, 0x00F7 },
	{ IMX415_THSPREPARE, 0x003F },
	{ IMX415_THSZERO, 0x006F },
	{ IMX415_THSTRAIL, 0x003F },
	{ IMX415_THSEXIT, 0x005F },
	{ IMX415_TLPX, 0x002F },
};

/*
 * radxa-zero2pro-camera: all-pixel 4-lane 1440 Mbps 60.038 Hz mode. NEW, not
 * in mainline. This is the mode this board actually wants: it halves the
 * rolling-shutter skew of the 2-lane 1440 mode (32.45 ms -> 16.23 ms) and
 * doubles the frame rate, using hardware that is already wired.
 *
 * ------------------------- why this is reachable -------------------------
 * The overlay used to claim 4-lane was out of reach because imx415's only
 * 4-lane mode (891 Mbps) needs a 27 MHz INCK and this board gives 24 MHz.
 * That conflates two independent things. imx415_clk_params[] is keyed on
 * (lane_rate, inck) ONLY -- see imx415_check_inck() and the lookup at the end
 * of imx415_parse_hw_config(); lane count never enters it -- and
 * imx415_set_mode() writes the clk_params block verbatim after the mode's own
 * register list. So a 4-lane mode at an already-supported lane rate needs no
 * new INCK settings at all. The 1440 Mbps @ 24 MHz entry is present and is
 * proven working on this board today at 2 lanes.
 *
 * Corroboration from Sony's own mode list (the table below this comment):
 * "1440 / 2 lanes / 30.019 / 4510 / 304615385" and
 * "1440 / 4 lanes / 30.019 / 4510 / 304615385" are the same line twice with a
 * different lane count -- identical hmax_pix and identical pixel_rate, i.e.
 * identical HMAX and identical system clock. Lane count does not move the
 * timing base.
 *
 * ---------------------------- deriving HMAX ------------------------------
 * frame period = VMAX * HMAX / 72e6 (see IMX415_HMAX_CLK_24MHZ above for why
 * 72e6, and for the check that this reproduces all three existing modes).
 * VMAX stays 0x08CA = 2250 = 2192 active + 58 blank.
 *
 *   HMAX = 72e6 / (60.0375 * 2250) = 533   -> 0x0215
 *
 * Cross-check against the documented table row "1440 / 4 / 60.038 / 4510 /
 * 609230769", using the identity that makes hmax_pix/pixel_rate consistent
 * with the real register (both describe the same line period):
 *
 *   pixel_rate = hmax_pix * 72e6 / HMAX = 4510 * 72e6 / 533 = 609230769.2
 *
 * which truncates to exactly the documented 609230769. And
 * 72e6 / (2250 * 533) = 60.0375 fps, i.e. the documented "60.038". Two
 * independent derivations agree, and the same method reproduces HMAX for all
 * three pre-existing modes (2032, 1066, 1100). HMAX = 533.
 *
 * NOTE: 550 would be the answer if the system clock were 74.25 MHz. It is not,
 * at a 24 MHz INCK -- 74.25 MHz is the 27 MHz-INCK value, which is where the
 * 4-lane 891 mode's HMAX of 1100 comes from. 550 reproduces neither the
 * documented fps nor the documented pixel_rate.
 *
 * -------------------------- MIPI feasibility -----------------------------
 * One line is 3864 px * 10 bit = 38640 bits. 4 lanes * 1440 Mbps = 5760 Mbps.
 *
 *   payload time  = 38640 / 5.76e9   = 6.708 us
 *   line period   = 533 / 72e6       = 7.403 us
 *   margin        = 0.694 us          (90.6 % link utilisation)
 *
 * That is tight, but it is EXACTLY the utilisation the working 2-lane 1440
 * mode already runs at (13.417 us payload in a 14.806 us line = 90.6 %) --
 * both halves of the ratio doubled. Sony ships this mode, so the packet
 * overhead fits in the same 9.4 %.
 *
 * ----------------------------- D-PHY timings -----------------------------
 * TCLKPOST..TLPX are copied verbatim from imx415_mode_2_1440. They are a
 * function of the lane RATE (they are unit-interval counts), not the lane
 * count: across the three existing modes TLPX runs 39 / 47 / 79 for
 * 720 / 891 / 1440 Mbps -- linear in lane rate, and the 891 entry is the
 * 4-lane one, sitting on the same line as the two 2-lane entries.
 */
static const struct imx415_reg imx415_mode_4_1440[] = {
	{ IMX415_VMAX, 0x08CA },
	{ IMX415_HMAX, 0x0215 },
	{ IMX415_LANEMODE, IMX415_LANEMODE_4 },
	{ IMX415_TCLKPOST, 0x009F },
	{ IMX415_TCLKPREPARE, 0x0057 },
	{ IMX415_TCLKTRAIL, 0x0057 },
	{ IMX415_TCLKZERO, 0x0187 },
	{ IMX415_THSPREPARE, 0x005F },
	{ IMX415_THSZERO, 0x00A7 },
	{ IMX415_THSTRAIL, 0x005F },
	{ IMX415_THSEXIT, 0x0097 },
	{ IMX415_TLPX, 0x004F },
};

struct imx415_mode_reg_list {
	u32 num_of_regs;
	const struct imx415_reg *regs;
};

/*
 * Mode : number of lanes, lane rate and frame rate dependent settings
 *
 * pixel_rate and hmax_pix are needed to calculate hblank for the v4l2 ctrl
 * interface. These values can not be found in the data sheet and should be
 * treated as virtual values. Use following table when adding new modes.
 *
 * lane_rate  lanes    fps     hmax_pix   pixel_rate
 *
 *     594      2     10.000     4400       99000000
 *     891      2     15.000     4400      148500000
 *     720      2     15.748     4064      144000000
 *    1782      2     30.000     4400      297000000
 *    2079      2     30.000     4400      297000000
 *    1440      2     30.019     4510      304615385
 *
 *     594      4     20.000     5500      247500000
 *     594      4     25.000     4400      247500000
 *     720      4     25.000     4400      247500000
 *     720      4     30.019     4510      304615385
 *     891      4     30.000     4400      297000000
 *    1440      4     30.019     4510      304615385
 *    1440      4     60.038     4510      609230769
 *    1485      4     60.000     4400      594000000
 *    1782      4     60.000     4400      594000000
 *    2079      4     60.000     4400      594000000
 *    2376      4     90.164     4392      891000000
 */
struct imx415_mode {
	u64 lane_rate;
	u32 lanes;
	u32 hmax_pix;
	u64 pixel_rate;
	struct imx415_mode_reg_list reg_list;
};

/* mode configs */
static const struct imx415_mode supported_modes[] = {
	{
		.lane_rate = 720000000,
		.lanes = 2,
		.hmax_pix = 4064,
		.pixel_rate = 144000000,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_mode_2_720),
			.regs = imx415_mode_2_720,
		},
	},
	{
		.lane_rate = 1440000000,
		.lanes = 2,
		.hmax_pix = 4510,
		.pixel_rate = 304615385,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_mode_2_1440),
			.regs = imx415_mode_2_1440,
		},
	},
	{
		.lane_rate = 891000000,
		.lanes = 4,
		.hmax_pix = 4400,
		.pixel_rate = 297000000,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_mode_4_891),
			.regs = imx415_mode_4_891,
		},
	},
	/*
	 * radxa-zero2pro-camera: the fast mode. Selected by
	 * data-lanes = <1 2 3 4> + link-frequencies = 720000000 in the overlay;
	 * imx415_parse_hw_config() matches on (lanes, lane_rate), and this is
	 * only entry with (4, 1440000000), so there is no ambiguity with the
	 * 2-lane 1440 entry above. See imx415_mode_4_1440[] for the derivation
	 * of every number here.
	 */
	{
		.lane_rate = 1440000000,
		.lanes = 4,
		.hmax_pix = 4510,
		.pixel_rate = 609230769,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx415_mode_4_1440),
			.regs = imx415_mode_4_1440,
		},
	},
};

static const struct regmap_config imx415_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static const char *const imx415_test_pattern_menu[] = {
	"disabled",
	"solid black",
	"solid white",
	"solid dark gray",
	"solid light gray",
	"stripes light/dark grey",
	"stripes dark/light grey",
	"stripes black/dark grey",
	"stripes dark grey/black",
	"stripes black/white",
	"stripes white/black",
	"horizontal color bar",
	"vertical color bar",
};

struct imx415 {
	struct device *dev;
	struct clk *clk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx415_supply_names)];
	struct gpio_desc *reset;
	struct regmap *regmap;

	const struct imx415_clk_params *clk_params;

	bool streaming;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	/*
	 * radxa-zero2pro-camera: the VMAX currently programmed (or about to
	 * be),
	 * i.e. readout lines + vertical blanking.
	 *
	 * Cached rather than recomputed from (format->height + vblank->cur.val)
	 * at every use, because of a v4l2-ctrls ordering trap: inside a
	 * control's
	 * own s_ctrl() its ->cur.val is still the OLD value (new_to_cur() runs
	 * after s_ctrl() returns). Updating VBLANK re-publishes the EXPOSURE
	 * range, and __v4l2_ctrl_modify_range() re-enters s_ctrl() for EXPOSURE
	 * if the clamp moved the value -- which would then have read a stale
	 * vblank and written an SHR0 inconsistent with the VMAX programmed
	 * in the very same call. The VBLANK handler updates this field first,
	 * so
	 * everything downstream sees one coherent VMAX.
	 */
	unsigned int cur_vmax;

	unsigned int cur_mode;
	unsigned int num_data_lanes;
};

/*
 * This table includes fixed register settings and a bunch of undocumented
 * registers that have to be set to another value than default.
 */
static const struct imx415_reg imx415_init_table[] = {
	/* use all-pixel readout mode, no flip */
	{ IMX415_WINMODE, 0x00 },
	{ IMX415_ADDMODE, 0x00 },
	{ IMX415_REVERSE, 0x00 },
	/* use RAW 10-bit mode */
	{ IMX415_ADBIT, 0x00 },
	{ IMX415_MDBIT, 0x00 },
	/* output VSYNC on XVS and low on XHS */
	{ IMX415_OUTSEL, 0x22 },
	{ IMX415_DRV, 0x00 },

	/* SONY magic registers */
	{ IMX415_REG_8BIT(0x32D4), 0x21 },
	{ IMX415_REG_8BIT(0x32EC), 0xA1 },
	{ IMX415_REG_8BIT(0x3452), 0x7F },
	{ IMX415_REG_8BIT(0x3453), 0x03 },
	{ IMX415_REG_8BIT(0x358A), 0x04 },
	{ IMX415_REG_8BIT(0x35A1), 0x02 },
	{ IMX415_REG_8BIT(0x36BC), 0x0C },
	{ IMX415_REG_8BIT(0x36CC), 0x53 },
	{ IMX415_REG_8BIT(0x36CD), 0x00 },
	{ IMX415_REG_8BIT(0x36CE), 0x3C },
	{ IMX415_REG_8BIT(0x36D0), 0x8C },
	{ IMX415_REG_8BIT(0x36D1), 0x00 },
	{ IMX415_REG_8BIT(0x36D2), 0x71 },
	{ IMX415_REG_8BIT(0x36D4), 0x3C },
	{ IMX415_REG_8BIT(0x36D6), 0x53 },
	{ IMX415_REG_8BIT(0x36D7), 0x00 },
	{ IMX415_REG_8BIT(0x36D8), 0x71 },
	{ IMX415_REG_8BIT(0x36DA), 0x8C },
	{ IMX415_REG_8BIT(0x36DB), 0x00 },
	{ IMX415_REG_8BIT(0x3724), 0x02 },
	{ IMX415_REG_8BIT(0x3726), 0x02 },
	{ IMX415_REG_8BIT(0x3732), 0x02 },
	{ IMX415_REG_8BIT(0x3734), 0x03 },
	{ IMX415_REG_8BIT(0x3736), 0x03 },
	{ IMX415_REG_8BIT(0x3742), 0x03 },
	{ IMX415_REG_8BIT(0x3862), 0xE0 },
	{ IMX415_REG_8BIT(0x38CC), 0x30 },
	{ IMX415_REG_8BIT(0x38CD), 0x2F },
	{ IMX415_REG_8BIT(0x395C), 0x0C },
	{ IMX415_REG_8BIT(0x3A42), 0xD1 },
	{ IMX415_REG_8BIT(0x3A4C), 0x77 },
	{ IMX415_REG_8BIT(0x3AE0), 0x02 },
	{ IMX415_REG_8BIT(0x3AEC), 0x0C },
	{ IMX415_REG_8BIT(0x3B00), 0x2E },
	{ IMX415_REG_8BIT(0x3B06), 0x29 },
	{ IMX415_REG_8BIT(0x3B98), 0x25 },
	{ IMX415_REG_8BIT(0x3B99), 0x21 },
	{ IMX415_REG_8BIT(0x3B9B), 0x13 },
	{ IMX415_REG_8BIT(0x3B9C), 0x13 },
	{ IMX415_REG_8BIT(0x3B9D), 0x13 },
	{ IMX415_REG_8BIT(0x3B9E), 0x13 },
	{ IMX415_REG_8BIT(0x3BA1), 0x00 },
	{ IMX415_REG_8BIT(0x3BA2), 0x06 },
	{ IMX415_REG_8BIT(0x3BA3), 0x0B },
	{ IMX415_REG_8BIT(0x3BA4), 0x10 },
	{ IMX415_REG_8BIT(0x3BA5), 0x14 },
	{ IMX415_REG_8BIT(0x3BA6), 0x18 },
	{ IMX415_REG_8BIT(0x3BA7), 0x1A },
	{ IMX415_REG_8BIT(0x3BA8), 0x1A },
	{ IMX415_REG_8BIT(0x3BA9), 0x1A },
	{ IMX415_REG_8BIT(0x3BAC), 0xED },
	{ IMX415_REG_8BIT(0x3BAD), 0x01 },
	{ IMX415_REG_8BIT(0x3BAE), 0xF6 },
	{ IMX415_REG_8BIT(0x3BAF), 0x02 },
	{ IMX415_REG_8BIT(0x3BB0), 0xA2 },
	{ IMX415_REG_8BIT(0x3BB1), 0x03 },
	{ IMX415_REG_8BIT(0x3BB2), 0xE0 },
	{ IMX415_REG_8BIT(0x3BB3), 0x03 },
	{ IMX415_REG_8BIT(0x3BB4), 0xE0 },
	{ IMX415_REG_8BIT(0x3BB5), 0x03 },
	{ IMX415_REG_8BIT(0x3BB6), 0xE0 },
	{ IMX415_REG_8BIT(0x3BB7), 0x03 },
	{ IMX415_REG_8BIT(0x3BB8), 0xE0 },
	{ IMX415_REG_8BIT(0x3BBA), 0xE0 },
	{ IMX415_REG_8BIT(0x3BBC), 0xDA },
	{ IMX415_REG_8BIT(0x3BBE), 0x88 },
	{ IMX415_REG_8BIT(0x3BC0), 0x44 },
	{ IMX415_REG_8BIT(0x3BC2), 0x7B },
	{ IMX415_REG_8BIT(0x3BC4), 0xA2 },
	{ IMX415_REG_8BIT(0x3BC8), 0xBD },
	{ IMX415_REG_8BIT(0x3BCA), 0xBD },
};

static inline struct imx415 *to_imx415(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx415, subdev);
}

static int imx415_read(struct imx415 *sensor, u32 addr)
{
	u8 data[3] = { 0 };
	int ret;

	ret = regmap_raw_read(sensor->regmap, addr & IMX415_REG_ADDR_MASK, data,
			      (addr >> IMX415_REG_SIZE_SHIFT) & 3);
	if (ret < 0)
		return ret;

	return (data[2] << 16) | (data[1] << 8) | data[0];
}

static int imx415_write(struct imx415 *sensor, u32 addr, u32 value)
{
	u8 data[3] = { value & 0xff, (value >> 8) & 0xff, value >> 16 };
	int ret;

	ret = regmap_raw_write(sensor->regmap, addr & IMX415_REG_ADDR_MASK,
			       data, (addr >> IMX415_REG_SIZE_SHIFT) & 3);
	if (ret < 0)
		dev_err_ratelimited(sensor->dev,
				    "%u-bit write to 0x%04x failed: %d\n",
				    ((addr >> IMX415_REG_SIZE_SHIFT) & 3) * 8,
				    addr & IMX415_REG_ADDR_MASK, ret);

	return 0;
}

static int imx415_set_testpattern(struct imx415 *sensor, int val)
{
	int ret;

	if (val) {
		ret = imx415_write(sensor, IMX415_BLKLEVEL, 0x00);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_TPG_EN_DUOUT, 0x01);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_TPG_PATSEL_DUOUT, val - 1);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_TPG_COLORWIDTH, 0x01);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_TESTCLKEN_MIPI, 0x20);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_DIG_CLP_MODE, 0x00);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_WRJ_OPEN, 0x00);
	} else {
		ret = imx415_write(sensor, IMX415_BLKLEVEL,
				   IMX415_BLKLEVEL_DEFAULT);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_TPG_EN_DUOUT, 0x00);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_TESTCLKEN_MIPI, 0x00);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_DIG_CLP_MODE, 0x01);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_WRJ_OPEN, 0x01);
	}
	return 0;
}

/*
 * radxa-zero2pro-camera: re-publish the EXPOSURE range for the current VMAX.
 *
 * Must be called with the control handler lock held -- which, in this driver,
 * is the same mutex as the subdev state lock (see imx415_subdev_init()). Hence
 * the __-prefixed, caller-locks variant.
 */
static void imx415_update_exposure_range(struct imx415 *sensor)
{
	u32 max = sensor->cur_vmax - IMX415_SHR0_MIN;

	__v4l2_ctrl_modify_range(sensor->exposure, IMX415_EXPOSURE_MIN, max, 1,
				 max);
}

/*
 * radxa-zero2pro-camera: re-publish the blanking + exposure ranges after the
 * readout window changed. Same locking rule as above.
 */
static void imx415_update_geometry_ctrls(struct imx415 *sensor, u32 width,
					 u32 height)
{
	u32 hblank = supported_modes[sensor->cur_mode].hmax_pix - width;

	/*
	 * HBLANK stays READ_ONLY -- HMAX is owned by the mode register list and
	 * is not a runtime knob here (see the note above imx415_set_window()).
	 * Its *value* still has to track the crop width, because userspace and
	 * the ISP bridge compute the frame rate as
	 *
	 *   fps = pixel_rate / ((width + hblank) * (height + vblank))
	 *
	 * and that only comes out right while (width + hblank) == hmax_pix.
	 */
	__v4l2_ctrl_modify_range(sensor->hblank, hblank, hblank, 1, hblank);

	/*
	 * VBLANK's minimum is the sensor's fixed 58 lines of vertical blanking;
	 * its maximum is whatever still fits in VMAX. Cropping does not change
	 * the minimum, only how many active lines it is added to.
	 */
	__v4l2_ctrl_modify_range(sensor->vblank, IMX415_PIXEL_ARRAY_VBLANK,
				 IMX415_VMAX_MAX - height, 1,
				 IMX415_PIXEL_ARRAY_VBLANK);

	/*
	 * Read vblank->cur.val back rather than assuming the default: a user
	 * value set before the crop survives, clamped into the new range by the
	 * modify_range above.
	 */
	sensor->cur_vmax = height + sensor->vblank->cur.val;
	imx415_update_exposure_range(sensor);
}

static int imx415_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx415 *sensor = container_of(ctrl->handler, struct imx415,
					     ctrls);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	unsigned int vmax;
	unsigned int flip;
	int ret;

	/*
	 * radxa-zero2pro-camera: the state accessors below dereference
	 * sd->active_state unconditionally, and that only exists from
	 * v4l2_subdev_init_finalize() onwards. Mainline got away without this
	 * guard because the !streaming early-return came first; VBLANK now has
	 * work to do while not streaming, so the guard has to be explicit.
	 */
	if (!sensor->subdev.active_state)
		return 0;

	state = v4l2_subdev_get_locked_active_state(&sensor->subdev);
	format = v4l2_subdev_get_pad_format(&sensor->subdev, state, 0);

	/*
	 * radxa-zero2pro-camera: VBLANK is handled before the !streaming
	 * early-return because it moves the EXPOSURE ceiling, which has to be
	 * visible to userspace (and to the ISP bridge, which reads the EXPOSURE
	 * control's min/max in sensor_update_parameters()) whether or not
	 * pixels
	 * are flowing.
	 *
	 * Note VBLANK only makes the sensor SLOWER: it adds idle lines after
	 * the
	 * frame, so it lengthens the frame period without changing the line
	 * period, i.e. it costs frame rate and buys nothing in rolling-shutter
	 * skew. It exists as a debugging lever (back off the frame rate to see
	 * whether a problem is receiver bandwidth) and to let the AE loop reach
	 * longer exposures. Cropping, not VBLANK, is the lever for going
	 * faster.
	 */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* ctrl->val, not sensor->vblank->cur.val -- see cur_vmax. */
		sensor->cur_vmax = format->height + ctrl->val;

		if (sensor->streaming) {
			ret = imx415_write(sensor, IMX415_VMAX,
					   sensor->cur_vmax);
			if (ret)
				return ret;
		}

		imx415_update_exposure_range(sensor);
		return 0;
	}

	if (!sensor->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/*
		 * radxa-zero2pro-camera: clamp against the cached VMAX (see
		 * cur_vmax), and clamp to VMAX - IMX415_SHR0_MIN rather than to
		 * VMAX. Mainline's min_t(int, ctrl->val, vmax) allowed
		 * SHR0 == 0, which is below the sensor's minimum and would be
		 * rejected or produce a corrupt frame.
		 */
		vmax = sensor->cur_vmax;
		ctrl->val = clamp_t(int, ctrl->val, IMX415_EXPOSURE_MIN,
				    (int)vmax - IMX415_SHR0_MIN);
		return imx415_write(sensor, IMX415_SHR0, vmax - ctrl->val);

	case V4L2_CID_HBLANK:
	case V4L2_CID_LINK_FREQ:
		/*
		 * radxa-zero2pro-camera: read-only informational controls.
		 * __v4l2_ctrl_handler_setup() skips READ_ONLY controls, so
		 * mainline never saw these here -- but
		 * __v4l2_ctrl_modify_range() does NOT skip them: it re-enters
		 * s_ctrl() whenever the clamped value actually moves. HBLANK's
		 * value moves on every crop change, so without this case it
		 * would fall through to -EINVAL and fail set_fmt().
		 * There is nothing to program: HMAX comes from the mode's
		 * register list.
		 */
		return 0;

	case V4L2_CID_ANALOGUE_GAIN:
		/* analogue gain in 0.3 dB step size */
		return imx415_write(sensor, IMX415_GAIN_PCG_0, ctrl->val);

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		flip = (sensor->hflip->val << IMX415_HREVERSE_SHIFT) |
		       (sensor->vflip->val << IMX415_VREVERSE_SHIFT);
		return imx415_write(sensor, IMX415_REVERSE, flip);

	case V4L2_CID_TEST_PATTERN:
		return imx415_set_testpattern(sensor, ctrl->val);

	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops imx415_ctrl_ops = {
	.s_ctrl = imx415_s_ctrl,
};

static int imx415_ctrls_init(struct imx415 *sensor)
{
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	u64 pixel_rate = supported_modes[sensor->cur_mode].pixel_rate;
	u64 lane_rate = supported_modes[sensor->cur_mode].lane_rate;
	u32 exposure_max = IMX415_PIXEL_ARRAY_HEIGHT +
			   IMX415_PIXEL_ARRAY_VBLANK - IMX415_SHR0_MIN;
	u32 hblank;
	unsigned int i;
	int ret;

	/*
	 * radxa-zero2pro-camera: seed the cached VMAX for the default all-pixel
	 * geometry. imx415_init_cfg() runs later and sets the same value, but
	 * it
	 * goes through imx415_set_format() with which == 0 (== _FORMAT_TRY), so
	 * it deliberately does not touch controls; this has to be right before
	 * any s_ctrl() can run.
	 */
	sensor->cur_vmax = IMX415_PIXEL_ARRAY_HEIGHT +
			   IMX415_PIXEL_ARRAY_VBLANK;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 10);

	for (i = 0; i < ARRAY_SIZE(link_freq_menu_items); ++i) {
		if (lane_rate == link_freq_menu_items[i] * 2)
			break;
	}
	if (i == ARRAY_SIZE(link_freq_menu_items)) {
		return dev_err_probe(sensor->dev, -EINVAL,
				     "lane rate %llu not supported\n",
				     lane_rate);
	}

	ctrl = v4l2_ctrl_new_int_menu(&sensor->ctrls, &imx415_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(link_freq_menu_items) - 1, i,
				      link_freq_menu_items);

	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * radxa-zero2pro-camera: VBLANK is created BEFORE EXPOSURE now.
	 * __v4l2_ctrl_handler_setup() walks the handler in creation order, and
	 * VBLANK is no longer READ_ONLY, so it is now actually applied at
	 * stream-on. Programming VMAX before SHR0 keeps the pair consistent at
	 * every instant instead of relying on the fact that both happen while
	 * the sensor is still in standby.
	 *
	 * The range is real rather than pinned at 58: 58 is the sensor's
	 * *minimum* vertical blanking, not a fixed value, and being able to
	 * lengthen the frame period at runtime is the cheapest way to test
	 * whether a capture problem is receiver bandwidth. Maximum is whatever
	 * still fits in VMAX for the current readout height.
	 */
	sensor->vblank = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
					   V4L2_CID_VBLANK,
					   IMX415_PIXEL_ARRAY_VBLANK,
					   IMX415_VMAX_MAX -
						   IMX415_PIXEL_ARRAY_HEIGHT,
					   1, IMX415_PIXEL_ARRAY_VBLANK);

	sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX415_EXPOSURE_MIN, exposure_max,
					     1, exposure_max);

	v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, IMX415_AGAIN_MIN,
			  IMX415_AGAIN_MAX, IMX415_AGAIN_STEP,
			  IMX415_AGAIN_MIN);

	hblank = supported_modes[sensor->cur_mode].hmax_pix -
		 IMX415_PIXEL_ARRAY_WIDTH;
	sensor->hblank = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * The pixel rate used here is a virtual value and can be used for
	 * calculating the frame rate together with hblank. It may not
	 * necessarily be the physically correct pixel clock.
	 */
	v4l2_ctrl_new_std(&sensor->ctrls, NULL, V4L2_CID_PIXEL_RATE, pixel_rate,
			  pixel_rate, 1, pixel_rate);

	sensor->hflip = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(&sensor->ctrls, &imx415_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(&sensor->ctrls, &imx415_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx415_test_pattern_menu) - 1,
				     0, 0, imx415_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &imx415_ctrl_ops,
					&props);

	if (sensor->ctrls.error) {
		dev_err_probe(sensor->dev, sensor->ctrls.error,
			      "failed to add controls\n");
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return sensor->ctrls.error;
	}
	sensor->subdev.ctrl_handler = &sensor->ctrls;

	return 0;
}

static int imx415_set_mode(struct imx415 *sensor, int mode)
{
	const struct imx415_reg *reg;
	unsigned int i;
	int ret = 0;

	if (mode >= ARRAY_SIZE(supported_modes)) {
		dev_err(sensor->dev, "Mode %d not supported\n", mode);
		return -EINVAL;
	}

	for (i = 0; i < supported_modes[mode].reg_list.num_of_regs; ++i) {
		reg = &supported_modes[mode].reg_list.regs[i];
		ret = imx415_write(sensor, reg->address, reg->val);
		if (ret)
			return ret;
	}

	for (i = 0; i < IMX415_NUM_CLK_PARAM_REGS; ++i) {
		reg = &sensor->clk_params->regs[i];
		ret = imx415_write(sensor, reg->address, reg->val);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * radxa-zero2pro-camera: program the readout window and the matching VMAX.
 * NEW, not in mainline (mainline only ever reads the full array).
 *
 * ======================= why this is the big win =========================
 * Rolling-shutter skew is (lines read out) * (line period). The line period is
 * HMAX / 72e6 and is fixed by the mode. So cutting the number of lines cuts
 * the skew proportionally -- and because VMAX shrinks with it, it raises the
 * frame rate by the same factor. For a shot trainer, both of those are the
 * whole point; the discarded rows cost nothing since the target occupies a
 * small part of the frame.
 *
 * On the 4-lane 1440 mode (line period 7.403 us):
 *
 *   window        VMAX  frame period     fps      skew
 *   3864 x 2192   2250    16.656 ms    60.04    16.227 ms
 *   1920 x 1080   1138     8.424 ms   118.70     7.995 ms
 *   1920 x  512    570     4.220 ms   236.99     3.790 ms
 *
 * On the 2-lane 1440 mode this board runs today (line period 14.806 us),
 * cropping alone -- no overlay change, no lane change -- already gives:
 *
 *   3864 x 2192   2250    33.313 ms    30.02    32.454 ms   (today)
 *   1920 x 1080   1138    16.849 ms    59.35    15.990 ms
 *   1920 x  512    570     8.439 ms   118.50     7.580 ms
 *
 * ================= why horizontal cropping does not help =================
 * HMAX is a programmed line PERIOD, not a consequence of the line width, so
 * narrowing the window does not shorten the line -- it only frees MIPI
 * bandwidth (a 1920-px line needs 3.33 us of a 7.40 us line period at 4 lanes,
 * 45 % utilisation, versus 90.6 % at full width) and reduces the pixel volume
 * the ISP has to chew through. Horizontal cropping is supported because it is
 * free to support and it does buy those two things, but every skew figure
 * above depends only on the height.
 *
 * There IS a further factor-of-1.5 available by shrinking HMAX once the line
 * is narrower, which would shorten the line period itself. It is deliberately
 * NOT done here: the sensor's minimum HMAX is a datasheet number this port
 * does not have. The only bound available is empirical -- Sony's own mode list
 * (the table above struct imx415_mode) contains a 4-lane 2376 Mbps mode with
 * hmax_pix 4392 at 90.164 fps, i.e. HMAX = 366 at 74.25 MHz = 4.93 us, so the
 * analog front end sustains at least a 4.93 us line. Scaled to this board's
 * 72 MHz that is HMAX >= 355 against the 533 used here. Exploiting it means
 * making V4L2_CID_HBLANK writable and clamping HMAX at max(355, MIPI payload
 * time); that is a hardware-validation job, not a by-construction one.
 *
 * ========================== the VMAX floor ===============================
 * VMAX = readout height + vertical blanking, blanking >= 58 lines. The 58 is
 * mainline's IMX415_PIXEL_ARRAY_VBLANK and is confirmed by the existing modes
 * (2192 + 58 = 2250 = 0x08CA). ASSUMED, and the main thing to check on
 * hardware: that 58 remains sufficient in cropping mode, and that there is no
 * separate absolute VMAX floor that a 570-line frame would violate. If a
 * cropped mode produces broken frames, raising V4L2_CID_VBLANK is the first
 * thing to try.
 */
static int imx415_set_window(struct imx415 *sensor,
			     struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *crop;
	int ret;

	format = v4l2_subdev_get_pad_format(&sensor->subdev, state, 0);
	crop = v4l2_subdev_get_pad_crop(&sensor->subdev, state, 0);

	if (crop->width == IMX415_PIXEL_ARRAY_WIDTH &&
	    crop->height == IMX415_PIXEL_ARRAY_HEIGHT) {
		/*
		 * Full array: stay in all-pixel mode rather than programming a
		 * full-size crop window. imx415_init_table[] already wrote
		 * WINMODE = 0, but write it again so that a stream_off/crop/
		 * stream_on cycle cannot leave the cropping mode latched.
		 */
		ret = imx415_write(sensor, IMX415_WINMODE,
				   IMX415_WINMODE_ALL_PIXEL);
		if (ret)
			return ret;
	} else {
		ret = imx415_write(sensor, IMX415_PIX_HST, crop->left);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_PIX_HWIDTH, crop->width);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_PIX_VST, crop->top);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_PIX_VWIDTH, crop->height);
		if (ret)
			return ret;
		ret = imx415_write(sensor, IMX415_WINMODE, IMX415_WINMODE_CROP);
		if (ret)
			return ret;
	}

	/*
	 * Overrides the VMAX from the mode's register list (which always states
	 * the all-pixel 0x08CA). Recomputed here rather than trusted from
	 * cur_vmax so that a stream-on after a TRY-only format, or after a
	 * control range clamp, cannot program a VMAX that disagrees with the
	 * window actually being read out.
	 */
	sensor->cur_vmax = format->height + sensor->vblank->cur.val;

	return imx415_write(sensor, IMX415_VMAX, sensor->cur_vmax);
}

static int imx415_setup(struct imx415 *sensor, struct v4l2_subdev_state *state)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(imx415_init_table); ++i) {
		ret = imx415_write(sensor, imx415_init_table[i].address,
				   imx415_init_table[i].val);
		if (ret)
			return ret;
	}

	ret = imx415_set_mode(sensor, sensor->cur_mode);
	if (ret)
		return ret;

	/*
	 * radxa-zero2pro-camera: after the mode list, so that the window's VMAX
	 * wins over the mode's all-pixel default. The controls applied by
	 * __v4l2_ctrl_handler_setup() in imx415_s_stream() run after this and
	 * will program the same VMAX again from the same cur_vmax.
	 */
	return imx415_set_window(sensor, state);
}

static int imx415_wakeup(struct imx415 *sensor)
{
	int ret;

	ret = imx415_write(sensor, IMX415_MODE, IMX415_MODE_OPERATING);
	if (ret)
		return ret;

	/*
	 * According to the datasheet we have to wait at least 63 us after
	 * leaving standby mode. But this doesn't work even after 30 ms.
	 * So probably this should be 63 ms and therefore we wait for 80 ms.
	 */
	msleep(80);

	return 0;
}

static int imx415_stream_on(struct imx415 *sensor)
{
	int ret;

	ret = imx415_wakeup(sensor);
	if (ret)
		return ret;

	return imx415_write(sensor, IMX415_XMSTA, IMX415_XMSTA_START);
}

static int imx415_stream_off(struct imx415 *sensor)
{
	int ret;

	ret = imx415_write(sensor, IMX415_XMSTA, IMX415_XMSTA_STOP);
	if (ret)
		return ret;

	return imx415_write(sensor, IMX415_MODE, IMX415_MODE_STANDBY);
}

static int imx415_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx415 *sensor = to_imx415(sd);
	struct v4l2_subdev_state *state;
	int ret;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (!enable) {
		ret = imx415_stream_off(sensor);

		pm_runtime_mark_last_busy(sensor->dev);
		pm_runtime_put_autosuspend(sensor->dev);

		sensor->streaming = false;

		goto unlock;
	}

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		goto unlock;

	ret = imx415_setup(sensor, state);
	if (ret)
		goto err_pm;

	/*
	 * Set streaming to true to ensure __v4l2_ctrl_handler_setup() will set
	 * the controls. The flag is reset to false further down if an error
	 * occurs.
	 */
	sensor->streaming = true;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret < 0)
		goto err_pm;

	ret = imx415_stream_on(sensor);
	if (ret)
		goto err_pm;

	ret = 0;

unlock:
	v4l2_subdev_unlock_state(state);

	return ret;

err_pm:
	/*
	 * In case of error, turn the power off synchronously as the device
	 * likely has no other chance to recover.
	 */
	pm_runtime_put_sync(sensor->dev);
	sensor->streaming = false;

	goto unlock;
}

static int imx415_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGBRG10_1X10;

	return 0;
}

static int imx415_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_get_pad_format(sd, state, fse->pad);

	if (fse->index > 0 || fse->code != format->code)
		return -EINVAL;

	/*
	 * radxa-zero2pro-camera: a continuous range now, not a single size --
	 * any window between the crop minimum and the full array can be read
	 * out. imx415_set_format() aligns whatever is asked for.
	 */
	fse->min_width = IMX415_CROP_MIN_WIDTH;
	fse->max_width = IMX415_PIXEL_ARRAY_WIDTH;
	fse->min_height = IMX415_CROP_MIN_HEIGHT;
	fse->max_height = IMX415_PIXEL_ARRAY_HEIGHT;
	return 0;
}

static int imx415_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	fmt->format = *v4l2_subdev_get_pad_format(sd, state, fmt->pad);

	return 0;
}

/*
 * radxa-zero2pro-camera: fill in the fixed half of a media bus format. The
 * Bayer order stays SGBRG10 for every crop this driver will accept, because
 * both crop alignments are even (see IMX415_CROP_*_ALIGN) -- an odd left or
 * top would shift the Bayer phase and make this code lie.
 */
static void imx415_fill_format(struct v4l2_mbus_framefmt *format, u32 width,
			       u32 height)
{
	format->width = width;
	format->height = height;
	format->code = MEDIA_BUS_FMT_SGBRG10_1X10;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_DEFAULT;
	format->xfer_func = V4L2_XFER_FUNC_NONE;
}

/*
 * radxa-zero2pro-camera: set_fmt now selects a CENTRED CROP, where mainline
 * only ever had one geometry and copied the requested size through unchecked.
 *
 * Why the crop rides on set_fmt rather than living only behind
 * .set_selection: the consumer on this board is the ISP bridge
 * (isp-module/src/driver/sensor/V4L2_drv.c), which configures the sensor with
 * exactly one call -- v4l2_subdev_call(pad, set_fmt) -- and never touches the
 * selection API. Making set_fmt mean "give me a window this size, centred"
 * means the ROI is reachable by changing two #defines in that bridge, with no
 * new call into the driver. .set_selection is implemented too, for the general
 * case where the caller wants an off-centre window, and it is the canonical
 * interface; set_fmt is the convenience path that happens to be the one this
 * board uses.
 *
 * The alternative -- extra cropped entries in supported_modes[] -- fits this
 * driver badly. supported_modes[] is indexed once at probe by
 * imx415_parse_hw_config(), keyed on (lanes, lane_rate) from the devicetree,
 * and sensor->cur_mode never changes afterwards; two entries sharing a
 * (lanes, lane_rate) pair would be indistinguishable to that lookup. Keeping
 * supported_modes[] meaning "link + timing configuration" and treating the
 * window as an orthogonal runtime property leaves both concepts clean and
 * lets any window compose with any link mode.
 */
static int imx415_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct imx415 *sensor = to_imx415(sd);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	u32 width, height;

	/* Round the request up to the alignment, then into the array. */
	width = clamp_t(u32, ALIGN(fmt->format.width, IMX415_CROP_WIDTH_ALIGN),
			IMX415_CROP_MIN_WIDTH, IMX415_PIXEL_ARRAY_WIDTH);
	height = clamp_t(u32,
			 ALIGN(fmt->format.height, IMX415_CROP_HEIGHT_ALIGN),
			 IMX415_CROP_MIN_HEIGHT, IMX415_PIXEL_ARRAY_HEIGHT);

	format = v4l2_subdev_get_pad_format(sd, state, fmt->pad);
	crop = v4l2_subdev_get_pad_crop(sd, state, fmt->pad);

	crop->width = width;
	crop->height = height;
	crop->left = ALIGN_DOWN((IMX415_PIXEL_ARRAY_WIDTH - width) / 2,
			       IMX415_CROP_LEFT_ALIGN);
	crop->top = ALIGN_DOWN((IMX415_PIXEL_ARRAY_HEIGHT - height) / 2,
			      IMX415_CROP_TOP_ALIGN);

	imx415_fill_format(format, width, height);
	fmt->format = *format;

	/*
	 * Only the ACTIVE format owns the controls; a TRY format is scratch
	 * state. This also keeps imx415_init_cfg() -- which calls in with a
	 * zeroed struct, so which == 0 == V4L2_SUBDEV_FORMAT_TRY -- from
	 * touching controls before v4l2_subdev_init_finalize() has finished.
	 *
	 * The control handler lock and the subdev state lock are the same mutex
	 * in this driver (imx415_subdev_init()), and the state is locked around
	 * every set_fmt path, so the __-prefixed range helpers are correct
	 * here.
	 */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		imx415_update_geometry_ctrls(sensor, width, height);

	return 0;
}

static int imx415_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		/*
		 * radxa-zero2pro-camera: the live window, not the array.
		 * Mainline returned the full array for all three targets
		 * because it could not crop.
		 */
		sel->r = *v4l2_subdev_get_pad_crop(sd, sd_state, sel->pad);
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = IMX415_PIXEL_ARRAY_TOP;
		sel->r.left = IMX415_PIXEL_ARRAY_LEFT;
		sel->r.width = IMX415_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX415_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/*
 * radxa-zero2pro-camera: NEW. Arbitrary (not necessarily centred) window.
 * The format follows the crop 1:1 -- there is no scaler or binner in play
 * here, so the output size is the window size.
 */
static int imx415_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct imx415 *sensor = to_imx415(sd);
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	u32 width, height;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	width = clamp_t(u32, ALIGN(sel->r.width, IMX415_CROP_WIDTH_ALIGN),
			IMX415_CROP_MIN_WIDTH, IMX415_PIXEL_ARRAY_WIDTH);
	height = clamp_t(u32, ALIGN(sel->r.height, IMX415_CROP_HEIGHT_ALIGN),
			 IMX415_CROP_MIN_HEIGHT, IMX415_PIXEL_ARRAY_HEIGHT);

	crop = v4l2_subdev_get_pad_crop(sd, sd_state, sel->pad);

	crop->width = width;
	crop->height = height;
	/*
	 * Clamp the origin so the window stays inside the array, then align it
	 * DOWN -- aligning up could push it back out.
	 */
	crop->left = ALIGN_DOWN(clamp_t(s32, sel->r.left, 0,
					IMX415_PIXEL_ARRAY_WIDTH - (s32)width),
			       IMX415_CROP_LEFT_ALIGN);
	crop->top = ALIGN_DOWN(clamp_t(s32, sel->r.top, 0,
				       IMX415_PIXEL_ARRAY_HEIGHT - (s32)height),
			      IMX415_CROP_TOP_ALIGN);

	format = v4l2_subdev_get_pad_format(sd, sd_state, sel->pad);
	imx415_fill_format(format, width, height);

	sel->r = *crop;

	if (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		imx415_update_geometry_ctrls(sensor, width, height);

	return 0;
}

static int imx415_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.format = {
			.width = IMX415_PIXEL_ARRAY_WIDTH,
			.height = IMX415_PIXEL_ARRAY_HEIGHT,
		},
	};

	/*
	 * .which is 0 == V4L2_SUBDEV_FORMAT_TRY, deliberately: this runs from
	 * v4l2_subdev_init_finalize() and must not reach into the controls.
	 * It still initialises the crop rectangle to the full array, which is
	 * what imx415_set_window() needs to see for the all-pixel path.
	 */
	imx415_set_format(sd, state, &format);

	return 0;
}

static const struct v4l2_subdev_video_ops imx415_subdev_video_ops = {
	.s_stream = imx415_s_stream,
};

static const struct v4l2_subdev_pad_ops imx415_subdev_pad_ops = {
	.enum_mbus_code = imx415_enum_mbus_code,
	.enum_frame_size = imx415_enum_frame_size,
	.get_fmt = imx415_get_format,
	.set_fmt = imx415_set_format,
	.get_selection = imx415_get_selection,
	.set_selection = imx415_set_selection,
	.init_cfg = imx415_init_cfg,
};

static const struct v4l2_subdev_ops imx415_subdev_ops = {
	.video = &imx415_subdev_video_ops,
	.pad = &imx415_subdev_pad_ops,
};

static int imx415_subdev_init(struct imx415 *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int ret;

	v4l2_i2c_subdev_init(&sensor->subdev, client, &imx415_subdev_ops);

	ret = imx415_ctrls_init(sensor);
	if (ret)
		return ret;

	sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->subdev.entity, 1, &sensor->pad);
	if (ret < 0) {
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return ret;
	}

	sensor->subdev.state_lock = sensor->subdev.ctrl_handler->lock;
	v4l2_subdev_init_finalize(&sensor->subdev);

	return 0;
}

static void imx415_subdev_cleanup(struct imx415 *sensor)
{
	media_entity_cleanup(&sensor->subdev.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
}

static int imx415_power_on(struct imx415 *sensor)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret < 0)
		return ret;

	gpiod_set_value_cansleep(sensor->reset, 0);

	udelay(1);

	ret = clk_prepare_enable(sensor->clk);
	if (ret < 0)
		goto err_reset;

	/*
	 * Data sheet states that 20 us are required before communication start,
	 * but this doesn't work in all cases. Use 100 us to be on the safe
	 * side.
	 *
	 * radxa-zero2pro-camera: 100 us is far too short for the Radxa Camera
	 * 4K on this board -- measured from userspace, the module does not
	 * appear on i2c until ~10 ms after XCLR release. 50 ms is used here.
	 * NOTE: this alone does NOT make probe succeed; see README "Where it
	 * stands". The XCLR low-pulse width turned out not to matter.
	 */
	msleep(50);

	return 0;

err_reset:
	gpiod_set_value_cansleep(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
	return ret;
}

static void imx415_power_off(struct imx415 *sensor)
{
	clk_disable_unprepare(sensor->clk);
	gpiod_set_value_cansleep(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
}

static int imx415_identify_model(struct imx415 *sensor)
{
	int model, ret;

	/*
	 * While most registers can be read when the sensor is in standby, this
	 * is not the case of the sensor info register :-(
	 */
	ret = imx415_wakeup(sensor);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to get sensor out of standby\n");

	ret = imx415_read(sensor, IMX415_SENSOR_INFO);
	if (ret < 0) {
		dev_err_probe(sensor->dev, ret,
			      "failed to read sensor information\n");
		goto done;
	}

	model = ret & IMX415_SENSOR_INFO_MASK;

	switch (model) {
	case IMX415_CHIP_ID:
		dev_info(sensor->dev, "Detected IMX415 image sensor\n");
		break;
	default:
		ret = dev_err_probe(sensor->dev, -ENODEV,
				    "invalid device model 0x%04x\n", model);
		goto done;
	}

	ret = 0;

done:
	imx415_write(sensor, IMX415_MODE, IMX415_MODE_STANDBY);
	return ret;
}

static int imx415_check_inck(unsigned long inck, u64 link_frequency)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx415_clk_params); ++i) {
		if ((imx415_clk_params[i].lane_rate == link_frequency * 2) &&
		    imx415_clk_params[i].inck == inck)
			break;
	}

	if (i == ARRAY_SIZE(imx415_clk_params))
		return -EINVAL;
	else
		return 0;
}

static int imx415_parse_hw_config(struct imx415 *sensor)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	u64 lane_rate;
	unsigned long inck;
	unsigned int i, j;
	int ret;

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); ++i)
		sensor->supplies[i].supply = imx415_supply_names[i];

	ret = devm_regulator_bulk_get(sensor->dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(sensor->dev, ret,
				     "failed to get supplies\n");

	sensor->reset = devm_gpiod_get_optional(sensor->dev, "reset",
						GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->reset),
				     "failed to get reset GPIO\n");

	sensor->clk = devm_clk_get(sensor->dev, "inck");
	if (IS_ERR(sensor->clk))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->clk),
				     "failed to get clock\n");

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	switch (bus_cfg.bus.mipi_csi2.num_data_lanes) {
	case 2:
	case 4:
		sensor->num_data_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "invalid number of CSI2 data lanes %d\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "no link frequencies defined");
		goto done_endpoint_free;
	}

	/*
	 * Check if there exists a sensor mode defined for current INCK,
	 * number of lanes and given lane rates.
	 */
	inck = clk_get_rate(sensor->clk);
	for (i = 0; i < bus_cfg.nr_of_link_frequencies; ++i) {
		if (imx415_check_inck(inck, bus_cfg.link_frequencies[i])) {
			dev_dbg(sensor->dev,
				"INCK %lu Hz not supported for this link freq",
				inck);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(supported_modes); ++j) {
			if (sensor->num_data_lanes != supported_modes[j].lanes)
				continue;
			if (bus_cfg.link_frequencies[i] * 2 !=
			    supported_modes[j].lane_rate)
				continue;
			sensor->cur_mode = j;
			break;
		}
		if (j < ARRAY_SIZE(supported_modes))
			break;
	}
	if (i == bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "no valid sensor mode defined\n");
		goto done_endpoint_free;
	}

	lane_rate = supported_modes[sensor->cur_mode].lane_rate;
	for (i = 0; i < ARRAY_SIZE(imx415_clk_params); ++i) {
		if (lane_rate == imx415_clk_params[i].lane_rate &&
		    inck == imx415_clk_params[i].inck) {
			sensor->clk_params = &imx415_clk_params[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(imx415_clk_params)) {
		ret = dev_err_probe(sensor->dev, -EINVAL,
				    "Mode %d not supported\n",
				    sensor->cur_mode);
		goto done_endpoint_free;
	}

	ret = 0;
	dev_dbg(sensor->dev, "clock: %lu Hz, lane_rate: %llu bps, lanes: %d\n",
		inck, lane_rate, sensor->num_data_lanes);

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int imx415_probe(struct i2c_client *client)
{
	struct imx415 *sensor;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = &client->dev;

	ret = imx415_parse_hw_config(sensor);
	if (ret)
		return ret;

	sensor->regmap = devm_regmap_init_i2c(client, &imx415_regmap_config);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	/*
	 * Enable power management. The driver supports runtime PM, but needs to
	 * work when runtime PM is disabled in the kernel. To that end, power
	 * the sensor on manually here, identify it, and fully initialize it.
	 */
	ret = imx415_power_on(sensor);
	if (ret)
		return ret;

	ret = imx415_identify_model(sensor);
	if (ret)
		goto err_power;

	ret = imx415_subdev_init(sensor);
	if (ret)
		goto err_power;

	/*
	 * Enable runtime PM. As the device has been powered manually, mark it
	 * as active, and increase the usage count without resuming the device.
	 */
	pm_runtime_set_active(sensor->dev);
	pm_runtime_get_noresume(sensor->dev);
	pm_runtime_enable(sensor->dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->subdev);
	if (ret < 0)
		goto err_pm;

	/*
	 * Finally, enable autosuspend and decrease the usage count. The device
	 * will get suspended after the autosuspend delay, turning the power
	 * off.
	 */
	pm_runtime_set_autosuspend_delay(sensor->dev, 1000);
	pm_runtime_use_autosuspend(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	return 0;

err_pm:
	pm_runtime_disable(sensor->dev);
	pm_runtime_put_noidle(sensor->dev);
	imx415_subdev_cleanup(sensor);
err_power:
	imx415_power_off(sensor);
	return ret;
}

static void imx415_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx415 *sensor = to_imx415(subdev);

	v4l2_async_unregister_subdev(subdev);

	imx415_subdev_cleanup(sensor);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		imx415_power_off(sensor);
	pm_runtime_set_suspended(sensor->dev);
}

static int imx415_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx415 *sensor = to_imx415(subdev);

	return imx415_power_on(sensor);
}

static int imx415_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx415 *sensor = to_imx415(subdev);

	imx415_power_off(sensor);

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx415_pm_ops, imx415_runtime_suspend,
				 imx415_runtime_resume, NULL);

static const struct of_device_id imx415_of_match[] = {
	{ .compatible = "sony,imx415" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, imx415_of_match);

static struct i2c_driver imx415_driver = {
	.probe_new = imx415_probe,
	.remove = imx415_remove,
	.driver = {
		.name = "imx415",
		.of_match_table = imx415_of_match,
		.pm = pm_ptr(&imx415_pm_ops),
	},
};

module_i2c_driver(imx415_driver);

MODULE_DESCRIPTION("Sony IMX415 image sensor driver");
MODULE_AUTHOR("Gerald Loacker <gerald.loacker@wolfvision.net>");
MODULE_AUTHOR("Michael Riesch <michael.riesch@wolfvision.net>");
MODULE_LICENSE("GPL");
