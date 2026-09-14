// SPDX-License-Identifier: GPL-2.0
/*
 * ao_mclk: gives the camera its INCK (master clock).
 *
 * VIM3 vs Radxa
 * -------------
 * The Radxa Zero 2 Pro wires the CSI connector's CM_MCLK to GPIOAO_10, whose
 * only clock function is the SoC's CLK12_24 output (mux 7).  The Khadas VIM3
 * does NOT share that routing, and an earlier version of this module carried
 * the Radxa pad over unverified -- which left the sensor with no clock at all,
 * so it never ACKed on i2c no matter how reset was driven.
 *
 * The VIM3 vendor DTS (khadas/common_drivers, khadas-vims-5.15.y,
 * arch/arm64/boot/dts/amlogic/kvim3.dts) drives the camera clock as:
 *
 *   pinctrl-0   = <&gen_clk_ee_ao>;
 *   clocks      = <&clkc CLKID_GEN>;
 *   clock-names = "gen_clk";
 *
 * and that pinmux group is defined in the vendor pinctrl driver
 * (drivers/gpio/pinctrl/pinctrl-meson-g12a.c) as:
 *
 *   static const unsigned int gen_clk_ee_ao_pins[] = { GPIOAO_11 };
 *   GROUP(gen_clk_ee_ao, 4),
 *
 * So on VIM3 the camera MCLK is GPIOAO_11 at mux 4, fed by the SoC's "gen_clk"
 * generator -- a different pad AND a different clock source than on Radxa.
 *
 * Mainline models neither: pinctrl-meson-g12a.c has no gen_clk_ee_ao group, and
 * drivers/clk/meson/g12a.c does not register gen_clk at all (no CLKID_GEN in
 * the g12a-clkc.h binding).  Neither a pinctrl-0 property nor a clocks phandle
 * can reach this hardware from DT, hence the direct register writes below.
 *
 * Register locations
 * ------------------
 * Both register files are addressed as (base + offset*4) in the S922X/A311D
 * datasheet, which states for each block: "Each register final address =
 * 0xFF800000 + offset * 4" (AO) and "... = 0xFF63C000 + offset * 4" (HHI).
 *
 *   AO_RTI_PINMUX_REG1  offset 0x06 -> 0xff800018, 4 bits per pin.
 *     GPIOAO_8..11 occupy bits [3:0],[7:4],[11:8],[15:12], so GPIOAO_11 is
 *     bits [15:12] and GPIOAO_10 is bits [11:8].  Cross-checked against the
 *     live devicetree: the aobus pinctrl bank@14 node has reg = <0x14 0x8> for
 *     its "mux" range, i.e. 0xff800014 and 0xff800018.
 *
 *   HHI_GEN_CLK_CNTL    offset 0x8a -> 0xff63c228.  The layout is NOT the same
 *     across the family, which is an easy and costly mistake here: gxbb and axg
 *     gate this clock at bit 7, but G12A/G12B gate it at bit 11 and widen the
 *     parent select to 5 bits.  See the G12A field definitions next to
 *     GEN_CLK_EN below.  Selecting the xtal parent with a divider of 0 yields
 *     exactly 24 MHz, which is what the IMX415 wants for INCK.
 *
 *   HHI_XTAL_DIVN_CNTL  offset 0x2f -> 0xff63c0bc (the Radxa/CLK12_24 path):
 *     [11] crt_clk24_en     - enables the CLK12_24 output
 *     [10] clk24_div2_en    - 0 => 24 MHz, 1 => 12 MHz
 *
 * Both the pinmux field and the clock register are saved on load and restored
 * on unload, so rmmod leaves the hardware exactly as it was found.
 */

#include <linux/io.h>
#include <linux/module.h>

#define AO_PINMUX_REG1		0xff800018
#define PAD_FIELD_SHIFT(pad)	(((pad) - 8) * 4)	/* GPIOAO_8..11 */
#define PAD_FIELD_MASK(pad)	(0xfu << PAD_FIELD_SHIFT(pad))

/*
 * G12A/G12B layout -- deliberately NOT the gxbb/axg one.  The older SoCs put
 * the gate at bit 7, which on G12 lands inside the divider field instead (an
 * earlier version of this module did exactly that: it selected a /129 divide
 * and never opened the gate, so the pad stayed dead).  Per the vendor G12A
 * clock driver, drivers/clk/meson/g12a.c:
 *   g12a_gen_mux: .mask = 0x1f, .shift = 12   -> sel is bits [16:12]
 *   g12a_gen_div: .shift = 0, .width = 11     -> div is bits [10:0]
 *   g12a_gen:     .bit_idx = 11               -> gate is bit 11
 * and g12a_gen_mux_table[0] == 0 selects the "xtal" parent, i.e. 24 MHz.
 */
#define HHI_GEN_CLK_CNTL	0xff63c228
#define GEN_CLK_EN		BIT(11)
#define GEN_CLK_SEL_SHIFT	12
#define GEN_CLK_SEL_MASK	(0x1fu << GEN_CLK_SEL_SHIFT)
#define GEN_CLK_SEL_XTAL	0u
#define GEN_CLK_DIV_MASK	0x7ffu
#define GEN_CLK_OURS		(GEN_CLK_EN | GEN_CLK_SEL_MASK | GEN_CLK_DIV_MASK)

#define HHI_XTAL_DIVN_CNTL	0xff63c0bc
#define CRT_CLK24_EN		BIT(11)
#define CLK24_DIV2_EN		BIT(10)
#define XTAL_DIVN_OURS		(CRT_CLK24_EN | CLK24_DIV2_EN)

/*
 * Default to the VIM3 routing.  Set source=clk12_24 (with pad=10 mux=7) to get
 * the old Radxa behaviour back for an A/B comparison on the bench.
 */
static char *source = "gen";
module_param(source, charp, 0444);
MODULE_PARM_DESC(source, "Clock source: 'gen' (VIM3, gen_clk) or 'clk12_24' (Radxa)");

static unsigned int pad = 11;
module_param(pad, uint, 0444);
MODULE_PARM_DESC(pad, "GPIOAO pad carrying MCLK (VIM3: 11, Radxa: 10)");

static unsigned int mux = 4;
module_param(mux, uint, 0444);
MODULE_PARM_DESC(mux, "Pad mux value (VIM3 gen_clk_ee_ao: 4, Radxa CLK12_24: 7)");

/* gen_clk divider: output = 24 MHz / (div + 1). 0 => 24 MHz. */
static unsigned int div;
module_param(div, uint, 0444);
MODULE_PARM_DESC(div, "gen_clk divider; output = 24 MHz / (div + 1)");

/* CLK12_24 path only: 0 => 24 MHz, 1 => 12 MHz. */
static unsigned int div2;
module_param(div2, uint, 0444);
MODULE_PARM_DESC(div2, "clk12_24 path: divide by 2 (0 = 24 MHz, 1 = 12 MHz)");

static void __iomem *pinmux;
static void __iomem *clkreg;
static u32 saved_pinmux, saved_clkreg;
static bool use_gen;

static int __init ao_mclk_init(void)
{
	u32 val;

	if (mux > 15 || pad < 8 || pad > 11)
		return -EINVAL;

	use_gen = strcmp(source, "clk12_24") != 0;
	if (use_gen && div > GEN_CLK_DIV_MASK)
		return -EINVAL;

	pinmux = ioremap(AO_PINMUX_REG1, 4);
	if (!pinmux)
		return -ENOMEM;

	clkreg = ioremap(use_gen ? HHI_GEN_CLK_CNTL : HHI_XTAL_DIVN_CNTL, 4);
	if (!clkreg) {
		iounmap(pinmux);
		return -ENOMEM;
	}

	/* Generator first, then route it out, so the pad never briefly
	 * presents a selected-but-dead clock function. */
	saved_clkreg = readl(clkreg);
	if (use_gen) {
		/* xtal parent, requested divider, gate on. */
		val = saved_clkreg & ~(GEN_CLK_SEL_MASK | GEN_CLK_DIV_MASK);
		val |= (GEN_CLK_SEL_XTAL << GEN_CLK_SEL_SHIFT) | div |
		       GEN_CLK_EN;
	} else {
		val = saved_clkreg | CRT_CLK24_EN;
		if (div2)
			val |= CLK24_DIV2_EN;
		else
			val &= ~CLK24_DIV2_EN;
	}
	writel(val, clkreg);

	saved_pinmux = readl(pinmux);
	writel((saved_pinmux & ~PAD_FIELD_MASK(pad)) |
	       (mux << PAD_FIELD_SHIFT(pad)), pinmux);

	pr_info("ao_mclk: %s on at %u MHz (%s 0x%08x -> 0x%08x)\n",
		use_gen ? "gen_clk" : "CLK12_24",
		use_gen ? 24 / (div + 1) : (div2 ? 12 : 24),
		use_gen ? "HHI_GEN_CLK_CNTL" : "HHI_XTAL_DIVN_CNTL",
		saved_clkreg, readl(clkreg));
	pr_info("ao_mclk: GPIOAO_%u mux %u -> %u (AO_RTI_PINMUX_REG1 0x%08x -> 0x%08x)\n",
		pad, (saved_pinmux & PAD_FIELD_MASK(pad)) >> PAD_FIELD_SHIFT(pad),
		mux, saved_pinmux, readl(pinmux));
	return 0;
}

static void __exit ao_mclk_exit(void)
{
	u32 ours = use_gen ? GEN_CLK_OURS : XTAL_DIVN_OURS;

	/* Restore only our own fields; anything else that touched these
	 * registers while we were loaded keeps its change. */
	writel((readl(pinmux) & ~PAD_FIELD_MASK(pad)) |
	       (saved_pinmux & PAD_FIELD_MASK(pad)), pinmux);
	writel((readl(clkreg) & ~ours) | (saved_clkreg & ours), clkreg);

	pr_info("ao_mclk: restored (pinmux 0x%08x, clk 0x%08x)\n",
		readl(pinmux), readl(clkreg));
	iounmap(clkreg);
	iounmap(pinmux);
}

module_init(ao_mclk_init);
module_exit(ao_mclk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Route a 24 MHz camera MCLK to the VIM3 CSI connector");
