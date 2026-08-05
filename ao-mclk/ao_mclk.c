// SPDX-License-Identifier: GPL-2.0
/*
 * ao_mclk: gives the camera its INCK (master clock).
 *
 * The Radxa Zero 2 Pro wires the CSI connector's CM_MCLK to GPIOAO_10, and the
 * only clock function that ball has is the SoC's CLK12_24 output. From the
 * board schematic (page 9, "CPU I/O"):
 *
 *   CM_MCLK  BF16  GPIOAO_10(AO_CEC_A//AO_CEC_B//PWMAO_D//SPDIF_OUT//
 *                            TDMB_D1//TDMB_DIN1//CLK12_24)
 *
 * Seven alt-functions, so CLK12_24 is mux value 7. Mainline's
 * pinctrl-meson-g12a.c models exactly the first six of these in the same order
 * (cec_ao_a=1, cec_ao_b=2, pwm_ao_d_10=3, spdif_ao_out=4, tdm_ao_b_dout1=5,
 * tdm_ao_b_din1=6) and simply omits CLK12_24 -- there is no pinctrl group for
 * it at all, so it cannot be selected with a pinctrl-0 property from DT. Hence
 * the direct register write below.
 *
 * Two separate things are needed, and doing only the first is why an earlier
 * version of this module appeared to "work" (the sensor ACKed its i2c address)
 * while register access still failed:
 *
 *   1. Route CLK12_24 to the pad     -- AO_RTI_PINMUX_REG1, GPIOAO_10 field.
 *   2. Switch the generator ON       -- HHI_XTAL_DIVN_CNTL. Routing an output
 *                                      that nothing is generating just gives a
 *                                      dead pin.
 *
 * Register locations
 * ------------------
 * Both register files are addressed as (base + offset*4) in the S922X/A311D
 * datasheet, which states for each block: "Each register final address =
 * 0xFF800000 + offset * 4" (AO) and "... = 0xFF63C000 + offset * 4" (HHI).
 *
 *   AO_RTI_PINMUX_REG1  offset 0x06 -> 0xff800018, 4 bits per pin.
 *     GPIOAO_8..11 occupy bits [3:0],[7:4],[11:8],[15:12], so GPIOAO_10 is
 *     bits [11:8]. Cross-checked against the live devicetree: the aobus
 *     pinctrl bank@14 node has reg = <0x14 0x8> for its "mux" range, i.e.
 *     0xff800014 and 0xff800018.
 *
 *   HHI_XTAL_DIVN_CNTL  offset 0x2f -> 0xff63c0bc:
 *     [12] crt_clk25_en
 *     [11] crt_clk24_en     - enables the CLK12_24 output
 *     [10] clk24_div2_en    - 0 => 24 MHz, 1 => 12 MHz
 *     [7:0] clk25_div
 *   The IMX415 accepts only 24 or 27 MHz INCK, so this drives 24 MHz:
 *   crt_clk24_en set, clk24_div2_en cleared.
 *
 * Rollback: both registers are saved on load and restored on unload, so rmmod
 * leaves the hardware exactly as it was found.
 */

#include <linux/io.h>
#include <linux/module.h>

#define AO_PINMUX_REG1		0xff800018
#define GPIOAO_10_SHIFT		8
#define GPIOAO_10_MASK		(0xfu << GPIOAO_10_SHIFT)

#define HHI_XTAL_DIVN_CNTL	0xff63c0bc
#define CRT_CLK24_EN		BIT(11)
#define CLK24_DIV2_EN		BIT(10)

/* CLK12_24 is the 7th alt-function on GPIOAO_10; see the comment above. */
static unsigned int mux = 7;
module_param(mux, uint, 0444);
MODULE_PARM_DESC(mux, "GPIOAO_10 mux value (7 = CLK12_24, the camera MCLK)");

/* 0 => 24 MHz (what the IMX415 wants), 1 => 12 MHz. */
static unsigned int div2;
module_param(div2, uint, 0444);
MODULE_PARM_DESC(div2, "Divide CLK12_24 by 2: 0 = 24 MHz, 1 = 12 MHz");

static void __iomem *pinmux;
static void __iomem *xtaldiv;
static u32 saved_pinmux, saved_xtaldiv;

static int __init ao_mclk_init(void)
{
	u32 val;

	if (mux > 15)
		return -EINVAL;

	pinmux = ioremap(AO_PINMUX_REG1, 4);
	if (!pinmux)
		return -ENOMEM;

	xtaldiv = ioremap(HHI_XTAL_DIVN_CNTL, 4);
	if (!xtaldiv) {
		iounmap(pinmux);
		return -ENOMEM;
	}

	/* Generator first, then route it out, so the pad never briefly
	 * presents a selected-but-dead clock function. */
	saved_xtaldiv = readl(xtaldiv);
	val = saved_xtaldiv | CRT_CLK24_EN;
	if (div2)
		val |= CLK24_DIV2_EN;
	else
		val &= ~CLK24_DIV2_EN;
	writel(val, xtaldiv);

	saved_pinmux = readl(pinmux);
	writel((saved_pinmux & ~GPIOAO_10_MASK) | (mux << GPIOAO_10_SHIFT),
	       pinmux);

	pr_info("ao_mclk: CLK12_24 on at %u MHz (HHI_XTAL_DIVN_CNTL 0x%08x -> 0x%08x)\n",
		div2 ? 12 : 24, saved_xtaldiv, readl(xtaldiv));
	pr_info("ao_mclk: GPIOAO_10 mux %u -> %u (AO_RTI_PINMUX_REG1 0x%08x -> 0x%08x)\n",
		(saved_pinmux & GPIOAO_10_MASK) >> GPIOAO_10_SHIFT, mux,
		saved_pinmux, readl(pinmux));
	return 0;
}

static void __exit ao_mclk_exit(void)
{
	/* Restore only our own fields; anything else that touched these
	 * registers while we were loaded keeps its change. */
	writel((readl(pinmux) & ~GPIOAO_10_MASK) |
	       (saved_pinmux & GPIOAO_10_MASK), pinmux);
	writel((readl(xtaldiv) & ~(CRT_CLK24_EN | CLK24_DIV2_EN)) |
	       (saved_xtaldiv & (CRT_CLK24_EN | CLK24_DIV2_EN)), xtaldiv);

	pr_info("ao_mclk: restored (pinmux 0x%08x, xtal_divn 0x%08x)\n",
		readl(pinmux), readl(xtaldiv));
	iounmap(xtaldiv);
	iounmap(pinmux);
}

module_init(ao_mclk_init);
module_exit(ao_mclk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Enable CLK12_24 and route it to GPIOAO_10 as the camera MCLK");
