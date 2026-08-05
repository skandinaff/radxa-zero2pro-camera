// SPDX-License-Identifier: GPL-2.0
/*
 * ao_mclk: routes the SoC's CLK12_24 output to GPIOAO_10, which is the pin
 * the Radxa Zero 2 Pro wires to the camera connector's CM_MCLK (J5002 pin 21,
 * "CAM0-CLK", ball AU36). Without a clock on that pin the IMX415 never brings
 * up its internal logic and does not ACK on i2c at all -- verified on hardware:
 * with no MCLK, 0x1a is absent from `i2cdetect -y 3` for every combination of
 * the reset / power-down / frame-sync GPIOs.
 *
 * Why this exists as a register poke instead of proper pinctrl:
 * mainline's pinctrl-meson-g12a.c simply has no group for the CLK12_24
 * function. On GPIOAO_10 it defines only cec_ao_a (mux 1), cec_ao_b (mux 2)
 * and tdm_ao_b_din1 (mux 6); the CLK12_24 alt-function that the schematic
 * (page 9: "GPIOAO_10(AO_CEC_A//AO_CEC_B//PWMAO_D//SPDIF_OUT//TDMB_D1//
 * CLK12_24)") shows on this ball is not modelled at all. Since the mux value
 * is therefore not documented anywhere in-tree, `mux` is a module parameter so
 * it can be determined empirically -- see scripts/find-mclk-mux.sh.
 *
 * Register location, derived from the live devicetree rather than assumed:
 *   /soc/bus@ff800000/sys-ctrl@0/pinctrl@14/bank@14
 *      reg = <0x14 0x8>  ("mux"), so the AO mux registers are
 *      0xff800014 (GPIOAO_0..7) and 0xff800018 (GPIOAO_8..11, GPIOE_0..2),
 *      4 bits per pin. GPIOAO_10 is therefore 0xff800018 bits [11:8].
 *   This matches the vendor's AO_RTI_PINMUX_REG1 (register index 6 -> byte
 *   offset 0x18) as a cross-check.
 *
 * Rollback: the previous field value is saved at load and written back at
 * unload, so rmmod leaves the pin exactly as it was found.
 */

#include <linux/io.h>
#include <linux/module.h>

#define AO_PINMUX_REG1		0xff800018
#define GPIOAO_10_SHIFT		8
#define GPIOAO_10_MASK		(0xfu << GPIOAO_10_SHIFT)

/*
 * 3 is CLK12_24, determined empirically with scripts/find-mclk-mux.sh: of all
 * 16 possible field values it is the only one that makes the IMX415 ACK at
 * 0x1a on i2c-3. (It is NOT 6 -- mainline assigns 6 to tdm_ao_b_din1 on this
 * pin, so the schematic's function ordering is not the mux ordering.)
 */
static unsigned int mux = 3;
module_param(mux, uint, 0444);
MODULE_PARM_DESC(mux, "Mux value for GPIOAO_10 (3 = CLK12_24, the camera MCLK)");

static void __iomem *reg;
static u32 saved;

static int __init ao_mclk_init(void)
{
	u32 val;

	if (mux > 15) {
		pr_err("ao_mclk: mux must be 0-15\n");
		return -EINVAL;
	}

	reg = ioremap(AO_PINMUX_REG1, 4);
	if (!reg)
		return -ENOMEM;

	saved = readl(reg);
	val = (saved & ~GPIOAO_10_MASK) | (mux << GPIOAO_10_SHIFT);
	writel(val, reg);

	pr_info("ao_mclk: GPIOAO_10 mux %u -> %u (reg 0x%08x: 0x%08x -> 0x%08x)\n",
		(saved & GPIOAO_10_MASK) >> GPIOAO_10_SHIFT, mux,
		AO_PINMUX_REG1, saved, readl(reg));
	return 0;
}

static void __exit ao_mclk_exit(void)
{
	if (!reg)
		return;

	/* Restore only our field; anything else that touched this register
	 * while we were loaded keeps its change. */
	writel((readl(reg) & ~GPIOAO_10_MASK) | (saved & GPIOAO_10_MASK), reg);
	pr_info("ao_mclk: GPIOAO_10 mux restored (reg now 0x%08x)\n", readl(reg));
	iounmap(reg);
}

module_init(ao_mclk_init);
module_exit(ao_mclk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Route CLK12_24 to GPIOAO_10 for the Zero 2 Pro camera MCLK");
