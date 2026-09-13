// SPDX-License-Identifier: GPL-2.0
/*
 * isp_clkc: out-of-tree clk provider that resurrects the two G12B
 * MIPI-ISP / MIPI-CSI-PHY "composite" clocks that this kernel's built-in
 * amlogic,g12b-clkc driver does not provide:
 *
 *   - cts_mipi_isp_clk_composite      (mux -> div -> gate)
 *   - cts_mipi_csi_phy_clk0_composite (mux -> div -> gate)
 *
 * plus four single-bit MPEG-bus gate clocks associated with the ISP/CSI2
 * block (csi_dig, mipi_isp, csi2_phy0, csi2_phy1). MIPI-DSI (display
 * output) clocks were mainlined into drivers/clk/meson/g12a.c; MIPI-CSI
 * (camera input) clocks were not, so these simply do not exist anywhere
 * in this kernel (verified: neither dt-bindings/clock/g12a-clkc.h nor
 * /sys/kernel/debug/clk/clk_summary mention them).
 *
 * Register layout/bitfields are taken from Amlogic's vendor out-of-tree
 * driver (github.com/khadas/linux, khadas-vims-4.9.y,
 * drivers/amlogic/clk/g12b/g12b.c), which implements these as raw
 * ioremap'd struct clk_mux / clk_divider / clk_gate against a *private*
 * of_iomap() of the whole HHI block. We deliberately do NOT copy that:
 * this driver instead shares the regmap already exposed by the live
 * "amlogic,meson-gx-hhi-sysctrl" simple-mfd/syscon node, via
 * device_node_to_regmap() on our parent device node - see
 * PORT_NOTES.md for why a private ioremap of the same physical range
 * would be redundant (and the locking hazard it would reintroduce), and
 * for the reasoning behind using .prepare/.unprepare instead of
 * .enable/.disable for the gate stages.
 *
 * Binds as a child platform device of the syscon node (sibling of the
 * live "clock-controller" and "power-controller" nodes) - see
 * aux-clk-overlay.dts for exactly how it's attached.
 *
 * #clock-cells = <1>. Output index mapping:
 *   0 = cts_mipi_isp_clk_composite      (mux->div->gate @ HHI_MIPI_ISP_CLK_CNTL, 0x1c0)
 *   1 = cts_mipi_csi_phy_clk0_composite (mux->div->gate @ HHI_MIPI_CSI_PHY_CLK_CNTL, 0x340)
 *   2 = csi_dig    gate (HHI_GCLK_MPEG1 bit 18)
 *   3 = mipi_isp   gate (HHI_GCLK_MPEG2 bit 17)
 *   4 = csi2_phy0  gate (HHI_GCLK_MPEG2 bit 29)
 *   5 = csi2_phy1  gate (HHI_GCLK_MPEG2 bit 28)
 *   6 = gen_clk    composite (mux->div->gate @ HHI_GEN_CLK_CNTL, 0x228) --
 *       this is the sensor MCLK source (GPIOAO_10's "CLK12_24" alt
 *       function per the Zero 2 Pro schematic). Same missing-from-mainline
 *       story as 0/1: dt-bindings/clock/g12a-clkc.h and clk_summary on the
 *       live board both have zero CLKID_GEN_CLK/"gen_clk" hits, confirmed
 *       the same way as the ISP/CSI-PHY clocks were. Unlike 0/1, this
 *       mux's register field is NOT an identity map onto the parent index
 *       (see gen_clk_mux_table below) -- ported from vendor
 *       drivers/amlogic/clk/g12a/g12a.c's g12a_gen_clk_sel/_div/_gate,
 *       which is a *base* G12A clock (not a G12B-only addition like 0-5),
 *       consistent with it being present in the vendor tree's own base
 *       mesong12b.dtsi sensor node (`clocks = <&clkc CLKID_GEN_CLK>`) as a
 *       sibling of the ISP node rather than something isp/adapter/phycsi
 *       themselves reference.
 *
 * Only the clk0 CSI-PHY composite is registered (per the task brief: the
 * isp_module driver this exists to support only references clk0, per
 * mesong12b.dtsi's isp@ff140000 node). The vendor file's clk1 variant
 * (shift 25/16/24 in HHI_MIPI_CSI_PHY_CLK_CNTL) and the clk0/clk1 output
 * selector mux (bit 31) are intentionally not implemented here.
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* Byte offsets into the shared HHI syscon register block. */
#define HHI_GCLK_MPEG1			0x144
#define HHI_GCLK_MPEG2			0x148
#define HHI_MIPI_ISP_CLK_CNTL		0x1c0
#define HHI_GEN_CLK_CNTL		0x228
#define HHI_MIPI_CSI_PHY_CLK_CNTL	0x340

#define NR_CLKS			7
#define CLKID_ISP_COMP		0
#define CLKID_CSI_PHY0_COMP	1
#define CLKID_CSI_DIG_GATE	2
#define CLKID_MIPI_ISP_GATE	3
#define CLKID_CSI2_PHY0_GATE	4
#define CLKID_CSI2_PHY1_GATE	5
#define CLKID_GEN_CLK_COMP	6

/*
 * media_parent_names / media_parent_names_mipi, verified present verbatim
 * in /sys/kernel/debug/clk/clk_summary on the live board (see
 * PORT_NOTES.md) - registered by the base amlogic,g12b-clkc driver, and
 * resolved by the clk core across providers by name, no phandle needed.
 */
static const char * const isp_clkc_parents_isp[] = {
	"xtal", "gp0_pll", "hifi_pll", "fclk_div2p5", "fclk_div3",
	"fclk_div4", "fclk_div5", "fclk_div7",
};

static const char * const isp_clkc_parents_csi_phy[] = {
	"xtal", "gp0_pll", "mpll1", "mpll2", "fclk_div3",
	"fclk_div4", "fclk_div5", "fclk_div7",
};

/*
 * gen_clk's parent list/table, ported from vendor
 * drivers/amlogic/clk/g12a/g12a.c (mux_table_gen_clk /
 * gen_clk_parent_names). Unlike the two composites above, this mux's 5-bit
 * register field is NOT the parent index directly -- gen_clk_table[index]
 * is the raw value written to the field. "gp1_pll" (index 2, raw value 6)
 * was checked against /sys/kernel/debug/clk/clk_summary on the live board
 * and does NOT exist on this SoC (only entry of the 13 that's missing) --
 * left in the list anyway (matching the vendor's own table) since the clk
 * core tolerates a mux listing a parent name that doesn't currently
 * resolve, as long as nothing ever selects that particular index. We only
 * ever select index 0 (xtal) for the sensor MCLK, so this is inert.
 */
static const u32 isp_clkc_gen_clk_table[] = {
	0, 5, 6, 7, 20, 21, 22, 23, 24, 25, 26, 27, 28,
};

static const char * const isp_clkc_parents_gen_clk[] = {
	"xtal", "gp0_pll", "gp1_pll", "hifi_pll", "fclk_div2", "fclk_div3",
	"fclk_div4", "fclk_div5", "fclk_div7", "mpll0", "mpll1", "mpll2",
	"mpll3",
};

struct isp_clkc_mux {
	struct clk_hw hw;
	struct regmap *map;
	u32 reg;
	u8 shift;
	u32 mask;
	/* NULL => register field value IS the parent index (isp/csi_phy0).
	 * Non-NULL => register field value is table[index] (gen_clk).
	 */
	const u32 *table;
	u8 num_parents;
};

struct isp_clkc_div {
	struct clk_hw hw;
	struct regmap *map;
	u32 reg;
	u8 shift;
	u8 width;
};

struct isp_clkc_gate {
	struct clk_hw hw;
	struct regmap *map;
	u32 reg;
	u8 bit;
};

#define to_isp_clkc_mux(_hw)  container_of(_hw, struct isp_clkc_mux, hw)
#define to_isp_clkc_div(_hw)  container_of(_hw, struct isp_clkc_div, hw)
#define to_isp_clkc_gate(_hw) container_of(_hw, struct isp_clkc_gate, hw)

struct isp_clkc_priv {
	struct isp_clkc_mux isp_mux;
	struct isp_clkc_div isp_div;
	struct isp_clkc_gate isp_gate;

	struct isp_clkc_mux csi_phy0_mux;
	struct isp_clkc_div csi_phy0_div;
	struct isp_clkc_gate csi_phy0_gate;

	struct isp_clkc_gate csi_dig_gate;
	struct isp_clkc_gate mipi_isp_gate;
	struct isp_clkc_gate csi2_phy0_gate;
	struct isp_clkc_gate csi2_phy1_gate;

	struct isp_clkc_mux gen_clk_mux;
	struct isp_clkc_div gen_clk_div;
	struct isp_clkc_gate gen_clk_gate;
};

/*
 * --- mux ---
 *
 * .get_parent/.set_parent are only ever called under the clk core's
 * prepare_lock (a mutex - see the "Called with prepare_lock held"
 * contract in linux/clk-provider.h, and __clk_init_parent() for
 * .get_parent specifically), so it's safe to call the syscon's
 * mutex-backed regmap accessors here. For isp/csi_phy0 the 3-bit mux
 * field value IS the parent index directly (table == NULL); gen_clk uses
 * a real remapping table (its register field isn't a dense 0..N-1 index)
 * - see isp_clkc_gen_clk_table.
 */
static u8 isp_clkc_mux_get_parent(struct clk_hw *hw)
{
	struct isp_clkc_mux *mux = to_isp_clkc_mux(hw);
	unsigned int val, field;
	u8 i;

	if (regmap_read(mux->map, mux->reg, &val))
		return 0;

	field = (val >> mux->shift) & mux->mask;

	if (!mux->table)
		return field;

	/* Table-mapped mux (gen_clk): reverse-lookup the raw field value
	 * back to a parent index, same linear scan the core's own
	 * clk_mux_get_parent() does for a .table mux. Falls back to index 0
	 * if the register holds a value not in our table (shouldn't happen
	 * since we only ever write table[] values via set_parent below).
	 */
	for (i = 0; i < mux->num_parents; i++)
		if (mux->table[i] == field)
			return i;

	return 0;
}

static int isp_clkc_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct isp_clkc_mux *mux = to_isp_clkc_mux(hw);
	unsigned int field = mux->table ? mux->table[index] : index;

	/*
	 * regmap_update_bits() does its own locked read-modify-write
	 * against the shared syscon regmap - this is the mechanism that
	 * keeps this field-write safe both against our own sibling
	 * div/gate on the same register word, and against the base
	 * clk-g12b driver's unrelated fields on other shared HHI words
	 * (e.g. the MPEG gate registers below). See PORT_NOTES.md.
	 */
	return regmap_update_bits(mux->map, mux->reg,
				   mux->mask << mux->shift,
				   field << mux->shift);
}

/*
 * .determine_rate is required, not optional, because we register each
 * vendor "composite" as three separate clks (mux -> div -> gate) rather
 * than one clk_register_composite(). In the vendor's single-clk form,
 * clk_composite_determine_rate() walks the mux's parents internally to
 * find one that can supply the requested rate. Splitting the stages
 * loses that for free, so it has to be rebuilt explicitly: the divider
 * (which carries CLK_SET_RATE_PARENT) asks its parent - this mux - to
 * round, and without a .determine_rate here clk_core_can_round() fails
 * on the mux and the whole set_rate silently leaves the rate alone.
 *
 * __clk_mux_determine_rate is the generic core helper (the same one
 * in-tree clk_mux_ops installs); it works against any clk_hw - it only
 * uses clk_hw_get_parent_by_index() - so it does not care that our mux
 * is regmap-backed rather than a struct clk_mux. It picks the parent
 * giving the highest rate <= the request, which is what we want: a
 * camera pipeline should never be clocked faster than asked for.
 *
 * This matters concretely: iv009_isp asks for 666666667 Hz, which is
 * only reachable by reparenting to fclk_div3 (2000/3 MHz, divider 1).
 * The MIPI clock asks for 200000000 Hz -> fclk_div5 (400 MHz) / 2.
 * Both are impossible from the xtal (24 MHz) the mux powers up on.
 */
static const struct clk_ops isp_clkc_mux_ops = {
	.get_parent = isp_clkc_mux_get_parent,
	.set_parent = isp_clkc_mux_set_parent,
	.determine_rate = __clk_mux_determine_rate,
};

/*
 * --- divider ---
 *
 * .recalc_rate/.round_rate/.set_rate all run under prepare_lock too, so
 * again safe for the mutex-backed regmap. Math is delegated to the same
 * exported helpers the core clk_divider_ops uses internally, rather than
 * hand-rolled, so behaviour matches a "real" 7-bit, zero-based divider
 * exactly (no CLK_DIVIDER_* flags set, matching the vendor's plain
 * clk_divider_ops usage with flags = 0).
 */
static unsigned long isp_clkc_div_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct isp_clkc_div *div = to_isp_clkc_div(hw);
	unsigned int val;

	if (regmap_read(div->map, div->reg, &val))
		return 0;

	val = (val >> div->shift) & clk_div_mask(div->width);

	return divider_recalc_rate(hw, parent_rate, val, NULL, 0, div->width);
}

static long isp_clkc_div_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	struct isp_clkc_div *div = to_isp_clkc_div(hw);

	return divider_round_rate_parent(hw, clk_hw_get_parent(hw), rate,
					  parent_rate, NULL, div->width, 0);
}

static int isp_clkc_div_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct isp_clkc_div *div = to_isp_clkc_div(hw);
	int val;

	val = divider_get_val(rate, parent_rate, NULL, div->width, 0);
	if (val < 0)
		return val;

	return regmap_update_bits(div->map, div->reg,
				   clk_div_mask(div->width) << div->shift,
				   (unsigned int)val << div->shift);
}

static const struct clk_ops isp_clkc_div_ops = {
	.recalc_rate = isp_clkc_div_recalc_rate,
	.round_rate = isp_clkc_div_round_rate,
	.set_rate = isp_clkc_div_set_rate,
};

/*
 * --- gate ---
 *
 * Deliberately implemented via .prepare/.unprepare/.is_prepared, NOT
 * .enable/.disable/.is_enabled.
 *
 * The clk core calls .enable/.disable with enable_lock (a spinlock)
 * held and documents "This function must not sleep"; .is_enabled has
 * the same "must not sleep" contract. The regmap we get from
 * device_node_to_regmap() is a plain MMIO regmap created by the syscon
 * core (drivers/mfd/syscon.c's syscon_regmap_config has no
 * .fast_io), so it defaults to mutex-based locking internally -
 * regmap_read()/regmap_update_bits() on it CAN sleep. Calling them from
 * .enable/.disable would be "sleeping while atomic" the moment this
 * runs under the spinlock.
 *
 * .prepare/.unprepare/.is_prepared, by contrast, are documented as
 * running under prepare_lock (a mutex) and explicitly "allowed to
 * sleep". Doing all the hardware work here instead is the officially
 * sanctioned split (see the big comment block at the end of `struct
 * clk_ops` in linux/clk-provider.h: "Clock enable code that will never
 * be called in a sleepable context may be implemented in clk_enable" -
 * implying the converse is equally fine, and is exactly what several
 * in-tree "slow" gate drivers, e.g. regulator- or I2C-backed clocks, do
 * for the same reason). Leaving .enable/.disable NULL means the clk
 * core treats "prepared" as "enabled" with no separate atomic step,
 * which is correct here since there is no genuinely atomic-only part of
 * gating this bit.
 *
 * Practical consequence for consumers (the ISP/CSI2 driver on the other
 * side of this): they must call clk_prepare_enable() (or prepare() then
 * enable()), not a bare clk_enable() without a preceding clk_prepare().
 * This is already virtually universal practice for clk consumers, so in
 * practice this is a non-issue, but it's worth flagging explicitly.
 */
static int isp_clkc_gate_prepare(struct clk_hw *hw)
{
	struct isp_clkc_gate *gate = to_isp_clkc_gate(hw);

	return regmap_update_bits(gate->map, gate->reg,
				   BIT(gate->bit), BIT(gate->bit));
}

static void isp_clkc_gate_unprepare(struct clk_hw *hw)
{
	struct isp_clkc_gate *gate = to_isp_clkc_gate(hw);

	regmap_update_bits(gate->map, gate->reg, BIT(gate->bit), 0);
}

static int isp_clkc_gate_is_prepared(struct clk_hw *hw)
{
	struct isp_clkc_gate *gate = to_isp_clkc_gate(hw);
	unsigned int val;

	if (regmap_read(gate->map, gate->reg, &val))
		return 0;

	return !!(val & BIT(gate->bit));
}

static const struct clk_ops isp_clkc_gate_ops = {
	.prepare = isp_clkc_gate_prepare,
	.unprepare = isp_clkc_gate_unprepare,
	.is_prepared = isp_clkc_gate_is_prepared,
};

/* --- registration helpers --- */

static int isp_clkc_register_mux(struct device *dev, struct regmap *map,
				  struct isp_clkc_mux *mux, const char *name,
				  u32 reg, u8 shift, u32 mask,
				  const u32 *table,
				  const char * const *parent_names,
				  u8 num_parents)
{
	struct clk_init_data init = {};

	mux->map = map;
	mux->reg = reg;
	mux->shift = shift;
	mux->mask = mask;
	mux->table = table;
	mux->num_parents = num_parents;

	init.name = name;
	init.ops = &isp_clkc_mux_ops;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	init.flags = 0;

	mux->hw.init = &init;

	/* devm_clk_hw_register() copies what it needs out of init/name
	 * arrays before returning - see clk-provider.h: "This pointer
	 * will be set to NULL once a clk_register() variant is called",
	 * so it's fine that `init` is a stack local here. parent_names
	 * itself must stay alive forever, which is why the two arrays
	 * above are static file-scope const, not stack-built.
	 */
	return devm_clk_hw_register(dev, &mux->hw);
}

static int isp_clkc_register_div(struct device *dev, struct regmap *map,
				  struct isp_clkc_div *div, const char *name,
				  const char *parent_name,
				  u32 reg, u8 shift, u8 width,
				  unsigned long flags)
{
	struct clk_init_data init = {};

	div->map = map;
	div->reg = reg;
	div->shift = shift;
	div->width = width;

	init.name = name;
	init.ops = &isp_clkc_div_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	div->hw.init = &init;

	return devm_clk_hw_register(dev, &div->hw);
}

static int isp_clkc_register_gate(struct device *dev, struct regmap *map,
				   struct isp_clkc_gate *gate, const char *name,
				   const char *parent_name,
				   u32 reg, u8 bit, unsigned long flags)
{
	struct clk_init_data init = {};

	gate->map = map;
	gate->reg = reg;
	gate->bit = bit;

	init.name = name;
	init.ops = &isp_clkc_gate_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = flags;

	gate->hw.init = &init;

	return devm_clk_hw_register(dev, &gate->hw);
}

static int isp_clkc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct isp_clkc_priv *priv;
	struct clk_hw_onecell_data *onecell;
	struct regmap *map;
	int ret;

	if (!dev->parent || !dev->parent->of_node) {
		dev_err(dev, "no parent syscon device/of_node\n");
		return -ENODEV;
	}

	/*
	 * Shared regmap for the whole HHI block, owned by the syscon
	 * core (drivers/mfd/syscon.c) against our parent
	 * "amlogic,meson-gx-hhi-sysctrl" node - the same regmap instance
	 * the live built-in amlogic,g12b-clkc driver uses (confirmed:
	 * this kernel has CONFIG_COMMON_CLK_MESON_REGMAP=y and
	 * CONFIG_COMMON_CLK_MESON_EE_CLKC=y, i.e. it's the mainline
	 * regmap-based meson clk stack, and /sys/kernel/debug/regmap/
	 * shows exactly one regmap for this physical range, named
	 * "dummy-system-controller@0x00000000ff63c000" - see
	 * PORT_NOTES.md). We must NOT ioremap this range ourselves.
	 */
	map = device_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(map)) {
		dev_err(dev, "failed to get parent syscon regmap: %ld\n",
			PTR_ERR(map));
		return PTR_ERR(map);
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	onecell = devm_kzalloc(dev, struct_size(onecell, hws, NR_CLKS),
				GFP_KERNEL);
	if (!onecell)
		return -ENOMEM;
	onecell->num = NR_CLKS;

	/* cts_mipi_isp_clk_composite: mux -> div -> gate @ 0x1c0 */
	ret = isp_clkc_register_mux(dev, map, &priv->isp_mux,
				     "cts_mipi_isp_clk_mux",
				     HHI_MIPI_ISP_CLK_CNTL, 9, 0x7, NULL,
				     isp_clkc_parents_isp,
				     ARRAY_SIZE(isp_clkc_parents_isp));
	if (ret)
		return dev_err_probe(dev, ret, "isp mux register failed\n");

	ret = isp_clkc_register_div(dev, map, &priv->isp_div,
				     "cts_mipi_isp_clk_div",
				     "cts_mipi_isp_clk_mux",
				     HHI_MIPI_ISP_CLK_CNTL, 0, 7,
				     CLK_SET_RATE_PARENT);
	if (ret)
		return dev_err_probe(dev, ret, "isp div register failed\n");

	/*
	 * CLK_SET_RATE_PARENT on both the gate and the divider above is
	 * NOT what the vendor sets on its cts_mipi_isp_clk_gate, and that
	 * difference is deliberate. The vendor builds one clk via
	 * clk_register_composite(), where set_rate lands on the divider
	 * stage directly; its flags describe a clk that already contains
	 * the divider. Here the gate is its own clk with no .set_rate and
	 * no .determine_rate, so without CLK_SET_RATE_PARENT the core's
	 * clk_core_can_round() gives up at the gate and clk_set_rate()
	 * returns 0 having changed nothing - the ISP would silently run at
	 * the 24 MHz xtal default instead of 666 MHz. Copying the vendor
	 * flag verbatim into a split topology was a bug; these two flags
	 * reconstruct the linkage clk_register_composite() gave implicitly.
	 *
	 * CLK_GET_RATE_NOCACHE is dropped as it was always a no-op here:
	 * isp_clkc_div_recalc_rate() does a live regmap_read every time and
	 * never caches.
	 */
	ret = isp_clkc_register_gate(dev, map, &priv->isp_gate,
				      "cts_mipi_isp_clk_composite",
				      "cts_mipi_isp_clk_div",
				      HHI_MIPI_ISP_CLK_CNTL, 8,
				      CLK_SET_RATE_PARENT);
	if (ret)
		return dev_err_probe(dev, ret, "isp gate register failed\n");

	onecell->hws[CLKID_ISP_COMP] = &priv->isp_gate.hw;

	/* cts_mipi_csi_phy_clk0_composite: mux -> div -> gate @ 0x340 */
	ret = isp_clkc_register_mux(dev, map, &priv->csi_phy0_mux,
				     "cts_mipi_csi_phy_clk0_mux",
				     HHI_MIPI_CSI_PHY_CLK_CNTL, 9, 0x7, NULL,
				     isp_clkc_parents_csi_phy,
				     ARRAY_SIZE(isp_clkc_parents_csi_phy));
	if (ret)
		return dev_err_probe(dev, ret, "csi_phy0 mux register failed\n");

	/* CLK_SET_RATE_PARENT on div+gate for the same reason as the ISP
	 * composite above - see that comment. Target here is 200 MHz.
	 */
	ret = isp_clkc_register_div(dev, map, &priv->csi_phy0_div,
				     "cts_mipi_csi_phy_clk0_div",
				     "cts_mipi_csi_phy_clk0_mux",
				     HHI_MIPI_CSI_PHY_CLK_CNTL, 0, 7,
				     CLK_SET_RATE_PARENT);
	if (ret)
		return dev_err_probe(dev, ret, "csi_phy0 div register failed\n");

	ret = isp_clkc_register_gate(dev, map, &priv->csi_phy0_gate,
				      "cts_mipi_csi_phy_clk0_composite",
				      "cts_mipi_csi_phy_clk0_div",
				      HHI_MIPI_CSI_PHY_CLK_CNTL, 8,
				      CLK_SET_RATE_PARENT);
	if (ret)
		return dev_err_probe(dev, ret, "csi_phy0 gate register failed\n");

	onecell->hws[CLKID_CSI_PHY0_COMP] = &priv->csi_phy0_gate.hw;

	/* Simple MPEG-bus gates, parent "clk81" (matches vendor MESON_GATE(),
	 * confirmed present in clk_summary on the live board). Flags match
	 * MESON_GATE()'s exactly: CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED
	 * (the composite terminal gates above intentionally do NOT get
	 * these - the vendor's cts_mipi_isp_clk_gate/cts_mipi_csi_phy_clk0_gate
	 * only set CLK_GET_RATE_NOCACHE, see above).
	 */
	ret = isp_clkc_register_gate(dev, map, &priv->csi_dig_gate,
				      "csi_dig", "clk81",
				      HHI_GCLK_MPEG1, 18,
				      CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED);
	if (ret)
		return dev_err_probe(dev, ret, "csi_dig gate register failed\n");
	onecell->hws[CLKID_CSI_DIG_GATE] = &priv->csi_dig_gate.hw;

	ret = isp_clkc_register_gate(dev, map, &priv->mipi_isp_gate,
				      "mipi_isp", "clk81",
				      HHI_GCLK_MPEG2, 17,
				      CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED);
	if (ret)
		return dev_err_probe(dev, ret, "mipi_isp gate register failed\n");
	onecell->hws[CLKID_MIPI_ISP_GATE] = &priv->mipi_isp_gate.hw;

	ret = isp_clkc_register_gate(dev, map, &priv->csi2_phy0_gate,
				      "csi2_phy0", "clk81",
				      HHI_GCLK_MPEG2, 29,
				      CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED);
	if (ret)
		return dev_err_probe(dev, ret, "csi2_phy0 gate register failed\n");
	onecell->hws[CLKID_CSI2_PHY0_GATE] = &priv->csi2_phy0_gate.hw;

	ret = isp_clkc_register_gate(dev, map, &priv->csi2_phy1_gate,
				      "csi2_phy1", "clk81",
				      HHI_GCLK_MPEG2, 28,
				      CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED);
	if (ret)
		return dev_err_probe(dev, ret, "csi2_phy1 gate register failed\n");
	onecell->hws[CLKID_CSI2_PHY1_GATE] = &priv->csi2_phy1_gate.hw;

	/* gen_clk: mux(table) -> div -> gate @ HHI_GEN_CLK_CNTL, 0x228.
	 * Sensor MCLK source. Flags match vendor g12a_gen_clk_{sel,div,gate}
	 * exactly: CLK_SET_RATE_PARENT throughout, no CLK_IGNORE_UNUSED (the
	 * vendor doesn't set it here - unlike the MESON_GATE() simple gates
	 * above, this one predates/isn't generated by that macro).
	 */
	ret = isp_clkc_register_mux(dev, map, &priv->gen_clk_mux,
				     "gen_clk_sel",
				     HHI_GEN_CLK_CNTL, 12, 0x1f,
				     isp_clkc_gen_clk_table,
				     isp_clkc_parents_gen_clk,
				     ARRAY_SIZE(isp_clkc_parents_gen_clk));
	if (ret)
		return dev_err_probe(dev, ret, "gen_clk mux register failed\n");

	ret = isp_clkc_register_div(dev, map, &priv->gen_clk_div,
				     "gen_clk_div", "gen_clk_sel",
				     HHI_GEN_CLK_CNTL, 0, 11,
				     CLK_SET_RATE_PARENT);
	if (ret)
		return dev_err_probe(dev, ret, "gen_clk div register failed\n");

	ret = isp_clkc_register_gate(dev, map, &priv->gen_clk_gate,
				      "gen_clk", "gen_clk_div",
				      HHI_GEN_CLK_CNTL, 11,
				      CLK_SET_RATE_PARENT);
	if (ret)
		return dev_err_probe(dev, ret, "gen_clk gate register failed\n");

	onecell->hws[CLKID_GEN_CLK_COMP] = &priv->gen_clk_gate.hw;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, onecell);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register clk provider\n");

	platform_set_drvdata(pdev, priv);
	dev_info(dev, "registered %d aux ISP/CSI clocks via shared HHI syscon regmap\n",
		 NR_CLKS);
	return 0;
}

static const struct of_device_id isp_clkc_of_match[] = {
	{ .compatible = "khadas,vim3-isp-clkc" },
	{ }
};
MODULE_DEVICE_TABLE(of, isp_clkc_of_match);

static struct platform_driver isp_clkc_driver = {
	.probe = isp_clkc_probe,
	.driver = {
		.name = "isp_clkc",
		.of_match_table = isp_clkc_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(isp_clkc_driver);

MODULE_DESCRIPTION("Out-of-tree G12B MIPI-ISP/CSI-PHY aux clock provider (regmap-backed, syscon-shared)");
MODULE_LICENSE("GPL");
