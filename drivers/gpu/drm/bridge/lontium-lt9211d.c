// SPDX-License-Identifier: GPL-2.0
/*
 * Lontium LT9211D bridge driver (minimal bring-up)
 *
 * LT9211D: MIPI DSI -> dual-channel LVDS.
 * This file provides I2C/regmap/DRM/DSI infrastructure only.
 * MIPI RX, timing, PLL/PCR and LVDS TX setup come later.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#define REG_PAGE_CONTROL	0xff
#define REG_CHIPID0		0x8100
#define REG_CHIPID1		0x8101
#define REG_CHIPID2		0x8102

struct lt9211d {
	struct drm_bridge	bridge;
	struct device		*dev;
	struct regmap		*regmap;
	struct mipi_dsi_device	*dsi;
	struct drm_bridge	*panel_bridge;
	struct gpio_desc	*reset_gpio;
	struct regulator	*vccio;
};

static const struct regmap_range lt9211d_rw_ranges[] = {
	regmap_reg_range(REG_PAGE_CONTROL, REG_PAGE_CONTROL),
	regmap_reg_range(REG_CHIPID0, REG_CHIPID2),
	/* MIPI RX D-PHY reset */
	regmap_reg_range(0x8109, 0x8109),
	/* System clock routing / video check source select */
	regmap_reg_range(0x8180, 0x8181),
	/* MIPI RX PHY power-on */
	regmap_reg_range(0x8201, 0x8209),
	regmap_reg_range(0x8213, 0x8213),
	regmap_reg_range(0x8218, 0x8218),
	/* Active RX source select */
	regmap_reg_range(0x8530, 0x8530),
	/* MIPI RX lane mapping */
	regmap_reg_range(0x853f, 0x8549),
	regmap_reg_range(0x85e9, 0x85e9),
	regmap_reg_range(0x8632, 0x8632),
	regmap_reg_range(0x863f, 0x863f),
	/* MIPI RX PHY lane count / power-on / input select */
	regmap_reg_range(0xd000, 0xd005),
	regmap_reg_range(0xd00a, 0xd00b),
	regmap_reg_range(0xd021, 0xd021),
	/* Video Check reset/latch */
	regmap_reg_range(0x810b, 0x810b),
	/* Frame rate diagnostics */
	regmap_reg_range(0x8643, 0x8645),
	/* Frequency meter */
	regmap_reg_range(0x8690, 0x8690),
	regmap_reg_range(0x8698, 0x869a),
	/* MIPI RX video parameters / SOT diagnostics */
	regmap_reg_range(0xd082, 0xd08f),
	regmap_reg_range(0xd09c, 0xd09c),
};

static const struct regmap_access_table lt9211d_rw_table = {
	.yes_ranges = lt9211d_rw_ranges,
	.n_yes_ranges = ARRAY_SIZE(lt9211d_rw_ranges),
};

static const struct regmap_range_cfg lt9211d_range = {
	.name = "lt9211d",
	.range_min = 0x0000,
	.range_max = 0xd0ff,
	.selector_reg = REG_PAGE_CONTROL,
	.selector_mask = 0xff,
	.selector_shift = 0,
	.window_start = 0,
	.window_len = 0x100,
};

static const struct regmap_config lt9211d_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.rd_table = &lt9211d_rw_table,
	.wr_table = &lt9211d_rw_table,
	.volatile_table = &lt9211d_rw_table,
	.ranges = &lt9211d_range,
	.num_ranges = 1,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0xd0ff,
};

static struct lt9211d *bridge_to_lt9211d(struct drm_bridge *bridge)
{
	return container_of(bridge, struct lt9211d, bridge);
}

/* Vendor: Mod_LT9211D_Reset() */
static void lt9211d_reset(struct lt9211d *ctx)
{
	if (!ctx->reset_gpio)
		return;

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(100);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(100);
}

/* Vendor: Mod_ChipID_Read() — bank 0x81, regs 0x00..0x02 */
static int lt9211d_read_chipid(struct lt9211d *ctx)
{
	u8 chipid[3];
	int ret;

	ret = regmap_bulk_read(ctx->regmap, REG_CHIPID0, chipid, 3);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to read chip ID: %d\n", ret);
		return ret;
	}

	dev_info(ctx->dev, "LT9211D chip ID: %02x %02x %02x\n",
		 chipid[0], chipid[1], chipid[2]);
	return 0;
}

/* Vendor: Drv_MipiRx_PhyPowerOn(), PORTA, non-burst */
static const struct reg_sequence lt9211d_mipi_rx_phy_82_seq[] = {
	{ 0x8201, 0x11 },
	{ 0x8218, 0x48 },
	{ 0x8201, 0x91 },
	{ 0x8202, 0x00 },
	{ 0x8203, 0xee },
	{ 0x8209, 0x21 },
	{ 0x8204, 0x44 },
	{ 0x8205, 0xc4 },
	{ 0x8206, 0x44 },
	{ 0x8213, 0x0c },
};

static const struct reg_sequence lt9211d_mipi_rx_phy_d0_seq[] = {
	{ 0xd001, 0x00 },
	{ 0xd002, 0x0e },
	{ 0xd005, 0x00 },
	{ 0xd00a, 0x59 },
	{ 0xd00b, 0x20 },
};

/* D-PHY reset */
static const struct reg_sequence lt9211d_mipi_rx_phy_reset_seq[] = {
	{ 0x8109, 0xde },
	{ 0x8109, 0xdf },
};

static int lt9211d_mipi_rx_phy_poweron(struct lt9211d *ctx)
{
	unsigned int lane_cfg;
	int ret;

	switch (ctx->dsi->lanes) {
	case 4:
		lane_cfg = 0;
		break;
	case 1:
		lane_cfg = 1;
		break;
	case 2:
		lane_cfg = 2;
		break;
	case 3:
		lane_cfg = 3;
		break;
	default:
		dev_err(ctx->dev, "unsupported DSI lane count: %u\n",
			ctx->dsi->lanes);
		return -EINVAL;
	}

	ret = regmap_update_bits(ctx->regmap, 0xd000, GENMASK(1, 0), lane_cfg);
	if (ret) {
		dev_err(ctx->dev, "failed to set MIPI RX lane count: %d\n",
			ret);
		return ret;
	}

	ret = regmap_multi_reg_write(ctx->regmap, lt9211d_mipi_rx_phy_82_seq,
				     ARRAY_SIZE(lt9211d_mipi_rx_phy_82_seq));
	if (ret) {
		dev_err(ctx->dev, "failed to power on MIPI RX phy: %d\n", ret);
		return ret;
	}

	ret = regmap_multi_reg_write(ctx->regmap, lt9211d_mipi_rx_phy_d0_seq,
				     ARRAY_SIZE(lt9211d_mipi_rx_phy_d0_seq));
	if (ret) {
		dev_err(ctx->dev, "failed to power on MIPI RX phy: %d\n", ret);
		return ret;
	}

	ret = regmap_multi_reg_write(ctx->regmap, lt9211d_mipi_rx_phy_reset_seq,
				     ARRAY_SIZE(lt9211d_mipi_rx_phy_reset_seq));
	if (ret)
		dev_err(ctx->dev, "failed to reset MIPI RX D-PHY: %d\n", ret);

	return ret;
}

/* Vendor: Drv_MipiRxClk_Sel() + Drv_System_VidChkClk_SrcSel(MLRX_BYTE_CLK) */
static const struct reg_sequence lt9211d_mipi_rx_clk_seq[] = {
	{ 0x85e9, 0x88 },
	{ 0x8180, 0x51 },
	{ 0x8181, 0x10 },
	{ 0x8632, 0x03 },
};

static int lt9211d_mipi_rx_clk_sel(struct lt9211d *ctx)
{
	int ret;

	ret = regmap_multi_reg_write(ctx->regmap, lt9211d_mipi_rx_clk_seq,
				     ARRAY_SIZE(lt9211d_mipi_rx_clk_seq));
	if (ret) {
		dev_err(ctx->dev, "failed to select MIPI RX clock: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(ctx->regmap, 0x8180, GENMASK(1, 0), 0x03);
	if (ret)
		dev_err(ctx->dev,
			"failed to select video check clock source: %d\n",
			ret);

	return ret;
}

/* Vendor: Drv_System_VidChk_SrcSel(MIPIDEBUG) */
static int lt9211d_mipi_rx_vidchk_srcsel(struct lt9211d *ctx)
{
	int ret;

	ret = regmap_write(ctx->regmap, 0x863f, 0x05);
	if (ret)
		dev_err(ctx->dev,
			"failed to select video check source: %d\n", ret);

	return ret;
}

/* Vendor: Drv_SystemActRx_Sel(MIPIRX) */
static int lt9211d_mipi_rx_actrx_sel(struct lt9211d *ctx)
{
	int ret;

	ret = regmap_update_bits(ctx->regmap, 0x8530, GENMASK(2, 0), 0x01);
	if (ret) {
		dev_err(ctx->dev, "failed to select active RX source: %d\n",
			ret);
		return ret;
	}

	ret = regmap_update_bits(ctx->regmap, 0x8530, BIT(4), BIT(4));
	if (ret)
		dev_err(ctx->dev, "failed to enable active RX source: %d\n",
			ret);

	return ret;
}

/* Vendor: Drv_MipiRx_InputSel(), DSI input */
static const struct reg_sequence lt9211d_mipi_rx_input_seq[] = {
	{ 0xd004, 0x00 },
	{ 0xd021, 0x46 },
};

static int lt9211d_mipi_rx_input_sel(struct lt9211d *ctx)
{
	int ret;

	ret = regmap_multi_reg_write(ctx->regmap, lt9211d_mipi_rx_input_seq,
				     ARRAY_SIZE(lt9211d_mipi_rx_input_seq));
	if (ret)
		dev_err(ctx->dev, "failed to select MIPI RX input: %d\n", ret);

	return ret;
}

/* Vendor: Drv_MipiRx_LaneSet(), PORTA */
static const struct reg_sequence lt9211d_mipi_rx_lane_seq[] = {
	{ 0x853f, 0x08 },
	{ 0x8540, 0x04 },
	{ 0x8541, 0x03 },
	{ 0x8542, 0x02 },
	{ 0x8543, 0x01 },
	{ 0x8545, 0x04 },
	{ 0x8546, 0x03 },
	{ 0x8547, 0x02 },
	{ 0x8548, 0x01 },
	{ 0x8544, 0x00 },
	{ 0x8549, 0x00 },
};

static int lt9211d_mipi_rx_lane_set(struct lt9211d *ctx)
{
	int ret;

	ret = regmap_multi_reg_write(ctx->regmap, lt9211d_mipi_rx_lane_seq,
				     ARRAY_SIZE(lt9211d_mipi_rx_lane_seq));
	if (ret)
		dev_err(ctx->dev, "failed to set MIPI RX lane mapping: %d\n",
			ret);

	return ret;
}

/* Vendor state: STATE_CHIPRX_WAIT_SOURCE — MIPI RX init only */
static int lt9211d_configure_mipi_rx(struct lt9211d *ctx)
{
	int ret;

	dev_info(ctx->dev, "LT9211D: configure MIPI RX\n");

	ret = lt9211d_mipi_rx_phy_poweron(ctx);
	if (ret)
		return ret;

	ret = lt9211d_mipi_rx_clk_sel(ctx);
	if (ret)
		return ret;

	ret = lt9211d_mipi_rx_vidchk_srcsel(ctx);
	if (ret)
		return ret;

	ret = lt9211d_mipi_rx_actrx_sel(ctx);
	if (ret)
		return ret;

	ret = lt9211d_mipi_rx_input_sel(ctx);
	if (ret)
		return ret;

	return lt9211d_mipi_rx_lane_set(ctx);
}

#define LT9211D_FM_SRC_MLRXA_BYTE_CLK	0x18

/* Vendor: Drv_System_FmClkGet() */
static int lt9211d_fm_clk_get(struct lt9211d *ctx, u8 src, u32 *val)
{
	unsigned int status;
	u8 fm[3];
	int ret, cleanup_ret;

	ret = regmap_write(ctx->regmap, 0x8690, src);
	if (ret)
		return ret;

	msleep(5);

	ret = regmap_read(ctx->regmap, 0x8698, &status);
	if (ret)
		return ret;

	dev_info(ctx->dev, "MIPI RX FM status=0x%02x (%s)\n", status,
		 (status & 0x60) == 0x60 ? "stable" : "unstable");

	ret = regmap_write(ctx->regmap, 0x8690, src | BIT(7));
	if (ret)
		return ret;

	ret = regmap_bulk_read(ctx->regmap, 0x8698, fm, sizeof(fm));
	if (!ret)
		*val = ((fm[0] & 0x0f) << 16) | (fm[1] << 8) | fm[2];

	cleanup_ret = regmap_update_bits(ctx->regmap, 0x8690, BIT(7), 0);

	return ret ? ret : cleanup_ret;
}

/* Vendor: Video Check reset/latch, done before reading frame rate */
static int lt9211d_vidchk_reset(struct lt9211d *ctx)
{
	static const struct reg_sequence seq[] = {
		{ 0x810b, 0x7f },
		{ 0x810b, 0xff },
	};
	int ret;

	ret = regmap_multi_reg_write(ctx->regmap, seq, ARRAY_SIZE(seq));
	if (ret)
		return ret;

	msleep(80);

	return 0;
}

/* Vendor: diagnostics only — word count, format, VACT, PA_LPN, SOT, FM, frame rate */
static void lt9211d_dump_mipi_rx(struct lt9211d *ctx)
{
	u8 vid[5];
	u8 sot[8];
	u8 frt[3];
	unsigned int pa_lpn;
	u16 word_count, vact;
	unsigned int hact;
	u8 fmt;
	u32 byte_clk;
	u32 frametime;
	int ret;

	/* Vendor: Drv_MipiRx_HactGet() — word count / format / VACT */
	ret = regmap_bulk_read(ctx->regmap, 0xd082, vid, sizeof(vid));
	if (ret) {
		dev_warn(ctx->dev, "failed to read MIPI RX video params: %d\n",
			 ret);
		return;
	}

	ret = regmap_read(ctx->regmap, 0xd09c, &pa_lpn);
	if (ret) {
		dev_warn(ctx->dev, "failed to read MIPI RX pa_lpn: %d\n", ret);
		return;
	}

	word_count = (vid[0] << 8) | vid[1];
	fmt = vid[2] & GENMASK(3, 0);
	vact = (vid[3] << 8) | vid[4];
	hact = fmt == 0x0a ? word_count / 3 : 0;

	dev_info(ctx->dev,
		 "MIPI RX: wc=%u fmt=0x%02x hact=%u vact=%u pa_lpn=0x%02x\n",
		 word_count, fmt, hact, vact, pa_lpn);

	/* Vendor: Drv_MipiRx_SotGet() */
	ret = regmap_bulk_read(ctx->regmap, 0xd088, sot, sizeof(sot));
	if (ret) {
		dev_warn(ctx->dev, "failed to read MIPI RX SOT: %d\n", ret);
		return;
	}

	dev_info(ctx->dev,
		 "MIPI RX SOT: num=%02x %02x %02x %02x data=%02x %02x %02x %02x\n",
		 sot[0], sot[2], sot[4], sot[6],
		 sot[1], sot[3], sot[5], sot[7]);

	ret = lt9211d_fm_clk_get(ctx, LT9211D_FM_SRC_MLRXA_BYTE_CLK, &byte_clk);
	if (ret) {
		dev_warn(ctx->dev,
			 "failed to read MIPI RX PortA byte clock: %d\n", ret);
		return;
	}

	dev_info(ctx->dev, "MIPI RX PortA byte clock=%u\n", byte_clk);

	ret = lt9211d_vidchk_reset(ctx);
	if (ret) {
		dev_warn(ctx->dev, "failed to reset video check: %d\n", ret);
		return;
	}

	/* Vendor: Drv_VidChk_FrmRt_Get() */
	ret = regmap_bulk_read(ctx->regmap, 0x8643, frt, sizeof(frt));
	if (ret) {
		dev_warn(ctx->dev, "failed to read MIPI RX frame rate: %d\n",
			 ret);
		return;
	}

	frametime = (frt[0] << 16) | (frt[1] << 8) | frt[2];
	if (frametime)
		dev_info(ctx->dev, "MIPI RX frametime=%u fps=%u\n", frametime,
			 DIV_ROUND_CLOSEST(25000000, frametime));
	else
		dev_warn(ctx->dev, "MIPI RX frametime not available\n");
}

static int lt9211d_bridge_attach(struct drm_bridge *bridge,
				 enum drm_bridge_attach_flags flags)
{
	struct lt9211d *ctx = bridge_to_lt9211d(bridge);

	return drm_bridge_attach(bridge->encoder, ctx->panel_bridge,
				 &ctx->bridge, flags);
}

static void lt9211d_atomic_enable(struct drm_bridge *bridge,
				  struct drm_bridge_state *old_bridge_state)
{
	struct lt9211d *ctx = bridge_to_lt9211d(bridge);
	int ret;

	ret = regulator_enable(ctx->vccio);
	if (ret) {
		dev_err(ctx->dev, "failed to enable vccio: %d\n", ret);
		return;
	}

	lt9211d_reset(ctx);

	if (!ctx->reset_gpio)
		msleep(100);

	ret = lt9211d_read_chipid(ctx);
	if (ret) {
		int err = regulator_disable(ctx->vccio);

		if (err)
			dev_err(ctx->dev,
				"failed to disable vccio after chip ID error: %d\n",
				err);
		return;
	}

	ret = lt9211d_configure_mipi_rx(ctx);
	if (ret) {
		int err;

		dev_err(ctx->dev, "failed to configure MIPI RX: %d\n", ret);

		err = regulator_disable(ctx->vccio);
		if (err)
			dev_err(ctx->dev,
				"failed to disable vccio after MIPI RX config error: %d\n",
				err);
		return;
	}

	dev_info(ctx->dev, "LT9211D MIPI RX configured\n");

	msleep(100);
	lt9211d_dump_mipi_rx(ctx);

	dev_info(ctx->dev, "LT9211D bridge enabled\n");
}

static void lt9211d_atomic_disable(struct drm_bridge *bridge,
				   struct drm_bridge_state *old_bridge_state)
{
	struct lt9211d *ctx = bridge_to_lt9211d(bridge);
	int ret;

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(10);
	}

	ret = regulator_disable(ctx->vccio);
	if (ret)
		dev_err(ctx->dev, "failed to disable vccio: %d\n", ret);

	regcache_mark_dirty(ctx->regmap);
}

#define LT9211D_MAX_INPUT_SEL_FORMATS	1

static u32 *
lt9211d_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
				  struct drm_bridge_state *bridge_state,
				  struct drm_crtc_state *crtc_state,
				  struct drm_connector_state *conn_state,
				  u32 output_fmt,
				  unsigned int *num_input_fmts)
{
	u32 *input_fmts;

	*num_input_fmts = 0;

	input_fmts = kcalloc(LT9211D_MAX_INPUT_SEL_FORMATS, sizeof(*input_fmts),
			     GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	input_fmts[0] = MEDIA_BUS_FMT_RGB888_1X24;
	*num_input_fmts = 1;

	return input_fmts;
}

static const struct drm_bridge_funcs lt9211d_bridge_funcs = {
	.attach = lt9211d_bridge_attach,
	.atomic_enable = lt9211d_atomic_enable,
	.atomic_disable = lt9211d_atomic_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_get_input_bus_fmts = lt9211d_atomic_get_input_bus_fmts,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

static int lt9211d_parse_dt(struct lt9211d *ctx)
{
	struct drm_bridge *panel_bridge;
	struct drm_panel *panel;
	struct device *dev = ctx->dev;
	int ret;

	ctx->vccio = devm_regulator_get(dev, "vccio");
	if (IS_ERR(ctx->vccio))
		return dev_err_probe(dev, PTR_ERR(ctx->vccio),
				     "failed to get supply 'vccio'\n");

	ret = drm_of_find_panel_or_bridge(dev->of_node, 2, 0,
					  &panel, &panel_bridge);
	if (ret < 0)
		return ret;

	if (panel) {
		panel_bridge = devm_drm_panel_bridge_add(dev, panel);
		if (IS_ERR(panel_bridge))
			return PTR_ERR(panel_bridge);
	}

	ctx->panel_bridge = panel_bridge;
	return 0;
}

static int lt9211d_host_attach(struct lt9211d *ctx)
{
	const struct mipi_dsi_device_info info = {
		.type = "lt9211d",
		.channel = 0,
		.node = NULL,
	};
	struct device *dev = ctx->dev;
	struct device_node *host_node;
	struct device_node *endpoint;
	struct mipi_dsi_device *dsi;
	struct mipi_dsi_host *host;
	int dsi_lanes;
	int ret;

	endpoint = of_graph_get_endpoint_by_regs(dev->of_node, 0, -1);
	if (!endpoint)
		return -ENODEV;

	dsi_lanes = drm_of_get_data_lanes_count(endpoint, 1, 4);
	host_node = of_graph_get_remote_port_parent(endpoint);
	host = of_find_mipi_dsi_host_by_node(host_node);
	of_node_put(host_node);
	of_node_put(endpoint);

	if (!host)
		return -EPROBE_DEFER;

	if (dsi_lanes < 0)
		return dsi_lanes;

	dsi = devm_mipi_dsi_device_register_full(dev, host, &info);
	if (IS_ERR(dsi))
		return dev_err_probe(dev, PTR_ERR(dsi),
				     "failed to create dsi device\n");

	ctx->dsi = dsi;
	dsi->lanes = dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_VIDEO_HSE;

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0) {
		dev_err(dev, "failed to attach dsi to host: %d\n", ret);
		return ret;
	}

	return 0;
}

static int lt9211d_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct lt9211d *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return PTR_ERR(ctx->reset_gpio);


	ret = lt9211d_parse_dt(ctx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse DT\n");

	ctx->regmap = devm_regmap_init_i2c(client, &lt9211d_regmap_config);
	if (IS_ERR(ctx->regmap))
		return PTR_ERR(ctx->regmap);

	dev_set_drvdata(dev, ctx);
	i2c_set_clientdata(client, ctx);

	ctx->bridge.funcs = &lt9211d_bridge_funcs;
	ctx->bridge.of_node = dev->of_node;
	drm_bridge_add(&ctx->bridge);

	ret = lt9211d_host_attach(ctx);
	if (ret) {
		drm_bridge_remove(&ctx->bridge);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	dev_info(dev, "LT9211D bridge probed\n");
	return 0;
}

static void lt9211d_remove(struct i2c_client *client)
{
	struct lt9211d *ctx = i2c_get_clientdata(client);

	drm_bridge_remove(&ctx->bridge);
}

static const struct i2c_device_id lt9211d_id[] = {
	{ "lt9211d", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lt9211d_id);

static const struct of_device_id lt9211d_of_match[] = {
	{ .compatible = "lontium,lt9211d" },
	{ }
};
MODULE_DEVICE_TABLE(of, lt9211d_of_match);

static struct i2c_driver lt9211d_driver = {
	.probe = lt9211d_probe,
	.remove = lt9211d_remove,
	.id_table = lt9211d_id,
	.driver = {
		.name = "lt9211d",
		.of_match_table = lt9211d_of_match,
	},
};
module_i2c_driver(lt9211d_driver);

MODULE_AUTHOR("DIASOM");
MODULE_DESCRIPTION("Lontium LT9211D MIPI DSI to LVDS bridge driver");
MODULE_LICENSE("GPL");
