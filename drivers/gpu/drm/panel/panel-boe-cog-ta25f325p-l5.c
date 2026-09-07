// SPDX-License-Identifier: GPL-2.0+
/*
 * BOE COG-TA25F325P-L5 dual-port LVDS panel driver
 *
 * 1920x720, VESA 24-bit, DE mode.
 *
 * Unlike the generic panel-lvds driver, this panel requires a specific
 * RESET / STBYB / VDD power sequence (see VL-PS-COG-TA25F325P-L5 REV.A,
 * section 5.5 "Power On / Off Sequence", Figure 11), so it cannot be
 * driven by panel-lvds.c and needs a dedicated driver.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

/*
 * Minimum delays from Figure 11 (Power on/off sequence),
 * VL-PS-COG-TA25F325P-L5 REV.A, section 5.5.
 *
 * Power on:
 *   VDD valid       -> RESET HIGH              >= 10us (datasheet minimum)
 *   RESET HIGH      -> STBYB HIGH               >= 20ms (+T, T=0, SPI unused)
 *   STBYB HIGH      -> LVDS DATA                >= 140ms
 *   LVDS DATA       -> backlight ON              >= 40ms
 *
 * Power off (mirrors the above):
 *   backlight OFF   -> STBYB LOW                >= 150ms
 *   STBYB LOW       -> RESET LOW                 >= 150ms
 *   RESET LOW       -> VDD OFF                    >= 16ms
 *   VDD OFF         -> next power on               >= 2s
 */
#define BOE_COG_T_POWER_ON_SETTLE_US	10000
#define BOE_COG_T_RESET_TO_STBYB_MS	20
#define BOE_COG_T_STBYB_TO_DATA_MS	140
#define BOE_COG_T_DATA_TO_BL_MS		40

#define BOE_COG_T_BL_TO_STBYB_OFF_MS	150
#define BOE_COG_T_STBYB_TO_RESET_OFF_MS	150
#define BOE_COG_T_RESET_TO_VDD_OFF_MS	16
#define BOE_COG_T_POWER_OFF_MS		2000

struct boe_cog_panel {
	struct drm_panel panel;
	struct device *dev;

	struct regulator *supply;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;

	struct drm_display_mode dmode;
	u32 bus_flags;
	unsigned int bus_format;

	/* Timestamp of the last unprepare(), for the >2s power-off rule. */
	ktime_t last_unprepared;
};

static inline struct boe_cog_panel *to_boe_cog_panel(struct drm_panel *panel)
{
	return container_of(panel, struct boe_cog_panel, panel);
}

static int boe_cog_panel_prepare(struct drm_panel *panel)
{
	struct boe_cog_panel *boe = to_boe_cog_panel(panel);
	int ret;

	if (boe->last_unprepared) {
		s64 elapsed_ms = ktime_ms_delta(ktime_get(), boe->last_unprepared);

		if (elapsed_ms < BOE_COG_T_POWER_OFF_MS)
			msleep(BOE_COG_T_POWER_OFF_MS - elapsed_ms);
	}

	ret = regulator_enable(boe->supply);
	if (ret < 0) {
		dev_err(boe->dev, "failed to enable supply: %d\n", ret);
		return ret;
	}

	/*
	 * Datasheet requires RESET deassertion at least 10us after VDD
	 * reaches its valid level. Use a conservative 10ms board-level
	 * settling delay after regulator_enable().
	 */
	usleep_range(BOE_COG_T_POWER_ON_SETTLE_US,
		     BOE_COG_T_POWER_ON_SETTLE_US + 2000);

	/* GPIO_ACTIVE_LOW: logical 0 = physical HIGH = RESET normal. */
	gpiod_set_value_cansleep(boe->reset_gpio, 0);

	msleep(BOE_COG_T_RESET_TO_STBYB_MS);

	if (boe->enable_gpio)
		gpiod_set_value_cansleep(boe->enable_gpio, 1);

	msleep(BOE_COG_T_STBYB_TO_DATA_MS);

	return 0;
}

static int boe_cog_panel_unprepare(struct drm_panel *panel)
{
	struct boe_cog_panel *boe = to_boe_cog_panel(panel);
	int ret;

	msleep(BOE_COG_T_STBYB_TO_RESET_OFF_MS);

	/* GPIO_ACTIVE_LOW: logical 1 = physical LOW = RESET asserted. */
	gpiod_set_value_cansleep(boe->reset_gpio, 1);

	msleep(BOE_COG_T_RESET_TO_VDD_OFF_MS);

	ret = regulator_disable(boe->supply);
	if (ret < 0) {
		dev_err(boe->dev, "failed to disable supply: %d\n", ret);
		return ret;
	}

	boe->last_unprepared = ktime_get();

	return 0;
}

static int boe_cog_panel_enable(struct drm_panel *panel)
{
	/*
	 * LVDS data starts flowing once the encoder/bridge is enabled,
	 * which happens between prepare() and this call. Delay here so
	 * that drm_panel_enable()'s following backlight_enable() lands
	 * at least BOE_COG_T_DATA_TO_BL_MS after data became valid.
	 */
	msleep(BOE_COG_T_DATA_TO_BL_MS);

	return 0;
}

static int boe_cog_panel_disable(struct drm_panel *panel)
{
	/*
	 * drm_panel_disable() already turned the backlight off before
	 * calling us. Wait before driving STBYB LOW.
	 */
	struct boe_cog_panel *boe = to_boe_cog_panel(panel);

	msleep(BOE_COG_T_BL_TO_STBYB_OFF_MS);

	if (boe->enable_gpio)
		gpiod_set_value_cansleep(boe->enable_gpio, 0);

	return 0;
}

static int boe_cog_panel_get_modes(struct drm_panel *panel,
				    struct drm_connector *connector)
{
	struct boe_cog_panel *boe = to_boe_cog_panel(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &boe->dmode);
	if (!mode)
		return 0;

	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = boe->dmode.width_mm;
	connector->display_info.height_mm = boe->dmode.height_mm;
	drm_display_info_set_bus_formats(&connector->display_info,
					  &boe->bus_format, 1);
	connector->display_info.bus_flags = boe->bus_flags;

	return 1;
}

static const struct drm_panel_funcs boe_cog_panel_funcs = {
	.prepare = boe_cog_panel_prepare,
	.unprepare = boe_cog_panel_unprepare,
	.enable = boe_cog_panel_enable,
	.disable = boe_cog_panel_disable,
	.get_modes = boe_cog_panel_get_modes,
};

static int boe_cog_panel_parse_dt(struct boe_cog_panel *boe)
{
	struct device_node *np = boe->dev->of_node;
	int ret;

	ret = of_get_drm_panel_display_mode(np, &boe->dmode, &boe->bus_flags);
	if (ret < 0) {
		dev_err(boe->dev, "%pOF: problems parsing panel-timing (%d)\n",
			np, ret);
		return ret;
	}

	ret = drm_of_lvds_get_data_mapping(np);
	if (ret < 0) {
		dev_err(boe->dev, "%pOF: invalid or missing %s DT property\n",
			np, "data-mapping");
		return ret;
	}

	boe->bus_format = ret;

	boe->bus_flags |= of_property_read_bool(np, "data-mirror") ?
			  DRM_BUS_FLAG_DATA_LSB_TO_MSB :
			  DRM_BUS_FLAG_DATA_MSB_TO_LSB;

	return 0;
}

static int boe_cog_panel_probe(struct platform_device *pdev)
{
	struct boe_cog_panel *boe;
	int ret;

	boe = devm_kzalloc(&pdev->dev, sizeof(*boe), GFP_KERNEL);
	if (!boe)
		return -ENOMEM;

	boe->dev = &pdev->dev;

	ret = boe_cog_panel_parse_dt(boe);
	if (ret < 0)
		return ret;

	boe->supply = devm_regulator_get(boe->dev, "power");
	if (IS_ERR(boe->supply))
		return dev_err_probe(boe->dev, PTR_ERR(boe->supply),
				      "failed to request regulator\n");

	/*
	 * reset-gpios is mandatory: this panel requires RESET to be
	 * actively driven through the datasheet power sequence, and a
	 * missing/floating RESET is what previously caused a black
	 * screen. It is GPIO_ACTIVE_LOW: logical 1 = physical LOW =
	 * RESET asserted. Request it asserted so the panel stays in
	 * reset until prepare() runs the datasheet power sequence.
	 */
	boe->reset_gpio = devm_gpiod_get(boe->dev, "reset", GPIOD_ASIS);
	if (IS_ERR(boe->reset_gpio))
		return dev_err_probe(boe->dev, PTR_ERR(boe->reset_gpio),
				      "failed to request %s GPIO\n", "reset");

	/*
	 * enable-gpios drives STBYB and is GPIO_ACTIVE_HIGH: logical 0 =
	 * physical LOW = STBYB standby. Request it deasserted at probe.
	 */
	boe->enable_gpio = devm_gpiod_get_optional(boe->dev, "enable",
						    GPIOD_OUT_LOW);
	if (IS_ERR(boe->enable_gpio))
		return dev_err_probe(boe->dev, PTR_ERR(boe->enable_gpio),
				      "failed to request %s GPIO\n", "enable");

	drm_panel_init(&boe->panel, boe->dev, &boe_cog_panel_funcs,
		       DRM_MODE_CONNECTOR_LVDS);

	ret = drm_panel_of_backlight(&boe->panel);
	if (ret)
		return ret;

	drm_panel_add(&boe->panel);

	platform_set_drvdata(pdev, boe);

	return 0;
}

static int boe_cog_panel_remove(struct platform_device *pdev)
{
	struct boe_cog_panel *boe = platform_get_drvdata(pdev);

	drm_panel_remove(&boe->panel);
	drm_panel_disable(&boe->panel);
	drm_panel_unprepare(&boe->panel);

	return 0;
}

static const struct of_device_id boe_cog_panel_of_table[] = {
	{ .compatible = "boe,cog-ta25f325p-l5", },
	{ /* Sentinel */ },
};

MODULE_DEVICE_TABLE(of, boe_cog_panel_of_table);

static struct platform_driver boe_cog_panel_driver = {
	.probe		= boe_cog_panel_probe,
	.remove		= boe_cog_panel_remove,
	.driver		= {
		.name	= "panel-boe-cog-ta25f325p-l5",
		.of_match_table = boe_cog_panel_of_table,
	},
};

module_platform_driver(boe_cog_panel_driver);

MODULE_DESCRIPTION("BOE COG-TA25F325P-L5 LVDS Panel Driver");
MODULE_LICENSE("GPL");
