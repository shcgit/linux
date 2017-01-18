#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tw28xx.h>

static const struct regmap_range tw28xx_reg_ranges[] = {
	regmap_reg_range(0x000, 0x1bf),
	regmap_reg_range(0x1ff, 0x2ff),
};

static const struct regmap_range tw28xx_reg_ro_ranges[] = {
	regmap_reg_range(0x000, 0x001),
	regmap_reg_range(0x00d, 0x00d),
	regmap_reg_range(0x010, 0x011),
	regmap_reg_range(0x01d, 0x01d),
	regmap_reg_range(0x020, 0x021),
	regmap_reg_range(0x02d, 0x02d),
	regmap_reg_range(0x030, 0x031),
	regmap_reg_range(0x03d, 0x03d),
	regmap_reg_range(0x055, 0x055),
	regmap_reg_range(0x0d8, 0x0d9),
	regmap_reg_range(0x108, 0x10a),
	regmap_reg_range(0x15e, 0x15e),
	regmap_reg_range(0x16e, 0x16f),
	regmap_reg_range(0x17c, 0x17c),
	regmap_reg_range(0x18b, 0x18f),
	regmap_reg_range(0x198, 0x19f),
	regmap_reg_range(0x29e, 0x29e),
	regmap_reg_range(0x2be, 0x2be),
	regmap_reg_range(0x2de, 0x2de),
	regmap_reg_range(0x2fe, 0x2fe),
};

static const struct regmap_access_table tw28xx_write_ranges_table = {
	.yes_ranges	= tw28xx_reg_ranges,
	.n_yes_ranges	= ARRAY_SIZE(tw28xx_reg_ranges),
	.no_ranges	= tw28xx_reg_ro_ranges,
	.n_no_ranges	= ARRAY_SIZE(tw28xx_reg_ro_ranges),
};

static struct regmap_config tw28xx_regmap_i2c_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.cache_type	= REGCACHE_NONE,
	.max_register	= 0x2ff,
	.wr_table	= &tw28xx_write_ranges_table,
};

static const struct reg_sequence tw28xx_init_patch[] = {
	/* Recommended values to reserved registers */
	{ 0x17d, 0x00 },
	{ 0x1b7, 0x00 },
	{ 0x1b8, 0x00 },
	{ 0x1b9, 0x00 },
	{ 0x1ba, 0x00 },
	{ 0x1bb, 0x00 },
	{ 0x1bc, 0x00 },
	{ 0x1bd, 0x00 },
	{ 0x1be, 0x00 },
	{ 0x1bf, 0x00 },
	/* Initialize the operation mode of SDRAM */
	{ 0x17f, BIT(7) },
	/* Master operation mode */
	{ 0x1a4, BIT(7) },
	/* Power down Audio DAC, Audio ADC, Video ADCs */
	{ 0x04c, 0x3f },
	/* Power down Video DACs */
	{ 0x1a1, BIT(3) | BIT(7) },
	{ 0x1a2, BIT(3) },
};

static void tw28xx_shutdown(struct tw28xx *tw28xx)
{
	clk_disable_unprepare(tw28xx->clk);

	if (!IS_ERR(tw28xx->vddo))
		regulator_disable(tw28xx->vddo);
	if (!IS_ERR(tw28xx->vddi))
		regulator_disable(tw28xx->vddi);
}

static const struct mfd_cell tw28xx_devs[] = {
	{ .name = "tw28xx-audio", },
	{ .name = "tw28xx-video", },
};

static int tw28xx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct gpio_desc *reset;
	struct tw28xx *tw28xx;
	u32 data;
	int ret;

	tw28xx = devm_kzalloc(&client->dev, sizeof(*tw28xx), GFP_KERNEL);
	if (!tw28xx)
		return -ENOMEM;

	dev_set_drvdata(&client->dev, tw28xx);

	tw28xx->regmap = devm_regmap_init_i2c(client,
					      &tw28xx_regmap_i2c_config);
	if (IS_ERR(tw28xx->regmap))
		return PTR_ERR(tw28xx->regmap);

	tw28xx->vddi = devm_regulator_get_optional(&client->dev, "vddi");
	if (!IS_ERR(tw28xx->vddi)) {
		ret = regulator_enable(tw28xx->vddi);
		if (ret)
			return ret;
	} else if (PTR_ERR(tw28xx->vddi) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	}

	tw28xx->vddo = devm_regulator_get_optional(&client->dev, "vddo");
	if (!IS_ERR(tw28xx->vddo)) {
		ret = regulator_enable(tw28xx->vddo);
		if (ret)
			return ret;
	} else if (PTR_ERR(tw28xx->vddo) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	}

	reset = devm_gpiod_get(&client->dev, "reset", GPIOD_ASIS);
	if (PTR_ERR(reset) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	tw28xx->clk = devm_clk_get(&client->dev, NULL);
	if (IS_ERR(tw28xx->clk))
		return PTR_ERR(tw28xx->clk);

	ret = clk_prepare_enable(tw28xx->clk);
	if (ret)
		return ret;

	if (!IS_ERR(reset)) {
		gpiod_direction_output(reset, 1);
		usleep_range(5000, 10000);
		gpiod_set_value_cansleep(reset, 0);
		usleep_range(5000, 10000);
	}

	tw28xx->irq = client->irq;

	ret = regmap_multi_reg_write(tw28xx->regmap, tw28xx_init_patch,
				     ARRAY_SIZE(tw28xx_init_patch));
	if (ret) {
		tw28xx_shutdown(tw28xx);
		return ret;
	}

	/* Get Device ID */
	ret = regmap_read(tw28xx->regmap, 0x0fe, &data);
	if (ret) {
		dev_err(&client->dev, "Regmap read error.\n");
		tw28xx_shutdown(tw28xx);
		return -ENODEV;
	}

	switch (data >> 3) {
	case 5:
		dev_info(&client->dev, "Detected TW2835 chip, rev %u.\n",
			 data & 0x07);
		break;
	case 6:
		dev_info(&client->dev, "Detected TW2837 chip, rev %u.\n",
			 data & 0x07);
		break;
	default:
		dev_err(&client->dev, "No TW28xx found (0x%02x).\n", data);
		tw28xx_shutdown(tw28xx);
		return -ENODEV;
	}

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				   tw28xx_devs, ARRAY_SIZE(tw28xx_devs),
				   NULL, 0, NULL);
	if (ret)
		tw28xx_shutdown(tw28xx);

	return ret;
}

static int tw28xx_remove(struct i2c_client *client)
{
	tw28xx_shutdown(dev_get_drvdata(&client->dev));

	return 0;
}

static const struct of_device_id __maybe_unused tw28xx_dt_ids[] = {
	{ .compatible = "intersil,tw2835", },
	{ .compatible = "intersil,tw2837", },
	{ }
};
MODULE_DEVICE_TABLE(of, tw28xx_dt_ids);

static const struct i2c_device_id tw28xx_ids[] = {
	{ "tw2835", 0 },
	{ "tw2837", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, tw28xx_ids);

static struct i2c_driver tw28xx_driver = {
	.driver = {
		.name = "tw28xx",
		.of_match_table = of_match_ptr(tw28xx_dt_ids),
	},
	.probe = tw28xx_probe,
	.remove = tw28xx_remove,
	.id_table = tw28xx_ids,
};
module_i2c_driver(tw28xx_driver);

MODULE_AUTHOR("Alexander Shiyan <shc_work@mail.ru>");
MODULE_DESCRIPTION("Techwell/Intersil TW28xx MFD driver");
MODULE_LICENSE("GPL");
