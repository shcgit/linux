#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/tw28xx.h>

#define TW283X_REG_PWDOWN		0x004c
#define TW283X_REG_AFILTERS		0x0052
#define TW283X_REG_AFORMAT1		0x0062
#define TW283X_REG_AFORMAT2		0x006c
#define TW283X_REG_AMUTE		0x006d
#define TW283X_REG_DEVICE_ID		0x00fe
#define TW283X_REG_MEM_INIT		0x017f
#define TW283X_REG_MASTER_MODE		0x01a4

static struct regmap_config tw28xx_regmap_i2c_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0x2ff,
	.cache_type	= REGCACHE_NONE,
};

static const struct reg_sequence tw28xx_init_patch[] = {
	/* Initialize the operation mode of SDRAM */
	{ TW283X_REG_MEM_INIT, (1 << 7) },
	/* Master operation mode */
	{ TW283X_REG_MASTER_MODE, (1 << 7) },
	/* Power down the audio DAC */
	{ TW283X_REG_PWDOWN, (1 << 5) },
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

	ret = regmap_read(tw28xx->regmap, TW283X_REG_DEVICE_ID, &data);
	if (ret) {
		dev_err(&client->dev, "Regmap read error.\n");
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
		return -ENODEV;
	}

	ret = regmap_multi_reg_write(tw28xx->regmap, tw28xx_init_patch,
				     ARRAY_SIZE(tw28xx_init_patch));

	/* Initialize the operation mode of SDRAM */
//	regmap_write(tw28xx->regmap, TW283X_REG_MEM_INIT, (1 << 7));
	/* Master operation mode */
//	regmap_write(tw28xx->regmap, TW283X_REG_MASTER_MODE, (1 << 7));
	/* Power down the audio DAC */
//	regmap_write(tw28xx->regmap, TW283X_REG_PWDOWN, (1 << 5));
	/* Analog Audio input Anti-Aliasing Filter enabled */
//	regmap_write(tw28xx->regmap, TW283X_REG_AFILTERS, (1 << 4));
	/* I2S, 4 audios */
//	regmap_write(tw28xx->regmap, TW283X_REG_AFORMAT1, (1 << 0));
	/* I2S, Audio ADC Function mode, Master */
//	regmap_write(tw28xx->regmap, TW283X_REG_AFORMAT2, (1 << 6) | (1 << 5) | (1 << 0));
	/* Mute Playback audio input */
//	regmap_write(tw28xx->regmap, TW283X_REG_AMUTE, (1 << 4));

	return ret;
}

static int tw28xx_remove(struct i2c_client *client)
{
	struct tw28xx *tw28xx = dev_get_drvdata(&client->dev);

	if (!IS_ERR(tw28xx->vddo))
		regulator_disable(tw28xx->vddo);
	if (!IS_ERR(tw28xx->vddi))
		regulator_disable(tw28xx->vddi);

	clk_disable_unprepare(tw28xx->clk);

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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexander Shiyan <shc_work@mail.ru>");
MODULE_DESCRIPTION("Techwell/Intersil TW28xx driver");
