#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/err.h>

#define TW283X_REG_PWDOWN		0x004c
#define TW283X_REG_AFILTERS		0x0052
#define TW283X_REG_AFORMAT1		0x0062
#define TW283X_REG_AFORMAT2		0x006c
#define TW283X_REG_AMUTE		0x006d
#define TW283X_REG_DEVICE_ID		0x00fe
#define TW283X_REG_MEM_INIT		0x017f
#define TW283X_REG_MASTER_MODE		0x01a4

struct tw283x_t
{
	struct regmap	*regmap;
	struct device	*dev;
};

static struct regmap_config tw283x_regmap_i2c_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0x2ff,
	.cache_type	= REGCACHE_NONE,
};

static int tw283x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u32 data;
	int ret;
	struct tw283x_t *tw283x;

	tw283x = kzalloc(sizeof(struct tw283x_t), GFP_KERNEL);
	if (!tw283x)
		return -ENOMEM;

	dev_set_drvdata(&client->dev, tw283x);
//	tw283x->dev = &client->dev;

	tw283x->regmap = regmap_init_i2c(client, &tw283x_regmap_i2c_config);
	if (IS_ERR(tw283x->regmap)) {
		ret = PTR_ERR(tw283x->regmap);
		dev_err(tw283x->dev, "Failed to initialize register map: %d\n", ret);
		dev_set_drvdata(&client->dev, NULL);
		kfree(tw283x);
		return ret;
	}

	ret = regmap_read(tw283x->regmap, TW283X_REG_DEVICE_ID, &data);
	if (!ret) {
		switch (data >> 3) {
		case 5:
			dev_info(tw283x->dev, "Detected TW2835 chip, rev%u.\n", data & 0x07);
			break;
		case 6:
			dev_info(tw283x->dev, "Detected TW2837 chip, rev%u.\n", data & 0x07);
			break;
		default:
			dev_err(tw283x->dev, "No TW28xx found.\n");
			ret = -ENODEV;
			goto tw283x_out;
		}
		/* Initialize the operation mode of SDRAM */
		regmap_write(tw283x->regmap, TW283X_REG_MEM_INIT, (1 << 7));
		/* Master operation mode */
		regmap_write(tw283x->regmap, TW283X_REG_MASTER_MODE, (1 << 7));
		/* Power down the audio DAC */
		regmap_write(tw283x->regmap, TW283X_REG_PWDOWN, (1 << 5));
		/* Analog Audio input Anti-Aliasing Filter enabled */
		regmap_write(tw283x->regmap, TW283X_REG_AFILTERS, (1 << 4));
		/* I2S, 4 audios */
		regmap_write(tw283x->regmap, TW283X_REG_AFORMAT1, (1 << 0));
		/* I2S, Audio ADC Function mode, Master */
		regmap_write(tw283x->regmap, TW283X_REG_AFORMAT2, (1 << 6) | (1 << 5) | (1 << 0));
		/* Mute Playback audio input */
		regmap_write(tw283x->regmap, TW283X_REG_AMUTE, (1 << 4));
	} else {
		dev_err(tw283x->dev, "Failed to read device id register: %d\n", ret);
tw283x_out:
		dev_set_drvdata(&client->dev, NULL);
		regmap_exit(tw283x->regmap);
		kfree(tw283x);
	}

	return ret;
}

static int __devexit tw283x_remove(struct i2c_client *client)
{
	struct tw283x_t *tw283x = dev_get_drvdata(&client->dev);

	regmap_exit(tw283x->regmap);
	kfree(tw283x);

	return 0;
}

static const struct i2c_device_id tw283x_id[] = {
	{ "tw283x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tw283x_id);

static struct i2c_driver tw283x_i2c_driver = {
	.driver = {
		.name	= "tw283x",
	},
	.probe		= tw283x_probe,
	.remove		= __devexit_p(tw283x_remove),
	.id_table	= tw283x_id,
};

static int __init tw283x_mod_init(void)
{
	return i2c_add_driver(&tw283x_i2c_driver);
}
module_init(tw283x_mod_init);

static void __exit tw283x_mod_exit(void)
{
	i2c_del_driver(&tw283x_i2c_driver);
}
module_exit(tw283x_mod_exit);

MODULE_DESCRIPTION("Techwell TW28xx driver");
MODULE_AUTHOR("Alexander Shiyan <shc_work@mail.ru>");
MODULE_LICENSE("GPL");
