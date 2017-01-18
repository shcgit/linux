#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/tw28xx.h>
#include <sound/soc.h>

static int tw28xx_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	//
	return 0;
}

static int tw28xx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	//
	return 0;
}

static const struct snd_soc_dai_ops tw28xx_ops = {
	.hw_params	= tw28xx_hw_params,
	.set_fmt	= tw28xx_set_fmt,
//	.set_sysclk
//	.set_tdm_slot
};

static int tw28xx_codec_probe(struct snd_soc_codec *codec)
{
	dev_info(codec->dev, "=============== Codec probe\n");

//<------>struct wm8998_priv *priv = snd_soc_codec_get_drvdata(codec);
//<------>struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

//<------>priv->core.arizona->dapm = dapm;

//<------>arizona_init_spk(codec);
//<------>arizona_init_gpio(codec);

//<------>snd_soc_dapm_disable_pin(dapm, "HAPTICS");

	return 0;
}

static int tw28xx_codec_remove(struct snd_soc_codec *codec)
{
//<------>struct wm8998_priv *priv = snd_soc_codec_get_drvdata(codec);

//<------>priv->core.arizona->dapm = NULL;

//<------>arizona_free_spk(codec);

	return 0;
}

static struct regmap *tw28xx_get_regmap(struct device *dev)
{
	return dev_get_regmap(dev->parent, NULL);
}

static struct snd_soc_dai_driver tw28xx_soc_dai = {
	.name = "tw28xx-adc",
	.symmetric_rates = 1,
	.playback = {
		.stream_name = "ADC Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_MU_LAW | SNDRV_PCM_FMTBIT_A_LAW,
	},
	.ops = &tw28xx_ops,
};

static const struct snd_kcontrol_new tw28xx_controls[] = {
	SOC_SINGLE("AIN0 Gain", 0x060, 0, 0xf, 0),
	SOC_SINGLE("AIN1 Gain", 0x060, 4, 0xf, 0),
	SOC_SINGLE("AIN2 Gain", 0x061, 0, 0xf, 0),
	SOC_SINGLE("AIN3 Gain", 0x061, 4, 0xf, 0),
//SOC_SINGLE_EXT
//	SND_SOC_DAPM_MUX("AIN Selection L", SND_SOC_NOPM, 0, 0, &wm8753_mic_sel_mux_controls),
//SND_SOC_DAPM_MUX("Capture Mixer", SND_SOC_NOPM, 0, 0,
//	&wm8753_adc_mono_controls),
};

static const struct snd_soc_dapm_widget tw28xx_widgets[] = {
	SND_SOC_DAPM_ADC("ADC", NULL, 0x04c, 4, 1),
	SND_SOC_DAPM_INPUT("AIN0"),
	SND_SOC_DAPM_INPUT("AIN1"),
	SND_SOC_DAPM_INPUT("AIN2"),
	SND_SOC_DAPM_INPUT("AIN3"),

//	SND_SOC_DAPM_ADC("DAC", NULL, 0x04c, 5, 1);
};

static const struct snd_soc_dapm_route tw28xx_routes[] = {
	/* AIN Selector */
	{"AIN Selection L", "Input 0", "AIN0"},
	{"AIN Selection L", "Input 1", "AIN1"},
	{"AIN Selection L", "Input 2", "AIN2"},
	{"AIN Selection L", "Input 3", "AIN3"},
	{"AIN Selection R", "Input 0", "AIN0"},
	{"AIN Selection R", "Input 1", "AIN1"},
	{"AIN Selection R", "Input 2", "AIN2"},
	{"AIN Selection R", "Input 3", "AIN3"},

	{ "ADC", NULL, "AIN0" },
	{ "ADC", NULL, "AIN1" },
	{ "ADC", NULL, "AIN2" },
	{ "ADC", NULL, "AIN3" },
//	{ "ADC Capture", NULL, "ADC" },
};

static const struct snd_soc_codec_driver tw28xx_soc_codec = {
	.probe = tw28xx_codec_probe,
	.remove = tw28xx_codec_remove,
	.get_regmap = tw28xx_get_regmap,//FIXME: is this needed?
	.component_driver = {
		.controls		= tw28xx_controls,
		.num_controls		= ARRAY_SIZE(tw28xx_controls),
		.dapm_widgets		= tw28xx_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(tw28xx_widgets),
		.dapm_routes		= tw28xx_routes,
		.num_dapm_routes	= ARRAY_SIZE(tw28xx_routes),
	},
};

static int tw28xx_probe(struct platform_device *pdev)
{
	int ret;

	ret = snd_soc_register_codec(&pdev->dev, &tw28xx_soc_codec,
				     &tw28xx_soc_dai, 1);
	dev_info(&pdev->dev, "=============== Audio Register %i\n", ret);

	return ret;
}

static int tw28xx_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static struct platform_driver tw28xx_audio_driver = {
	.driver = {
		.name	= "tw28xx-audio",
	},
	.probe	= tw28xx_probe,
	.remove	= tw28xx_remove,
};
module_platform_driver(tw28xx_audio_driver);

MODULE_AUTHOR("Alexander Shiyan <shc_work@mail.ru>");
MODULE_DESCRIPTION("Techwell/Intersil TW28xx ASoC driver");
MODULE_LICENSE("GPL");
