/*
 *  Currus Logic CLPS711X DAI driver
 *
 *  Author: Alexander Shiyan <shc_work@mail.ru>, 2016-2018
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/clps711x.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/fiq.h>

#include "clps711x-dai.h"

#define CLPS711X_FMTS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE)
#define CLPS711X_RATES	(SNDRV_PCM_RATE_8000_48000)
#define DRV_NAME	"clps711x-dai"

struct clps711x_dai {
	u32				head;
	u32				last_ptr;

	struct hrtimer			hrt;
	unsigned long			reload;

	unsigned long			pll_hz;
	unsigned long			ext_hz;

	void __iomem			*base;
	int				irq;

	struct pt_regs			regs;
	struct fiq_handler		fiq;
	atomic_t			running;

	struct snd_pcm_substream	*substream;
};

static int clps711x_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_LEFT_J)
		return -EINVAL;

	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS)
		return -EINVAL;

	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	return 0;
}

static int clps711x_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id,
				   unsigned int freq, int dir)
{
	struct clps711x_dai *s = dev_get_drvdata(dai->dev);

	s->ext_hz = freq;

	return 0;
}

static int clps711x_dai_update_best_err(unsigned int clk, unsigned int div,
					unsigned int rate, int *besterr)
{
	int err = abs(DIV_ROUND_CLOSEST(clk, div) - rate);

	if ((*besterr < 0) || (*besterr > err)) {
		*besterr = err;
		return 0;
	}

	return 1;
}

static int clps711x_dai_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct clps711x_dai *s = dev_get_drvdata(dai->dev);
	int i, besterr = -1;
	u32 dai64 = 0;

	if ((params_rate(params) < 8000) || (params_rate(params) > 48000))
		return -EINVAL;

	if (!s->pll_hz && !s->ext_hz)
		return -EINVAL;

	/* Find best settings for desired samplerate */
	for (i = 0x01; (i < 0x80) && besterr && s->pll_hz; i++)
		if (!clps711x_dai_update_best_err(s->pll_hz, 128 * i,
						  params_rate(params),
						  &besterr))
			dai64 = DAI64FS_AUDIV(i);
	for (i = 0x01; (i < 0x80) && besterr && s->ext_hz; i++)
		if (!clps711x_dai_update_best_err(s->ext_hz, 128 * i,
						  params_rate(params),
						  &besterr))
			dai64 = DAI64FS_AUDIV(i) | DAI64FS_AUDIOCLKSRC;

	writel(DAI64FS_I2SF64 | DAI64FS_AUDIOCLKEN | DAI64FS_MCLK256EN | dai64,
	       s->base + DAI64FS);

	return 0;
}

static const struct snd_soc_dai_ops clps711x_dai_ops = {
	.hw_params	= clps711x_dai_hw_params,
	.set_fmt	= clps711x_dai_set_fmt,
	.set_sysclk	= clps711x_dai_set_sysclk,
};

static int clps711x_dai_probe(struct snd_soc_dai *dai)
{
	struct clps711x_dai *s = dev_get_drvdata(dai->dev);

	snd_soc_dai_set_drvdata(dai, s);

	return 0;
}

static struct snd_soc_dai_driver clps711x_dai_driver = {
	.probe		= clps711x_dai_probe,
	.playback	= {
		.stream_name	= "CPU-Playback",
		.formats	= CLPS711X_FMTS,
		.rates		= CLPS711X_RATES,
		.channels_min	= 2,
		.channels_max	= 2,
	},
	.ops		= &clps711x_dai_ops,
};

static const struct snd_soc_component_driver clps711x_i2s_component = {
	.name = "clps711x-i2s",
};

static int clps711x_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct clps711x_dai *dai = snd_soc_component_get_drvdata(component);
	int ret;

	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0)
		return ret;

	dai->substream = substream;
	dai->reload = (1000000000 / params_rate(params)) *
		    params_period_size(params) / 2;

	local_fiq_disable();
	get_fiq_regs(&dai->regs);
	dai->regs.ARM_r8 = (u32)substream->runtime->dma_area;
	set_fiq_regs(&dai->regs);
	local_fiq_enable();

	return 0;
}

static int clps711x_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct clps711x_dai *dai = snd_soc_component_get_drvdata(component);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		atomic_set(&dai->running, 1);
		hrtimer_start(&dai->hrt, ns_to_ktime(dai->reload),
			      HRTIMER_MODE_REL);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		atomic_set(&dai->running, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t clps711x_pcm_ptr(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct clps711x_dai *dai = snd_soc_component_get_drvdata(component);

	return bytes_to_frames(substream->runtime, dai->last_ptr);
}

static int clps711x_pcm_copy(struct snd_pcm_substream *substream, int channel,
			     unsigned long pos, void __user *buf,
			     unsigned long bytes)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct clps711x_dai *dai = snd_soc_component_get_drvdata(component);

	if (copy_from_user(substream->runtime->dma_area + dai->head, buf, bytes))
		return -EFAULT;

	local_fiq_disable();
	dai->head += bytes;
	dai->head %= CLPS711X_SNDBUF_SIZE;
	local_fiq_enable();

	return 0;
}

static enum hrtimer_restart clps711x_pcm_timer(struct hrtimer *hrt)
{
	struct clps711x_dai *dai = container_of(hrt, struct clps711x_dai, hrt);
	u32 delta;

	if (!atomic_read(&dai->running)) {
		dai->head = 0;
		dai->last_ptr = 0;

		local_fiq_disable();
		get_fiq_regs(&dai->regs);
		dai->regs.ARM_r10 = 0;
		set_fiq_regs(&dai->regs);
		local_fiq_enable();

		return HRTIMER_NORESTART;
	}

	get_fiq_regs(&dai->regs);

	delta = CLPS711X_SNDBUF_SIZE + dai->regs.ARM_r10 - dai->last_ptr;
	delta %= CLPS711X_SNDBUF_SIZE;
	if ((delta >= dai->substream->runtime->period_size) ||
	    (dai->regs.ARM_r10 == dai->head)) {
		dai->last_ptr = dai->regs.ARM_r10;
		snd_pcm_period_elapsed(dai->substream);
	}

	hrtimer_forward_now(hrt, ns_to_ktime(dai->reload));

	return HRTIMER_RESTART;
}

static const struct snd_pcm_hardware clps711x_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= CLPS711X_FMTS,
	.buffer_bytes_max	= CLPS711X_SNDBUF_SIZE,
	.period_bytes_min	= 32,
	.period_bytes_max	= CLPS711X_SNDBUF_SIZE / 8,
	.periods_min		= 4,
	.periods_max		= CLPS711X_SNDBUF_SIZE / 64 - 1,
};

static int clps711x_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct clps711x_dai *dai = snd_soc_component_get_drvdata(component);
	int ret;

	atomic_set(&dai->running, 0);
	hrtimer_init(&dai->hrt, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dai->hrt.function = clps711x_pcm_timer;

	dai->head = 0;
	dai->last_ptr = 0;

	get_fiq_regs(&dai->regs);
	dai->regs.ARM_r9 = (u32)&dai->head;
	dai->regs.ARM_r10 = 0;
	dai->regs.ARM_fp = (u32)dai->base;
	set_fiq_regs(&dai->regs);

	enable_irq(dai->irq);

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	return snd_soc_set_runtime_hwparams(substream, &clps711x_pcm_hardware);
}

static int clps711x_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct clps711x_dai *dai = snd_soc_component_get_drvdata(component);

	disable_irq(dai->irq);
	hrtimer_cancel(&dai->hrt);

	return 0;
}

static const struct snd_pcm_ops clps711x_pcm_ops = {
	.open		= clps711x_pcm_open,
	.close		= clps711x_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= clps711x_pcm_hw_params,
	.hw_free	= snd_pcm_lib_free_pages,
	.trigger	= clps711x_pcm_trigger,
	.pointer	= clps711x_pcm_ptr,
	.copy_user	= clps711x_pcm_copy,
};

static int clps711x_pcm_probe(struct snd_soc_component *component)
{
	struct platform_device *pdev = to_platform_device(component->dev);
	struct clps711x_dai *dai = platform_get_drvdata(pdev);

	snd_soc_component_set_drvdata(component, dai);

	return 0;
}

static int clps711x_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct clps711x_dai *dai = snd_soc_component_get_drvdata(component);
	int ret;

	/* Request FIQ */
	dai->fiq.name = DRV_NAME;
	ret = claim_fiq(&dai->fiq);
	if (ret)
		return ret;

	/* Request FIQ interrupt */
	ret = request_irq(dai->irq, no_action, 0, DRV_NAME, NULL);
	if (ret)
		return ret;

	/* Install FIQ handler */
	set_fiq_handler(&daifiq_start, &daifiq_end - &daifiq_start);

	return snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
		SNDRV_DMA_TYPE_CONTINUOUS, snd_dma_continuous_data(GFP_KERNEL),
		CLPS711X_SNDBUF_SIZE, CLPS711X_SNDBUF_SIZE);
}

static void clps711x_pcm_free(struct snd_pcm *pcm)
{
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	struct clps711x_dai *dai = snd_soc_component_get_drvdata(component);

	free_irq(dai->irq, NULL);

	release_fiq(&dai->fiq);

	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static struct snd_soc_component_driver clps711x_soc_component = {
	.name		= DRV_NAME,
	.ops		= &clps711x_pcm_ops,
	.probe		= clps711x_pcm_probe,
	.pcm_new	= clps711x_pcm_new,
	.pcm_free	= clps711x_pcm_free,
};

static int clps711x_enable_fifo(void __iomem *base, u32 channel)
{
	unsigned long timeout;

	writel(DAIDR2_FIFOEN | channel, base + DAIDR2);
	timeout = jiffies + msecs_to_jiffies(30);
	while (!(readl(base + DAISR) & DAISR_FIFO))
		if (time_is_before_jiffies(timeout))
			return -ETIMEDOUT;

	return 0;
}

static int clps711x_dai_platform_probe(struct platform_device *pdev)
{
	u32 dai64 = DAI64FS_AUDIOCLKEN | DAI64FS_MCLK256EN | DAI64FS_I2SF64;
	struct device *dev = &pdev->dev;
	struct clps711x_dai *dai;
	struct regmap *syscon;
	struct resource *res;
	struct clk *clk;
	int ret;

	/* Check for proper buffer size */
	BUG_ON(!is_power_of_2(CLPS711X_SNDBUF_SIZE));

	dai = devm_kzalloc(dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	dai->irq = platform_get_irq(pdev, 0);
	if (dai->irq < 0)
		return dai->irq;

	syscon = syscon_regmap_lookup_by_compatible("cirrus,ep7209-syscon3");
	if (IS_ERR(syscon))
		return PTR_ERR(syscon);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dai->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dai->base))
		return PTR_ERR(dai->base);

	clk = devm_clk_get(dev, "pll");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	dai->pll_hz = clk_get_rate(clk) / 2;
	if (!dai->pll_hz) {
		dai64 |= DAI64FS_AUDIOCLKSRC;
		dev_notice(dev, "External MCLK will be used\n");
	}

	clk = devm_clk_get(dev, "mclk");
	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	} else
		dai->ext_hz = clk_get_rate(clk);

	platform_set_drvdata(pdev, dai);

	ret = devm_snd_soc_register_component(dev, &clps711x_i2s_component,
					      &clps711x_dai_driver, 1);
	if (ret)
		return ret;

	/* Enable DAI interface */
	regmap_update_bits(syscon, SYSCON_OFFSET,
			   SYSCON3_DAISEL | SYSCON3_128FS, SYSCON3_DAISEL);

	/* Clear interrupt flags */
	writel(~0, dai->base + DAISR);

	/* Enable DAI */
	writel(DAIR_RESERVED | DAIR_DAIEN | DAIR_ECS | DAIR_LCTM | DAIR_RCTM,
	       dai->base + DAIR);

	/* Set initial value to DAI register */
	writel(dai64 | DAI64FS_AUDIV(10), dai->base + DAI64FS);

	/* Enable FIFOs */
	ret = clps711x_enable_fifo(dai->base, DAIDR2_FIFOLEFT);
	if (ret)
		goto out_err;
	ret = clps711x_enable_fifo(dai->base, DAIDR2_FIFORIGHT);
	if (ret)
		goto out_err;

	ret = devm_snd_soc_register_component(dev, &clps711x_soc_component,
					      NULL, 0);
	if (!ret)
		return 0;

out_err:
	/* Disable DAI */
	writel(DAIR_RESERVED, dai->base + DAIR);

	return ret;
}

static int clps711x_dai_platform_remove(struct platform_device *pdev)
{
	struct clps711x_dai *dai = platform_get_drvdata(pdev);

	/* Disable DAI */
	writel(DAIR_RESERVED, dai->base + DAIR);

	return 0;
}

static const struct of_device_id clps711x_dai_dt_ids[] = {
	{ .compatible = "cirrus,ep7209-dai", },
	{ }
};
MODULE_DEVICE_TABLE(of, clps711x_dai_dt_ids);

static struct platform_driver clps711x_dai_platform_driver = {
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= clps711x_dai_dt_ids,
	},
	.probe	= clps711x_dai_platform_probe,
	.remove	= clps711x_dai_platform_remove,
};
module_platform_driver(clps711x_dai_platform_driver);

MODULE_AUTHOR("Alexander Shiyan <shc_work@mail.ru>");
MODULE_DESCRIPTION("CLPS711X DAI driver");
MODULE_LICENSE("GPL");
