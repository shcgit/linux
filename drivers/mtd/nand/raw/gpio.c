/*
 * drivers/mtd/nand/gpio.c
 *
 * Updated, and converted to generic GPIO based driver by Russell King.
 *
 * Written by Ben Dooks <ben@simtec.co.uk>
 *   Based on 2.4 version by Mark Whittaker
 *
 * © 2004 Simtec Electronics
 *
 * Device driver for NAND flash that uses a memory mapped interface to
 * read/write the NAND commands and data, and GPIO pins for control signals
 * (the DT binding refers to this as "GPIO assisted NAND flash")
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand-gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#define MAX_NAND_PER_CHIP	4

struct gpiomtd {
	void __iomem		*io_sync;
	struct nand_chip	nand_chip;
	int			chip_index;
	struct gpio_nand_platdata plat;
	int gpio_nce[MAX_NAND_PER_CHIP];	/* NCE */
	int gpio_cle;				/* CLE */
	int gpio_ale;				/* ALE */
	int gpio_rdy[MAX_NAND_PER_CHIP];	/* RDY (Optional) */
	int gpio_nwp;				/* NWP (Optional) */
};

static inline struct gpiomtd *gpio_nand_getpriv(struct mtd_info *mtd)
{
	return container_of(mtd_to_nand(mtd), struct gpiomtd, nand_chip);
}

#ifdef CONFIG_ARM
/* gpio_nand_dosync()
 *
 * Make sure the GPIO state changes occur in-order with writes to NAND
 * memory region.
 * Needed on PXA due to bus-reordering within the SoC itself (see section on
 * I/O ordering in PXA manual (section 2.3, p35)
 */
static void gpio_nand_dosync(struct gpiomtd *gpiomtd)
{
	unsigned long tmp;

	if (gpiomtd->io_sync) {
		/*
		 * Linux memory barriers don't cater for what's required here.
		 * What's required is what's here - a read from a separate
		 * region with a dependency on that read.
		 */
		tmp = readl(gpiomtd->io_sync);
		asm volatile("mov %1, %0\n" : "=r" (tmp) : "r" (tmp));
	}
}
#else
static inline void gpio_nand_dosync(struct gpiomtd *gpiomtd) {}
#endif

static void gpio_nand_cmd_ctrl(struct nand_chip *chip, int cmd,
			       unsigned int ctrl)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(nand_to_mtd(chip));

	gpio_nand_dosync(gpiomtd);

	if (ctrl & NAND_CTRL_CHANGE) {
		gpio_set_value(gpiomtd->gpio_nce[gpiomtd->chip_index],
			       !(ctrl & NAND_NCE));
		gpio_set_value(gpiomtd->gpio_cle, !!(ctrl & NAND_CLE));
		gpio_set_value(gpiomtd->gpio_ale, !!(ctrl & NAND_ALE));
		gpio_nand_dosync(gpiomtd);
	}
	if (cmd == NAND_CMD_NONE)
		return;

	writeb(cmd, gpiomtd->nand_chip.legacy.IO_ADDR_W);
	gpio_nand_dosync(gpiomtd);
}

static int gpio_nand_devready(struct nand_chip *chip)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(nand_to_mtd(chip));
	int idx = gpiomtd->chip_index;

	if (!gpio_is_valid(gpiomtd->gpio_rdy[idx]))
		idx = 0;

	return gpio_get_value(gpiomtd->gpio_rdy[idx]);
}

static void gpio_nand_select_chip(struct nand_chip *chip, int chipnr)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(nand_to_mtd(chip));

	switch (chipnr) {
	case -1:
		gpio_nand_cmd_ctrl(chip, NAND_CMD_NONE, 0 | NAND_CTRL_CHANGE);
		break;
	default:
		gpiomtd->chip_index = chipnr;
		break;
	}
}

#ifdef CONFIG_OF
static const struct of_device_id gpio_nand_id_table[] = {
	{ .compatible = "gpio-control-nand" },
	{}
};
MODULE_DEVICE_TABLE(of, gpio_nand_id_table);

static int gpio_nand_get_config_of(const struct device *dev,
				   struct gpio_nand_platdata *plat)
{
	struct device_node *np = dev->of_node;
	struct gpiomtd *gpiomtd = dev_get_drvdata(dev);
	u32 val;
	int i;

	if (!dev->of_node)
		return -ENODEV;

//	if (!of_property_read_u32(np, "bank-width", &val)) {
//		if (val == 2) {
//			plat->options |= NAND_BUSWIDTH_16;
//		} else if (val != 1) {
//			dev_err(dev, "invalid bank-width %u\n", val);
//			return -EINVAL;
//		}
//	} else {
//		if (of_get_property(np,
//				    "gpio-control-nand,bank-width-auto", NULL))
//			plat->options |= NAND_BUSWIDTH_AUTO;
//	}

	if (of_find_property(np, "gpios", NULL)) {
		dev_notice(dev, "Property \"gpios\" is deprecated");

		gpiomtd->gpio_rdy[0] = of_get_gpio(np, 0);
		gpiomtd->gpio_nce[0] = of_get_gpio(np, 1);
		gpiomtd->gpio_ale = of_get_gpio(np, 2);
		gpiomtd->gpio_cle = of_get_gpio(np, 3);
		gpiomtd->gpio_nwp = of_get_gpio(np, 4);
	} else {
		gpiomtd->gpio_ale = of_get_named_gpio(np, "ale-gpios", 0);
		gpiomtd->gpio_cle = of_get_named_gpio(np, "cle-gpios", 0);
		gpiomtd->gpio_nwp = of_get_named_gpio(np, "nwp-gpios", 0);

		for (i = 0; i < MAX_NAND_PER_CHIP; i++) {
			gpiomtd->gpio_nce[i] = of_get_named_gpio(np,
							      "nce-gpios", i);
			gpiomtd->gpio_rdy[i] = of_get_named_gpio(np,
							      "rdy-gpios", i);
		}
	}

	if (!of_property_read_u32(np, "chip-delay", &val))
		plat->chip_delay = val;

	return 0;
}

static struct resource *gpio_nand_get_io_sync_of(struct platform_device *pdev)
{
	struct resource *r = devm_kzalloc(&pdev->dev, sizeof(*r), GFP_KERNEL);
	u64 addr;

	if (!r || of_property_read_u64(pdev->dev.of_node,
				       "gpio-control-nand,io-sync-reg", &addr))
		return NULL;

	r->start = addr;
	r->end = r->start + 0x3;
	r->flags = IORESOURCE_MEM;

	return r;
}
#else /* CONFIG_OF */
static inline int gpio_nand_get_config_of(const struct device *dev,
					  struct gpio_nand_platdata *plat)
{
	return -ENOSYS;
}

static inline struct resource *
gpio_nand_get_io_sync_of(struct platform_device *pdev)
{
	return NULL;
}
#endif /* CONFIG_OF */

static inline int gpio_nand_get_config(const struct device *dev,
				       struct gpio_nand_platdata *plat)
{
	int ret = gpio_nand_get_config_of(dev, plat);

	if (!ret)
		return ret;

	if (dev_get_platdata(dev)) {
		memcpy(plat, dev_get_platdata(dev), sizeof(*plat));
		return 0;
	}

	return -EINVAL;
}

static inline struct resource *
gpio_nand_get_io_sync(struct platform_device *pdev)
{
	struct resource *r = gpio_nand_get_io_sync_of(pdev);

	if (r)
		return r;

	return platform_get_resource(pdev, IORESOURCE_MEM, 1);
}

static void gpio_nand_set_wp(struct gpiomtd *gpiomtd, int val)
{
	if (gpio_is_valid(gpiomtd->gpio_nwp))
		gpio_set_value(gpiomtd->gpio_nwp, val);
}

static int gpio_nand_remove(struct platform_device *pdev)
{
	struct gpiomtd *gpiomtd = platform_get_drvdata(pdev);
	int i;

	nand_release(&gpiomtd->nand_chip);

	gpio_nand_set_wp(gpiomtd, 0);

	for (i = 0; i < MAX_NAND_PER_CHIP; i++)
		if (gpio_is_valid(gpiomtd->gpio_nce[i]))
			gpio_set_value(gpiomtd->gpio_nce[i], 1);

	return 0;
}

static int gpio_nand_probe(struct platform_device *pdev)
{
	struct gpiomtd *gpiomtd;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct resource *res;
	struct device *dev = &pdev->dev;
	int i, ret, found = 0;

	if (!dev->of_node && !dev_get_platdata(dev))
		return -EINVAL;

	gpiomtd = devm_kzalloc(dev, sizeof(*gpiomtd), GFP_KERNEL);
	if (!gpiomtd)
		return -ENOMEM;

	chip = &gpiomtd->nand_chip;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->legacy.IO_ADDR_R = devm_ioremap_resource(dev, res);
	if (IS_ERR(chip->legacy.IO_ADDR_R))
		return PTR_ERR(chip->legacy.IO_ADDR_R);

	ret = gpio_nand_get_config(dev, &gpiomtd->plat);
	if (ret)
		return ret;

//	if (resource_size(res) < 2)
//		gpiomtd->plat.options &= ~(NAND_BUSWIDTH_16 |
//					   NAND_BUSWIDTH_AUTO);

	res = gpio_nand_get_io_sync(pdev);
	if (res) {
		gpiomtd->io_sync = devm_ioremap_resource(dev, res);
		if (IS_ERR(gpiomtd->io_sync))
			return PTR_ERR(gpiomtd->io_sync);
	}

	ret = devm_gpio_request_one(dev, gpiomtd->gpio_ale,
				    GPIOF_OUT_INIT_LOW, "NAND ALE");
	if (ret)
		return ret;

	ret = devm_gpio_request_one(dev, gpiomtd->gpio_cle,
				    GPIOF_OUT_INIT_LOW, "NAND CLE");
	if (ret)
		return ret;

	if (gpio_is_valid(gpiomtd->gpio_nwp)) {
		ret = devm_gpio_request_one(dev, gpiomtd->gpio_nwp,
					    GPIOF_OUT_INIT_LOW, "NAND NWP");
		if (ret)
			return ret;
	}

	for (i = 0; i < MAX_NAND_PER_CHIP; i++) {
		if (!gpio_is_valid(gpiomtd->gpio_nce[i]))
			break;
		if (devm_gpio_request_one(dev, gpiomtd->gpio_nce[i],
					  GPIOF_OUT_INIT_HIGH, NULL))
			break;

		found++;

		if (gpio_is_valid(gpiomtd->gpio_rdy[i])) {
			if (devm_gpio_request_one(dev,
						  gpiomtd->gpio_rdy[i],
						  GPIOF_IN, NULL))
				gpiomtd->gpio_rdy[i] = -1;
			else if (!i)
				chip->legacy.dev_ready = gpio_nand_devready;
		}
	}

	if (!found)
		return -EINVAL;

	nand_set_flash_node(chip, pdev->dev.of_node);
	chip->legacy.IO_ADDR_W	= chip->legacy.IO_ADDR_R;
	chip->ecc.mode		= NAND_ECC_SOFT;
	chip->ecc.algo		= NAND_ECC_HAMMING;
	chip->options		= gpiomtd->plat.options;
	chip->legacy.chip_delay	= gpiomtd->plat.chip_delay;
	chip->legacy.cmd_ctrl	= gpio_nand_cmd_ctrl;
	chip->select_chip	= gpio_nand_select_chip;

	mtd			= nand_to_mtd(chip);
	mtd->dev.parent		= dev;

	platform_set_drvdata(pdev, gpiomtd);

	gpio_nand_set_wp(gpiomtd, 1);

	ret = nand_scan(chip, found);
	if (ret)
		goto err_wp;

	if (gpiomtd->plat.adjust_parts)
		gpiomtd->plat.adjust_parts(&gpiomtd->plat, mtd->size);

	ret = mtd_device_register(mtd, gpiomtd->plat.parts,
				  gpiomtd->plat.num_parts);
	if (!ret)
		return 0;

err_wp:
	gpio_nand_set_wp(gpiomtd, 0);

	return ret;
}

static struct platform_driver gpio_nand_driver = {
	.probe		= gpio_nand_probe,
	.remove		= gpio_nand_remove,
	.driver		= {
		.name	= "gpio-nand",
		.of_match_table = of_match_ptr(gpio_nand_id_table),
	},
};

module_platform_driver(gpio_nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_DESCRIPTION("GPIO NAND Driver");
