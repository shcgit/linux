// SPDX-License-Identifier: GPL-2.0-only
/*
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
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand-gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>

#define MAX_NAND_PER_CHIP	4

struct gpiomtd {
	struct nand_controller	base;
	void __iomem		*io;
	void __iomem		*io_sync;
	struct nand_chip	nand_chip;
	struct gpio_nand_platdata plat;
	struct gpio_desc *nce[MAX_NAND_PER_CHIP]; /* Optional chip enable */
	struct gpio_desc *cle;
	struct gpio_desc *ale;
	struct gpio_desc *rdy[MAX_NAND_PER_CHIP];
	struct gpio_desc *nwp; /* Optional write protection */
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

static int gpio_nand_exec_instr(struct nand_chip *chip, int cs,
				const struct nand_op_instr *instr)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(nand_to_mtd(chip));
	unsigned int i;
	int rdyidx = 0;

	switch (instr->type) {
	case NAND_OP_CMD_INSTR:
		gpio_nand_dosync(gpiomtd);
		gpiod_set_value(gpiomtd->cle, 1);
		gpio_nand_dosync(gpiomtd);
		writeb(instr->ctx.cmd.opcode, gpiomtd->io);
		gpio_nand_dosync(gpiomtd);
		gpiod_set_value(gpiomtd->cle, 0);
		return 0;

	case NAND_OP_ADDR_INSTR:
		gpio_nand_dosync(gpiomtd);
		gpiod_set_value(gpiomtd->ale, 1);
		gpio_nand_dosync(gpiomtd);
		for (i = 0; i < instr->ctx.addr.naddrs; i++)
			writeb(instr->ctx.addr.addrs[i], gpiomtd->io);
		gpio_nand_dosync(gpiomtd);
		gpiod_set_value(gpiomtd->ale, 0);
		return 0;

	case NAND_OP_DATA_IN_INSTR:
		gpio_nand_dosync(gpiomtd);
		if ((chip->options & NAND_BUSWIDTH_16) &&
		    !instr->ctx.data.force_8bit)
			ioread16_rep(gpiomtd->io, instr->ctx.data.buf.in,
				     instr->ctx.data.len / 2);
		else
			ioread8_rep(gpiomtd->io, instr->ctx.data.buf.in,
				    instr->ctx.data.len);
		return 0;

	case NAND_OP_DATA_OUT_INSTR:
		gpio_nand_dosync(gpiomtd);
		if ((chip->options & NAND_BUSWIDTH_16) &&
		    !instr->ctx.data.force_8bit)
			iowrite16_rep(gpiomtd->io, instr->ctx.data.buf.out,
				      instr->ctx.data.len / 2);
		else
			iowrite8_rep(gpiomtd->io, instr->ctx.data.buf.out,
				     instr->ctx.data.len);
		return 0;

	case NAND_OP_WAITRDY_INSTR:
		if (gpiomtd->rdy[cs] && !IS_ERR(gpiomtd->rdy[cs]))
			rdyidx = cs;
		if (!gpiomtd->rdy[rdyidx])
			return nand_soft_waitrdy(chip, instr->ctx.waitrdy.timeout_ms);

		return nand_gpio_waitrdy(chip, gpiomtd->rdy[rdyidx],
					 instr->ctx.waitrdy.timeout_ms);

	default:
		return -EINVAL;
	}

	return 0;
}

static int gpio_nand_exec_op(struct nand_chip *chip,
			     const struct nand_operation *op,
			     bool check_only)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(nand_to_mtd(chip));
	unsigned int i;
	int ret = 0;

	if (check_only)
		return 0;

	gpio_nand_dosync(gpiomtd);
	gpiod_set_value(gpiomtd->nce[op->cs], 0);
	for (i = 0; i < op->ninstrs; i++) {
		ret = gpio_nand_exec_instr(chip, op->cs, &op->instrs[i]);
		if (ret)
			break;

		if (op->instrs[i].delay_ns)
			ndelay(op->instrs[i].delay_ns);
	}
	gpio_nand_dosync(gpiomtd);
	gpiod_set_value(gpiomtd->nce[op->cs], 1);

	return ret;
}

static const struct nand_controller_ops gpio_nand_ops = {
	.exec_op = gpio_nand_exec_op,
};

#ifdef CONFIG_OF
static const struct of_device_id gpio_nand_id_table[] = {
	{ .compatible = "gpio-control-nand" },
	{}
};
MODULE_DEVICE_TABLE(of, gpio_nand_id_table);

static int gpio_nand_get_config_of(const struct device *dev,
				   struct gpio_nand_platdata *plat)
{
	u32 val;

	if (!dev->of_node)
		return -ENODEV;

	if (!of_property_read_u32(dev->of_node, "bank-width", &val)) {
		if (val == 2) {
			plat->options |= NAND_BUSWIDTH_16;
		} else if (val != 1) {
			dev_err(dev, "invalid bank-width %u\n", val);
			return -EINVAL;
		}
	} else {
		plat->options |= NAND_BUSWIDTH_AUTO;
		dev_info(dev, "Using auto bank-width\n");
	}

	if (!of_property_read_u32(dev->of_node, "chip-delay", &val))
		plat->chip_delay = val;

	return 0;
}

static struct resource *gpio_nand_get_io_sync_of(struct platform_device *pdev)
{
	struct resource *r;
	u64 addr;

	if (of_property_read_u64(pdev->dev.of_node,
				       "gpio-control-nand,io-sync-reg", &addr))
		return NULL;

	r = devm_kzalloc(&pdev->dev, sizeof(*r), GFP_KERNEL);
	if (!r)
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

static int gpio_nand_remove(struct platform_device *pdev)
{
	struct gpiomtd *gpiomtd = platform_get_drvdata(pdev);
	struct nand_chip *chip = &gpiomtd->nand_chip;
	int i, ret;

	ret = mtd_device_unregister(nand_to_mtd(chip));
	WARN_ON(ret);
	nand_cleanup(chip);

	/* Enable write protection and disable the chip */
	if (gpiomtd->nwp && !IS_ERR(gpiomtd->nwp))
		gpiod_set_value(gpiomtd->nwp, 0);

	for (i = 0; i < MAX_NAND_PER_CHIP; i++)
		if (gpiomtd->nce[i] && !IS_ERR(gpiomtd->nce[i]))
			gpiod_set_value(gpiomtd->nce[i], 1);

	return 0;
}

static int gpio_nand_probe(struct platform_device *pdev)
{
	struct gpiomtd *gpiomtd;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct resource *res;
	struct device *dev = &pdev->dev;
	int i, ret, cscount = 0;

	if (!dev->of_node && !dev_get_platdata(dev))
		return -EINVAL;

	gpiomtd = devm_kzalloc(dev, sizeof(*gpiomtd), GFP_KERNEL);
	if (!gpiomtd)
		return -ENOMEM;

	chip = &gpiomtd->nand_chip;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gpiomtd->io = devm_ioremap_resource(dev, res);
	if (IS_ERR(gpiomtd->io))
		return PTR_ERR(gpiomtd->io);

	ret = gpio_nand_get_config(dev, &gpiomtd->plat);
	if (ret)
		return ret;

	/* Sanity check */
	if (resource_size(res) < 2)
		gpiomtd->plat.options &= ~(NAND_BUSWIDTH_16 |
					   NAND_BUSWIDTH_AUTO);

	res = gpio_nand_get_io_sync(pdev);
	if (res) {
		gpiomtd->io_sync = devm_ioremap_resource(dev, res);
		if (IS_ERR(gpiomtd->io_sync))
			return PTR_ERR(gpiomtd->io_sync);
	}

	for (i = 0; i < MAX_NAND_PER_CHIP; i++) {
		gpiomtd->nce[i] = devm_gpiod_get_index_optional(dev, "nce", i,
								GPIOD_OUT_HIGH);
		if (IS_ERR(gpiomtd->nce[i]))
			return PTR_ERR(gpiomtd->nce[i]);
		if (!gpiomtd->nce[i])
			break;
	}

	gpiomtd->nwp = devm_gpiod_get_optional(dev, "nwp", GPIOD_OUT_HIGH);
	if (IS_ERR(gpiomtd->nwp)) {
		ret = PTR_ERR(gpiomtd->nwp);
		goto out_ce;
	}

	gpiomtd->ale = devm_gpiod_get(dev, "ale", GPIOD_OUT_LOW);
	if (IS_ERR(gpiomtd->ale)) {
		ret = PTR_ERR(gpiomtd->ale);
		goto out_ce;
	}

	gpiomtd->cle = devm_gpiod_get(dev, "cle", GPIOD_OUT_LOW);
	if (IS_ERR(gpiomtd->cle)) {
		ret = PTR_ERR(gpiomtd->cle);
		goto out_ce;
	}

	for (i = 0; i < MAX_NAND_PER_CHIP; i++) {
		if (!gpiomtd->nce[i])
			break;

		cscount++;

		gpiomtd->rdy[i] = devm_gpiod_get_index_optional(dev, "rdy", i,
								GPIOD_IN);
		if (IS_ERR(gpiomtd->rdy[i])) {
			ret = PTR_ERR(gpiomtd->rdy[i]);
			goto out_ce;
		}
	}

	if (!cscount) {
		ret = -EINVAL;
		goto out_ce;
	}

	nand_controller_init(&gpiomtd->base);
	gpiomtd->base.ops = &gpio_nand_ops;

	nand_set_flash_node(chip, pdev->dev.of_node);
	chip->ecc.mode		= NAND_ECC_SOFT;
	chip->ecc.algo		= NAND_ECC_HAMMING;
	chip->options		= gpiomtd->plat.options;
	chip->controller	= &gpiomtd->base;
//	chip->legacy.chip_delay	= gpiomtd->plat.chip_delay;
//	chip->legacy.cmd_ctrl	= gpio_nand_cmd_ctrl;
//	chip->legacy.select_chip = gpio_nand_select_chip;

	mtd			= nand_to_mtd(chip);
	mtd->dev.parent		= dev;

	/* Using RDY pin */
//	if (gpiomtd->rdy[0])
//		chip->legacy.dev_ready = gpio_nand_devready;

	platform_set_drvdata(pdev, gpiomtd);

	if (gpiomtd->nwp && !IS_ERR(gpiomtd->nwp))
		gpiod_set_value(gpiomtd->nwp, 1);

	ret = nand_scan(chip, cscount);
	if (ret)
		goto err_wp;

	if (gpiomtd->plat.adjust_parts)
		gpiomtd->plat.adjust_parts(&gpiomtd->plat, mtd->size);

	ret = mtd_device_register(mtd, gpiomtd->plat.parts,
				  gpiomtd->plat.num_parts);
	if (!ret)
		return 0;

err_wp:
	if (gpiomtd->nwp && !IS_ERR(gpiomtd->nwp))
		gpiod_set_value(gpiomtd->nwp, 0);
out_ce:
	for (i = 0; i < MAX_NAND_PER_CHIP; i++)
		if (gpiomtd->nce[i] && !IS_ERR(gpiomtd->nce[i]))
			gpiod_set_value(gpiomtd->nce[i], 1);

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
