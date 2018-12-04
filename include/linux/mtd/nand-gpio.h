/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_MTD_NAND_GPIO_H
#define __LINUX_MTD_NAND_GPIO_H

#include <linux/mtd/rawnand.h>

#define MAX_NAND_PER_CHIP	4

struct gpio_nand_platdata {
	int	gpio_ale;			/* ALE */
	int	gpio_cle;			/* CLE */
	int	gpio_nwp;			/* NWP (Optional) */
	int	gpio_nce[MAX_NAND_PER_CHIP];	/* NCE */
	int	gpio_rdy[MAX_NAND_PER_CHIP];	/* RDY (Optional) */
	void	(*adjust_parts)(struct gpio_nand_platdata *, size_t);
	struct mtd_partition *parts;
	unsigned int num_parts;
	unsigned int options;
	int	chip_delay;
};

#endif
