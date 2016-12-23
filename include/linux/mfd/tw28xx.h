#ifndef __MFD_TW28XX_H__
#define __MFD_TW28XX_H__

#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

struct tw28xx {
	struct regmap		*regmap;
	struct clk		*clk;
	int			irq;
	struct regulator	*vddi;
	struct regulator	*vddo;
};

#endif
