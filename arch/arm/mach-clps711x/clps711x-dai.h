/*
 *  Currus Logic CLPS711X DAI definitions
 *
 *  Author: Alexander Shiyan <shc_work@mail.ru>, 2016-2018
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __CLPS711X_DAI_H
#define __CLPS711X_DAI_H

#include <linux/sizes.h>

#define CLPS711X_SNDBUF_SIZE	SZ_128K

#define DAIR			(0x0000)
# define DAIR_RESERVED		(0x0404)
# define DAIR_DAIEN		(1 << 16)
# define DAIR_ECS		(1 << 17)
# define DAIR_LCTM		(1 << 19)
# define DAIR_LCRM		(1 << 20)
# define DAIR_RCTM		(1 << 21)
# define DAIR_RCRM		(1 << 22)
# define DAIR_LBM		(1 << 23)
#define DAIDR0			(0x0040)
#define DAIDR1			(0x0080)
#define DAIDR2			(0x00c0)
# define DAIDR2_FIFOEN		(1 << 15)
# define DAIDR2_FIFOLEFT	(0x0d << 16)
# define DAIDR2_FIFORIGHT	(0x11 << 16)
#define DAISR			(0x0100)
# define DAISR_RCTS		(1 << 0)
# define DAISR_RCRS		(1 << 1)
# define DAISR_LCTS		(1 << 2)
# define DAISR_LCRS		(1 << 3)
# define DAISR_RCTU		(1 << 4)
# define DAISR_RCRO		(1 << 5)
# define DAISR_LCTU		(1 << 6)
# define DAISR_LCRO		(1 << 7)
# define DAISR_RCNF		(1 << 8)
# define DAISR_RCNE		(1 << 9)
# define DAISR_LCNF		(1 << 10)
# define DAISR_LCNE		(1 << 11)
# define DAISR_FIFO		(1 << 12)
#define DAI64FS			(0x0600)
# define DAI64FS_I2SF64		(1 << 0)
# define DAI64FS_AUDIOCLKEN	(1 << 1)
# define DAI64FS_AUDIOCLKSRC	(1 << 2)
# define DAI64FS_MCLK256EN	(1 << 3)
# define DAI64FS_LOOPBACK	(1 << 5)
# define DAI64FS_AUDIV_MASK	(0x7f00)
# define DAI64FS_AUDIV(x)	(((x) << 8) & DAI64FS_AUDIV_MASK)

#ifndef __ASSEMBLY__
extern unsigned char daifiq_start, daifiq_end;
#endif

#endif
