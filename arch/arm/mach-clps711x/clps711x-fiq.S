/*
 *  Currus Logic CLPS711X DAI FIQ Handler
 *
 *  Author: Alexander Shiyan <shc_work@mail.ru>, 2016-2018
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/assembler.h>
#include <linux/linkage.h>

#include "clps711x-dai.h"

	.text
	.arm

	.global daifiq_end

ENTRY(daifiq_start)
	@ r8  - Buffer address
	@ r9  - Head Pointer
	@ r10 - Tail Pointer
	@ r11 - Registers base virtual address
	@ r12 - Temp

daifiq_loop:
	ldr	r12, [r9]			@ Get Head
	subs	r12, r12, r10			@ Compare Head & Tail
	beq	daifiq_play			@ Send zeroes if Head = Tail

	ldr	r12, [r8, r10]			@ Get buffer value
	add	r10, r10, #4			@ Increment pointer
	bic	r10, r10, #CLPS711X_SNDBUF_SIZE	@ Mask with buffer size

daifiq_play:
	@ Put data to FIFOs
	strh	r12, [r11, #DAIDR1]		@ Left channel
	mov	r12, r12, lsr #16
	strh	r12, [r11, #DAIDR0]		@ Right channel

	@ Check DAI Flags (FIFOs not full)
	ldr	r12, [r11, #DAISR]
	and	r12, r12, #DAISR_RCNF | DAISR_LCNF
	cmp	r12, #DAISR_RCNF | DAISR_LCNF
	beq	daifiq_loop

	@ Clear DAI Interrupt Flags
	mvn	r12, #0
	str	r12, [r11, #DAISR]

	@ Return from FIQ
	subs	pc, lr, #4
daifiq_end:
