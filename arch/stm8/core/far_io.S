/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/toolchain.h>
#include <zephyr/arch/stm8/far_io.h>

	.stm8_model "large"
	.comm __stm8_far_io_address,3,1
	.section .text.arch_stm8_far_read8,"ax",@progbits
	.global _arch_stm8_far_read8
	.type _arch_stm8_far_read8,@function
_arch_stm8_far_read8:
	/* The shared indirect-address operand is protected against preemption. */
	push cc
	sim
	ld a,(6,sp)
	ld __stm8_far_io_address,a
	ldw x,(7,sp)
	ldw __stm8_far_io_address+1,x
	ldf a,[__stm8_far_io_address.e]
	pop cc
	retf
	.section .text.arch_stm8_far_write8,"ax",@progbits
	.global _arch_stm8_far_write8
	.type _arch_stm8_far_write8,@function
_arch_stm8_far_write8:
	push cc
	sim
	ld a,(6,sp)
	ld __stm8_far_io_address,a
	ldw x,(7,sp)
	ldw __stm8_far_io_address+1,x
	ld a,(9,sp)
	ldf [__stm8_far_io_address.e],a
	pop cc
	retf
#ifdef CONFIG_FLASH_STM8
	.comm __stm8_far_io_status,1,1
	.comm __stm8_far_io_mask,1,1
	.section .ramfunc.arch_stm8_far_write4,"ax",@progbits
	.global _arch_stm8_far_write4
	.type _arch_stm8_far_write4,@function
_arch_stm8_far_write4:
	/* Complete the transfer and status polling without fetching from Flash. */
	push cc
	sim
	ldw y,(11,sp)
	ldw x,y
	ldw x,(x)
	ld a,(ARCH_STM8_FAR_WRITE4_SETUP,y)
	ld (x),a
	ld a,(ARCH_STM8_FAR_WRITE4_SETUP+1,y)
	ld (1,x),a
	ld a,(6,sp)
	ld __stm8_far_io_address,a
	ldw x,(7,sp)
	ldw __stm8_far_io_address+1,x
	ldw y,(9,sp)
	ldw x,#4
.Lfar_write4:
	ld a,(y)
	ldf [__stm8_far_io_address.e],a
	incw y
	inc __stm8_far_io_address+2
	jrne .Lfar_next
	inc __stm8_far_io_address+1
	jrne .Lfar_next
	inc __stm8_far_io_address
.Lfar_next:
	decw x
	jrne .Lfar_write4
	ldw y,(11,sp)
	ld a,(ARCH_STM8_FAR_WRITE4_MASK,y)
	ld __stm8_far_io_mask,a
	ldw y,(ARCH_STM8_FAR_WRITE4_STATUS,y)
	ldw x,#0xffff
.Lfar_wait:
	ld a,(y)
	ld __stm8_far_io_status,a
	and a,__stm8_far_io_mask
	jrne .Lfar_ready
	decw x
	jrne .Lfar_wait
	clr __stm8_far_io_status
.Lfar_ready:
	ldw y,(11,sp)
	ldw x,y
	ldw x,(x)
	ld a,(ARCH_STM8_FAR_WRITE4_FINISH,y)
	ld (x),a
	ld a,(ARCH_STM8_FAR_WRITE4_FINISH+1,y)
	ld (1,x),a
	ld a,__stm8_far_io_status
	pop cc
	retf
#endif
	.section .note.GNU-stack,"",@progbits
