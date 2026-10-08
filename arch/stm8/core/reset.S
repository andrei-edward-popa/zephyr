/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/toolchain.h>

	.stm8_model "large"
	.comm __stm8_frame_pointer,2,1
	.section .vectors,"ax",@progbits
	.global __stm8_vectors
__stm8_vectors:
	.byte 0x82
	.3byte __start
	.byte 0x82
#ifdef CONFIG_IRQ_OFFLOAD
	.3byte __stm8_trap
#else
	.3byte __stm8_unhandled_irq
#endif
	.irp irq,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29
	.byte 0x82
	.3byte __stm8_irq_\irq
	.endr

	.section .text.startup,"ax",@progbits
	.global __start
	.type __start,@function
__start:
	sim
	ldw x, #(_z_interrupt_stacks + CONFIG_ISR_STACK_SIZE - 1)
	ldw sp, x
	ldw x, #__bss_start
	ldw y, #__bss_end
.Lzero:
	pushw y
	cpw x, (1,sp)
	popw y
	jreq .Lcopy_begin
	clr (x)
	incw x
	jra .Lzero
.Lcopy_begin:
	ldw x, #__data_region_load_start
	ldw y, #__data_region_start
.Lcopy:
	cpw y, #__data_region_end
	jreq .Lrun
	ld a, (x)
	ld (y), a
	incw x
	incw y
	jra .Lcopy
.Lrun:
	callf _z_cstart
__stm8_unhandled_irq:
	sim
	halt
	jra __stm8_unhandled_irq
	.section .note.GNU-stack,"",@progbits
