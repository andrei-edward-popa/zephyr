/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/toolchain.h>

	.stm8_model "large"
	.section .text.irq,"ax",@progbits
	.macro STM8_IRQ_ENTRY number
	.global __stm8_irq_\number
__stm8_irq_\number:
	/* Hardware saved CC, A, X, Y and the complete 24-bit PC. */
	/* Set the ISR priority explicitly and clear the DIV/DIVW erratum flag. */
	push #0x28
	pop cc
	ldw x, __stm8_frame_pointer
	pushw x
	/* Match the normal switch frame: RETF continuation followed by software FP. */
	push #lo8(__stm8_irq_resume)
	push #hi8(__stm8_irq_resume)
	push #hh8(__stm8_irq_resume)
	pushw x
	ldw y, sp
	ldw x, #\number
	jpf __stm8_irq_entry
	.endm

	.irp irq,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20
	STM8_IRQ_ENTRY \irq
	.endr
	STM8_IRQ_ENTRY 21
	.irp irq,22,23,24,25,26,27,28,29
	STM8_IRQ_ENTRY \irq
	.endr

#ifdef CONFIG_IRQ_OFFLOAD
	/* TRAP has its own vector and uses the ordinary interrupt context. */
	.global __stm8_trap
__stm8_trap:
	STM8_IRQ_ENTRY CONFIG_NUM_IRQS
#endif

__stm8_irq_entry:
	ld a, xl
#ifdef CONFIG_MULTITHREADING
	ldw x, #(_z_interrupt_stacks + CONFIG_ISR_STACK_SIZE - 1)
	ldw sp, x
#endif
	/* SDCC large ABI: IRQ number in X, interrupted switch handle on the stack. */
	pushw y
	clrw x
	ld xl, a
	callf _z_stm8_irq_dispatch
	ldw sp, x
	popw y
	ldw __stm8_frame_pointer, y
	retf

__stm8_irq_resume:
	popw x
	ldw __stm8_frame_pointer, x
	iret
	.section .note.GNU-stack,"",@progbits
