/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

	.stm8_model "large"
	.section .text.z_stm8_arch_switch,"ax",@progbits
	.global _z_stm8_arch_switch
	.type _z_stm8_arch_switch,@function
_z_stm8_arch_switch:
	/* SDCC large ABI: switch_to in X, switched_from at SP + 4. */
	/* IRQs are locked by the scheduler. All CPU registers are call-clobbered. */
	ldw y, __stm8_frame_pointer
	pushw y
	ldw y, (6,sp)
	pushw x
	ldw x, sp
	addw x, #2
	ldw (y), x
	popw x
	ldw sp, x
	popw y
	ldw __stm8_frame_pointer, y
	retf
	.section .note.GNU-stack,"",@progbits
