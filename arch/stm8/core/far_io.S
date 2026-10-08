/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/toolchain.h>
#include <zephyr/arch/stm8/far_io.h>

	.stm8_model "large"
	.cfi_sections .debug_frame

	.macro STM8_FAR_IO_CFI_START
	.cfi_startproc
	/* Return addresses contain three bytes, unlike the DWARF PC container. */
	.cfi_escape 0x16,11,5,0x11,126,0x22,0x94,3
	.endm

	.comm __stm8_far_io_address,3,1
	.section .text.arch_stm8_far_read8,"ax",@progbits
	.global _arch_stm8_far_read8
	.type _arch_stm8_far_read8,@function
_arch_stm8_far_read8:
	STM8_FAR_IO_CFI_START
	/* The shared indirect-address operand is protected against preemption. */
	push cc
	.cfi_adjust_cfa_offset 1
	.cfi_offset 6,-3
	sim
	ld a,(6,sp)
	ld __stm8_far_io_address,a
	ldw x,(7,sp)
	ldw __stm8_far_io_address+1,x
	ldf a,[__stm8_far_io_address.e]
	pop cc
	.cfi_adjust_cfa_offset -1
	.cfi_restore 6
	retf
	.cfi_endproc
	.size _arch_stm8_far_read8, .-_arch_stm8_far_read8
	.section .text.arch_stm8_far_write8,"ax",@progbits
	.global _arch_stm8_far_write8
	.type _arch_stm8_far_write8,@function
_arch_stm8_far_write8:
	STM8_FAR_IO_CFI_START
	push cc
	.cfi_adjust_cfa_offset 1
	.cfi_offset 6,-3
	sim
	ld a,(6,sp)
	ld __stm8_far_io_address,a
	ldw x,(7,sp)
	ldw __stm8_far_io_address+1,x
	ld a,(9,sp)
	ldf [__stm8_far_io_address.e],a
	pop cc
	.cfi_adjust_cfa_offset -1
	.cfi_restore 6
	retf
	.cfi_endproc
	.size _arch_stm8_far_write8, .-_arch_stm8_far_write8
#ifdef CONFIG_FLASH_STM8
	.comm __stm8_far_io_status,1,1
	.comm __stm8_far_io_mask,1,1
	.comm __stm8_far_io_poll,3,1
	.section .ramfunc.arch_stm8_far_write,"ax",@progbits
	.global _arch_stm8_far_write
	.type _arch_stm8_far_write,@function
_arch_stm8_far_write:
	STM8_FAR_IO_CFI_START
	/* Complete the transfer and status polling without fetching from Flash. */
	push cc
	.cfi_adjust_cfa_offset 1
	.cfi_offset 6,-3
	sim
	ldw y,(11,sp)
	ld a,(ARCH_STM8_FAR_WRITE_POLL,y)
	ld __stm8_far_io_poll,a
	ldw x,y
	ldw x,(ARCH_STM8_FAR_WRITE_POLL+1,x)
	ldw __stm8_far_io_poll+1,x
	ld a,(ARCH_STM8_FAR_WRITE_COUNT,y)
	tnz a
	jreq .Lfar_invalid
	cp a,#4
	jrugt .Lfar_invalid
	push a
	.cfi_adjust_cfa_offset 1
	ldw x,y
	ldw x,(x)
	ld a,(ARCH_STM8_FAR_WRITE_SETUP,y)
	ld (x),a
	ld a,(ARCH_STM8_FAR_WRITE_SETUP+1,y)
	ld (1,x),a
	pop a
	.cfi_adjust_cfa_offset -1
	clrw x
	ld xl,a
	pushw x
	.cfi_adjust_cfa_offset 2
	ld a,(8,sp)
	ld __stm8_far_io_address,a
	ldw x,(9,sp)
	ldw __stm8_far_io_address+1,x
	ldw y,(11,sp)
	popw x
	.cfi_adjust_cfa_offset -2
.Lfar_write:
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
	jrne .Lfar_write
	ldw y,(11,sp)
	ld a,(ARCH_STM8_FAR_WRITE_MASK,y)
	ld __stm8_far_io_mask,a
	ldw y,(ARCH_STM8_FAR_WRITE_STATUS,y)
	ldw x,#0xffff
.Lfar_wait:
	ld a,__stm8_far_io_poll
	or a,__stm8_far_io_poll+1
	or a,__stm8_far_io_poll+2
	jreq .Lfar_status
	pushw x
	.cfi_adjust_cfa_offset 2
	pushw y
	.cfi_adjust_cfa_offset 2
	callf [__stm8_far_io_poll.e]
	popw y
	.cfi_adjust_cfa_offset -2
	popw x
	.cfi_adjust_cfa_offset -2
.Lfar_status:
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
	ld a,(ARCH_STM8_FAR_WRITE_FINISH,y)
	ld (x),a
	ld a,(ARCH_STM8_FAR_WRITE_FINISH+1,y)
	ld (1,x),a
	ld a,__stm8_far_io_status
	.cfi_remember_state
	pop cc
	.cfi_adjust_cfa_offset -1
	.cfi_restore 6
	retf
.Lfar_invalid:
	.cfi_restore_state
	clr a
	pop cc
	.cfi_adjust_cfa_offset -1
	.cfi_restore 6
	retf
	.cfi_endproc
	.size _arch_stm8_far_write, .-_arch_stm8_far_write
#endif
	.section .note.GNU-stack,"",@progbits
