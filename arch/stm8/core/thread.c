/* SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa */
/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/kernel.h>
#include <kernel_internal.h>

struct stm8_initial_frame {
	uint16_t frame_pointer;
	void (*pc)(void);
};

static FUNC_NORETURN void stm8_thread_start(void)
{
	struct k_thread *thread = _current;

	__asm__ volatile("rim" : : : "cc", "memory");
	z_thread_entry(thread->arch.entry, thread->arch.p1, thread->arch.p2, thread->arch.p3);
	CODE_UNREACHABLE;
}

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack, char *stack_ptr,
		     k_thread_entry_t entry, void *p1, void *p2, void *p3)
{
	struct stm8_initial_frame *frame = (void *)(stack_ptr - sizeof(*frame));

	ARG_UNUSED(stack);
	thread->arch.entry = entry;
	thread->arch.p1 = p1;
	thread->arch.p2 = p2;
	thread->arch.p3 = p3;
	frame->frame_pointer = 0;
	frame->pc = stm8_thread_start;
	/* POPW and RETF read above SP, which points below the saved frame. */
	thread->switch_handle = (char *)frame - 1;
}

int arch_coprocessors_disable(struct k_thread *thread)
{
	ARG_UNUSED(thread);
	return 0;
}
