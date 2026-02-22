/*
 * Copyright (c) 2025 Andrei-Edward Popa
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>

#define STACK_SIZE 256
#define PRIORITY 5

K_THREAD_STACK_DEFINE(stack1, STACK_SIZE);
K_THREAD_STACK_DEFINE(stack2, STACK_SIZE);

struct k_thread thread1_data;
struct k_thread thread2_data;

K_SEM_DEFINE(my_sem, 0, 1);

void thread_take(void *p1, void *p2, void *p3)
{
	while (1) {
		printf("Before take\n");
		k_sem_take(&my_sem, K_FOREVER);
		printf("After take!\n");
	}
}

void thread_give(void *p1, void *p2, void *p3)
{
	while (1) {
		printf("Before give\n");
		k_sem_give(&my_sem);
		printf("After give\n");
		k_msleep(100);
	}
}

int main(void)
{
	printf("Start semaphore test\n");

	k_thread_create(&thread1_data, stack1, STACK_SIZE,
			thread_take, NULL, NULL, NULL,
			PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&thread2_data, stack2, STACK_SIZE,
			thread_give, NULL, NULL, NULL,
			PRIORITY, 0, K_NO_WAIT);

	while(1) {
		printf("Main thread\n");
		k_msleep(1000);
	}

	return 0;
}
