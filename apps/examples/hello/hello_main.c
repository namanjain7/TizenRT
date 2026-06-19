/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int atexit_called = 0;
static int onexit_called = 0;
static int destructor_called = 0;
static pthread_key_t key;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_atexit_func
 * Description: atexit callback - tests task_atexit() path
 ****************************************************************************/

void test_atexit_func(void)
{
	printf("[EXIT_TEST] atexit callback executed!\n");
	atexit_called = 1;
}

/****************************************************************************
 * Name: test_onexit_func
 * Description: on_exit callback - tests task_onexit() path
 ****************************************************************************/

void test_onexit_func(int status, void *arg)
{
	printf("[EXIT_TEST] on_exit callback executed! status=%d, arg=%p\n", status, arg);
	onexit_called = 1;
}

/****************************************************************************
 * Name: test_destructor
 * Description: pthread destructor - tests pthread_key_destroy() path
 ****************************************************************************/

void test_destructor(void *arg)
{
#if defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_8M_BASE__) || defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_7A__)
	uint32_t control;
	__asm__ volatile ("mrs %0, control" : "=r" (control));
	printf("[EXIT_TEST] pthread destructor executed! arg=%p\n", arg);
	printf("[EXIT_TEST] CONTROL register: 0x%08x (bit0=1 means unprivileged)\n", control);
#else
	printf("[EXIT_TEST] pthread destructor executed! arg=%p\n", arg);
	printf("[EXIT_TEST] CONTROL register read not supported on this architecture\n");
#endif
	destructor_called = 1;
}

/****************************************************************************
 * Name: pthread_func
 * Description: pthread that sets key data to trigger destructor
 ****************************************************************************/

void *pthread_func(void *arg)
{
	printf("[EXIT_TEST] pthread running, setting key data...\n");
	pthread_setspecific(key, (void*)0x12345678);
	return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hello_main / main
 * Description: Test exit callbacks (atexit, on_exit, pthread destructor)
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("[EXIT_TEST] Exit Callback Test Starting\n");
	printf("[EXIT_TEST] This test verifies that exit callbacks are invoked correctly\n");
	printf("[EXIT_TEST] In protected/kernel builds, callbacks should execute in user mode\n\n");
	
	// Register atexit - this will test task_atexit() path
	printf("[EXIT_TEST] Registering atexit...\n");
	atexit(test_atexit_func);
	
	// Register on_exit - this will test task_onexit() path
	printf("[EXIT_TEST] Registering on_exit...\n");
	on_exit(test_onexit_func, (void*)0xDEADBEEF);
	
	// Create pthread key with destructor - this will test pthread_key_destroy() path
	printf("[EXIT_TEST] Creating pthread key with destructor...\n");
	pthread_key_create(&key, test_destructor);
	
	// Create and join pthread to trigger destructor
	pthread_t thread;
	pthread_create(&thread, NULL, pthread_func, NULL);
	pthread_join(thread, NULL);
	printf("[EXIT_TEST] pthread joined, destructor should have been called\n\n");
	
	printf("[EXIT_TEST] Test Complete. Exiting...\n");
	printf("[EXIT_TEST] Results: atexit=%d, onexit=%d, destructor=%d\n", 
		   atexit_called, onexit_called, destructor_called);
	
	return 0;
}
