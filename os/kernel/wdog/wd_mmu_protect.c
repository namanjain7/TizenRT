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
 * kernel/wdog/wd_mmu_protect.c
 *
 *   Copyright (C) 2007, 2009, 2014 Gregory Nutt. All rights reserved.
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

/************************************************************************
 * Included Files
 ************************************************************************/

#include <tinyara/config.h>

#ifdef CONFIG_WDOG_MMU_PROTECT

#include <stdint.h>
#include <tinyara/irq.h>
#include <tinyara/mmu.h>
#include "wdog/wdog.h"

/* External reference to watchdog pool defined in wd_initialize.c */
extern struct wdog_s g_wdpool[CONFIG_PREALLOC_WDOGS];

/************************************************************************
 * Private Data
 ************************************************************************/

/* Base address of the watchdog pool */
static uintptr_t g_wdog_pool_vaddr;

/* Flag indicating if MMU permission control is initialized */
static bool g_wdog_mmu_initialized = false;

/* Nesting counter for reentrant wd_mmu_write_begin/end calls */
static int g_wdog_mmu_nest_count = 0;

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Name: wd_mmu_protect_init
 *
 * Description:
 *   Initialize MMU permission control for the watchdog pool.
 *   Uses ARM-specific MMU functions for L1 page table manipulation.
 ************************************************************************/
void wd_mmu_protect_init(void)
{
	uint32_t *l1table;

	if (g_wdog_mmu_initialized) {
		return;
	}

	g_wdog_pool_vaddr = (uintptr_t)&g_wdpool[0];

	/* Get L1 page table */
	l1table = mmu_get_os_l1_pgtbl();

	/* Initialize ARM-specific watchdog pool protection */
	arm_mmu_wdog_pool_init(g_wdog_pool_vaddr, l1table);

	g_wdog_mmu_initialized = true;

	lldbg("WDOG_MMU: protection initialized (DACR domain 1, RO)\n");
}

/************************************************************************
 * Name: wd_mmu_write_begin
 *
 * Description:
 *   Change the watchdog pool memory region to read-write access.
 *   Uses DACR domain manager mode to allow writes.
 *   Uses a nesting counter to handle reentrant calls.
 ************************************************************************/
void wd_mmu_write_begin(void)
{
	if (!g_wdog_mmu_initialized) {
		return;
	}

	/* Nesting: if already in a write section, just increment counter */
	if (g_wdog_mmu_nest_count > 0) {
		g_wdog_mmu_nest_count++;
		return;
	}

	/* Set Read-Write: set domain 1 = manager (ignores AP bits, full access)
	 * Just one mcr instruction — no TLB invalidation, no barriers needed.
	 */
	arm_mmu_wdog_set_readwrite();

	g_wdog_mmu_nest_count = 1;
}

/************************************************************************
 * Name: wd_mmu_write_end
 *
 * Description:
 *   Change the watchdog pool memory region back to read-only access.
 *   Uses DACR domain client mode to enforce read-only.
 *   Uses a nesting counter to handle reentrant calls.
 ************************************************************************/
void wd_mmu_write_end(void)
{
	if (!g_wdog_mmu_initialized) {
		return;
	}

	/* Nesting: decrement counter, only re-protect when count reaches 0 */
	if (g_wdog_mmu_nest_count > 1) {
		g_wdog_mmu_nest_count--;
		return;
	}

	g_wdog_mmu_nest_count = 0;

	/* Set Read-Only: set domain 1 = client (enforces AP bits = RO)
	 * Just one mcr instruction — no TLB invalidation, no barriers needed.
	 */
	arm_mmu_wdog_set_readonly();
}

#endif /* CONFIG_WDOG_MMU_PROTECT */
