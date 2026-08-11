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
/************************************************************************
 * kernel/wdog/wd_initialize.c
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>
#include <queue.h>

#include "wdog/wdog.h"
#include <tinyara/irq.h>

/* MMU Page Table constants for ARMv7-A (Short Descriptor format) */
/* L1 Page Table Entry format */
#define PMD_TYPE_MASK       (3 << 0)
#define PMD_TYPE_PTE        (1 << 0)
#define PMD_PTE_PADDR_MASK  (0xfffffc00)

/* L2 Page Table Entry format - Access Permission bits */
#define PTE_AP0             (1 << 4)
#define PTE_AP1             (2 << 4)
#define PTE_AP2             (1 << 9)

/* External function to get page table base */
extern uint32_t *mmu_get_os_l1_pgtbl(void);
#define PGTABLE_BASE_VADDR  ((uint32_t)mmu_get_os_l1_pgtbl())

/* Inline assembly for cache and TLB operations */
static inline void cp15_clean_dcache_bymva(uint32_t vaddr)
{
	__asm__ volatile ("mcr p15, 0, %0, c7, c14, 1" :: "r"(vaddr) : "memory");
}

static inline void cp15_invalidate_tlb_bymva(uint32_t vaddr)
{
	__asm__ volatile ("mcr p15, 0, %0, c8, c7, 1" :: "r"(vaddr) : "memory");
	__asm__ volatile ("dsb" ::: "memory");
	__asm__ volatile ("isb" ::: "memory");
}

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Public Variables
 ************************************************************************/

/* The g_wdfreelist data structure is a singly linked list of watchdogs
 * available to the system for delayed function use.
 */

sq_queue_t g_wdfreelist;

/* The g_wdactivelist data structure is a singly linked list ordered by
 * watchdog expiration time. When watchdog timers expire,the functions on
 * this linked list are removed and the function is called.
 */

sq_queue_t g_wdactivelist;

/* This is the number of free, pre-allocated watchdog structures in the
 * g_wdfreelist.  This value is used to enforce a reserve for interrupt
 * handlers.
 */

uint16_t g_wdnfree;

/************************************************************************
 * Private Data
 ************************************************************************/

/* g_wdpool is a list of pre-allocated watchdogs. The number of watchdogs
 * in the pool is a configuration item.
 * This is placed in a separate 4KB-aligned section in DDR2 memory.
 */

static struct wdog_s g_wdpool[CONFIG_PREALLOC_WDOGS]
    __attribute__((aligned(4096), section(".wdog_pool")));

/************************************************************************
 * MMU Permission Control for Watchdog Pool
 ************************************************************************/

/* Base address of the watchdog pool */
static uintptr_t g_wdog_pool_vaddr;

/* Original L2 page table entry for the watchdog pool region */
static uint32_t g_wdog_pool_pte_saved;

/* Flag indicating if the watchdog pool is currently read-only */
static bool g_wdog_pool_is_ro = false;

/* Flag indicating if MMU permission control is initialized */
static bool g_wdog_mmu_initialized = false;

/************************************************************************
 * Name: wd_mmu_init
 *
 * Description:
 *   Initialize MMU permission control for the watchdog pool.
 *   This function saves the original page table entry for the watchdog pool.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   This function must be called after the MMU is enabled and after
 *   wd_initialize() has been called.
 *
 ************************************************************************/

void wd_mmu_init(void)
{
	uint32_t *l1table;
	uint32_t l1entry;
	uint32_t *l2table;
	uint32_t index;
	
	if (g_wdog_mmu_initialized) {
		return;
	}
	
	g_wdog_pool_vaddr = (uintptr_t)&g_wdpool[0];
	
	/* Get L1 page table entry */
	l1table = (uint32_t *)PGTABLE_BASE_VADDR;
	index = (g_wdog_pool_vaddr >> 20) & 0xfff;
	l1entry = l1table[index];
	
	/* Check if this is a page table entry (L2) */
	if ((l1entry & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
		/* Get L2 table base address */
		l2table = (uint32_t *)(l1entry & PMD_PTE_PADDR_MASK);
		
		/* Get L2 page table entry */
		index = (g_wdog_pool_vaddr >> 12) & 0xff;
		g_wdog_pool_pte_saved = l2table[index];
		
		g_wdog_mmu_initialized = true;
		g_wdog_pool_is_ro = false; /* Start with read-write */
	}
}

/************************************************************************
 * Name: wd_set_wdogpool_rw
 *
 * Description:
 *   Change the watchdog pool memory region to read-write access.
 *   This allows modification of watchdog structures in the pool.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   wd_mmu_init() must have been called first.
 *   This function disables interrupts during the page table modification.
 *
 ************************************************************************/

void wd_set_wdogpool_rw(void)
{
	uint32_t *l1table;
	uint32_t l1entry;
	uint32_t *l2table;
	uint32_t index;
	uint32_t newpte;
	irqstate_t flags;
	
	if (!g_wdog_mmu_initialized) {
		return;
	}
	
	if (!g_wdog_pool_is_ro) {
		return; /* Already read-write */
	}
	
	/* Disable interrupts during page table modification */
	flags = enter_critical_section();
	
	/* Get L1 page table entry */
	l1table = (uint32_t *)PGTABLE_BASE_VADDR;
	index = (g_wdog_pool_vaddr >> 20) & 0xfff;
	l1entry = l1table[index];
	
	/* Check if this is a page table entry (L2) */
	if ((l1entry & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
		/* Get L2 table base address */
		l2table = (uint32_t *)(l1entry & PMD_PTE_PADDR_MASK);
		
		/* Get L2 page table entry index */
		index = (g_wdog_pool_vaddr >> 12) & 0xff;
		
		/* Set page table entry to read-write (clear RO bit, set RW) */
		newpte = g_wdog_pool_pte_saved & ~PTE_AP2; /* Clear AP2 for RW */
		newpte |= PTE_AP1; /* Set AP1 for RW */
		l2table[index] = newpte;
		
		/* Flush data cache for the modified page table entry */
		cp15_clean_dcache_bymva((uint32_t)&l2table[index]);
		
		/* Invalidate TLB for the watchdog pool address */
		cp15_invalidate_tlb_bymva(g_wdog_pool_vaddr);
		
		g_wdog_pool_is_ro = false;
	}
	
	leave_critical_section(flags);
}

/************************************************************************
 * Name: wd_set_wdogpool_ro
 *
 * Description:
 *   Change the watchdog pool memory region back to read-only access.
 *   This protects watchdog structures from unintended modifications.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   wd_mmu_init() must have been called first.
 *   This function disables interrupts during the page table modification.
 *
 ************************************************************************/

void wd_set_wdogpool_ro(void)
{
	uint32_t *l1table;
	uint32_t l1entry;
	uint32_t *l2table;
	uint32_t index;
	uint32_t newpte;
	irqstate_t flags;
	
	if (!g_wdog_mmu_initialized) {
		return;
	}
	
	if (g_wdog_pool_is_ro) {
		return; /* Already read-only */
	}
	
	/* Disable interrupts during page table modification */
	flags = enter_critical_section();
	
	/* Get L1 page table entry */
	l1table = (uint32_t *)PGTABLE_BASE_VADDR;
	index = (g_wdog_pool_vaddr >> 20) & 0xfff;
	l1entry = l1table[index];
	
	/* Check if this is a page table entry (L2) */
	if ((l1entry & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
		/* Get L2 table base address */
		l2table = (uint32_t *)(l1entry & PMD_PTE_PADDR_MASK);
		
		/* Get L2 page table entry index */
		index = (g_wdog_pool_vaddr >> 12) & 0xff;
		
		/* Set page table entry to read-only (set AP2, clear AP1) */
		newpte = g_wdog_pool_pte_saved | PTE_AP2; /* Set AP2 for RO */
		newpte &= ~PTE_AP1; /* Clear AP1 for RO */
		l2table[index] = newpte;
		
		/* Flush data cache for the modified page table entry */
		cp15_clean_dcache_bymva((uint32_t)&l2table[index]);
		
		/* Invalidate TLB for the watchdog pool address */
		cp15_invalidate_tlb_bymva(g_wdog_pool_vaddr);
		
		g_wdog_pool_is_ro = true;
	}
	
	leave_critical_section(flags);
}

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: wd_is_prealloc
 *
 * Description:
 * This function checks if the wdog is pre- allocated or not
 *
 * Parameters:
 *   wdog - the address of wdog (WDOG_ID)
 *
 * Return Value:
 *   true  - if wdog is preallocated
 *   false - otherwise
 *
 ************************************************************************/

bool wd_is_prealloc(WDOG_ID wdog)
{
	uintptr_t wdog_ptr = (uintptr_t)wdog;
	uintptr_t start = (uintptr_t)(&g_wdpool[0]);
	uintptr_t end = (uintptr_t)(&g_wdpool[CONFIG_PREALLOC_WDOGS - 1]);
	
	if (end < start) {
		start = start ^ end;
		end = start ^ end;
		start = start ^ end;
	}

	return (wdog_ptr >= start) && (wdog_ptr <= end) && (((wdog_ptr - start) % sizeof(struct wdog_s)) == 0);
}

/************************************************************************
 * Name: wd_initialize
 *
 * Description:
 * This function initializes the watchdog data structures
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   This function must be called early in the initialization sequence
 *   before the timer interrupt is attached and before any watchdog
 *   services are used.
 *
 ************************************************************************/

void wd_initialize(void)
{
	FAR struct wdog_s *wdog = g_wdpool;
	int i;

	/* Initialize watchdog lists */

	sq_init(&g_wdfreelist);
	sq_init(&g_wdactivelist);

	/* The g_wdfreelist must be loaded at initialization time to hold the
	 * configured number of watchdogs.
	 */

	for (i = 0; i < CONFIG_PREALLOC_WDOGS; i++) {
		sq_addlast((FAR sq_entry_t *)wdog++, &g_wdfreelist);
	}

	/* All watchdogs are free */

	g_wdnfree = CONFIG_PREALLOC_WDOGS;
}
