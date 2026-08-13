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
#ifndef __KERNEL_WDOG_WD_MMU_PROTECT_H
#define __KERNEL_WDOG_WD_MMU_PROTECT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#ifdef CONFIG_WDOG_MMU_PROTECT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* External function to get page table base */
extern uint32_t *mmu_get_os_l1_pgtbl(void);
#define PGTABLE_BASE_VADDR  ((uint32_t)mmu_get_os_l1_pgtbl())

/* Write Domain Access Control Register (DACR) — takes effect immediately, no TLB flush */
static inline void cp15_write_dacr(uint32_t val)
{
	__asm__ volatile("mcr p15, 0, %0, c3, c0, 0" :: "r"(val) : "memory");
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void wd_mmu_protect_init(void);

#endif /* CONFIG_WDOG_MMU_PROTECT */

#endif /* __KERNEL_WDOG_WD_MMU_PROTECT_H */
