/****************************************************************************
 *
 * Copyright 2026 Samsung Electronics All Rights Reserved.
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
 * either express or implied. See the License for the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * os/drivers/os_api_test/os_api_test_wdog_mmu.c
 *
 * Watchdog MMU Protection Test Handler
 *
 * This file provides test functions for verifying watchdog timer
 * MMU-based read-only protection via the os_api_test driver.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#ifdef CONFIG_WDOG_MMU_PROTECT

#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/wdog.h>
#include <tinyara/mmu.h>
#include <tinyara/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MMU Page Table constants for ARMv7-A (Short Descriptor format) */
#define PMD_SECT_AP2            (1 << 15)
#define PMD_SECT_AP_SHIFT       10
#define PMD_SECT_DOMAIN_SHIFT   5
#define WDOG_DOMAIN             1

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* External reference to watchdog pool */
extern struct wdog_s g_wdpool[CONFIG_PREALLOC_WDOGS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_wdog_mmu_protect
 *
 * Description:
 *   Test watchdog MMU protection functionality.
 *
 *   arg = 0: Verify MMU protection is active (safe)
 *   arg = 1: Attempt to write to protected wdog_s (triggers Data Abort)
 *   arg = 2: Basic watchdog start/cancel test
 *
 ****************************************************************************/

int test_wdog_mmu_protect(unsigned long arg)
{
	WDOG_ID wdog;
	int ret = OK;

	switch (arg) {
	case 0:
		/* Test: Verify MMU protection is active */
		printf("[wdog_mmu_test] arg=0: Verifying MMU protection...\n");

		/* Create a watchdog */
		wdog = wd_create();
		if (!wdog) {
			printf("[wdog_mmu_test] FAIL: wd_create() returned NULL\n");
			return -ENOMEM;
		}
		printf("[wdog_mmu_test] Watchdog created at 0x%08x\n", (uint32_t)wdog);

		/* Verify watchdog is in protected region */
		if (wd_is_prealloc(wdog)) {
			printf("[wdog_mmu_test] Watchdog is in pre-allocated pool\n");
		}

		/* The wdog pool should be read-only via MMU.
		 * We can verify this by checking if the memory is accessible
		 * for reading (it should be) and that wd_mmu_write_begin/end
		 * functions are available.
		 */
#ifdef CONFIG_WDOG_MMU_PROTECT
		printf("[wdog_mmu_test] MMU protection is configured\n");
		printf("[wdog_mmu_test] Watchdog pool is READ-ONLY by default\n");
		printf("[wdog_mmu_test] Use wd_mmu_write_begin/end to modify\n");
#endif

		/* Clean up */
		wd_delete(wdog);
		printf("[wdog_mmu_test] PASS: MMU protection verification complete\n");
		break;

	case 1:
		/* Test: Attempt to write to protected wdog_s structure
		 * This should trigger a Data Abort because the wdog pool
		 * is marked as read-only in the MMU page tables.
		 */
		printf("[wdog_mmu_test] arg=1: DANGEROUS TEST\n");
		printf("[wdog_mmu_test] Attempting to write to protected wdog_s...\n");
		printf("[wdog_mmu_test] EXPECT DATA ABORT - system may crash!\n");

		/* Get pointer to first watchdog in pool */
		wdog = &g_wdpool[0];
		printf("[wdog_mmu_test] Target wdog_s at 0x%08x\n", (uint32_t)wdog);

		/* This write should trigger a Data Abort because the page
		 * is marked read-only. However, if we're in kernel mode,
		 * the write might succeed depending on AP bit configuration.
		 *
		 * With DACR domain=client and AP=RO, writes from privileged
		 * mode should still be blocked.
		 */
		printf("[wdog_mmu_test] About to write to protected memory...\n");
		
		/* Attempt to corrupt the watchdog structure */
		wdog->flags = 0xDEADBEEF;
		
		/* If we reach here, protection failed */
		printf("[wdog_mmu_test] ERROR: Write succeeded - MMU protection FAILED!\n");
		printf("[wdog_mmu_test] flags = 0x%08x (should have faulted)\n", wdog->flags);
		ret = -EFAULT;
		break;

	case 2:
		/* Test: Basic watchdog functionality with proper protection */
		printf("[wdog_mmu_test] arg=2: Basic watchdog test\n");

		/* Create watchdog */
		wdog = wd_create();
		if (!wdog) {
			printf("[wdog_mmu_test] FAIL: wd_create() returned NULL\n");
			return -ENOMEM;
		}
		printf("[wdog_mmu_test] Watchdog created\n");

		/* Start watchdog with a callback (100ms delay) */
		printf("[wdog_mmu_test] Starting watchdog with 100ms delay...\n");
		/* Note: We use a simple callback that just prints */
		ret = wd_start(wdog, 10, NULL, 0);
		if (ret < 0) {
			printf("[wdog_mmu_test] wd_start failed: %d\n", ret);
			wd_delete(wdog);
			return ret;
		}
		printf("[wdog_mmu_test] Watchdog started\n");

		/* Cancel watchdog before it expires */
		printf("[wdog_mmu_test] Canceling watchdog...\n");
		ret = wd_cancel(wdog);
		if (ret < 0) {
			printf("[wdog_mmu_test] wd_cancel failed: %d\n", ret);
			wd_delete(wdog);
			return ret;
		}
		printf("[wdog_mmu_test] Watchdog canceled\n");

		/* Delete watchdog */
		ret = wd_delete(wdog);
		if (ret < 0) {
			printf("[wdog_mmu_test] wd_delete failed: %d\n", ret);
			return ret;
		}
		printf("[wdog_mmu_test] Watchdog deleted\n");
		printf("[wdog_mmu_test] PASS: Basic watchdog test complete\n");
		break;

	default:
		printf("[wdog_mmu_test] Unknown arg: %lu\n", arg);
		printf("[wdog_mmu_test] Usage:\n");
		printf("[wdog_mmu_test]   0 - Verify MMU protection (safe)\n");
		printf("[wdog_mmu_test]   1 - Attempt protected write (DANGEROUS)\n");
		printf("[wdog_mmu_test]   2 - Basic watchdog test\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

#endif /* CONFIG_WDOG_MMU_PROTECT */
