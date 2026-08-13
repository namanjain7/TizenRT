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
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * apps/examples/watchdog/wdog_align_test.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <tinyara/wdog.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int wdog_align_test_main(int argc, char *argv[])
#endif
{
	struct wdog_s *wdog;
	uintptr_t addr;
	uintptr_t remainder;
	int is_aligned;

	printf("=== Watchdog Structure 4096-byte Alignment Test ===\n\n");

	/* Allocate a watchdog */
	wdog = wd_create();
	if (!wdog) {
		printf("ERROR: Failed to create watchdog\n");
		return -1;
	}

	/* Get the address of the watchdog structure */
	addr = (uintptr_t)wdog;

	/* Check if the address is 4096-byte aligned */
	remainder = addr % 4096;
	is_aligned = (remainder == 0);

	printf("Watchdog structure address: 0x%lx\n", (unsigned long)addr);
	printf("Address %% 4096 = %lu\n", (unsigned long)remainder);
	printf("Expected remainder: 0\n");
	printf("Alignment test: %s\n\n", is_aligned ? "PASSED" : "FAILED");

	/* Also print struct size for reference */
	printf("sizeof(struct wdog_s) = %lu bytes\n", (unsigned long)sizeof(struct wdog_s));

	/* Cleanup */
	wd_delete(wdog);

	if (is_aligned) {
		printf("\nSUCCESS: struct wdog_s is properly 4096-byte aligned!\n");
		return 0;
	} else {
		printf("\nFAILURE: struct wdog_s is NOT 4096-byte aligned!\n");
		return -1;
	}
}
