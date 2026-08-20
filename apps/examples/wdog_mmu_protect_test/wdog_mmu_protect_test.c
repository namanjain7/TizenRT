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
 * apps/examples/wdog_mmu_protect_test/wdog_mmu_protect_test.c
 *
 * Watchdog MMU Protection Test Application
 *
 * This test application verifies the MMU-based Read-Only protection for
 * watchdog timer structures (struct wdog_s) via the os_api_test driver.
 *
 * Usage from TASH:
 *   wdog_mmu_protect_test 0   - Verify MMU protection is active (safe)
 *   wdog_mmu_protect_test 1   - Attempt write to protected wdog_s (Data Abort!)
 *   wdog_mmu_protect_test 2   - Basic watchdog start/cancel test
 *
 * NOTE: The '1' command triggers a fatal Data Abort!
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <tinyara/config.h>
#include <tinyara/fs/ioctl.h>
#include <tinyara/os_api_test_drv.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: show_usage
 ****************************************************************************/
static void show_usage(void)
{
	printf("\n=== Watchdog MMU Protection Test ===\n");
	printf("Usage: wdog_mmu_protect_test <arg>\n\n");
	printf("Arguments:\n");
	printf("  0 - Verify MMU protection is active (safe test)\n");
	printf("      Creates a watchdog and confirms MMU protection is configured.\n");
	printf("\n");
	printf("  1 - Attempt write to protected wdog_s (DANGEROUS!)\n");
	printf("      Attempts to directly write to the protected watchdog pool.\n");
	printf("      This SHOULD trigger a Data Abort exception.\n");
	printf("\n");
	printf("  2 - Basic watchdog functionality test\n");
	printf("      Tests watchdog create/start/cancel/delete operations.\n");
	printf("\n");
	printf("NOTE: Argument '1' triggers a Data Abort to demonstrate\n");
	printf("      that MMU protection is working. System may crash!\n");
	printf("\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int wdog_mmu_protect_test_main(int argc, char *argv[])
#endif
{
	int fd;
	int ret;
	long arg = 0;

	if (argc < 2) {
		show_usage();
		return -1;
	}

	arg = atol(argv[1]);

	printf("\n=== Watchdog MMU Protection Test (arg=%ld) ===\n", arg);

	if (arg == 1) {
		printf("WARNING: This will trigger a Data Abort!\n");
		printf("The system may crash or reboot.\n\n");
	}

	printf("[wdog_mmu_protect_test] Opening %s...\n", OS_API_TEST_DRVPATH);

	fd = open(OS_API_TEST_DRVPATH, O_RDWR);
	if (fd < 0) {
		printf("[wdog_mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
		       OS_API_TEST_DRVPATH, errno);
		printf("[wdog_mmu_protect_test] Is CONFIG_DRIVERS_OS_API_TEST enabled?\n");
		return -1;
	}

	printf("[wdog_mmu_protect_test] Device opened successfully (fd=%d)\n", fd);

	if (arg == 1) {
		printf("[wdog_mmu_protect_test] >>> EXPECT DATA ABORT - system will crash <<<\n");
	}

	ret = ioctl(fd, TESTIOC_WDOG_MMU_PROTECT, arg);

	if (arg == 1 && ret == OK) {
		/* Should not reach here - MMU protection should have triggered Data Abort */
		printf("\n[wdog_mmu_protect_test] ERROR: ioctl returned OK, Data Abort was NOT triggered!\n");
		printf("[wdog_mmu_protect_test] MMU protection FAILED to detect corruption attempt\n");
		close(fd);
		return -1;
	}

	if (ret < 0) {
		printf("[wdog_mmu_protect_test] ioctl returned %d (errno %d)\n", ret, errno);
		close(fd);
		return ret;
	}

	printf("[wdog_mmu_protect_test] Test completed successfully\n");
	close(fd);
	return 0;
}
