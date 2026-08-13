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
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <tinyara/os_api_test_drv.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: show_usage
 ****************************************************************************/
static void show_usage(void)
{
	printf("\n=== Watchdog Protection Test ===\n");
	printf("Usage: hello <test_num>\n\n");
	printf("Test numbers:\n");
	printf("  0 - Basic watchdog functionality test (create/start/cancel/delete)\n");
	printf("  1 - Verify MMU protection is active (safe read-only check)\n");
	printf("  2 - Attempt write to protected wdog_s (DANGEROUS - may crash!)\n");
	printf("\n");
}

/****************************************************************************
 * Name: run_wdog_test
 ****************************************************************************/
static int run_wdog_test(int test_num)
{
	int fd;
	int ret;

	printf("\n[hello] Opening %s...\n", OS_API_TEST_DRVPATH);

	fd = open(OS_API_TEST_DRVPATH, O_RDWR);
	if (fd < 0) {
		printf("[hello] FAIL: cannot open %s (errno %d)\n",
		       OS_API_TEST_DRVPATH, errno);
		printf("[hello] Is CONFIG_DRIVERS_OS_API_TEST enabled?\n");
		return -1;
	}

	printf("[hello] Running watchdog test %d...\n", test_num);

	if (test_num == 2) {
		printf("\n");
		printf("!!! WARNING !!!\n");
		printf("This test will attempt to write to read-only memory.\n");
		printf("The system MAY CRASH or REBOOT!\n");
		printf("\n");
	}

	ret = ioctl(fd, TESTIOC_WDOG_MMU_PROTECT, test_num);

	close(fd);

	if (ret < 0) {
		printf("[hello] Test returned error: %d (errno: %d)\n", ret, errno);
	} else {
		printf("[hello] Test completed successfully\n");
	}

	return ret;
}

/****************************************************************************
 * hello_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	int test_num;

	if (argc < 2) {
		show_usage();
		return 0;
	}

	test_num = atoi(argv[1]);

	if (test_num < 0 || test_num > 2) {
		printf("Invalid test number: %d\n", test_num);
		show_usage();
		return -1;
	}

	return run_wdog_test(test_num);
}
