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
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <tinyara/sramlog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SRAMLOG_CMD_READ	"read"
#define SRAMLOG_CMD_WRITE	"write"
#define SRAMLOG_CMD_ERASE	"erase"

#define SRAMLOG_BYTES_PER_LINE	16

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Kept in .bss of the application image, not in the SRAM window itself */

static uint8_t g_sramlog_buf[SRAMLOG_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sramlog_print_usage
 ****************************************************************************/

static void sramlog_print_usage(FAR const char *progname)
{
	printf("Usage: %s <" SRAMLOG_CMD_READ "|" SRAMLOG_CMD_WRITE "|"
		SRAMLOG_CMD_ERASE ">\n", progname);
	printf("  " SRAMLOG_CMD_READ "  : dump the %lu byte SRAM window at 0x%08lx\n",
		(unsigned long)SRAMLOG_SIZE, (unsigned long)SRAMLOG_BASE);
	printf("  " SRAMLOG_CMD_WRITE ": fill the window with a test pattern\n");
	printf("  " SRAMLOG_CMD_ERASE ": clear the window to zero\n");
}

/****************************************************************************
 * Name: sramlog_hexdump
 *
 * Description:
 *   Dump the window contents that were copied out of the driver, one line
 *   of 16 bytes at a time.
 *
 ****************************************************************************/

static void sramlog_hexdump(FAR const uint8_t *buf, uint32_t len)
{
	uint32_t offset;
	uint32_t i;

	printf("sramlog: %lu bytes read from 0x%08lx\n", (unsigned long)len,
		(unsigned long)SRAMLOG_BASE);

	for (offset = 0; offset < len; offset += SRAMLOG_BYTES_PER_LINE) {
		printf("  %04lx:", (unsigned long)offset);

		for (i = 0; i < SRAMLOG_BYTES_PER_LINE && (offset + i) < len; i++) {
			printf(" %02x", buf[offset + i]);
		}

		printf("\n");
	}
}

/****************************************************************************
 * Name: sramlog_handle_read
 ****************************************************************************/

static int sramlog_handle_read(int fd)
{
	struct sramlog_data_s data;
	int nbytes;

	memset(&data, 0, sizeof(data));
	data.buf = g_sramlog_buf;
	data.len = SRAMLOG_SIZE;

	nbytes = ioctl(fd, SRAMLOGIOC_READ, (unsigned long)&data);
	if (nbytes < 0) {
		printf("sramlog: SRAMLOGIOC_READ failed: %d\n", nbytes);
		return EXIT_FAILURE;
	}

	sramlog_hexdump(g_sramlog_buf, (uint32_t)nbytes);

	return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: sramlog_handle_write
 ****************************************************************************/

static int sramlog_handle_write(int fd)
{
	int ret;

	/* A NULL argument selects the default address tagged pattern */

	ret = ioctl(fd, SRAMLOGIOC_WRITE, 0);
	if (ret < 0) {
		printf("sramlog: SRAMLOGIOC_WRITE failed: %d\n", ret);
		return EXIT_FAILURE;
	}

	printf("sramlog: wrote pattern to %lu bytes at 0x%08lx\n",
		(unsigned long)SRAMLOG_SIZE, (unsigned long)SRAMLOG_BASE);
	printf("sramlog: reboot and run '%s %s' to verify retention\n",
		"sramlog", SRAMLOG_CMD_READ);

	return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: sramlog_handle_erase
 ****************************************************************************/

static int sramlog_handle_erase(int fd)
{
	int ret;

	ret = ioctl(fd, SRAMLOGIOC_ERASE, 0);
	if (ret < 0) {
		printf("sramlog: SRAMLOGIOC_ERASE failed: %d\n", ret);
		return EXIT_FAILURE;
	}

	printf("sramlog: erased %lu bytes at 0x%08lx\n",
		(unsigned long)SRAMLOG_SIZE, (unsigned long)SRAMLOG_BASE);

	return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sramlog_main
 *
 * Description:
 *   TASH entry point: sramlog <read|write|erase>
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int sramlog_main(int argc, char *argv[])
#endif
{
	int fd;
	int status;

	if (argc != 2) {
		sramlog_print_usage(argv[0]);
		return EXIT_FAILURE;
	}

	/* The sramlog driver is ioctl-only: its read/write methods are
	 * stubs that exist only so inode_checkflags() accepts the open
	 * (this fork defines O_RDONLY as (1 << 0), so even an O_RDONLY
	 * open is rejected when the read method is NULL).  ioctl() does
	 * not require write access, so O_RDONLY is the right mode.
	 */

	fd = open(CONFIG_SRAMLOG_DEVPATH, O_RDONLY);
	if (fd < 0) {
		printf("sramlog: failed to open %s: %d\n",
			CONFIG_SRAMLOG_DEVPATH, errno);
		return EXIT_FAILURE;
	}

	if (strcmp(argv[1], SRAMLOG_CMD_READ) == 0) {
		status = sramlog_handle_read(fd);
	} else if (strcmp(argv[1], SRAMLOG_CMD_WRITE) == 0) {
		status = sramlog_handle_write(fd);
	} else if (strcmp(argv[1], SRAMLOG_CMD_ERASE) == 0) {
		status = sramlog_handle_erase(fd);
	} else {
		sramlog_print_usage(argv[0]);
		status = EXIT_FAILURE;
	}

	close(fd);

	return status;
}
