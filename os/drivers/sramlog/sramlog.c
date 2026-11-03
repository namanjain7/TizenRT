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

#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/fs/fs.h>
#include <tinyara/fs/ioctl.h>
#include <tinyara/sramlog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SRAMLOG_BYTES_PER_LINE	16

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The cache maintenance helpers live in the armv7-a architecture layer
 * (os/arch/arm/src/armv7-a/arm_cache.c) but are not exported through any
 * public header, so declare the two that are needed here.
 */

extern void up_clean_dcache(uintptr_t start, uintptr_t end);
extern void up_invalidate_dcache(uintptr_t start, uintptr_t end);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sramlog_dev_s {
	uintptr_t base;		/* Physical base of the retention window */
	uint32_t size;		/* Size of the window in bytes */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sramlog_open(FAR struct file *filep);
static int sramlog_close(FAR struct file *filep);
static ssize_t sramlog_read(FAR struct file *filep, FAR char *buffer,
	size_t buflen);
static ssize_t sramlog_write(FAR struct file *filep,
	FAR const char *buffer, size_t buflen);
static int sramlog_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static int sramlog_dump(FAR struct sramlog_dev_s *priv, uint32_t len);
static int sramlog_copyout(FAR struct sramlog_dev_s *priv,
	FAR struct sramlog_data_s *data);
static int sramlog_fill(FAR struct sramlog_dev_s *priv, uint32_t pattern);
static int sramlog_erase(FAR struct sramlog_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sramlogops = {
	sramlog_open,		/* open */
	sramlog_close,		/* close */
	sramlog_read,		/* read */
	sramlog_write,		/* write */
	NULL,			/* seek */
	sramlog_ioctl		/* ioctl */
#ifndef CONFIG_DISABLE_POLL
	, NULL			/* poll */
#endif
};

/* Single, statically described instance.  Nothing initializes the window it
 * points at: it is only ever touched from the ioctl handler below.
 */

static struct sramlog_dev_s g_sramlogdev = {
	SRAMLOG_BASE,
	SRAMLOG_SIZE
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sramlog_dump
 *
 * Description:
 *   Print the first len bytes of the window to the console, 16 bytes per
 *   line.  Used when SRAMLOGIOC_READ is issued with a NULL argument.
 *
 ****************************************************************************/

static int sramlog_dump(FAR struct sramlog_dev_s *priv, uint32_t len)
{
	FAR const uint8_t *src = (FAR const uint8_t *)priv->base;
	uint32_t offset;
	uint32_t i;

	if (len > priv->size) {
		len = priv->size;
	}

	/* Same reason as in sramlog_copyout(): the window may hold data written
	 * before a reset, which this core has never cached.
	 */

	up_invalidate_dcache(priv->base, priv->base + priv->size);

	printf("sramlog: dump of %lu bytes at 0x%08lx\n", (unsigned long)len,
		(unsigned long)priv->base);

	for (offset = 0; offset < len; offset += SRAMLOG_BYTES_PER_LINE) {
		printf("  %04lu:", (unsigned long)offset);

		for (i = 0;
			i < SRAMLOG_BYTES_PER_LINE && (offset + i) < len; i++) {
			printf(" %02x", src[offset + i]);
		}

		printf("\n");
	}

	return OK;
}

/****************************************************************************
 * Name: sramlog_copyout
 *
 * Description:
 *   Copy data->len bytes of the window into the caller supplied buffer.
 *
 ****************************************************************************/

static int sramlog_copyout(FAR struct sramlog_dev_s *priv,
	FAR struct sramlog_data_s *data)
{
	if (data->buf == NULL || data->len == 0 || data->len > priv->size) {
		return -EINVAL;
	}

	/* Drop any stale cache lines before reading.  The window normally holds
	 * data written by a previous boot or by another core, which this core
	 * has never cached, so this cannot discard anything dirty: every write
	 * path below cleans the cache once it is done.
	 */

	up_invalidate_dcache(priv->base, priv->base + priv->size);

	memcpy(data->buf, (FAR const void *)priv->base, data->len);

	return (int)data->len;
}

/****************************************************************************
 * Name: sramlog_fill
 *
 * Description:
 *   Fill the whole window with either the caller supplied 32-bit pattern,
 *   or with the default address tagged pattern.
 *
 ****************************************************************************/

static int sramlog_fill(FAR struct sramlog_dev_s *priv, uint32_t pattern)
{
	FAR volatile uint32_t *dst = (FAR volatile uint32_t *)priv->base;
	uint32_t nwords = priv->size / sizeof(uint32_t);
	uint32_t i;

	for (i = 0; i < nwords; i++) {
		dst[i] = (pattern != 0) ? pattern : (SRAMLOG_PATTERN_MAGIC | i);
	}

	/* The window is mapped cacheable write-back (see ameba_xlat_table.c),
	 * so these writes may still be sitting in the D-cache.  Clean it now,
	 * otherwise a reset that follows immediately would lose them.
	 */

	up_clean_dcache(priv->base, priv->base + priv->size);

	return OK;
}

/****************************************************************************
 * Name: sramlog_erase
 *
 * Description:
 *   Clear the whole window to zero.
 *
 ****************************************************************************/

static int sramlog_erase(FAR struct sramlog_dev_s *priv)
{
	(void)memset((FAR void *)priv->base, 0, priv->size);

	up_clean_dcache(priv->base, priv->base + priv->size);

	return OK;
}

/****************************************************************************
 * Name: sramlog_open
 ****************************************************************************/

static int sramlog_open(FAR struct file *filep)
{
	return OK;
}

/****************************************************************************
 * Name: sramlog_close
 ****************************************************************************/

static int sramlog_close(FAR struct file *filep)
{
	return OK;
}

/****************************************************************************
 * Name: sramlog_read
 ****************************************************************************/

static ssize_t sramlog_read(FAR struct file *filep, FAR char *buffer,
	size_t buflen)
{
	/* Stubs only.  The retention window is accessed exclusively through
	 * ioctls.  These methods exist because inode_checkflags() in the VFS
	 * open path rejects an open with -EACCES when the access requested
	 * by oflags has a NULL method, and this fork defines O_RDONLY as
	 * (1 << 0), so every open() requests the read bit.  A NULL read or
	 * write method therefore makes it impossible to open this driver at
	 * all.  See also the silent_reboot driver, which does the same.
	 */

	return 0;
}

/****************************************************************************
 * Name: sramlog_write
 ****************************************************************************/

static ssize_t sramlog_write(FAR struct file *filep,
	FAR const char *buffer, size_t buflen)
{
	return 0;
}

/****************************************************************************
 * Name: sramlog_ioctl
 ****************************************************************************/

static int sramlog_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	FAR struct inode *inode;
	FAR struct sramlog_dev_s *priv;
	int ret;

	if (_IOC_TYPE(cmd) != _SRAMLOGIOCBASE) {
		return -EINVAL;
	}

	inode = (FAR struct inode *)filep->f_inode;
	priv = (FAR struct sramlog_dev_s *)inode->i_private;

	switch (cmd) {
		case SRAMLOGIOC_READ:
			if (arg == 0) {
				ret = sramlog_dump(priv, priv->size);
			} else {
				ret = sramlog_copyout(priv,
					(FAR struct sramlog_data_s *)arg);
			}
			break;

		case SRAMLOGIOC_WRITE:
			ret = sramlog_fill(priv, (uint32_t)arg);
			break;

		case SRAMLOGIOC_ERASE:
			ret = sramlog_erase(priv);
			break;

		default:
			lldbg("ERROR: Unrecognized cmd: %d\n", cmd);
			ret = -ENOTTY;
			break;
	}

	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sramlog_register
 *
 * Description:
 *   Register the SRAM log character driver.  The retention window is
 *   deliberately left untouched: it lives outside .data, .bss and .noinit,
 *   so whatever a previous boot stored there is still valid when this
 *   function runs.
 *
 ****************************************************************************/

void sramlog_register(void)
{
	int ret;

	ret = register_driver(CONFIG_SRAMLOG_DEVPATH, &g_sramlogops, 0666,
		(FAR void *)&g_sramlogdev);
	if (ret < 0) {
		printf("sramlog: register_driver() failed: %d\n", ret);
	}
}
