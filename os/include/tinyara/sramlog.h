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

#ifndef __INCLUDE_TINYARA_SRAMLOG_H
#define __INCLUDE_TINYARA_SRAMLOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <tinyara/fs/ioctl.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL command definitions.  Bases 0x0100-0x1800 and 0x2000-0x2b00 are
 * fully claimed in <tinyara/fs/ioctl.h> (0x1500 _DJOYBASE, 0x1600
 * _AJOYBASE, 0x1700 _PIPEBASE, 0x1800 _RTCBASE, ...), as are 0x3900,
 * 0x3a00, 0x3b00, 0xfe00 and 0xff00.  0x1900 is the next free base.
 */

#define _SRAMLOGIOCBASE          (0x1900)
#define _SRAMLOGIOC(nr)          _IOC(_SRAMLOGIOCBASE, nr)

/* Size of the SRAM window managed by the driver */

#define SRAMLOG_SIZE             ((uint32_t)CONFIG_SRAMLOG_SIZE)

/* Physical base of the retention window (see SRAMLOG_BASE in Kconfig) */

#define SRAMLOG_BASE             ((uintptr_t)CONFIG_SRAMLOG_BASE)

/* Default fill used by SRAMLOGIOC_WRITE when the caller does not provide
 * one.  Each 32-bit word is written as (SRAMLOG_PATTERN_MAGIC | index) so
 * that a later dump clearly shows which offsets retained their value
 * across a reset.
 */

#define SRAMLOG_PATTERN_MAGIC    (0x5a5a0000)

/* SRAMLOGIOC_READ: copy bytes out of the retention window.  Argument is
 * either 0, in which case the driver dumps the whole window itself, or a
 * pointer to struct sramlog_data_s filled in by the caller.  The driver
 * invalidates the D-cache over the window first so that stale cache lines
 * cannot mask data written before a reset.
 */

#define SRAMLOGIOC_READ          _SRAMLOGIOC(1)

/* SRAMLOGIOC_WRITE: fill the retention window with a pattern.  Argument is
 * either 0 to select the default address-tagged pattern, or a 32-bit fill
 * word used for the whole window.  The driver cleans the D-cache
 * afterwards so the data is committed to SRAM before any reset.
 */

#define SRAMLOGIOC_WRITE         _SRAMLOGIOC(2)

/* SRAMLOGIOC_ERASE: clear the whole retention window to zero.  Argument is
 * ignored.
 */

#define SRAMLOGIOC_ERASE         _SRAMLOGIOC(3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Argument to SRAMLOGIOC_READ */

struct sramlog_data_s {
	FAR uint8_t *buf;	/* Caller buffer receiving the window contents */
	uint32_t len;		/* Number of bytes to copy (<= SRAMLOG_SIZE) */
};

/****************************************************************************
 * Public Functions Namespaces
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: sramlog_register
 *
 * Description:
 *   Register the SRAM log character driver at CONFIG_SRAMLOG_DEVPATH.
 *   This function does NOT touch the retention window in any way: the
 *   window lives outside .data/.bss/.noinit, so its contents are left
 *   exactly as a previous boot left them.
 *
 ****************************************************************************/

void sramlog_register(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_TINYARA_SRAMLOG_H */
