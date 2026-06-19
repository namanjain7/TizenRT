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
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <tinyara/arch.h>

#if defined(CONFIG_ARCH_ARMV7A_FAMILY)
#  include <arch/syscall.h>
#else
#  include "svcall.h"
#endif
#include "up_internal.h"

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
	!defined(CONFIG_DISABLE_SIGNALS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_user_callback
 *
 * Description:
 *   In this kernel mode build, this function will be called to execute a
 *   a user callback in user-space. When the callback is invoked, a
 *   kernel-mode stub will first run to perform some housekeeping functions.
 *   This kernel-mode stub will then be called transfer control to the user
 *   mode callback by calling this function.
 *
 *   Normally a user-mode callback handling stub will also execute
 *   before the ultimate callback is called. This function is the
 *   user-space, callback trampoline function. It is called from
 *   up_user_callback() in user-mode.
 *
 * Inputs:
 *   callback - The address user-space callback function
 *   signo, info, and ucontext - Standard arguments to be passed to the
 *     callback function.
 *
 * Return:
 *   None. This function does not return in the normal sense. It returns
 *   via an architecture specific system call made by up_signal_handler().
 *   However, this will look like a normal return by the caller of
 *   up_user_callback.
 *
 ****************************************************************************/

void up_user_callback(_sa_sigaction_t callback, uintptr_t arg1, uintptr_t arg2)
{
	/* Let sys_call4() do all of the work */

	(void)sys_call4(SYS_signal_handler, (uintptr_t)callback, 0, NULL, (uintptr_t)arg1);
}

#endif							/* (CONFIG_BUILD_PROTECTED || CONFIG_BUILD_KERNEL) && !CONFIG_DISABLE_PTHREAD */
