/****************************************************************************
 *
 * Copyright 2023 Samsung Electronics All Rights Reserved.
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
 * arch/arm/src/armv7-a/arm_sigdeliver.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/irq.h>
#include <tinyara/arch.h>
#include <tinyara/board.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "up_internal.h"
#include "arm.h"
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
void arm_sigdeliver(void)
{
	struct tcb_s *rtcb = this_task();
	uint32_t *regs = rtcb->xcp.saved_regs;

#ifdef CONFIG_SMP
	/* In the SMP case, we must terminate the critical section while the signal
	 * handler executes, but we also need to restore the irqcount when the
	 * we resume the main thread of the task.
	 */

	int16_t saved_irqcount;
#endif

	board_autoled_on(LED_SIGNAL);

	svdbg("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n", rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
	DEBUGASSERT(rtcb->xcp.sigdeliver != NULL);

#ifdef CONFIG_SMP
	/* In the SMP case, up_schedule_sigaction(0) will have incremented
	 * 'irqcount' in order to force us into a critical section.  Save the
	 * pre-incremented irqcount.
	 */

	saved_irqcount = rtcb->irqcount - 1;
	DEBUGASSERT(saved_irqcount >= 0);

	/* Now we need call leave_critical_section() repeatedly to get the irqcount
	 * to zero, freeing all global spinlocks that enforce the critical section.
	 */

	do {
		regs[REG_CPSR] |= PSR_MODE_SYS;
		leave_critical_section(regs[REG_CPSR]);
	} while (rtcb->irqcount > 0);
#endif							/* CONFIG_SMP */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
	/* Then make sure that interrupts are enabled.  Signal handlers must always
	 * run with interrupts enabled.
	 */

	up_irq_enable();
#endif

	/* Deliver the signal */

	((sig_deliver_t)rtcb->xcp.sigdeliver)(rtcb);

	/* Output any debug messages BEFORE restoring errno (because they may
	 * alter errno), then disable interrupts again and restore the original
	 * errno that is needed by the user logic (it is probably EINTR).
	 *
	 * I would prefer that all interrupts are disabled when
	 * arm_fullcontextrestore() is called, but that may not be necessary.
	 */

	svdbg("Resuming\n");

#ifdef CONFIG_SMP
	/* Restore the saved 'irqcount' and recover the critical section
	 * spinlocks.
	 */

	DEBUGASSERT(rtcb->irqcount == 0);
	while (rtcb->irqcount < saved_irqcount) {
		enter_critical_section();
	}
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS
	irqsave();
#endif

	/* Modify the saved return state with the actual saved values in the
	 * TCB.  This depends on the fact that nested signal handling is
	 * not supported.  Therefore, these values will persist throughout the
	 * signal handling action.
	 *
	 * Keeping this data in the TCB resolves a security problem in protected
	 * and kernel mode:  The regs[] array is visible on the user stack and
	 * could be modified by a hostile program.
	 */

	rtcb->xcp.sigdeliver = NULL;	/* Allows next handler to be scheduled */

	/* Then restore the correct state for this thread of execution. */

	board_autoled_off(LED_SIGNAL);
	arm_fullcontextrestore(regs);
}
#endif
