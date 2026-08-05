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

/// @file stc_sched_rr.c

/// @brief Scenario test cases for round-robin fairness (family A)
///
/// These assert that round-robin scheduling actually rotates the CPU between
/// equal priority tasks.  The existing API level test for round-robin,
/// tc_sched_sched_rr_get_interval(), only checks that the configured
/// interval is reported back; a kernel in which round-robin never yields
/// passes it.  The scenarios here fail in that case.
///
/// Each scenario states its oracle as two separate assertions:
///
///   - a hard correctness assertion, which can never be flaky, and
///   - a fairness assertion carrying a tolerance.
///
/// They are kept separate deliberately.  When the suite goes red the failing
/// assertion alone identifies whether behaviour is wrong or merely uneven.

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdio.h>
#include <sched.h>

#include "tc_internal.h"
#include "stc_sched_common.h"
#include "stc_sched_monitor.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Fairness floor for the unpinned, oversubscribed case.
 *
 * With ncpus + 1 workers on ncpus CPUs the ideal share is ncpus/(ncpus + 1)
 * each, so a perfectly fair run gives 100%.  A tight bound would be flaky:
 * over a 3 s window at CONFIG_RR_INTERVAL of 10 ms each worker sees roughly
 * 300 slice boundaries, and quantisation at those boundaries plus background
 * interrupt load both perturb the ratio.  50% sits far below fair and far
 * above the failure mode, which is one worker never running at all.
 */

#define STC_RR_FAIR_UNPINNED    50

/* Fairness floor when every worker is pinned to one CPU.
 *
 * A single run queue removes CPU selection and migration from the picture,
 * so rotation is a plain cycle through the priority band and the spread
 * should be much tighter.
 */

#define STC_RR_FAIR_PINNED      80

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Outcome of one monitored run.  Every field is captured before any
 * assertion is evaluated, so that a failing assertion cannot leave the
 * monitor task or the workers running.
 */

struct stc_rr_result_s {
	int monitor_start;
	int workload;
	int monitor_stop;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stc_rr_run_monitored
 *
 * Description:
 *   Applies a workload with the starvation monitor running alongside it.
 *
 *   The monitor is a second, independent detector: the fairness counters
 *   say whether work was shared, while the monitor says whether any task
 *   was passed over for longer than it should have been.  A defect that
 *   trips both is localised more precisely than one that trips either
 *   alone.
 *
 *   Nothing is asserted here.  Teardown must complete before the caller
 *   evaluates anything, because an assertion returns immediately and would
 *   otherwise strand the monitor.
 *
 ****************************************************************************/

static void stc_rr_run_monitored(const struct stc_workload_s *wl,
				 struct stc_rr_result_s *res)
{
	res->monitor_start = stc_monitor_start(wl->priority);
	res->workload = stc_run_workload(wl);
	res->monitor_stop = (res->monitor_start == OK) ?
			    stc_monitor_stop() : ERROR;
}

/****************************************************************************
 * Name: stc_sched_rr01_fairness_oversubscribed
 *
 * @testcase             :stc_sched_rr01_fairness_oversubscribed
 * @brief                :SCN-RR-01, round-robin fairness with more runnable
 *                        tasks than CPUs
 * @scenario             :Create ncpus + 1 CPU bound tasks, all at the same
 *                        priority, all SCHED_RR, all with the default
 *                        affinity so the scheduler is free to place them.
 *                        Let them run for a fixed window and compare the
 *                        work each completed.
 *                        The surplus task cannot be given a CPU at creation
 *                        time, because a CPU is only handed to a candidate
 *                        that strictly outranks the task already running
 *                        there.  Round-robin rotation is the only mechanism
 *                        that can ever admit it, so if rotation does not
 *                        work the surplus task records no work at all.
 * @apicovered           :task_create, sched_setscheduler, sched_getcpucount
 * @precondition         :CONFIG_RR_INTERVAL greater than zero
 * @postcondition        :all workers exited
 * @return               :void
 *
 ****************************************************************************/

static void stc_sched_rr01_fairness_oversubscribed(void)
{
	struct stc_workload_s wl;
	struct stc_rr_result_s res;
	int ncpus = stc_ncpus();
	int nworkers = ncpus + 1;

	stc_workload_init(&wl, nworkers, SCHED_PRIORITY_DEFAULT, SCHED_RR);

	stc_rr_run_monitored(&wl, &res);

	stc_report("SCN-RR-01", nworkers);
	stc_monitor_report("SCN-RR-01");

	TC_ASSERT_EQ("rr01_monitor_start", res.monitor_start, OK);
	TC_ASSERT_EQ("rr01_workload", res.workload, OK);
	TC_ASSERT_EQ("rr01_monitor_stop", res.monitor_stop, OK);

	/* Hard correctness assertion.  A zero here means a runnable task was
	 * never scheduled for the whole window, which is a starvation defect
	 * and not a fairness tolerance question.
	 */

	TC_ASSERT_GT("rr01_all_workers_ran", stc_min(nworkers), 0);

	/* Fairness assertion, tolerance based. */

	TC_ASSERT_GEQ("rr01_fair_share", stc_fair_pct(nworkers),
		      STC_RR_FAIR_UNPINNED);

	/* Independent detector: no task was passed over for longer than the
	 * monitor's grace period.
	 */

	TC_ASSERT_EQ("rr01_no_starvation", stc_monitor_violations(), 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sched_rr02_fairness_single_cpu
 *
 * @testcase             :stc_sched_rr02_fairness_single_cpu
 * @brief                :SCN-RR-02, round-robin fairness within one CPU
 * @scenario             :Create three CPU bound tasks at equal priority and
 *                        pin all of them to CPU0, so they share a single run
 *                        queue.  This is the case in which rotation is
 *                        purely a cycle through one priority band, with no
 *                        CPU selection or migration involved, so the work
 *                        completed should be tightly clustered.
 *                        It is also the configuration in which round-robin
 *                        can appear to work while being broken elsewhere,
 *                        which is why it is tested separately from RR-01.
 * @apicovered           :task_create, sched_setscheduler, sched_setaffinity
 * @precondition         :CONFIG_RR_INTERVAL greater than zero
 * @postcondition        :all workers exited
 * @return               :void
 *
 ****************************************************************************/

static void stc_sched_rr02_fairness_single_cpu(void)
{
	struct stc_workload_s wl;
	struct stc_rr_result_s res;
	int nworkers = 3;

	stc_workload_init(&wl, nworkers, SCHED_PRIORITY_DEFAULT, SCHED_RR);
#ifdef CONFIG_SMP
	CPU_SET(0, &wl.affinity);
	wl.pinned = true;
#else
	/* Without SMP there is only one CPU and nothing to pin to.  The
	 * scenario still applies: three equal priority tasks must share it.
	 */

	wl.pinned = false;
#endif

	stc_rr_run_monitored(&wl, &res);

	stc_report("SCN-RR-02", nworkers);
	stc_monitor_report("SCN-RR-02");

	TC_ASSERT_EQ("rr02_monitor_start", res.monitor_start, OK);
	TC_ASSERT_EQ("rr02_workload", res.workload, OK);
	TC_ASSERT_EQ("rr02_monitor_stop", res.monitor_stop, OK);

	TC_ASSERT_GT("rr02_all_workers_ran", stc_min(nworkers), 0);

	TC_ASSERT_GEQ("rr02_fair_share", stc_fair_pct(nworkers),
		      STC_RR_FAIR_PINNED);

	TC_ASSERT_EQ("rr02_no_starvation", stc_monitor_violations(), 0);

	TC_SUCCESS_RESULT();
}

#ifdef CONFIG_SMP
/****************************************************************************
 * Name: stc_sched_rr03_fairness_secondary_cpu
 *
 * @testcase             :stc_sched_rr03_fairness_secondary_cpu
 * @brief                :SCN-RR-03, round-robin fairness on a CPU that does
 *                        not receive the system tick
 * @scenario             :Create two CPU bound tasks at equal priority and
 *                        pin both to the highest numbered CPU.  On targets
 *                        where the system tick is enabled on CPU0 only, the
 *                        timeslice check for every other CPU is performed
 *                        remotely, by CPU0, from inside its own critical
 *                        section.  Any predicate in that path that asks
 *                        about the wrong CPU therefore evaluates the same
 *                        way on every tick and suppresses rotation on the
 *                        secondary CPU permanently, while CPU0 continues to
 *                        rotate normally.
 *                        RR-01 can mask that, because its unpinned workers
 *                        may all be placed on CPU0.  Pinning to the
 *                        secondary CPU isolates it.
 * @apicovered           :task_create, sched_setscheduler, sched_setaffinity
 * @precondition         :CONFIG_SMP_NCPUS greater than one
 * @postcondition        :all workers exited
 * @return               :void
 *
 ****************************************************************************/

static void stc_sched_rr03_fairness_secondary_cpu(void)
{
	struct stc_workload_s wl;
	struct stc_rr_result_s res;
	int ncpus = stc_ncpus();
	int nworkers = 2;
	int target;

	if (ncpus < 2) {
		printf("[SCN-RR-03] skipped, single CPU configuration\n");
		return;
	}

	target = ncpus - 1;

	stc_workload_init(&wl, nworkers, SCHED_PRIORITY_DEFAULT, SCHED_RR);
	wl.pinned = true;
	CPU_SET(target, &wl.affinity);

	stc_rr_run_monitored(&wl, &res);

	stc_report("SCN-RR-03", nworkers);
	stc_monitor_report("SCN-RR-03");

	TC_ASSERT_EQ("rr03_monitor_start", res.monitor_start, OK);
	TC_ASSERT_EQ("rr03_workload", res.workload, OK);
	TC_ASSERT_EQ("rr03_monitor_stop", res.monitor_stop, OK);

	/* Hard correctness assertion.  Both workers are pinned to the same
	 * CPU, so exactly one of them can be running at any instant and the
	 * other only ever runs if rotation happens on that CPU.
	 */

	TC_ASSERT_GT("rr03_all_workers_ran", stc_min(nworkers), 0);

	TC_ASSERT_GEQ("rr03_fair_share", stc_fair_pct(nworkers),
		      STC_RR_FAIR_PINNED);

	TC_ASSERT_EQ("rr03_no_starvation", stc_monitor_violations(), 0);

	TC_SUCCESS_RESULT();
}
#endif							/* CONFIG_SMP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sched_rr_main
 ****************************************************************************/

int stc_sched_rr_main(void)
{
	printf("[SCN-RR] round-robin fairness scenarios, ncpus=%d interval=%dms\n",
	       stc_ncpus(), CONFIG_RR_INTERVAL);

	stc_sched_rr01_fairness_oversubscribed();
	stc_sched_rr02_fairness_single_cpu();
#ifdef CONFIG_SMP
	stc_sched_rr03_fairness_secondary_cpu();
#endif

	return 0;
}
