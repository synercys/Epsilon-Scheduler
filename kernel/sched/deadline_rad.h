/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_DL_RAD_H
#define _LINUX_DL_RAD_H

//#include "sched.h"

void update_taskset_dl_rad_parameters(struct dl_rad_taskset *taskset);
void remove_dl_rad_task_pointer(struct task_struct *p);
struct sched_dl_entity *pick_rad_next_dl_entity(struct rq *rq,
						       struct dl_rq *dl_rq);
void update_rib_after_pi_idle_time(struct dl_rq *dl_rq);
void cancel_dl_rad_pi_timer(struct hrtimer *timer);
void init_dl_rad_pi_timer(struct hrtimer *timer);
enum hrtimer_restart dl_rad_pi_timer(struct hrtimer *timer);

#define	DL_RAD_MIN_EXEC_DURATION	1000000	// 1ms
#define	DL_RAD_MIN_IDLE_TIME_DURATION	1000000	// 1ms
#define	DL_RAD_OVERHEAD_UPPER_BOUND_NS	100000	// 100us

#endif
