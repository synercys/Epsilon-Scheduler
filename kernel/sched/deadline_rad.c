// SPDX-License-Identifier: GPL-2.0
/*
 * Randomization Protocol for SCHED_DEADLINE
 *
 * Utility functions for implementing the randomization protocol.
 * This includes functions for TaskShuffler (RM) and REORDER (EDF).
 *
 * Copyright (C) 2020 Chien-Ying Chen <cchen140@illinois.edu>
 */
#include <linux/random.h>	// This is for randomization
#include <asm/div64.h>		// This is for 64/32 bits devide operations. (do_div())
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/timekeeping.h>

#include "sched.h"
#include "deadline_rad.h"
#include "pelt.h"


static int start_dl_rad_pi_timer(struct hrtimer *timer, s64 pi_time_budget);


static inline struct task_struct *dl_task_of(struct sched_dl_entity *dl_se)
{
	return container_of(dl_se, struct task_struct, dl);
}

static inline struct rq *rq_of_dl_rq(struct dl_rq *dl_rq)
{
	return container_of(dl_rq, struct rq, dl);
}

static inline struct dl_rq *dl_rq_of_se(struct sched_dl_entity *dl_se)
{
	struct task_struct *p = dl_task_of(dl_se);
	struct rq *rq = task_rq(p);

	return &rq->dl;
}


/*
 * Given task i (task_index), compute the upper bound of the experienced interference:
 * I_i(t) = sum{ min[ceil(Di/Tj)+1, 1+floor[(t+Di-Dj)/Tj]+1] * Cj} | j~=i, Dj <= t+Di
 */
static u64 calculate_interference_relative_to_time(struct dl_rad_taskset *taskset, int task_index, u64 t) {
	int j; // for task iteration
	u64 interference=0, interference_from_j, interference_from_j_2, remainder;

	/* Cacluate I_i(t) */
	for (j=0; j<taskset->task_count; j++) {
		if (j == task_index)
			continue;	// Skip 
		else if (taskset->tasks[j]->dl.dl_deadline > (t+taskset->tasks[task_index]->dl.dl_deadline))
			continue;	// Only take the tasks having deadlines within t+D_i into account.
		else {
			/* ceil(D_i/T_j)+1 */
			interference_from_j = taskset->tasks[task_index]->dl.dl_deadline;	// Storing the deadline in interference_from_j for div calculation.
			//remainder = do_div(interference_from_j, taskset->tasks[j]->dl.dl_period);	// The quotient stores in the interference_from_j.
			interference_from_j = div64_u64_rem(interference_from_j, taskset->tasks[j]->dl.dl_period, &remainder);
			interference_from_j += 1;
			if (remainder > 0)
				interference_from_j++;

			/* 1+floor[(t+Di-Dj)/Tj]+1 */
			interference_from_j_2 = t + taskset->tasks[task_index]->dl.dl_deadline - taskset->tasks[j]->dl.dl_deadline;
			//do_div(interference_from_j_2, taskset->tasks[j]->dl.dl_period);	// The quotient stores in the interference_from_j_2.
			interference_from_j_2 = div64_u64(interference_from_j_2, taskset->tasks[j]->dl.dl_period);
			interference_from_j_2 += 2;

			interference_from_j = (interference_from_j<interference_from_j_2) ? interference_from_j : interference_from_j_2;
			interference_from_j *= taskset->tasks[j]->dl.dl_runtime;
			interference += interference_from_j;
		}
	}
	return interference;
}

/*
 * R_i(t) = max{Ci, W_i(t) - t}
 */
static u64 calculate_response_time_relative_to_time(struct dl_rad_taskset *taskset, int task_index, u64 t) {
	u64 w_i_at_t;

	/* W_i(t) = [floor(t/Ti) + 1]*Ci + I_i(t) */
	w_i_at_t = t;	// Storing t in w_i_at_t for div calculation below.
	//do_div(w_i_at_t, taskset->tasks[task_index]->dl.dl_period);
	w_i_at_t = div64_u64(w_i_at_t, taskset->tasks[task_index]->dl.dl_period);
	w_i_at_t = (w_i_at_t+1)*taskset->tasks[task_index]->dl.dl_runtime + calculate_interference_relative_to_time(taskset, task_index, t);

	if (taskset->tasks[task_index]->dl.dl_runtime > (w_i_at_t-t))
		return taskset->tasks[task_index]->dl.dl_runtime;
	else 
		return w_i_at_t-t;
}

/*
 * cap(R) = r^k when r^k == r^{k+1}, r^0 = sum(Cj)
 */
static u64 calculate_r_cap_recursion(struct dl_rad_taskset *taskset, int progress, u64 last_r_value) {
	int j; // for task iteration
	u64 current_r_value = 0;
	u64 last_r_value_dividend;
	u64 remainder;

	if (progress == 0) {
		for (j=0; j<taskset->task_count; j++) {
			current_r_value += taskset->tasks[j]->dl.dl_runtime;
		}	
		return calculate_r_cap_recursion(taskset, 1, current_r_value);
	} else {
		for (j=0; j<taskset->task_count; j++) {
			last_r_value_dividend = last_r_value;
			//remainder = do_div(last_r_value_dividend, taskset->tasks[j]->dl.dl_period);	// The quotient stores in the last_r_value_dividend.
			last_r_value_dividend = div64_u64_rem(last_r_value_dividend, taskset->tasks[j]->dl.dl_period, &remainder);
			if (remainder > 0)
				last_r_value_dividend++;			
			current_r_value = current_r_value + last_r_value_dividend* taskset->tasks[j]->dl.dl_runtime;
		}
		if (current_r_value == last_r_value)
			return current_r_value;
		else return calculate_r_cap_recursion(taskset, progress+1, current_r_value);
	}
}

/*
 * Calculate R cap which is needed for calculating a task's WCRT.
 */
static u64 calculate_r_cap(struct dl_rad_taskset *taskset) {
	return calculate_r_cap_recursion(taskset, 0, 0);
}

/*
 * Calculate the worst case response time (WCRT) of the given task under EDF scheduling.
 */
static u64 calculate_edf_wcrt(struct dl_rad_taskset *taskset, int task_index) {
	u64 t;
	u64 wcrt = 0, wcrt_at_t = 0;
	u64 t_max = taskset->r_cap - taskset->tasks[task_index]->dl.dl_runtime;

	// A restriction is placed here to reduce the search space: task's parameters should be multiple of miniseconds.
	for (t=0; (t<t_max)||((t==0)&&(t_max==0)); t+=1000000) {
		wcrt_at_t = calculate_response_time_relative_to_time(taskset, task_index, t);
		wcrt = (wcrt_at_t>wcrt) ? wcrt_at_t : wcrt;
	}

	return wcrt;
}

/*
 * Calculate the worst case response time (WCRT) of the given task under RM-FP scheduling.
 */
static u64 calculate_rm_wcrt(struct dl_rad_taskset *taskset, int task_index) {
	return 0;
}

/*
 * Calculate the worst case inversion budget (WCIB) of the given task. 
 * V_i = D_i - R_i, where D_i is the relative deadline and R_i is the worst
 * case response time (WCRT).
 */
static s64 calculate_wcib(struct dl_rad_taskset *taskset, int task_index) {
	if (sysctl_sched_dl_mode == SCHED_DLMODE_DL) {
		// for SCHED_DLMODE_DL
		return taskset->tasks[task_index]->dl.dl_deadline - calculate_edf_wcrt(taskset, task_index);
	} else {
		// for SCHED_DLMODE_RM
		return taskset->tasks[task_index]->dl.dl_deadline - calculate_rm_wcrt(taskset, task_index);
	}
}

void update_taskset_wcib(struct dl_rad_taskset *taskset) {
	int i;

	if (sysctl_sched_dl_mode == SCHED_DLMODE_DL) {
		taskset->r_cap = calculate_r_cap(taskset);
		printk("DLRAD: new r_cap value = %llu", taskset->r_cap);
	}

	printk("DLRAD: new V_i values:");
	/* Compute and update Vi (WCIB) for each task (while including the new task) */
	for (i=0; i<taskset->task_count; i++) {
		taskset->tasks[i]->dl.wcib = calculate_wcib(taskset, i);
		printk("DLRAD: V_%d = %llu.", i, taskset->tasks[i]->dl.wcib);
	}
}


/* Note that updating the WCIBs of the remaining tasks is not done here.
 * They should be updated after this function returns. This gives the caller
 * the ability to decide when to update the WCIBs (e.g., deleting 2 tasks at
 * a time before updating the WCIBs at once.)
 */
void remove_dl_rad_task_pointer(struct task_struct *p) {
	int j, deleted_task_index;
	struct dl_rad_taskset *taskset = &dl_rq_of_se(&p->dl)->dl_rad_taskset;

	/* Find the index of the deleted task in redf_taskset. */
	for (j=0; j<taskset->task_count; j++) {
		if (taskset->tasks[j] == p) {
			deleted_task_index = j;
			break;
		}
	}

	/* Swap the deleted one and the last one in the array. */
	if (deleted_task_index != (taskset->task_count-1)) {
		taskset->tasks[deleted_task_index] = taskset->tasks[taskset->task_count-1];
	}
	taskset->task_count--;
	//printk("DLRAD: pid[%d] is deleted.", p->pid);

	if (taskset->task_count == 0) {
		printk("DLRAD: all tasks are deleted.");
	}
}


struct sched_dl_entity *pick_rad_next_dl_entity(struct rq *rq,
						       struct dl_rq *dl_rq)
{
	/* Algorithm: 
	 * Step 1 (candidate selection):
	 *	Check if the highest priority task's rib is zero.
	 *	if (yes)
	 *		no randomization will be proceeded, go ahead with the highest priority task. 
	 *	else
	 *		a. compute m^t_{HP}
	 *		b. add tasks with deadline <= m^t_{HP} to candidate list/
	 * Setp 2 (random schedule):
	 * 	Randomly choose a task from the candidate list.
	 */

	int j;
	u64 rad_number;
	struct task_struct *rad_candidates[30];
	u64 rq_task_count = 0;
	u64 candidate_count = 0;
	struct task_struct *rad_task;
	struct dl_rad_taskset *taskset = &dl_rq->dl_rad_taskset;
	struct sched_dl_entity *leftmost_dl_se = rb_entry(dl_rq->root.rb_leftmost, struct sched_dl_entity, rb_node);
	u64 min_inversion_deadline = (~(u64)0);	// initialized with the max value
	s64 min_inversion_budget = 0;
	u64 scheduled_pi_timer_duration;
	s64 rq_min_task_rib = 0;
	int idle_time_scheduling_allowed = 1;
	struct task_struct dummy_idle_task;


	/* Step 1 */

	/* Does current highest priority task allow priority inversion? */
	if (leftmost_dl_se->rib <= 0)
		return leftmost_dl_se;

	/* Compute the minimum inversion deadline m^t_{HP} */
	rq_min_task_rib = leftmost_dl_se->rib;
	for (j=0; j<taskset->task_count; j++) {
		if (RB_EMPTY_NODE(&taskset->tasks[j]->dl.rb_node))
			continue;	// This is not in dl_rq.

		rq_task_count++;

		/* Check if every task has positive rib so taht idle time scheduling is possible. */
		if (taskset->tasks[j]->dl.rib <= 0)
			idle_time_scheduling_allowed = 0;

		if (taskset->tasks[j]->dl.rib < rq_min_task_rib)
			rq_min_task_rib = taskset->tasks[j]->dl.rib;

		/* Comparison for the minimum inversion deadline */
		if ( (taskset->tasks[j]->dl.deadline > leftmost_dl_se->deadline) && (taskset->tasks[j]->dl.rib < 0) ) 
			min_inversion_deadline = (taskset->tasks[j]->dl.deadline<min_inversion_deadline)?taskset->tasks[j]->dl.deadline:min_inversion_deadline;
	}


	/* Create a candidate list */
	for (j=0; j<taskset->task_count; j++) {
		if (RB_EMPTY_NODE(&taskset->tasks[j]->dl.rb_node))
			continue;	// This is not in dl_rq.

		if (taskset->tasks[j]->dl.deadline <= min_inversion_deadline) {
			rad_candidates[candidate_count] = taskset->tasks[j];
			candidate_count++;
		}
	}

	/* Note that at least the highest priority task will be in the candidate 
	 * list at this moment. We place this check here to track the status.
	 */
	if (candidate_count == 0) {
		printk(KERN_ERR "ERROR: redf: candidate list is empty.");
		return leftmost_dl_se; 
	}


	/* If it's NORMAL mode and there is only one candidate (the leftmost one) 
	 * then pick that one. 
	 */
	if ((candidate_count<=1) && !sysctl_sched_dl_rad_idle_enabled && 
				    !sysctl_sched_dl_rad_fg_enabled &&
				    !sysctl_sched_dl_rad_utr_enabled)
		return leftmost_dl_se; 


	if (sysctl_sched_dl_rad_idle_enabled) {
		/* Check if idle task should be included. */
		if ((candidate_count==rq_task_count) && (idle_time_scheduling_allowed==1)) {
			rad_candidates[candidate_count] = &dummy_idle_task;
			candidate_count++;
		}
	}

	/* Step 2 */

	/* Select a random task. */
	get_random_bytes(&rad_number, sizeof(rad_number)); // It's a system call from linux/random.h
	rad_task = rad_candidates[do_div(rad_number,candidate_count)];

	/* If the "idle task" is selected: */
	if ( sysctl_sched_dl_rad_idle_enabled && (rad_task == &dummy_idle_task) ) {
		if (sysctl_sched_dl_rad_fg_enabled) {
			/* Randomize the scheduled idle time */
			get_random_bytes(&rad_number, sizeof(rad_number));
			scheduled_pi_timer_duration = do_div(rad_number, (u64)rq_min_task_rib);
		} else {
			scheduled_pi_timer_duration = (u64)rq_min_task_rib;
		}

		/* Take the rad overhead into account, some time should be deducted. */
		if (scheduled_pi_timer_duration > DL_RAD_OVERHEAD_UPPER_BOUND_NS) {
			scheduled_pi_timer_duration -= DL_RAD_OVERHEAD_UPPER_BOUND_NS;
		}

		if (scheduled_pi_timer_duration < DL_RAD_MIN_IDLE_TIME_DURATION) {
			//printk("redf: idle time too small, so return leftmost => %llu", scheduled_pi_timer_duration);
			return leftmost_dl_se;
		}

		if (0 == start_dl_rad_pi_timer(&dl_rq->dl_rad_pi_timer, scheduled_pi_timer_duration)) {
			/* The timer somehow fails to start, so be safe and choose the leftmost task. */
			printk(KERN_ERR "ERROR: DLRAD: idle timer failed to start.");
			return leftmost_dl_se;
		} else {
			dl_rq->dl_rad_idle_time_acting = true;
			//printk("redf: idle task is selected => run for %lld", scheduled_pi_timer_duration);
			//printk("redf: next scheduling point: %llu", scheduled_pi_timer_duration);
			return NULL;	// return null for idle time scheduling.
		}
	}

	/* If it is the leftmost task, then just return it. */
	if (dl_rq->root.rb_leftmost == &rad_task->dl.rb_node)
		return leftmost_dl_se;

	/* 
	 * To this point, we are sure that: 
	 *   1. at least 2 candidates are in the list.
	 *   2. the chosen job is not the leftmost one.
	 */

	/* Get the allowed minimum inversion duration. */
	min_inversion_budget = leftmost_dl_se->rib;
	for (j=0; j<taskset->task_count; j++) {
		if (RB_EMPTY_NODE(&taskset->tasks[j]->dl.rb_node))
			continue;	// This is not in dl_rq.

		if (taskset->tasks[j]->dl.deadline < rad_task->dl.deadline) {
			if (taskset->tasks[j]->dl.rib < min_inversion_budget)
				min_inversion_budget = taskset->tasks[j]->dl.rib;
		}
	}

	/* Although the theory tells us that min_inversion_budget will definitely 
	 * be positive, we place this check to verify this claim. */
	if (min_inversion_budget <= 0) {
		/* Priority inversion is not allowed for the chosen task. */
		printk(KERN_ERR "ERROR: DLRAD: min_inversion_budget becomes %lld.", min_inversion_budget);
		return leftmost_dl_se;
	}


	if (sysctl_sched_dl_rad_fg_enabled) {
		/* If the remaining runtime is too small then don't bother to randomize. */
		if ((rad_task->dl.runtime<=min_inversion_budget) && (rad_task->dl.runtime<=DL_RAD_MIN_EXEC_DURATION)) {
			return &rad_task->dl;
		}

		scheduled_pi_timer_duration = (rad_task->dl.runtime<min_inversion_budget)?rad_task->dl.runtime:min_inversion_budget;

		/* Randomize the duration if it's larger than the given minimum. */
		if (scheduled_pi_timer_duration > DL_RAD_MIN_EXEC_DURATION) {
			/* Randomize time for next scheduling point.*/
			get_random_bytes(&rad_number, sizeof(rad_number)); 
			scheduled_pi_timer_duration = do_div(rad_number, scheduled_pi_timer_duration);

			/* Make sure the duration is greater than the given minimum after randomization. */
			if (scheduled_pi_timer_duration < DL_RAD_MIN_EXEC_DURATION)
				scheduled_pi_timer_duration = DL_RAD_MIN_EXEC_DURATION;
		} 

		//printk("DLRAD: next scheduling point: %llu", scheduled_pi_timer_duration);
		if (0 == start_dl_rad_pi_timer(&dl_rq->dl_rad_pi_timer, scheduled_pi_timer_duration)) {
			/* The timer somehow fails to start, so be safe and choose the leftmost task. */
			return leftmost_dl_se;
		} 

		return &rad_task->dl;
	} else { // without fine-grained switching 
		 if (rad_task->dl.runtime > min_inversion_budget) {
			/* Start the priority inversion timer for the next scheduling point if 
			 * the minimum inversion budget of higher priority tasks is smaller than
			 * the chosen task's remaining computation time. */
			if (0 == start_dl_rad_pi_timer(&dl_rq->dl_rad_pi_timer, min_inversion_budget)) {
				/* The timer somehow fails to start, so be safe and choose the leftmost task. */
				return leftmost_dl_se;
			}
		} 

		return &rad_task->dl;
	}
}

/* pi_time_budget is in nanoseconds. */
static int start_dl_rad_pi_timer(struct hrtimer *timer, s64 pi_time_budget) {
	struct dl_rq *dl_rq = container_of(timer, struct dl_rq, dl_rad_pi_timer);
	struct rq *rq = rq_of_dl_rq(dl_rq);

	ktime_t now, act;

	now = hrtimer_cb_get_time(timer);
	act = ktime_add_ns(now, pi_time_budget);

	/*
	 * If the expiry time already passed, e.g., because the budget is too small, 
	 * don't even try to start the timer in the past!
	 */
	if (ktime_us_delta(act, now) < 0)
		return 0;

	if (hrtimer_active(timer)) {
		hrtimer_try_to_cancel(timer);
	}

	hrtimer_start(timer, act, HRTIMER_MODE_ABS);

	/* This is set for the idle time scheduling. */
	dl_rq->dl_rad_pi_timer_start_time = rq_clock_task(rq);

	//printk("DLRAD: pi_timer starts %lld", pi_time_budget);

	return 1;
}

/* Randomized priority inversion timer is up. */
enum hrtimer_restart dl_rad_pi_timer(struct hrtimer *timer) {
	struct dl_rq *dl_rq = container_of(timer, struct dl_rq, dl_rad_pi_timer);
	struct rq *rq = rq_of_dl_rq(dl_rq);

	raw_spin_lock(&rq->lock);

	sched_clock_tick();
	update_rq_clock(rq);

	//printk("DLRAD: pi_timer is up");
	if (sysctl_sched_dl_rad_idle_enabled && dl_rq->dl_rad_idle_time_acting) {
		update_rib_after_pi_idle_time(dl_rq);
		dl_rq->dl_rad_idle_time_acting = false;
	}

	resched_curr(rq);

	raw_spin_unlock(&rq->lock);

	return HRTIMER_NORESTART;
}

void init_dl_rad_pi_timer(struct hrtimer *timer) {
	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer->function = dl_rad_pi_timer;
}

void cancel_dl_rad_pi_timer(struct hrtimer *timer) {
	struct dl_rq *dl_rq = container_of(timer, struct dl_rq, dl_rad_pi_timer);

	if (hrtimer_active(timer)) {
		hrtimer_try_to_cancel(timer);
		//printk("DLRAD: pi_timer is canceled.");
	}

	/* Reset for the idle time scheduling mode. */
	dl_rq->dl_rad_idle_time_acting = false;
}

void update_rib_after_pi_idle_time(struct dl_rq *dl_rq) {
	int i;
	struct rq *rq = rq_of_dl_rq(dl_rq);
	u64 delta_idle_time = rq_clock_task(rq) - dl_rq->dl_rad_pi_timer_start_time;
	struct dl_rad_taskset *taskset = &dl_rq->dl_rad_taskset;
	u64 rq_task_count = 0;

	/* DLRAD: update the remaining inversion budget (RIB) for each task in the rq */
	for (i=0; i<taskset->task_count; i++) {
		if (RB_EMPTY_NODE(&taskset->tasks[i]->dl.rb_node))
			continue;	// This is not in dl_rq.

		rq_task_count++;

		taskset->tasks[i]->dl.rib -= delta_idle_time;

		/* Due to context swtich overhead, consumed time may be more than what we scheduled. 
		 * This should already be avoided by taking the overhead into account when setting
		 * the scheduled idle time. */
		/*
		if (taskset->tasks[i]->dl.redf_rib < 0)
			printk("redf: rib becomes 0 after idle time. - %lld", taskset->tasks[i]->dl.redf_rib);
		*/
	}
	//printk("redf: %llu update rib after idle time.", rq_task_count);
}
