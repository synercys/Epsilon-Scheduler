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
 * Calculate the upper-bound inteference that a job of \tau_j can experience due to
 * the higher priority tasks during D_i in TaskShuffler algorithms (RM-rad scheduling).
 * Equation: (TaskShuffler -- Eq.2)
 * Ii = \sum_hp{ [ceil(Di/Tj) + 1] * Cj }
 */
static u64 calculate_rm_upper_interference(struct dl_rad_taskset *taskset, int task_index) {
	int j;
	u64 interference = 0, remainder;
	u64 dl_deadline = taskset->tasks[task_index]->dl.dl_deadline;
	u64 dl_period = taskset->tasks[task_index]->dl.dl_period;
	u64 j_runtime, j_period;

	for (j=0; j<taskset->task_count; j++) {
		j_period = taskset->tasks[j]->dl.dl_period;
		j_runtime = taskset->tasks[j]->dl.dl_runtime;		

		/* Only account for those that have higher priorities */
		if (j==task_index || dl_period<j_period)
			continue;

		/* [ceil(Di/Tj)+1]*Cj */
		interference += (j_runtime * (div64_u64_rem(dl_deadline, j_period, &remainder)+1));
		if (remainder > 0)
			interference += j_runtime;
	}

	return interference;
}

/*
 * Calculate the worst case inversion budget (WCIB) of the given task. 
 * V_i = D_i - R_i, where D_i is the relative deadline and R_i is the worst
 * case response time (WCRT).
 */
static s64 calculate_wcib(struct dl_rad_taskset *taskset, int task_index) {
	u64 dl_deadline = taskset->tasks[task_index]->dl.dl_deadline;
	u64 dl_runtime = taskset->tasks[task_index]->dl.dl_runtime;

	if (sysctl_sched_dl_mode == SCHED_DLMODE_DL) {
		// for SCHED_DLMODE_DL
		return dl_deadline - calculate_edf_wcrt(taskset, task_index);
	} else {
		// for SCHED_DLMODE_RM
		// Vi = di - (ei + Ii)
		return dl_deadline - (dl_runtime + calculate_rm_upper_interference(taskset, task_index));
	}
}

/*
 * Calculate the minimum inversion period of the given task.
 * This is the same as computing the miminum inversion "priority". 
 * Period is used here for better efficiency since we don't stroe
 * priorities for RM (periods are used to determine priorities in RM).
 */
u64 calculate_mip(struct dl_rad_taskset *taskset, int task_index) {
	int j;
	u64 dl_period = taskset->tasks[task_index]->dl.dl_period;
	u64 j_period;
	s64 j_wcib;
	u64 mip = (~(u64)0);	// initialized with the max value

	for (j=0; j<taskset->task_count; j++) {
		j_period = taskset->tasks[j]->dl.dl_period;
		j_wcib = taskset->tasks[j]->dl.wcib;
		if (j==task_index || dl_period>j_period)
			continue;

		if (j_wcib<0 && j_period<mip)
			mip = j_period;
	}

	return mip;
}

void update_taskset_dl_rad_parameters(struct dl_rad_taskset *taskset) {
	int i;

	if (sysctl_sched_dl_mode == SCHED_DLMODE_DL) {
		taskset->r_cap = calculate_r_cap(taskset);
		printk("DLRAD: new r_cap value = %llu", taskset->r_cap);
	}

	/* Compute and update Vi (WCIB) for each task (while including the new task) */
	printk("DLRAD: new V_i values:");
	for (i=0; i<taskset->task_count; i++) {
		taskset->tasks[i]->dl.wcib = calculate_wcib(taskset, i);
		printk("| V_%d = %lld.", i, taskset->tasks[i]->dl.wcib);
	}

	/* Compute the minimum inversion priority of each task (for TaskShuffler)
	 * This calculation has to be done when all V_i are computed.
	 */
	printk("DLRAD: new M_i values:");
	if (sysctl_sched_dl_mode == SCHED_DLMODE_RM) {
		for (i=0; i<taskset->task_count; i++) {
			taskset->tasks[i]->dl.mip = calculate_mip(taskset, i);
			printk("| M_%d = %llu.", i, taskset->tasks[i]->dl.mip);
		}
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
	u64 min_inversion_deadline;
	s64 min_inversion_budget = 0;
	u64 scheduled_pi_timer_duration;
	s64 rq_min_task_rib = 0;
	int idle_time_scheduling_allowed = 1;
	struct task_struct dummy_idle_task;
	u64 true_min_inversion_period;


	/* Step 1 */

	/* Does current highest priority task allow priority inversion? */
	if (leftmost_dl_se->rib <= 0)
		return leftmost_dl_se;


	/* Compute the minimum inversion property (deadline m^t_{HP} for EDF, period for RM) */
	if (sysctl_sched_dl_mode == SCHED_DLMODE_DL) {
		min_inversion_deadline = (~(u64)0);	// initialized with the max value
	} else { // SCHED_DLMODE_RM
		true_min_inversion_period = leftmost_dl_se->mip;
	}
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

		if (sysctl_sched_dl_mode == SCHED_DLMODE_DL) {
			// for SCHED_DLMODE_DL
			/* Comparison for the minimum inversion deadline */
			if ( (taskset->tasks[j]->dl.deadline > leftmost_dl_se->deadline) && (taskset->tasks[j]->dl.rib < 0) ) 
				min_inversion_deadline = (taskset->tasks[j]->dl.deadline<min_inversion_deadline)?taskset->tasks[j]->dl.deadline:min_inversion_deadline;
		} else {
			// for SCHED_DLMODE_RM
			if (taskset->tasks[j]->dl.rib<=0 && taskset->tasks[j]->dl.dl_period<true_min_inversion_period)
				true_min_inversion_period = taskset->tasks[j]->dl.dl_period;
		}
	}


	/* Create a candidate list */
	/* Note that leftmost_dl_se->rib is > 0 for sure if reaching this point. 
	 * So there will be at least one candidate after this loop.
	 */
	for (j=0; j<taskset->task_count; j++) {
		if (RB_EMPTY_NODE(&taskset->tasks[j]->dl.rb_node))
			continue;	// This is not in dl_rq.

		if (((sysctl_sched_dl_mode==SCHED_DLMODE_DL) && (taskset->tasks[j]->dl.deadline <= min_inversion_deadline)) ||
		    ((sysctl_sched_dl_mode==SCHED_DLMODE_RM) && (taskset->tasks[j]->dl.dl_period <= true_min_inversion_period))) {
			rad_candidates[candidate_count] = taskset->tasks[j];
			candidate_count++;
		}
	}


	/* Note that at least the highest priority task will be in the candidate 
	 * list at this moment. We place this check here to track the status.
	 */
	if (candidate_count == 0) {
		printk(KERN_ERR "ERROR: DLRAD: candidate list is empty.");
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


// Hard-coded, flattened Laplace distribution, in 100us (10 is 1ms)
// j=50, delta=190, location=0
u64 flattened_laplace_distributions[4][100] = {	// [4][100]
	{380, 2270, 4179, 6108, 8056, 10024, 12013, 14023, 16055, 18108, 20184, 22283, 24406, 26552, 28723, 30919, 33140, 35388, 37663, 39965, 42296, 44656, 47045, 49464, 51915, 54398, 56914, 59464, 62048, 64668, 67324, 70018, 72751, 75524, 78338, 81194, 84094, 87038, 90029, 93068, 96156, 99296, 102488, 105734, 109037, 112399, 115821, 119306, 122856, 126473, 130161, 133921, 137758, 141674, 145672, 149756, 153929, 158197, 162563, 167031, 171607, 176295, 181103, 186035, 191099, 196301, 201650, 207153, 212821, 218663, 224691, 230916, 237352, 244014, 250917, 258081, 265526, 273275, 281352, 289789, 298618, 307877, 317610, 327869, 338714, 350216, 362459, 375545, 389600, 404779, 421275, 439342, 459309, 481623, 506911, 536089, 570575, 612741, 667015, 743284},	// epsilon = 1
	{38, 227, 417, 610, 805, 1002, 1201, 1402, 1605, 1810, 2018, 2228, 2440, 2655, 2872, 3091, 3314, 3538, 3766, 3996, 4229, 4465, 4704, 4946, 5191, 5439, 5691, 5946, 6204, 6466, 6732, 7001, 7275, 7552, 7833, 8119, 8409, 8703, 9002, 9306, 9615, 9929, 10248, 10573, 10903, 11239, 11582, 11930, 12285, 12647, 13016, 13392, 13775, 14167, 14567, 14975, 15392, 15819, 16256, 16703, 17160, 17629, 18110, 18603, 19109, 19630, 20165, 20715, 21282, 21866, 22469, 23091, 23735, 24401, 25091, 25808, 26552, 27327, 28135, 28978, 29861, 30787, 31761, 32786, 33871, 35021, 36245, 37554, 38960, 40477, 42127, 43934, 45930, 48162, 50691, 53608, 57057, 61274, 66701, 74328},	// epsilon = 10
	{3, 22, 41, 61, 80, 100, 120, 140, 160, 181, 201, 222, 244, 265, 287, 309, 331, 353, 376, 399, 422, 446, 470, 494, 519, 543, 569, 594, 620, 646, 673, 700, 727, 755, 783, 811, 840, 870, 900, 930, 961, 992, 1024, 1057, 1090, 1123, 1158, 1193, 1228, 1264, 1301, 1339, 1377, 1416, 1456, 1497, 1539, 1581, 1625, 1670, 1716, 1762, 1811, 1860, 1910, 1963, 2016, 2071, 2128, 2186, 2246, 2309, 2373, 2440, 2509, 2580, 2655, 2732, 2813, 2897, 2986, 3078, 3176, 3278, 3387, 3502, 3624, 3755, 3896, 4047, 4212, 4393, 4593, 4816, 5069, 5360, 5705, 6127, 6670, 7432},	// epsilon = 100
	{0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 33, 35, 37, 39, 42, 44, 47, 49, 51, 54, 56, 59, 62, 64, 67, 70, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 109, 112, 115, 119, 122, 126, 130, 133, 137, 141, 145, 149, 153, 158, 162, 167, 171, 176, 181, 186, 191, 196, 201, 207, 212, 218, 224, 230, 237, 244, 250, 258, 265, 273, 281, 289, 298, 307, 317, 327, 338, 350, 362, 375, 389, 404, 421, 439, 459, 481, 506, 536, 570, 612, 667, 743},	// epsilon = 1000

};
s64 get_laplace_distribution_sample(u64 epsilon) {
	u64 rad_number;
	u64 distribution_index;
	u64 distribution_column_count;
	s64 sample;

	switch (epsilon) {
		case 1:
			distribution_index = 0;
			break;
		case 10:
			distribution_index = 1;
			break;
		case 100:
			distribution_index = 2;
			break;
		case 1000:
		default:
			distribution_index = 3;
			break;
	}

	get_random_bytes(&rad_number, sizeof(rad_number));
	distribution_column_count = sizeof(flattened_laplace_distributions[distribution_index])/sizeof(flattened_laplace_distributions[distribution_index][0]);
	rad_number = do_div(rad_number, 2*distribution_column_count);
	if (rad_number >= distribution_column_count) {
		sample = -flattened_laplace_distributions[distribution_index][rad_number-distribution_column_count];
	} else {
		sample = flattened_laplace_distributions[distribution_index][rad_number];
	}	
	sample *= 100000;	// convert 100us to ns
	return sample;
}

u64 get_laplace_inter_arrival_time(struct sched_dl_entity *dl_se) {
	u64 randomized_inter_arrival_time;
	do {
		randomized_inter_arrival_time = dl_se->dl_period + get_laplace_distribution_sample(dl_se->epsilon);
	} while( (randomized_inter_arrival_time<DL_LAPLACE_LOWER_PERIOD) || (randomized_inter_arrival_time>DL_LAPLACE_UPPER_PERIOD));
	printk(KERN_INFO "%llu -> %llu", dl_se->dl_period, randomized_inter_arrival_time);
	return randomized_inter_arrival_time;
}

