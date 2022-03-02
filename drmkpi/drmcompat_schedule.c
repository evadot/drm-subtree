/*-
 * Copyright (c) 2017 Mark Johnston <markj@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conds
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conds, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conds and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/signalvar.h>
#include <sys/sleepqueue.h>

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock.h>

#include <drmkpi/completion.h>
#include <drmkpi/sched.h>
#include <drmkpi/wait.h>
#include <drmkpi/ww_mutex.h>

static int
drmkpi_add_to_sleepqueue(void *wchan, struct task_struct *task,
    const char *wmesg, int timeout, int state)
{
	int flags, ret;

	MPASS((state & ~(TASK_PARKED | TASK_NORMAL)) == 0);

	flags = SLEEPQ_SLEEP | ((state & TASK_INTERRUPTIBLE) != 0 ?
	    SLEEPQ_INTERRUPTIBLE : 0);

	sleepq_add(wchan, NULL, wmesg, flags, 0);
	if (timeout != 0)
		sleepq_set_timeout(wchan, timeout);

	DROP_GIANT();
	if ((state & TASK_INTERRUPTIBLE) != 0) {
		if (timeout == 0)
			ret = -sleepq_wait_sig(wchan, 0);
		else
			ret = -sleepq_timedwait_sig(wchan, 0);
	} else {
		if (timeout == 0) {
			sleepq_wait(wchan, 0);
			ret = 0;
		} else
			ret = -sleepq_timedwait(wchan, 0);
	}
	PICKUP_GIANT();

	/* filter return value */
	if (ret != 0 && ret != -EWOULDBLOCK) {
		ret = -ERESTARTSYS;
	}
	return (ret);
}

static int
wake_up_task(struct task_struct *task, unsigned int state)
{
	int ret, wakeup_swapper;

	ret = wakeup_swapper = 0;
	sleepq_lock(task);
	if ((atomic_read(&task->state) & state) != 0) {
		set_task_state(task, TASK_WAKING);
		wakeup_swapper = sleepq_signal(task, SLEEPQ_SLEEP, 0, 0);
		ret = 1;
	}
	sleepq_release(task);
	if (wakeup_swapper)
		kick_proc0();
	return (ret);
}

bool
drmkpi_signal_pending(struct task_struct *task)
{
	struct thread *td;
	sigset_t pending;

	td = task->task_thread;
	PROC_LOCK(td->td_proc);
	pending = td->td_siglist;
	SIGSETOR(pending, td->td_proc->p_siglist);
	SIGSETNAND(pending, td->td_sigmask);
	PROC_UNLOCK(td->td_proc);
	return (!SIGISEMPTY(pending));
}

int
drmkpi_autoremove_wake_function(wait_queue_entry_t *wq, unsigned int state, int flags,
    void *key __unused)
{
	struct task_struct *task;
	int ret;

	task = wq->private;
	if ((ret = wake_up_task(task, state)) != 0)
		list_del_init(&wq->entry);
	return (ret);
}

void
drmkpi_wake_up(wait_queue_head_t *wqh, unsigned int state, int nr, bool locked)
{
	wait_queue_entry_t *pos, *next;

	if (!locked)
		spin_lock(&wqh->lock);
	list_for_each_entry_safe(pos, next, &wqh->head, entry) {
		if (pos->func == NULL) {
			if (wake_up_task(pos->private, state) != 0 && --nr == 0)
				break;
		} else {
			if (pos->func(pos, state, 0, NULL) != 0 && --nr == 0)
				break;
		}
	}
	if (!locked)
		spin_unlock(&wqh->lock);
}

void
drmkpi_prepare_to_wait(wait_queue_head_t *wqh, wait_queue_entry_t *wq, int state)
{

	spin_lock(&wqh->lock);
	if (list_empty(&wq->entry))
		list_add(&wqh->head, &wq->entry);
	set_task_state(current, state);
	spin_unlock(&wqh->lock);
}

void
drmkpi_finish_wait(wait_queue_head_t *wqh, wait_queue_entry_t *wq)
{

	spin_lock(&wqh->lock);
	set_task_state(current, TASK_RUNNING);
	if (!list_empty(&wq->entry)) {
		list_del(&wq->entry);
		INIT_LIST_HEAD(&wq->entry);
	}
	spin_unlock(&wqh->lock);
}

int
drmkpi_wait_event_common(wait_queue_head_t *wqh, wait_queue_entry_t *wq, int timeout,
    unsigned int state, spinlock_t *lock)
{
	struct task_struct *task;
	int ret;

	if (lock != NULL)
		spin_unlock_irq(lock);

	/* range check timeout */
	if (timeout < 1)
		timeout = 1;
	else if (timeout == MAX_SCHEDULE_TIMEOUT)
		timeout = 0;

	task = current;

	/*
	 * Our wait queue entry is on the stack - make sure it doesn't
	 * get swapped out while we sleep.
	 */
	PHOLD(task->task_thread->td_proc);
	sleepq_lock(task);
	if (atomic_read(&task->state) != TASK_WAKING) {
		ret = drmkpi_add_to_sleepqueue(task, task, "wevent", timeout,
		    state);
	} else {
		sleepq_release(task);
		ret = 0;
	}
	PRELE(task->task_thread->td_proc);

	if (lock != NULL)
		spin_lock_irq(lock);
	return (ret);
}

int
drmkpi_schedule_timeout(int timeout)
{
	struct task_struct *task;
	int ret;
	int state;
	int remainder;

	task = current;

	/* range check timeout */
	if (timeout < 1)
		timeout = 1;
	else if (timeout == MAX_SCHEDULE_TIMEOUT)
		timeout = 0;

	remainder = ticks + timeout;

	sleepq_lock(task);
	state = atomic_read(&task->state);
	if (state != TASK_WAKING) {
		ret = drmkpi_add_to_sleepqueue(task, task, "sched", timeout,
		    state);
	} else {
		sleepq_release(task);
		ret = 0;
	}
	set_task_state(task, TASK_RUNNING);

	if (timeout == 0)
		return (MAX_SCHEDULE_TIMEOUT);

	/* range check return value */
	remainder -= ticks;

	/* range check return value */
	if (ret == -ERESTARTSYS && remainder < 1)
		remainder = 1;
	else if (remainder < 0)
		remainder = 0;
	else if (remainder > timeout)
		remainder = timeout;
	return (remainder);
}

bool
drmkpi_wake_up_state(struct task_struct *task, unsigned int state)
{

	return (wake_up_task(task, state) != 0);
}
