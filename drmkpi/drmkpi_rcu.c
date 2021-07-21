/*-
 * Copyright (c) 2016 Matthew Macy (mmacy@mattmacy.io)
 * Copyright (c) 2017-2020 Hans Petter Selasky (hselasky@freebsd.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
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

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <drmkpi/rcupdate.h>
#include <drmkpi/srcu.h>

struct mtx drmkpi_global_rcu_lock;

int
drmkpi_init_srcu_struct(struct srcu_struct *srcu)
{

	return (0);
}

void
drmkpi_cleanup_srcu_struct(struct srcu_struct *srcu)
{

}

int
drmkpi_srcu_read_lock(struct srcu_struct *srcu)
{

	drmkpi_rcu_read_lock(RCU_TYPE_SLEEPABLE);

	return (0);
}

void
drmkpi_srcu_read_unlock(struct srcu_struct *srcu, int key __unused)
{

	drmkpi_rcu_read_unlock(RCU_TYPE_SLEEPABLE);
}

void
drmkpi_synchronize_srcu(struct srcu_struct *srcu)
{

	drmkpi_synchronize_rcu(RCU_TYPE_SLEEPABLE);
}

void
drmkpi_srcu_barrier(struct srcu_struct *srcu)
{

	drmkpi_rcu_barrier(RCU_TYPE_SLEEPABLE);
}

static void
drmkpi_rcu_runtime_init(void *arg __unused)
{

	mtx_init(&drmkpi_global_rcu_lock, "global rcu", NULL, MTX_DEF);
}
SYSINIT(drmkpi_rcu_runtime, SI_SUB_CPU, SI_ORDER_ANY,
    drmkpi_rcu_runtime_init, NULL);

static void
drmkpi_rcu_runtime_uninit(void *arg __unused)
{

	mtx_destroy(&drmkpi_global_rcu_lock);
}
SYSUNINIT(drmkpi_rcu_runtime, SI_SUB_LOCK, SI_ORDER_SECOND,
    drmkpi_rcu_runtime_uninit, NULL);
