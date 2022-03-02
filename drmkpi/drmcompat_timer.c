/*-
 * Copyright (c) 2015-2018 Mellanox Technologies, Ltd.
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

#include <sys/param.h>
#include <sys/time.h>
#include <sys/kernel.h>

#include <linux/compat.h>	/* For linux_set_current */

#include <drmkpi/timer.h>

unsigned long drmkpi_timer_hz_mask;

uint64_t drmkpi_nsec2hz_rem;
uint64_t drmkpi_nsec2hz_div = 1000000000ULL;
uint64_t drmkpi_nsec2hz_max;

uint64_t drmkpi_usec2hz_rem;
uint64_t drmkpi_usec2hz_div = 1000000ULL;
uint64_t drmkpi_usec2hz_max;

uint64_t drmkpi_msec2hz_rem;
uint64_t drmkpi_msec2hz_div = 1000ULL;
uint64_t drmkpi_msec2hz_max;

static inline int
timer_jiffies_until(int expires)
{
	int delta = expires - ticks;
	/* guard against already expired values */
	if (delta < 1)
		delta = 1;
	return (delta);
}

/* greatest common divisor, Euclid equation */
static uint64_t
drmkpi_gcd_64(uint64_t a, uint64_t b)
{
	uint64_t an;
	uint64_t bn;

	while (b != 0) {
		an = b;
		bn = a % b;
		a = an;
		b = bn;
	}
	return (a);
}

static void
drmkpi_timer_callback_wrapper(void *context)
{
	struct timer_list *timer;

	linux_set_current(curthread);

	timer = context;
	timer->function(timer->data);
}

int
drmkpi_mod_timer(struct timer_list *timer, int expires)
{
	int ret;

	timer->expires = expires;
	ret = callout_reset(&timer->callout,
	    timer_jiffies_until(expires),
	    &drmkpi_timer_callback_wrapper, timer);

	MPASS(ret == 0 || ret == 1);

	return (ret == 1);
}

void
drmkpi_add_timer(struct timer_list *timer)
{

	callout_reset(&timer->callout,
	    timer_jiffies_until(timer->expires),
	    &drmkpi_timer_callback_wrapper, timer);
}

void
drmkpi_add_timer_on(struct timer_list *timer, int cpu)
{

	callout_reset_on(&timer->callout,
	    timer_jiffies_until(timer->expires),
	    &drmkpi_timer_callback_wrapper, timer, cpu);
}

int
drmkpi_del_timer(struct timer_list *timer)
{

	if (callout_stop(&(timer)->callout) == -1)
		return (0);
	return (1);
}

int
drmkpi_del_timer_sync(struct timer_list *timer)
{

	if (callout_drain(&(timer)->callout) == -1)
		return (0);
	return (1);
}

static void
drmkpi_timer_init(void *arg)
{
	uint64_t gcd;

	/*
	 * Compute an internal HZ value which can divide 2**32 to
	 * avoid timer rounding problems when the tick value wraps
	 * around 2**32:
	 */
	drmkpi_timer_hz_mask = 1;
	while (drmkpi_timer_hz_mask < (unsigned long)hz)
		drmkpi_timer_hz_mask *= 2;
	drmkpi_timer_hz_mask--;

	/* compute some internal constants */

	drmkpi_nsec2hz_rem = hz;
	drmkpi_usec2hz_rem = hz;
	drmkpi_msec2hz_rem = hz;

	gcd = drmkpi_gcd_64(drmkpi_nsec2hz_rem, drmkpi_nsec2hz_div);
	drmkpi_nsec2hz_rem /= gcd;
	drmkpi_nsec2hz_div /= gcd;
	drmkpi_nsec2hz_max = -1ULL / drmkpi_nsec2hz_rem;

	gcd = drmkpi_gcd_64(drmkpi_usec2hz_rem, drmkpi_usec2hz_div);
	drmkpi_usec2hz_rem /= gcd;
	drmkpi_usec2hz_div /= gcd;
	drmkpi_usec2hz_max = -1ULL / drmkpi_usec2hz_rem;

	gcd = drmkpi_gcd_64(drmkpi_msec2hz_rem, drmkpi_msec2hz_div);
	drmkpi_msec2hz_rem /= gcd;
	drmkpi_msec2hz_div /= gcd;
	drmkpi_msec2hz_max = -1ULL / drmkpi_msec2hz_rem;
}
SYSINIT(drmkpi_timer, SI_SUB_DRIVERS, SI_ORDER_FIRST, drmkpi_timer_init, NULL);
