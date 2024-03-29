/*-
 * Copyright (c) 2010 Isilon Systems, Inc.
 * Copyright (c) 2010 iX Systems, Inc.
 * Copyright (c) 2010 Panasas, Inc.
 * Copyright (c) 2013-2016 Mellanox Technologies, Ltd.
 * Copyright (c) 2015 François Tigeot
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
 *
 * $FreeBSD$
 */

#ifndef	__DRMCOMPAT_LINUX_UACCESS_H__
#define	__DRMCOMPAT_LINUX_UACCESS_H__

#include <drmcompat/uaccess.h>

#define	__get_user(_x, _p) ({					\
	int __err;						\
	__typeof(*(_p)) __x;					\
	__err = drmcompat_copyin((_p), &(__x), sizeof(*(_p)));	\
	(_x) = __x;						\
	__err;							\
})

#define	__put_user(_x, _p) ({				\
	__typeof(*(_p)) __x = (_x);			\
	drmcompat_copyout(&(__x), (_p), sizeof(*(_p)));	\
})

#define	get_user(_x, _p)	drmcompat_copyin((_p), &(_x), sizeof(*(_p)))
#define	put_user(_x, _p)	__put_user(_x, _p)
#define	clear_user(...)		drmcompat_clear_user(__VA_ARGS__)
#define	access_ok(...)		drmcompat_access_ok(__VA_ARGS__)

#endif	/* __DRMCOMPAT_LINUX_UACCESS_H__ */
