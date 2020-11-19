/*-
 * Copyright (c) 2010 Isilon Systems, Inc.
 * Copyright (c) 2010 iX Systems, Inc.
 * Copyright (c) 2010 Panasas, Inc.
 * Copyright (c) 2013-2016 Mellanox Technologies, Ltd.
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

#ifndef __LINUX_IDR_H__
#define	__LINUX_IDR_H__

#include <linux/types.h>

#include <drmkpi/idr.h>

/* NOTE: It is the applications responsibility to destroy the IDR */
#define	DEFINE_IDR(name)						\
	struct idr name;						\
	SYSINIT(name##_idr_sysinit, SI_SUB_DRIVERS, SI_ORDER_FIRST,	\
	    idr_init, &(name))

/* NOTE: It is the applications responsibility to destroy the IDA */
#define	DEFINE_IDA(name)						\
	struct ida name;						\
	SYSINIT(name##_ida_sysinit, SI_SUB_DRIVERS, SI_ORDER_FIRST,	\
	    ida_init, &(name))

#define	idr_preload(gfp)			drmkpi_idr_preload(gfp)
#define	idr_preload_end()			drmkpi_idr_preload_end()
#define	idr_find(idr, id)			drmkpi_idr_find(idr, id)
#define	idr_get_next(idr, id)			drmkpi_idr_get_next(idr, id)
#define	idr_is_empty(idr)			drmkpi_idr_is_empty(idr)
#define	idr_pre_get(idr, gfp)			drmkpi_idr_pre_get(idr, gfp)
#define	idr_get_new(idr, ptr, id)		drmkpi_idr_get_new(idr, ptr, id)
#define	idr_get_new_above(idr, ptr, s_id, id)	drmkpi_idr_get_new_above(idr, ptr, s_id, id)
#define	idr_replace(idr, ptr, id)		drmkpi_idr_replace(idr, ptr, id)
#define	idr_remove(idr, id)			drmkpi_idr_remove(idr, id)
#define	idr_remove_all(idr)			drmkpi_idr_remove_all(idr)
#define	idr_destroy(idr)			drmkpi_idr_destroy(idr)
#define	idr_init(idr)				drmkpi_idr_init(idr)
#define	idr_alloc(idr, ptr, s, e, gfp)		drmkpi_idr_alloc(idr, ptr, s, e, gfp)
#define	idr_alloc_cyclic(idr, ptr, s, e, gfp)	drmkpi_idr_alloc_cyclic(idr, ptr, s, e, gfp)
#define	idr_for_each(idr, fn, data)		drmkpi_idr_for_each(idr, fn, data)

#define	idr_for_each_entry(idp, entry, id)	\
	for ((id) = 0; ((entry) = idr_get_next(idp, &(id))) != NULL; ++(id))

#define	ida_pre_get(ida, id)			drmkpi_ida_pre_get(ida, id)
#define	ida_pre_get_new_above(ida, s_id, p_id)	drmkpi_ida_get_new_above(ida, s_id, p_id)
#define	ida_remove(ida, id)			drmkpi_ida_remove(ida, id)
#define	ida_destroy(ida)			drmkpi_ida_destroy(ida)
#define	ida_init(ida)				drmkpi_ida_init(ida)
#define	ida_simple_get(ida, s, e, gfp)		drmkpi_ida_simple_get(ida, s, e, gfp)
#define	ida_simple_remove(ida, id)		drmkpi_ida_simple_remove(ida, id)
#define	ida_get_new(ida, id)			drmkpi_ida_get_new_above(ida, 0, id)

#endif	/* __LINUX_IDR_H__ */
