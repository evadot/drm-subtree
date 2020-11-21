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

#ifndef	__DRMKPI_LINUX_MODULEPARAM_H__
#define	__DRMKPI_LINUX_MODULEPARAM_H__

#include <sys/types.h>
#include <sys/sysctl.h>

#include <linux/types.h>

#ifndef DRMKPI_PARAM_PARENT
#define	DRMKPI_PARAM_PARENT	_dev_drm
#endif

#ifndef DRMKPI_PARAM_PREFIX
#define	DRMKPI_PARAM_PREFIX	/* empty prefix is the default */
#endif

#ifndef DRMKPI_PARAM_PERM
#define	DRMKPI_PARAM_PERM(perm) (((perm) & 0222) ? CTLFLAG_RWTUN : CTLFLAG_RDTUN)
#endif

#define	DRMKPI_PARAM_CONCAT_SUB(a,b,c,d) a##b##c##d
#define	DRMKPI_PARAM_CONCAT(...) DRMKPI_PARAM_CONCAT_SUB(__VA_ARGS__)
#define	DRMKPI_PARAM_PASS(...) __VA_ARGS__
#define	DRMKPI_PARAM_DESC(name) DRMKPI_PARAM_CONCAT(drmkpi_,DRMKPI_PARAM_PREFIX,name,_desc)
#define	DRMKPI_PARAM_NAME(name) DRMKPI_PARAM_CONCAT(DRMKPI_PARAM_PREFIX,name,,)

#define	DRMKPI_PARAM_bool(name, var, perm)				\
	extern const char DRMKPI_PARAM_DESC(name)[];			\
	DRMKPI_PARAM_PASS(SYSCTL_BOOL(DRMKPI_PARAM_PARENT, OID_AUTO,\
	DRMKPI_PARAM_NAME(name), DRMKPI_PARAM_PERM(perm), &(var), 0, \
	DRMKPI_PARAM_DESC(name)))

#define	DRMKPI_PARAM_byte(name, var, perm)				\
	extern const char DRMKPI_PARAM_DESC(name)[];			\
	DRMKPI_PARAM_PASS(SYSCTL_U8(DRMKPI_PARAM_PARENT, OID_AUTO,	\
	DRMKPI_PARAM_NAME(name), DRMKPI_PARAM_PERM(perm), &(var), 0, \
	DRMKPI_PARAM_DESC(name)))

#define	DRMKPI_PARAM_short(name, var, perm)				\
	extern const char DRMKPI_PARAM_DESC(name)[];			\
	DRMKPI_PARAM_PASS(SYSCTL_S16(DRMKPI_PARAM_PARENT, OID_AUTO,	\
	DRMKPI_PARAM_NAME(name), DRMKPI_PARAM_PERM(perm), &(var), 0, \
	DRMKPI_PARAM_DESC(name)))

#define	DRMKPI_PARAM_ushort(name, var, perm)				\
	extern const char DRMKPI_PARAM_DESC(name)[];			\
	DRMKPI_PARAM_PASS(SYSCTL_U16(DRMKPI_PARAM_PARENT, OID_AUTO,	\
	DRMKPI_PARAM_NAME(name), DRMKPI_PARAM_PERM(perm), &(var), 0, \
	DRMKPI_PARAM_DESC(name)))

#define	DRMKPI_PARAM_int(name, var, perm)				\
	extern const char DRMKPI_PARAM_DESC(name)[];			\
	DRMKPI_PARAM_PASS(SYSCTL_INT(DRMKPI_PARAM_PARENT, OID_AUTO,	\
	DRMKPI_PARAM_NAME(name), DRMKPI_PARAM_PERM(perm), &(var), 0,\
	DRMKPI_PARAM_DESC(name)))

#define	DRMKPI_PARAM_uint(name, var, perm)				\
	extern const char DRMKPI_PARAM_DESC(name)[];			\
	DRMKPI_PARAM_PASS(SYSCTL_UINT(DRMKPI_PARAM_PARENT, OID_AUTO, \
	DRMKPI_PARAM_NAME(name), DRMKPI_PARAM_PERM(perm), &(var), 0, \
	DRMKPI_PARAM_DESC(name)))

#define	DRMKPI_PARAM_long(name, var, perm)				\
	extern const char DRMKPI_PARAM_DESC(name)[];			\
	DRMKPI_PARAM_PASS(SYSCTL_LONG(DRMKPI_PARAM_PARENT, OID_AUTO, \
	DRMKPI_PARAM_NAME(name), DRMKPI_PARAM_PERM(perm), &(var), 0, \
	DRMKPI_PARAM_DESC(name)))

#define	DRMKPI_PARAM_ulong(name, var, perm)				\
	extern const char DRMKPI_PARAM_DESC(name)[];			\
	DRMKPI_PARAM_PASS(SYSCTL_ULONG(DRMKPI_PARAM_PARENT, OID_AUTO, \
	DRMKPI_PARAM_NAME(name), DRMKPI_PARAM_PERM(perm), &(var), 0, \
	DRMKPI_PARAM_DESC(name)))

#define	module_param_string(name, str, len, perm)			\
	extern const char DRMKPI_PARAM_DESC(name)[];			\
	DRMKPI_PARAM_PASS(SYSCTL_STRING(DRMKPI_PARAM_PARENT, OID_AUTO, \
	DRMKPI_PARAM_NAME(name), DRMKPI_PARAM_PERM(perm), (str), (len), \
	DRMKPI_PARAM_DESC(name)))

#define	module_param_named(name, var, type, mode)	\
	DRMKPI_PARAM_##type(name, var, mode)

#define	module_param(var, type, mode)	\
	DRMKPI_PARAM_##type(var, var, mode)

#define	module_param_named_unsafe(name, var, type, mode) \
	DRMKPI_PARAM_##type(name, var, mode)

#define	module_param_unsafe(var, type, mode) \
	DRMKPI_PARAM_##type(var, var, mode)

#define	module_param_array(var, type, addr_argc, mode)

#define	MODULE_PARM_DESC(name, desc) \
	const char DRMKPI_PARAM_DESC(name)[] = { desc }

#define	kernel_param_lock(...) do {} while (0)
#define	kernel_param_unlock(...) do {} while (0)

SYSCTL_DECL(_dev_drm);

#endif	/* __DRMKPI_LINUX_MODULEPARAM_H__ */
