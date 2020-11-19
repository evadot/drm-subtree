/*-
 * Copyright (c) 2010 Isilon Systems, Inc.
 * Copyright (c) 2010 iX Systems, Inc.
 * Copyright (c) 2010 Panasas, Inc.
 * Copyright (c) 2013-2017 Mellanox Technologies, Ltd.
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

#ifndef __DRMKPI_LINUX_FILE_H__
#define	__DRMKPI_LINUX_FILE_H__

#include <sys/param.h>
#include <sys/file.h>
#include <sys/filedesc.h>
#include <sys/refcount.h>
#include <sys/capsicum.h>
#include <sys/proc.h>

#include <linux/fs.h>


static inline void
put_unused_fd(unsigned int fd)
{
	struct file *file;

	panic("%s: Not implemented yet.", __func__);
	if (fget_unlocked(curthread->td_proc->p_fd, fd,
	    &cap_no_rights, &file) != 0) {
		return;
	}
	/*
	 * NOTE: We should only get here when the "fd" has not been
	 * installed, so no need to free the associated Linux file
	 * structure.
	 */
	fdclose(curthread, file, fd);

	/* drop extra reference */
	fdrop(file, curthread);
}

static inline int
get_unused_fd(void)
{
	struct file *file;
	int rv;
	int fd;

	panic("%s: Not implemented yet.", __func__);
	rv = falloc(curthread, &file, &fd, 0);
	if (rv)
		return (-rv);
	/* drop the extra reference */
	fdrop(file, curthread);
	return (fd);
}

static inline int
get_unused_fd_flags(int flags)
{
	struct file *file;
	int rv;
	int fd;

	panic("%s: Not implemented yet.", __func__);
	rv = falloc(curthread, &file, &fd, flags);
	if (rv)
		return (-rv);
	/* drop the extra reference */
	fdrop(file, curthread);
	return (fd);
}

static inline void
fd_install(unsigned int fd, struct file *file)
{
	panic("%s: Not implemented yet.", __func__);
}

static inline void
fput(struct file *file)
{

	if (refcount_release(&file->f_count)) {
		if (file->f_ops != NULL)
			_fdrop(file, curthread);
//		else
//			vm_object_deallocate(filp->f_shmem);
	}
}

#endif	/* __DRMKPI_LINUX_FILE_H__ */
