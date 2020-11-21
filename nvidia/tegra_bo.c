/*-
 * Copyright (c) 2015 Michal Meloun
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/rwlock.h>
#include <sys/vmem.h>

#include <vm/vm.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pageout.h>
#include <vm/vm_pager.h>

#include <machine/bus.h>

#include <dev/extres/clk/clk.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem.h>

#include <dev/drm/nvidia/tegra_drm.h>

static void
tegra_bo_destruct(struct tegra_bo *bo)
{
	vm_page_t m;
	int i;

	if (bo->vbase != 0) {
		pmap_qremove(bo->vbase, bo->npages);
		vmem_free(kmem_arena, bo->vbase, round_page(bo->gem_obj.size));
	}

	for (i = 0; i < bo->npages; i++) {
		m = bo->m[i];
		vm_page_lock(m);
		m->flags &= ~PG_FICTITIOUS;
		vm_page_unwire(m, PQ_NONE);
		vm_page_free(m);
		vm_page_unlock(m);
	}

}

static void
tegra_bo_free_object(struct drm_gem_object *gem_obj)
{
	struct tegra_bo *bo;

	bo = container_of(gem_obj, struct tegra_bo, gem_obj);
	drm_gem_free_mmap_offset(gem_obj);
	drm_gem_object_release(gem_obj);

	tegra_bo_destruct(bo);

	free(bo->m, DRM_MEM_DRIVER);
	free(bo, DRM_MEM_DRIVER);
}

static int
tegra_bo_alloc_contig(size_t npages, u_long alignment, vm_memattr_t memattr,
    vm_page_t **ret_page)
{
	vm_page_t m;
	int pflags, tries, i;
	vm_paddr_t low, high, boundary;

	low = 0;
	high = -1UL;
	boundary = 0;
	pflags = VM_ALLOC_NORMAL  | VM_ALLOC_NOOBJ | VM_ALLOC_NOBUSY |
	    VM_ALLOC_WIRED | VM_ALLOC_ZERO;
	tries = 0;
retry:
	m = vm_page_alloc_contig(NULL, 0, pflags, npages, low, high, alignment,
	    boundary, memattr);
	if (m == NULL) {
		if (tries < 3) {
			if (!vm_page_reclaim_contig(pflags, npages, low, high,
			    alignment, boundary))
				vm_wait(NULL);
			tries++;
			goto retry;
		}
		return (ENOMEM);
	}

	for (i = 0; i < npages; i++, m++) {
		if ((m->flags & PG_ZERO) == 0)
			pmap_zero_page(m);
		m->valid = VM_PAGE_BITS_ALL;
		(*ret_page)[i] = m;
	}

	return (0);
}


/* Allocate memory for frame buffer */
static int
tegra_bo_alloc(struct drm_device *drm, struct tegra_bo *bo)
{
	size_t size;
	vm_page_t m;
	int i;	int rv;

	size = round_page(bo->gem_obj.size);
	bo->npages = atop(size);
	bo->size = round_page(size);
	bo->m = malloc(sizeof(vm_page_t *) * bo->npages, DRM_MEM_DRIVER,
	    M_WAITOK | M_ZERO);

	rv = tegra_bo_alloc_contig(bo->npages, PAGE_SIZE,
	    VM_MEMATTR_WRITE_COMBINING, &(bo->m));
	if (rv != 0) {
		DRM_WARN("Cannot allocate memory for gem object.\n");
		return (rv);
	}

	for (i = 0; i < bo->npages; i++) {
		m = bo->m[i];
		/*
		 * XXX This is a temporary hack.
		 * We need pager suitable for paging (mmap) managed
		 * real (non-fictitious) pages.
		 * - managed pages are needed for clean module unload.
		 * - aliasing fictitious page to real one is bad,
		 *   pmap cannot handle this situation without issues
		 *   It expects that
		 *    paddr = PHYS_TO_VM_PAGE(VM_PAGE_TO_PHYS(paddr))
		 *   for every single page passed to pmap.
		 */
		m->oflags &= ~VPO_UNMANAGED;
		m->flags |= PG_FICTITIOUS;
	}

	bo->pbase = VM_PAGE_TO_PHYS(bo->m[0]);
	return (0);
}

int
tegra_bo_create(struct drm_device *drm, size_t size, struct tegra_bo **res_bo)
{
	struct tegra_bo *bo;
	int rv;

	if (size <= 0)
		return (-EINVAL);

	bo = malloc(sizeof(*bo), DRM_MEM_DRIVER, M_WAITOK | M_ZERO);

	size = round_page(size);
	rv = drm_gem_object_init(drm, &bo->gem_obj, size);
	if (rv != 0) {
		free(bo, DRM_MEM_DRIVER);
		return (rv);
	}
	rv = drm_gem_create_mmap_offset(&bo->gem_obj);
	if (rv != 0) {
		drm_gem_object_release(&bo->gem_obj);
		free(bo, DRM_MEM_DRIVER);
		return (rv);
	}

	rv = tegra_bo_alloc(drm, bo);
	if (rv != 0) {
		tegra_bo_free_object(&bo->gem_obj);
		return (rv);
	}

	*res_bo = bo;
	return (0);
}



static int
tegra_bo_create_with_handle(struct drm_file *file, struct drm_device *drm,
    size_t size, uint32_t *handle, struct tegra_bo **res_bo)
{
	int rv;
	struct tegra_bo *bo;

	rv = tegra_bo_create(drm, size, &bo);
	if (rv != 0)
		return (rv);

	rv = drm_gem_handle_create(file, &bo->gem_obj, handle);
	if (rv != 0) {
		tegra_bo_free_object(&bo->gem_obj);
		drm_gem_object_release(&bo->gem_obj);
		return (rv);
	}

	drm_gem_object_put_unlocked(&bo->gem_obj);

	*res_bo = bo;
	return (0);
}

static int
tegra_bo_dumb_create(struct drm_file *file, struct drm_device *drm_dev,
    struct drm_mode_create_dumb *args)
{
	struct tegra_drm *drm;
	struct tegra_bo *bo;
	int rv;

	drm = container_of(drm_dev, struct tegra_drm, drm_dev);

	args->pitch= (args->width * args->bpp + 7) / 8;
	args->pitch = roundup(args->pitch, drm->pitch_align);
	args->size = args->pitch * args->height;
	rv = tegra_bo_create_with_handle(file, drm_dev, args->size,
	    &args->handle, &bo);

	return (rv);
}

static int
tegra_bo_fault(struct vm_area_struct *dummy, struct vm_fault *vmf)
{

	struct vm_area_struct *vma;
	struct drm_gem_object *gem_obj;
	struct tegra_bo *bo;
	vm_object_t obj;
	vm_pindex_t pidx;
	struct page *page;
	int i;

	vma = vmf->vma;
	gem_obj = vma->vm_private_data;
	bo = container_of(gem_obj, struct tegra_bo, gem_obj);
	obj = vma->vm_obj;

	if (!bo->m)
		return (VM_FAULT_SIGBUS);

	pidx = OFF_TO_IDX(vmf->address - vma->vm_start);
	if (pidx >= bo->npages)
		return (VM_FAULT_SIGBUS);

	VM_OBJECT_WLOCK(obj);
	for (i = 0; i < bo->npages; i++) {
		page = bo->m[i];
		if (vm_page_busied(page))
			goto fail_unlock;
		if (vm_page_insert(page, obj, i))
			goto fail_unlock;
		vm_page_xbusy(page);
		page->valid = VM_PAGE_BITS_ALL;
	}
	VM_OBJECT_WUNLOCK(obj);

	vma->vm_pfn_first = 0;
	vma->vm_pfn_count =  bo->npages;
printf("%s: pidx: %llu, start: 0x%08X, addr: 0x%08lX\n", __func__, pidx, vma->vm_start, vmf->address);

	return (VM_FAULT_NOPAGE);

fail_unlock:
	VM_OBJECT_WUNLOCK(obj);
	printf("%s: insert failed\n", __func__);
	return (VM_FAULT_SIGBUS);
}


static struct vm_operations_struct tegra_bo_vm_ops = {
	.fault = tegra_bo_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

int
tegra_bo_mmap(struct drm_gem_object *gem_obj, struct vm_area_struct *vma)
{
	struct tegra_bo *bo;

	bo = container_of(gem_obj, struct tegra_bo, gem_obj);
	if (bo->pbase == 0)
		return (0);

	vma->vm_pfn = OFF_TO_IDX(bo->pbase);
	return (0);
}

/* Fill up relevant fields in drm_driver ops */
void
tegra_bo_driver_register(struct drm_driver *drm_drv)
{
	drm_drv->gem_free_object = tegra_bo_free_object;
	drm_drv->gem_vm_ops = &tegra_bo_vm_ops,
	drm_drv->dumb_create = tegra_bo_dumb_create;
	drm_drv->dumb_map_offset = drm_gem_dumb_map_offset;
	drm_drv->dumb_destroy = drm_gem_dumb_destroy;
}
