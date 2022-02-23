/*-
 * Copyright (c) 2021 Ruslan Bukin <br@bsdpad.com>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>

#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_file.h>

#include <dev/drm/rockchip/rk_gem.h>
#include <dev/drm/drmkpi/include/linux/dma-buf.h>

struct rockchip_gem_object {
	struct drm_gem_object	base;   /* Must go first */
	struct sg_table		*sgt;
};

MALLOC_DECLARE(M_RKGEM);

MALLOC_DEFINE(M_RKGEM, "rk_gem", "Rockchip GEM");

struct sg_table *
rockchip_gem_prime_get_sg_table(struct drm_gem_object *obj)
{
	struct sg_table *sgt;
	vm_page_t *m;
	int npages;

	m = drm_gem_cma_get_pages(obj, &npages);
	if (m == NULL)
		return (NULL);

	sgt = drm_prime_pages_to_sg(m, npages);

	return (sgt);
}

static void
rockchip_gem_free_object(struct drm_gem_object *obj)
{
	struct rockchip_gem_object *bo;

	bo = (struct rockchip_gem_object *)obj;

	if (obj->import_attach)
		drm_prime_gem_destroy(obj, bo->sgt);

	drm_gem_object_release(obj);

	free(bo, M_RKGEM);
}

int
rockchip_gem_open(struct drm_gem_object *obj, struct drm_file *file_priv)
{

	return (0);
}

void
rockchip_gem_close(struct drm_gem_object *obj, struct drm_file *file_priv)
{

}

void
rockchip_gem_print_info(struct drm_printer *p, unsigned int indent,
    const struct drm_gem_object *obj)
{

	printf("%s\n", __func__);
}

static int
rockchip_gem_pin(struct drm_gem_object *obj)
{

	printf("%s\n", __func__);

	return (0);
}

void
rockchip_gem_unpin(struct drm_gem_object *obj)
{

	printf("%s\n", __func__);
}

struct sg_table *
rockchip_gem_get_sg_table(struct drm_gem_object *obj)
{

	printf("%s\n", __func__);

	return (NULL);
}

void *
rockchip_gem_vmap(struct drm_gem_object *obj)
{

	printf("%s\n", __func__);

	return (0);
}

void
rockchip_gem_vunmap(struct drm_gem_object *obj, void *vaddr)
{

	printf("%s\n", __func__);
}

int
rockchip_gem_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{

	printf("%s\n", __func__);

	return (0);
}

static const struct drm_gem_object_funcs rockchip_gem_funcs = {
	.free = rockchip_gem_free_object,
	.open = rockchip_gem_open,
	.close = rockchip_gem_close,
	.print_info = rockchip_gem_print_info,
	.pin = rockchip_gem_pin,
	.unpin = rockchip_gem_unpin,
	.get_sg_table = rockchip_gem_get_sg_table,
	.vmap = rockchip_gem_vmap,
	.vunmap = rockchip_gem_vunmap,
	.mmap = rockchip_gem_mmap,
};

struct drm_gem_object *
rockchip_gem_prime_import_sg_table(struct drm_device *dev,
    struct dma_buf_attachment *attach, struct sg_table *sg)
{
	struct rockchip_gem_object *bo;
	size_t size;
	int error;

	bo = malloc(sizeof(*bo), M_RKGEM, M_ZERO | M_WAITOK);
	bo->base.funcs = &rockchip_gem_funcs;
	bo->sgt = sg;

	size = PAGE_ALIGN(attach->dmabuf->size);

	drm_gem_object_init(dev, &bo->base, size);

	error = drm_gem_create_mmap_offset(&bo->base);
	if (error != 0) {
		printf("%s: Failed to create mmap offset.\n", __func__);
		return (NULL);
	}

	return (&bo->base);
}

static int
rockchip_drm_gem_object_mmap(struct drm_gem_object *obj,
    struct vm_area_struct *vma)
{
	struct drm_gem_cma_object *bo;
	int npages;
	int error;

	drm_gem_cma_get_pages(obj, &npages);

	bo = container_of(obj, struct drm_gem_cma_object, gem_obj);
	if (bo->pbase == 0)
		return (0);

	error = drm_gem_mmap_obj(obj, npages * PAGE_SIZE, vma);
	drm_gem_object_put_unlocked(obj);
	if (error)
		printf("%s: error %d\n", __func__, error);

	vma->vm_pfn = OFF_TO_IDX(bo->pbase);

	return (error);
}

int
rockchip_gem_mmap_buf(struct drm_gem_object *obj, struct vm_area_struct *vma)
{

	return rockchip_drm_gem_object_mmap(obj, vma);
}
