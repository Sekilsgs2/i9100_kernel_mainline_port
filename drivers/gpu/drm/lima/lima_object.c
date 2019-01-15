// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright 2018 Qiang Yu <yuq825@gmail.com> */

#include <drm/drm_prime.h>
#include <drm/drm_fourcc.h>

#include "lima_object.h"

static void lima_bo_init_placement(struct lima_bo *bo)
{
	struct ttm_placement *placement = &bo->placement;
	struct ttm_place *place = &bo->place;

	place->fpfn = 0;
	place->lpfn = 0;
	place->flags = TTM_PL_FLAG_TT | TTM_PL_FLAG_WC;

	/* pin all bo for now */
	place->flags |= TTM_PL_FLAG_NO_EVICT;

	placement->num_placement = 1;
	placement->placement = place;

	placement->num_busy_placement = 1;
	placement->busy_placement = place;
}

static void lima_bo_destroy(struct ttm_buffer_object *tbo)
{
	struct lima_bo *bo = ttm_to_lima_bo(tbo);

	if (bo->gem.import_attach)
		drm_prime_gem_destroy(&bo->gem, bo->tbo.sg);
	drm_gem_object_release(&bo->gem);
	kfree(bo);
}

struct lima_bo *lima_bo_create(struct lima_device *dev, u64 size,
			       u32 flags, enum ttm_bo_type type,
			       struct sg_table *sg,
			       struct reservation_object *resv)
{
	struct lima_bo *bo;
	struct ttm_mem_type_manager *man;
	size_t acc_size;
	int err;

	size = PAGE_ALIGN(size);
	man = dev->mman.bdev.man + TTM_PL_TT;
	if (size >= (man->size << PAGE_SHIFT))
		return ERR_PTR(-ENOMEM);

	acc_size = ttm_bo_dma_acc_size(&dev->mman.bdev, size,
				       sizeof(struct lima_bo));

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return ERR_PTR(-ENOMEM);

	drm_gem_private_object_init(dev->ddev, &bo->gem, size);

	INIT_LIST_HEAD(&bo->va);

	bo->tbo.bdev = &dev->mman.bdev;

	lima_bo_init_placement(bo);

	err = ttm_bo_init(&dev->mman.bdev, &bo->tbo, size, type,
			  &bo->placement, 0, type != ttm_bo_type_kernel,
			  acc_size, sg, resv, &lima_bo_destroy);
	if (err)
		goto err_out;

	bo->modifier = DRM_FORMAT_MOD_INVALID;
	return bo;

err_out:
	kfree(bo);
	return ERR_PTR(err);
}

dma_addr_t *lima_bo_get_pages(struct lima_bo *bo)
{
	struct lima_ttm_tt *ttm = (void *)bo->tbo.ttm;
	return ttm->ttm.dma_address;
}

void *lima_bo_kmap(struct lima_bo *bo)
{
	bool is_iomem;
	void *ret;
	int err;

	ret = ttm_kmap_obj_virtual(&bo->kmap, &is_iomem);
	if (ret)
		return ret;

	err = ttm_bo_kmap(&bo->tbo, 0, bo->tbo.num_pages, &bo->kmap);
	if (err)
		return ERR_PTR(err);

	return ttm_kmap_obj_virtual(&bo->kmap, &is_iomem);
}
