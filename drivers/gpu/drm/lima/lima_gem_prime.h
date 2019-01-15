/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright 2018 Qiang Yu <yuq825@gmail.com> */

#ifndef __LIMA_GEM_PRIME_H__
#define __LIMA_GEM_PRIME_H__

struct drm_gem_object *lima_gem_prime_import_sg_table(
	struct drm_device *dev, struct dma_buf_attachment *attach,
	struct sg_table *sgt);
struct sg_table *lima_gem_prime_get_sg_table(struct drm_gem_object *obj);
struct reservation_object *lima_gem_prime_res_obj(struct drm_gem_object *obj);
void *lima_gem_prime_vmap(struct drm_gem_object *obj);
void lima_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr);
int lima_gem_prime_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma);

int lima_gem_prime_dma_buf_mmap(struct file *filp, struct vm_area_struct *vma);

#endif
