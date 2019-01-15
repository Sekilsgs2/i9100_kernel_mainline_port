/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright 2017-2018 Qiang Yu <yuq825@gmail.com> */

#ifndef __LIMA_GEM_H__
#define __LIMA_GEM_H__

struct lima_bo;
struct lima_submit;

struct lima_bo *lima_gem_create_bo(struct drm_device *dev, u32 size, u32 flags);
int lima_gem_create_handle(struct drm_device *dev, struct drm_file *file,
			   u32 size, u32 flags, u32 *handle);
void lima_gem_free_object(struct drm_gem_object *obj);
int lima_gem_object_open(struct drm_gem_object *obj, struct drm_file *file);
void lima_gem_object_close(struct drm_gem_object *obj, struct drm_file *file);
int lima_gem_mmap_offset(struct drm_file *file, u32 handle, u64 *offset);
int lima_gem_mmap(struct file *filp, struct vm_area_struct *vma);
int lima_gem_va_map(struct drm_file *file, u32 handle, u32 flags, u32 va);
int lima_gem_va_unmap(struct drm_file *file, u32 handle, u32 va);
int lima_gem_submit(struct drm_file *file, struct lima_submit *submit);
int lima_gem_wait(struct drm_file *file, u32 handle, u32 op, u64 timeout_ns);
int lima_gem_get_modifier(struct drm_file *file, u32 handle, u64 *modifier);
int lima_gem_set_modifier(struct drm_file *file, u32 handle, u64 modifier);

#endif
