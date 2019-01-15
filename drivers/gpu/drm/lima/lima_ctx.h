/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright 2018 Qiang Yu <yuq825@gmail.com> */

#ifndef __LIMA_CTX_H__
#define __LIMA_CTX_H__

#include <linux/idr.h>

#include "lima_device.h"

struct lima_ctx {
	struct kref refcnt;
	struct lima_device *dev;
	struct lima_sched_context context[lima_pipe_num];
	atomic_t guilty;
};

struct lima_ctx_mgr {
	spinlock_t lock;
	struct idr handles;
};

int lima_ctx_create(struct lima_device *dev, struct lima_ctx_mgr *mgr, u32 *id);
int lima_ctx_free(struct lima_ctx_mgr *mgr, u32 id);
struct lima_ctx *lima_ctx_get(struct lima_ctx_mgr *mgr, u32 id);
void lima_ctx_put(struct lima_ctx *ctx);
void lima_ctx_mgr_init(struct lima_ctx_mgr *mgr);
void lima_ctx_mgr_fini(struct lima_ctx_mgr *mgr);

struct dma_fence *lima_ctx_get_native_fence(struct lima_ctx_mgr *mgr,
					    u32 ctx, u32 pipe, u32 seq);

#endif
