/*
 * videobuf2-vmalloc.h - vmalloc memory allocator for videobuf2
 *
 * Copyright (C) 2010 Samsung Electronics
 *
 * Author: Pawel Osciak <pawel@osciak.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#ifndef _MEDIA_VIDEOBUF2_CMAALLOC_H
#define _MEDIA_VIDEOBUF2_CMAALLOC_H

#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-memops.h>

extern const struct vb2_mem_ops vb2_cmalloc_memops;

struct vb2_cmalloc_buf {
	void				*vaddr;
	/* 4.9->6.1 port: dma_alloc_coherent()'s dma_handle out-param, needed
	 * to call dma_free_coherent() later -- see cma_alloc()/cma_free() in
	 * isp-vb2-cmalloc.c for why this replaced the old
	 * dma_alloc_from_contiguous()-based approach (that API isn't
	 * reachable from an out-of-tree module, confirmed at modpost time). */
	dma_addr_t			dma_handle;
	struct frame_vector		*vec;
	enum dma_data_direction		dma_dir;
	unsigned long			size;
	/* 4.9->6.1 port: struct vb2_vmarea_handler.refcount (in
	 * <media/videobuf2-memops.h>) changed type from atomic_t * to
	 * refcount_t * -- refcount_t is the "checked" refcounter type
	 * (catches use-after-free/overflow) that atomic_t-as-a-refcount
	 * was gradually replaced with across the kernel. Must match here
	 * since &buf->refcount is handed to handler.refcount by pointer. */
	refcount_t			refcount;
	struct vb2_vmarea_handler	handler;
	struct dma_buf			*dbuf;
};


#endif
