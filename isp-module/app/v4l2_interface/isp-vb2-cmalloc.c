/*
 * videobuf2-vmalloc.c - vmalloc memory allocator for videobuf2
 *
 * Copyright (C) 2010 Samsung Electronics
 *
 * Author: Pawel Osciak <pawel@osciak.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/iosys-map.h> /* struct iosys_map, for dma_buf_ops.vmap below */

#include "isp-vb2-cmalloc.h"

/* 4.9->6.1 port, REVISED after real modpost testing on the target kernel:
 * originally ported this to keep calling dma_alloc_from_contiguous()/
 * dma_release_from_contiguous() (just fixed up for their header move to
 * <linux/dma-map-ops.h> and the new `bool no_warn` param), with a comment
 * here flagging that neither symbol is EXPORT_SYMBOL'd in this kernel's
 * Module.symvers so it'd probably compile but fail modpost/insmod.
 * Confirmed exactly that on a real `make modules` run against
 * /usr/src/linux-headers-6.1.68-3-stable:
 *   ERROR: modpost: "dma_release_from_contiguous" [...] undefined!
 *   ERROR: modpost: "dma_alloc_from_contiguous" [...] undefined!
 * So this is not a maybe -- those two are genuinely unusable from an
 * out-of-tree module on this kernel, full stop. dma_alloc_contiguous()/
 * dma_free_contiguous() (the newer public wrapper) are ALSO not exported
 * here (checked Module.symvers, absent). The only CMA-backed allocation
 * path actually available to an external module is dma_alloc_coherent()/
 * dma_free_coherent() -- always exported, and its arch implementation
 * (dma_direct_alloc on arm64, no IOMMU on this device) internally uses
 * the device's declared CMA/reserved-memory region the same way
 * dma_alloc_from_contiguous() would have. So switched to it below.
 *
 * NOT VERIFIED -- flagged in PORT_NOTES.md: the rest of this driver (see
 * isp-vb2.c's virt_to_phys(vb2_plane_vaddr(...))) assumes the vaddr
 * handed back here is part of the kernel's linear map, so virt_to_phys()
 * on it is meaningful. That holds for dma_alloc_coherent() on arm64 when
 * the device is cache-coherent for DMA (typical for an on-SoC ISP with no
 * IOMMU in front of it, which matches this hardware), but would NOT hold
 * if the device ends up needing non-coherent (write-combine, non-cached
 * vmap'd) allocations -- that path returns memory outside the linear map
 * and virt_to_phys() on it would be wrong. Whether this device's DT node
 * carries (or needs) a `dma-coherent` property is a devicetree-authoring
 * question outside this pass's scope; flagging it as the #1 thing to
 * check before trusting frame addresses out of this allocator. */
static void *cma_alloc(struct device *dev, unsigned long size, dma_addr_t *dma_handle)
{
    void *vaddr = NULL;

    vaddr = dma_alloc_coherent(dev, size, dma_handle, GFP_KERNEL);
    if (!vaddr) {
        pr_err("Failed to alloc cma pages.\n");
        return NULL;
    }

    return vaddr;
}

static void cma_free(void *buf_priv)
{
    struct vb2_cmalloc_buf *buf = buf_priv;
    struct device *dev = NULL;

    dev = (void *)(buf->dbuf);

    dma_free_coherent(dev, buf->size, buf->vaddr, buf->dma_handle);

    buf->vaddr = NULL;
}


static void vb2_cmalloc_put(void *buf_priv)
{
	struct vb2_cmalloc_buf *buf = buf_priv;

	/* 4.9->6.1 port: atomic_dec_and_test -> refcount_dec_and_test,
	 * matching the atomic_t -> refcount_t field type change above. */
	if (refcount_dec_and_test(&buf->refcount)) {
		cma_free(buf_priv);
		kfree(buf);
	}
}

/* 4.9->6.1 port: struct vb2_mem_ops.alloc dropped its `unsigned long attrs`
 * and `enum dma_data_direction dma_dir` parameters and gained a leading
 * `struct vb2_buffer *vb`; the queue-wide attrs/dma_dir/gfp_flags that used
 * to be passed in directly now live on vb->vb2_queue and are read from
 * there instead (see struct vb2_queue in videobuf2-core.h). Same shape of
 * information, just reached through the queue instead of as call args. */
static void *vb2_cmalloc_alloc(struct vb2_buffer *vb, struct device *dev,
			       unsigned long size)
{
	struct vb2_cmalloc_buf *buf;
	gfp_t gfp_flags = vb->vb2_queue->gfp_flags;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL | gfp_flags);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->size = PAGE_ALIGN(size);
	buf->vaddr = cma_alloc(dev, buf->size, &buf->dma_handle);
	buf->dma_dir = vb->vb2_queue->dma_dir;
	buf->handler.refcount = &buf->refcount;
	buf->handler.put = vb2_cmalloc_put;
	buf->handler.arg = buf;
	buf->dev = dev;
	buf->dbuf = (void *)dev;

	if (!buf->vaddr) {
		pr_err("cmalloc of size %ld failed\n", buf->size);
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}

	/* 4.9->6.1 port: this establishes the buffer's initial refcount
	 * (kzalloc leaves it 0), so it must be refcount_set(1), not
	 * refcount_inc() -- refcount_inc() on a still-zero refcount_t is
	 * treated as a use-after-free and refuses to increment (by
	 * design, that's the whole point of refcount_t over atomic_t). */
	refcount_set(&buf->refcount, 1);
	return buf;
}

/* 4.9->6.1 port: get_userptr gained a leading struct vb2_buffer *vb and
 * lost its dma_dir parameter (read from vb->vb2_queue->dma_dir instead),
 * same story as alloc() above. */
static void *vb2_cmalloc_get_userptr(struct vb2_buffer *vb, struct device *dev,
				     unsigned long vaddr, unsigned long size)
{
	struct vb2_cmalloc_buf *buf;
	struct frame_vector *vec;
	int n_pages, offset, i;
	int ret = -ENOMEM;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->dma_dir = vb->vb2_queue->dma_dir;
	offset = vaddr & ~PAGE_MASK;
	buf->size = size;
	/* Linux 6.18 restored the explicit FOLL_WRITE selection.  The device
	 * writes capture data into USERPTR pages for DMA_FROM_DEVICE (and
	 * bidirectional) queues, matching the in-tree vb2 allocators. */
	vec = vb2_create_framevec(vaddr, size,
				 buf->dma_dir == DMA_FROM_DEVICE ||
				 buf->dma_dir == DMA_BIDIRECTIONAL);
	if (IS_ERR(vec)) {
		ret = PTR_ERR(vec);
		goto fail_pfnvec_create;
	}
	buf->vec = vec;
	n_pages = frame_vector_count(vec);
	if (frame_vector_to_pages(vec) < 0) {
		unsigned long *nums = frame_vector_pfns(vec);

		/*
		 * We cannot get page pointers for these pfns. Check memory is
		 * physically contiguous and use direct mapping.
		 */
		for (i = 1; i < n_pages; i++)
			if (nums[i-1] + 1 != nums[i])
				goto fail_map;
		buf->vaddr = (__force void *)
				ioremap(nums[0] << PAGE_SHIFT, size);
	} else {
		/* 4.9->6.1 port: vm_map_ram() dropped its trailing pgprot_t
		 * arg (always maps PAGE_KERNEL now). */
		buf->vaddr = vm_map_ram(frame_vector_pages(vec), n_pages, -1);
	}

	if (!buf->vaddr)
		goto fail_map;
	buf->vaddr += offset;
	return buf;

fail_map:
	vb2_destroy_framevec(vec);
fail_pfnvec_create:
	kfree(buf);

	return ERR_PTR(ret);
}

static void vb2_cmalloc_put_userptr(void *buf_priv)
{
	struct vb2_cmalloc_buf *buf = buf_priv;
	unsigned long vaddr = (unsigned long)buf->vaddr & PAGE_MASK;
	unsigned int i;
	struct page **pages;
	unsigned int n_pages;

	if (!buf->vec->is_pfns) {
		n_pages = frame_vector_count(buf->vec);
		pages = frame_vector_pages(buf->vec);
		if (vaddr)
			vm_unmap_ram((void *)vaddr, n_pages);
		if (buf->dma_dir == DMA_FROM_DEVICE)
			for (i = 0; i < n_pages; i++)
				set_page_dirty_lock(pages[i]);
	} else {
		iounmap((__force void __iomem *)buf->vaddr);
	}
	vb2_destroy_framevec(buf->vec);
	kfree(buf);
}

/* 4.9->6.1 port: vaddr() gained a leading struct vb2_buffer *vb (unused
 * here, we only need the buf_priv). */
static void *vb2_cmalloc_vaddr(struct vb2_buffer *vb, void *buf_priv)
{
	struct vb2_cmalloc_buf *buf = buf_priv;

	if (!buf->vaddr) {
		pr_err("Address of an unallocated plane requested "
		       "or cannot map user pointer\n");
		return NULL;
	}

	return buf->vaddr;
}

static unsigned int vb2_cmalloc_num_users(void *buf_priv)
{
	struct vb2_cmalloc_buf *buf = buf_priv;
	return refcount_read(&buf->refcount);
}

/*
 * Added during the 6.1 port hardening pass, 2026-08-05 -- this allocator had
 * no .cookie op, so the only way callers had to get a DMA-capable address
 * out of a buffer was virt_to_phys(vb2_plane_vaddr(...)) (see isp-vb2.c).
 * That is only valid if the vaddr dma_alloc_coherent() handed back sits in
 * the kernel's linear map, which is NOT guaranteed for a device that isn't
 * marked `dma-coherent` in its devicetree node -- and isp@ff140000 in
 * camera-overlay.dts is not. On that path arm64 can return a non-cached
 * remapped vaddr outside the linear map, and virt_to_phys() on it silently
 * produces a wrong physical address rather than failing loudly. Confirmed on
 * hardware 2026-08-05: streaming corrupted kernel memory (systemd aborted,
 * ext4 freed-inode bitmap mismatch) within ~260ms of the very first frame,
 * before any error-recovery code ran -- consistent with the DMA writer being
 * armed with a garbage target address from frame 0, not a race or a size bug
 * (both already ruled out separately).
 *
 * dma_alloc_coherent() already computed the one address that is guaranteed
 * correct regardless of coherency/remapping -- buf->dma_handle -- and just
 * never surfaced it. Wiring up .cookie is the standard vb2 mechanism for
 * exposing exactly this (see videobuf2-dma-contig's vb2_dc_cookie() for the
 * pattern this mirrors); isp-vb2.c now uses vb2_plane_cookie() instead of
 * virt_to_phys().
 */
static void *vb2_cmalloc_cookie(struct vb2_buffer *vb, void *buf_priv)
{
	struct vb2_cmalloc_buf *buf = buf_priv;
	return &buf->dma_handle;
}

static int vb2_cmalloc_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct vb2_cmalloc_buf *buf = buf_priv;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	int ret = -1;

	if (!buf || !vma) {
		pr_err("No memory to map\n");
		return -EINVAL;
	}

	/*
	 * dma_mmap_coherent(), not remap_pfn_range(virt_to_phys(vaddr)).
	 *
	 * This is the same bug that was fixed on the DMA side in isp-vb2.c, in
	 * the other direction. buf->vaddr comes from dma_alloc_coherent(), and
	 * for a device the kernel treats as non-coherent that vaddr is a fresh
	 * non-cacheable mapping created by the DMA layer, not a linear-map
	 * address. virt_to_phys() on it does not fault, it quietly returns a
	 * number computed as if it were linear-map -- so remap_pfn_range()
	 * mapped an unrelated run of physical pages into userspace.
	 *
	 * Symptom, and how this was found: capture "worked" -- 60 buffers
	 * dequeued at a sustained 29.99 fps, no errors -- but every buffer
	 * userspace read back was uninitialised kernel memory. `strings` on the
	 * frames returned kernel symbol names (rtnl_link_vf_put), Mesa GLSL
	 * builtins (floatBitsToInt), page-cache HTML and a wifi firmware version
	 * banner. Meanwhile /dev/mem showed the FR DMA writer correctly armed and
	 * cycling through real per-frame bank0_base addresses (0xc1000000,
	 * 0xbfe00000, 0xc0700000) with format 13 and line offset 0xf80. The ISP
	 * was writing the frames exactly as asked; userspace was reading a
	 * different piece of memory entirely.
	 *
	 * dma_mmap_coherent() maps the pages the allocation actually owns, with
	 * the attributes the allocation was made with, and is the only correct
	 * way to hand a coherent allocation to userspace.
	 */
	ret = dma_mmap_coherent(buf->dev, vma, buf->vaddr, buf->dma_handle, vsize);

	if (ret) {
		pr_err("dma_mmap_coherent failed, error: %d\n", ret);
		return ret;
	}
		/*
		* Make sure that vm_areas for 2 buffers won't be merged together
		*/
	vm_flags_set(vma, VM_DONTEXPAND);

		/*
		* Use common vm_area operations to track buffer refcount.
		*/
	vma->vm_private_data = &buf->handler;
	vma->vm_ops = &vb2_common_vm_ops;

	vma->vm_ops->open(vma);

	return 0;
}

struct vb2_cmalloc_attachment {
	struct sg_table sgt;
	enum dma_data_direction dma_dir;
};

/* 4.9->6.1 port: dma_buf_ops.attach dropped its `struct device *dev`
 * parameter (dbuf_attach->dev carries the same info, and this function
 * body never actually used the `dev` argument it used to be handed). */
static int vb2_cmalloc_dmabuf_ops_attach(struct dma_buf *dbuf,
		struct dma_buf_attachment *dbuf_attach)
{
	struct vb2_cmalloc_attachment *attach;
	struct vb2_cmalloc_buf *buf = dbuf->priv;
	int num_pages = PAGE_ALIGN(buf->size) / PAGE_SIZE;
	struct sg_table *sgt;
	struct scatterlist *sg;
	void *vaddr = buf->vaddr;
	int ret;
	int i;

	attach = kzalloc(sizeof(*attach), GFP_KERNEL);
	if (!attach)
		return -ENOMEM;

	sgt = &attach->sgt;
	ret = sg_alloc_table(sgt, num_pages, GFP_KERNEL);
	if (ret) {
		kfree(attach);
		return ret;
	}
	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		struct page *page = virt_to_page(vaddr);

		if (!page) {
			sg_free_table(sgt);
			kfree(attach);
			return -ENOMEM;
		}
		sg_set_page(sg, page, PAGE_SIZE, 0);
		vaddr += PAGE_SIZE;
	}

	attach->dma_dir = DMA_NONE;
	dbuf_attach->priv = attach;
	return 0;
}

static void vb2_cmalloc_dmabuf_ops_detach(struct dma_buf *dbuf,
	struct dma_buf_attachment *db_attach)
{
	struct vb2_cmalloc_attachment *attach = db_attach->priv;
	struct sg_table *sgt;

	if (!attach)
		return;

	sgt = &attach->sgt;

	/* release the scatterlist cache */
	if (attach->dma_dir != DMA_NONE)
		dma_unmap_sg(db_attach->dev, sgt->sgl, sgt->orig_nents,
			attach->dma_dir);
	sg_free_table(sgt);
	kfree(attach);
	db_attach->priv = NULL;
}


static struct sg_table *vb2_cmalloc_dmabuf_ops_map(
	struct dma_buf_attachment *db_attach, enum dma_data_direction dma_dir)
{
	struct vb2_cmalloc_attachment *attach = db_attach->priv;
	struct sg_table *sgt;

	/* map_dma_buf() is invoked with the dma-buf reservation lock held on
	 * Linux 6.18, so the attachment cache is already serialized. */

	sgt = &attach->sgt;
	/* return previously mapped sg table */
	if (attach->dma_dir == dma_dir) {
		return sgt;
	}

	/* release any previous cache */
	if (attach->dma_dir != DMA_NONE) {
		dma_unmap_sg(db_attach->dev, sgt->sgl, sgt->orig_nents,
			attach->dma_dir);
		attach->dma_dir = DMA_NONE;
	}

	/* mapping to the client with new direction */
	sgt->nents = dma_map_sg(db_attach->dev, sgt->sgl, sgt->orig_nents,
				dma_dir);
	if (!sgt->nents) {
		pr_err("failed to map scatterlist\n");
		return ERR_PTR(-EIO);
	}

	attach->dma_dir = dma_dir;

	return sgt;
}

static void vb2_cmalloc_dmabuf_ops_unmap(struct dma_buf_attachment *db_attach,
	struct sg_table *sgt, enum dma_data_direction dma_dir)
{
	/* nothing to be done here */
}


static void vb2_cmalloc_dmabuf_ops_release(struct dma_buf *dbuf)
{
	vb2_cmalloc_put(dbuf->priv);
}

/* 4.9->6.1 port: dma_buf_ops.kmap/.kmap_atomic were removed outright (no
 * replacement -- dma-buf callers are expected to use .vmap instead, there
 * is no more single-page kmap concept in the dma-buf API). .vmap's
 * signature also changed from "returns void *" to "fills in a struct
 * iosys_map, returns int status", with a new paired .vunmap callback
 * (mandatory alongside .vmap per the kerneldoc). Dropped kmap/kmap_atomic
 * entirely and ported vmap/added vunmap; nothing in this driver's other
 * files calls dma_buf_vmap()/dma_buf_kmap() on our own exported buffers
 * (checked -- only mmap/get_dmabuf/vaddr are exercised via v4l2_interface
 * and fw_lib), so this is a straight API-shape port, not a behavior
 * change for this driver's own use. */
static int vb2_cmalloc_dmabuf_ops_vmap(struct dma_buf *dbuf, struct iosys_map *map)
{
	struct vb2_cmalloc_buf *buf = dbuf->priv;

	iosys_map_set_vaddr(map, buf->vaddr);
	return 0;
}

static void vb2_cmalloc_dmabuf_ops_vunmap(struct dma_buf *dbuf, struct iosys_map *map)
{
	/* nothing to be done here -- buf->vaddr is owned by cma_alloc()/
	 * cma_free(), not by this vmap/vunmap pair */
}

static int vb2_cmalloc_dmabuf_ops_mmap(struct dma_buf *dbuf,
	struct vm_area_struct *vma)
{
	return vb2_cmalloc_mmap(dbuf->priv, vma);
}

static struct dma_buf_ops vb2_cmalloc_dmabuf_ops = {
	.attach = vb2_cmalloc_dmabuf_ops_attach,
	.detach = vb2_cmalloc_dmabuf_ops_detach,
	.map_dma_buf = vb2_cmalloc_dmabuf_ops_map,
	.unmap_dma_buf = vb2_cmalloc_dmabuf_ops_unmap,
	.vmap = vb2_cmalloc_dmabuf_ops_vmap,
	.vunmap = vb2_cmalloc_dmabuf_ops_vunmap,
	.mmap = vb2_cmalloc_dmabuf_ops_mmap,
	.release = vb2_cmalloc_dmabuf_ops_release,
};

/* 4.9->6.1 port: get_dmabuf gained a leading struct vb2_buffer *vb
 * (unused here, same as vaddr() above). */
static struct dma_buf *vb2_cmalloc_get_dmabuf(struct vb2_buffer *vb, void *buf_priv, unsigned long flags)
{
	struct vb2_cmalloc_buf *buf = buf_priv;
	struct dma_buf *dbuf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.ops = &vb2_cmalloc_dmabuf_ops;
	exp_info.size = buf->size;
	exp_info.flags = flags;
	exp_info.priv = buf;

	if (WARN_ON(!buf->vaddr))
		return NULL;

	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf))
		return NULL;

	/* 4.9->6.1 port: atomic_inc -> refcount_inc. Unlike the initial
	 * refcount_set(1) in alloc(), this one is fine as a plain increment
	 * -- the buffer's refcount is already non-zero (established at
	 * alloc time) by the time get_dmabuf() can be called on it. */
	refcount_inc(&buf->refcount);

	return dbuf;
}

const struct vb2_mem_ops vb2_cmalloc_memops = {
	.alloc		= vb2_cmalloc_alloc,
	.put		= vb2_cmalloc_put,
	.get_userptr	= vb2_cmalloc_get_userptr,
	.put_userptr	= vb2_cmalloc_put_userptr,
#ifdef CONFIG_HAS_DMA
	.get_dmabuf	= vb2_cmalloc_get_dmabuf,
#endif
	.map_dmabuf	= NULL,
	.unmap_dmabuf	= NULL,
	.attach_dmabuf	= NULL,
	.detach_dmabuf	= NULL,
	.vaddr		= vb2_cmalloc_vaddr,
	.cookie		= vb2_cmalloc_cookie,
	.mmap		= vb2_cmalloc_mmap,
	.num_users	= vb2_cmalloc_num_users,
};
EXPORT_SYMBOL_GPL(vb2_cmalloc_memops);

MODULE_DESCRIPTION("cmalloc memory handling routines for videobuf2");
MODULE_AUTHOR("Keke Li<keke.li@amlogic.com>");
MODULE_LICENSE("GPL");
/* 4.9->6.1 port: dma-buf exports were namespaced (MODULE_IMPORT_NS)
 * somewhere in this span; modpost now errors "uses symbol dma_buf_export
 * from namespace DMA_BUF, but does not import it" without this. Confirmed
 * via a real `make modules` run against the target headers, not a guess. */
MODULE_IMPORT_NS("DMA_BUF");
