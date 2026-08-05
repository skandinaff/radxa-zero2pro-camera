PORT_NOTES.md
=============

Forward-port of the Amlogic/ARM "ACamera" ISP driver (`iv009_isp`, from
Khadas's `khadas-vims-4.9.y` kernel fork, `drivers/amlogic/isp_module`) from
Linux 4.9 to our target: Radxa Zero 2 Pro (Amlogic A311D), Radxa-built
6.1.68-3-stable kernel. Sensor is Sony IMX415 via the mainline
`drivers/media/i2c/imx415.c` driver (vendored from kernel tag v6.3),
bridged in through the vendor tree's existing `V4L2_drv.c` generic
`v4l2_subdev` wrapper -- no raw sensor driver needed.

Bottom line up front
---------------------
- **Both modules build clean with a real `make modules` run** against the
  actual target headers at `/usr/src/linux-headers-6.1.68-3-stable` on
  `colin@192.168.8.157`, in a scratch dir under `~/camera-port/scratch/`
  on that host (not just static signature-checking). Verified with a full
  `make clean && make` cycle for both, twice, from a clean checkout of
  this `build/` tree. Zero errors. A handful of pre-existing warnings
  (see "Warnings left as-is" below) -- all expected/acceptable per the
  task brief.
- `iv009_isp.ko` (isp_module) -- 653600 bytes, links clean, all symbols
  resolved at modpost time.
- `imx415.ko` -- 24984 bytes, **zero source changes needed**. Compiles
  clean verbatim as vendored from v6.3. The prediction that a 2-minor-tag
  gap would need little to no porting was correct.
- Nothing was insmod'd, nothing touched outside `~/camera-port` on the
  build host, no `dtc`/`/boot`/reboot -- all read+build only, per the hard
  constraint.
- `subdev/iq` is **not** a hard link-time dependency of the module that
  matters (see below) -- it was left untouched/unbuilt.

Module layout
-------------
Preserved the vendor tree's own module boundary rather than inventing a
new one. `isp_module/Makefile` was `obj-y += subdev/ v4l2_dev/`, and
`v4l2_dev/Makefile` builds exactly one module, `iv009_isp.o`, from
`app/`, `src/platform/`, `src/fw_lib/`, `src/calibration/`, and
`src/driver/{sensor,lens}/`. `subdev/{iq,lens,sensor}/Makefile` each build
*separate*, *independent* modules (`iv009_isp_iq.ko`, `iv009_isp_lens.ko`,
`iv009_isp_sensor.ko`) that are legacy raw-register sensor drivers
(IMX227/290/307/481/OV08A10) plus their calibration data -- **not**
referenced anywhere by `v4l2_dev`'s source (grepped for `subdev/iq` and
`soc_iq` across `v4l2_dev/`; only hits are `v4l2_dev`'s own
`app/soc_iq.h` and `src/calibration/soc_iq_calibrations.c`, which are
distinct files under `v4l2_dev/` itself, not `subdev/iq/`). Since we're
using the generic `V4L2_drv.c` bridge + mainline `imx415.c` instead of a
raw vendor sensor driver, `subdev/{iq,lens,sensor}` are simply irrelevant
to this build and were not touched, ported, or built. This confirms the
task brief's expectation.

So the deliverable is two modules, matching two source dirs here:
- `build/isp_module/` -> `iv009_isp.ko` (ported from vendor `v4l2_dev/`,
  renamed dir to make the module-vs-subdev split obvious since `subdev/`
  isn't present in this tree at all)
- `build/imx415/` -> `imx415.ko` (vendored verbatim from mainline v6.3)

File-by-file changelog
-----------------------
Every file touched, with a one-line-ish reason. Everything not listed
here (including all of `src/fw_lib/` apart from the four files below, and
all of `app/control/`) compiled with **zero changes** -- the "fw_lib is
mostly portable C, don't over-invest" prediction from the task brief held
up; the standard cast-between-function-pointer-types warnings there are
pre-existing vendor code style (see "Warnings left as-is"), not something
introduced or fixed by this port.

### `isp_module/Makefile` (new)
Rewritten as a standalone out-of-tree Kbuild file (`obj-m += iv009_isp.o`,
`KDIR ?= /usr/src/linux-headers-$(shell uname -r)`, `make -C $(KDIR) M=$(PWD)
modules`), following the `dtbo_loader/Makefile` pattern given in the task
brief. Source-file list and include paths are otherwise a straight copy
of the vendor `v4l2_dev/Makefile`'s `FW_SRC_OBJ`/`ccflags-y`, with one
real bug fix: **the include paths originally used `$(PWD)` via a
shell-evaluated `:=`, which silently resolves to `$(KDIR)`, not this
module's own directory** -- because the Makefile is `include`d by Kbuild
while Kbuild's *own* cwd is `$(KDIR)`, and `$(shell pwd)` is evaluated at
that time. Manifested as `acamera_firmware_config.h: No such file or
directory` even though the file is right there in `inc/`. Fixed by using
kbuild's own `$(src)` variable instead of a hand-rolled `$(PWD)`. Also
added `ccflags-y += -isystem $(shell $(CC) -print-file-name=include)`:
this specific headers package's top-level Makefile builds
`NOSTDINC_FLAGS` as plain `-nostdinc` with no paired `-isystem <gcc's own
freestanding-headers dir>` (stock kernel Makefiles normally add both
together specifically so things like `<stdarg.h>`/`<stddef.h>` stay
reachable under `-nostdinc`) -- without it, any file that does
`#include <stdarg.h>` (the ISP logger, `system_log.h`, uses `va_list`)
fails to find it. Couldn't patch the shared headers tree (out of scope,
read-only), so added the flag locally to this module's own `ccflags-y`
instead.

### `app/main_kernel_juno_v4l2.c`
- `write_to_file()`: `mm_segment_t`/`get_fs()`/`set_fs(KERNEL_DS)` were
  removed entirely (the whole address-limit-override mechanism is gone on
  modern kernels) -- replaced `vfs_write()` on a kernel buffer with
  `kernel_write()`, the direct modern equivalent. Also fixed
  `filp_open()`'s error check from `if (!fp)` to `IS_ERR(fp)` while in
  there -- `filp_open()` has always returned `ERR_PTR()` on failure, never
  `NULL`; this was a **pre-existing bug** in the vendor code (not
  introduced by the port), just one I noticed and fixed since I was
  already rewriting the surrounding lines.
- `ioremap_nocache()` (4 call sites) -> `ioremap()`. `ioremap_nocache` was
  removed; `ioremap()` alone has been the uncached mapping on every arch
  that matters for years.
- The `struct acamera_v4l2_subdev_t.soc_async_sd[]`/`soc_async_sd_ptr[]`
  arrays and the whole async-subdev registration block in
  `isp_platform_probe()`/`isp_platform_remove()`: **architectural change,
  forced by the compiler, not a style choice.**
  `V4L2_ASYNC_MATCH_CUSTOM` (an arbitrary-match-function type the vendor
  used with `match.custom.match = NULL`, meaning "bind whatever async
  subdev shows up, no filtering") was removed from the v4l2_async core
  well before 6.1 -- mainline only supports `V4L2_ASYNC_MATCH_I2C` and
  `V4L2_ASYNC_MATCH_FWNODE` now. There is no "match anything" escape hatch
  left in the framework by design. The whole notifier registration API
  also changed shape: `struct v4l2_async_notifier` no longer has
  `.bound`/`.complete`/`.unbind`/`.subdevs`/`.num_subdevs` fields directly
  -- callbacks moved to a separate `const struct
  v4l2_async_notifier_operations *notifier->ops`, and subdevs are added
  one at a time via `v4l2_async_nf_add_fwnode()`/`_add_i2c()` etc. onto an
  internal list (after `v4l2_async_nf_init()`), not a fixed array.
  `v4l2_async_notifier_register()`/`_unregister()` were renamed to
  `v4l2_async_nf_register()`/`_unregister()` (plus a new
  `v4l2_async_nf_cleanup()` that must be called after unregister).
  **Replaced with the standard modern pattern** (same idea as e.g.
  rkisp1's notifier setup): walk this device's own OF-graph endpoints via
  `fwnode_graph_get_next_endpoint()`/`fwnode_graph_get_remote_port_parent()`
  and fwnode-match whatever's on the other end of each one with
  `v4l2_async_nf_add_fwnode()`. **This depends on an OF-graph
  port/endpoint under the `isp@ff140000` DT node linking to the imx415's
  endpoint, which does not exist yet** -- devicetree/overlay authoring is
  explicitly out of scope for this pass per the task brief (same kind of
  accepted gap as "no IMX415 calibration data exists upstream"). Until
  that endpoint is added, this loop finds zero endpoints, nothing gets
  added to the notifier, `bound()`/`complete()` never fire, and
  `isp_v4l2_create_instance()` is never called -- i.e. **the driver loads
  and sits idle rather than binding to anything**, which is a safe (if
  inert) failure mode, not a crash. **Flagging this as the #1 thing the
  next session needs for the driver to actually do anything**: add a
  `port`/`endpoint` (with `remote-endpoint`) under the ISP's DT node
  pointing at the imx415 i2c client's endpoint.
- Added `#include <linux/property.h>` for the `fwnode_graph_*`/`dev_fwnode()`
  calls above, and `#include <linux/version.h>` was *not* needed here
  (added instead in `isp-v4l2-stream.h`, see below).

### `app/main_firmware.c`
`PTR_RET()` -> `PTR_ERR_OR_ZERO()`. Plain rename (identical semantics: 0
if not an `ERR_PTR`, else the encoded error), `PTR_RET` was dropped after
the rename fully landed upstream.

### `app/v4l2_interface/isp-v4l2-ctrl.c`
`v4l2_ctrl_add_handler()` gained a trailing `bool from_other_dev` param.
Passed `false` -- both handlers being merged (`hdl_std_ctrl`,
`hdl_cst_ctrl`) belong to this driver's own `isp_v4l2_ctrl_t`, not a
handler borrowed from a different `v4l2_device`.

### `app/v4l2_interface/isp-v4l2-stream.h`
Added `#include <linux/version.h>` -- `LINUX_VERSION_CODE`/`KERNEL_VERSION`
were apparently pulled in transitively via some other header on 4.9; on
6.1 the `#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,4,0))` guards in this
header failed with "not defined, evaluates to 0" followed by a hard
`missing binary operator` error. Left the version-gated branches
themselves alone (target is always >= 4.4.0 so they're dead code either
way, but pruning them wasn't necessary to get a clean build and keeps the
diff smaller).

### `app/v4l2_interface/isp-v4l2-stream.c`
- Dropped the dead `#include <linux/dma-contiguous.h>` (removed header;
  nothing in this file actually calls `dma_alloc_from_contiguous()` or
  friends -- that's in `isp-vb2-cmalloc.c`/`isp-v4l2.c`, see below).
- `kzfree()` -> `kfree_sensitive()`. Plain rename once the
  transition/shim period ended.
- `isp_v4l2_stream_fill_buf()`: `struct timeval`/`do_gettimeofday()` were
  removed outright as part of the y2038 cleanup (32-bit `tv_sec`
  overflows in 2038; not deprecated, deleted). Only ever used here for a
  millisecond duration measurement around a memcpy, so replaced with
  `ktime_t`/`ktime_get()`/`ktime_to_ms(ktime_sub(end, begin))` -- simpler
  than reconstructing calendar-time semantics that weren't needed anyway.
  Added `#include <linux/ktime.h>`.

### `app/v4l2_interface/isp-v4l2.c`
- Same `<linux/dma-contiguous.h>` -> `<linux/dma-map-ops.h>` header move
  as `isp-vb2-cmalloc.c`, and same `kzfree()` -> `kfree_sensitive()` (two
  call sites: `isp_v4l2_fh_open()`, `isp_v4l2_fh_release()`).
- `isp_v4l2_qbuf()`: `vb2_qbuf()` gained a `struct media_device *mdev`
  parameter (used to sync Media Controller request-API pipeline links).
  Passed `NULL` -- grepped the whole tree, this driver never registers a
  `media_device` anywhere (no `.mdev` on `v4l2_dev`, no
  `media_device_init()` call), so `NULL` is correct, not a placeholder
  guess.
- `.vidioc_enum_fmt_vid_cap_mplane` doesn't exist as an ioctl_ops field --
  there's no mplane-specific variant for `VIDIOC_ENUM_FMT` (unlike
  g/s/try_fmt, format enumeration doesn't distinguish single- vs
  multi-planar; the field is identified by `fmtdesc->type`). Renamed the
  registration to `.vidioc_enum_fmt_vid_cap` (checked `v4l2-ioctl.h`,
  confirmed that's the only field that exists; `isp_v4l2_enum_fmt_vid_cap`'s
  existing signature already matched it exactly, no function-side change
  needed).
- `.vidioc_cropcap`/`.vidioc_g_crop`/`.vidioc_s_crop` were removed from
  `struct v4l2_ioctl_ops` entirely. The legacy `VIDIOC_CROPCAP`/
  `VIDIOC_G_CROP`/`VIDIOC_S_CROP` *ioctls* are still valid UAPI (v4l2-ioctl.c
  implements them generically now, translated into calls against
  `.vidioc_g_selection`/`.vidioc_s_selection` plus `.vidioc_g_pixelaspect`
  for the pixel-aspect part), so userspace compatibility is unaffected --
  only this driver's registration point moved. Replaced the three
  functions with `isp_v4l2_g_selection()`/`isp_v4l2_s_selection()`
  (targets: `V4L2_SEL_TGT_CROP` -> current crop rect,
  `V4L2_SEL_TGT_CROP_DEFAULT` -> `cropcap.defrect`,
  `V4L2_SEL_TGT_CROP_BOUNDS` -> `cropcap.bounds`) and
  `isp_v4l2_g_pixelaspect()`, all implemented as thin wrappers around the
  **unchanged** `isp_v4l2_get_cropcap()`/`isp_v4l2_get_crop()`/
  `isp_v4l2_set_crop()` in `isp-v4l2-stream.c` (those still take
  `struct v4l2_cropcap *`/`struct v4l2_crop *` -- still valid UAPI types,
  needed zero changes themselves).
- `VFL_TYPE_GRABBER` -> `VFL_TYPE_VIDEO`. Plain rename, same enum
  position/value (checked `v4l2-dev.h`).
- **`isp_cma_alloc()`/`isp_cma_free()`: revised after real modpost
  testing**, see the `isp-vb2-cmalloc.c` entry below for the full
  reasoning -- same fix applied here (`dma_alloc_from_contiguous()`/
  `dma_release_from_contiguous()` -> `dma_alloc_coherent()`/
  `dma_free_coherent()`), same NOT-VERIFIED linear-map caveat applies.

### `app/v4l2_interface/isp-vb2-cmalloc.c` + `.h`
The highest-risk file in the tree, and where the most real work went.
- `#include <linux/dma-contiguous.h>` -> initially redirected to
  `<linux/dma-map-ops.h>` (where `dma_alloc_from_contiguous()`/
  `dma_release_from_contiguous()` now live) with a `bool no_warn` param
  added to the alloc call (compiles fine this way) -- **but a real `make
  modules` run caught that neither symbol is actually usable from an
  out-of-tree module**: modpost failed with
  `"dma_release_from_contiguous" [...] undefined!` and
  `"dma_alloc_from_contiguous" [...] undefined!` (neither has
  `EXPORT_SYMBOL`/`EXPORT_SYMBOL_GPL` in this kernel's `Module.symvers` --
  that header is meant for DMA-ops *implementers* built into vmlinux, not
  driver consumers). Checked whether the newer public wrapper
  (`dma_alloc_contiguous()`/`dma_free_contiguous()`) is exported instead
  -- also absent from `Module.symvers`. **The only CMA-backed allocation
  path actually reachable from an external module on this kernel is
  `dma_alloc_coherent()`/`dma_free_coherent()`** (always exported; its
  arm64 `dma_direct_alloc` implementation internally uses the device's
  declared CMA/reserved-memory region the same way
  `dma_alloc_from_contiguous()` would have, when there's no IOMMU in the
  way -- true for this hardware). Rewrote `cma_alloc()`/`cma_free()` to
  use it, which required adding a `dma_addr_t dma_handle` field to
  `struct vb2_cmalloc_buf` (needed to call `dma_free_coherent()` later;
  the old code derived the physical page via `virt_to_page(buf->vaddr)`
  instead, which doesn't apply to `dma_alloc_coherent()`'s allocation).
  **NOT VERIFIED, flagged explicitly**: the rest of this driver (see
  `isp-vb2.c`'s `virt_to_phys(vb2_plane_vaddr(...))`) assumes the vaddr
  handed back here is part of the kernel's linear map, so `virt_to_phys()`
  on it is meaningful. That holds for `dma_alloc_coherent()` on arm64
  *when the device is cache-coherent for DMA* (typical for an on-SoC ISP
  with no IOMMU, which matches this hardware's description in the task
  brief), but would **not** hold if the device needs non-coherent
  (write-combine, non-cache, vmap'd-outside-the-linear-map) allocations
  instead -- that path would make `virt_to_phys()` silently wrong. Whether
  this device's eventual DT node carries (or needs) a `dma-coherent`
  property is a devicetree-authoring question outside this pass's scope.
  **This is the #1 thing to verify before trusting frame addresses out of
  either allocator (this file and `isp-v4l2.c`'s `isp_cma_alloc()`) on
  real hardware.**
- `struct vb2_mem_ops` callback signatures all changed shape (checked
  `media/videobuf2-core.h` for every one rather than guessing):
  - `.alloc`: `(struct device *, unsigned long attrs, unsigned long size,
    enum dma_data_direction, gfp_t)` -> `(struct vb2_buffer *vb, struct
    device *, unsigned long size)`. The dropped `attrs`/`dma_dir`/
    `gfp_flags` now live on `vb->vb2_queue->{dma_attrs,dma_dir,gfp_flags}`
    instead of being passed as call args -- read from there.
  - `.get_userptr`: dropped its `dma_dir` param for the same reason (read
    `vb->vb2_queue->dma_dir` instead), gained the leading `vb2_buffer *`.
  - `.vaddr`, `.get_dmabuf`: both gained a leading (unused-by-us)
    `struct vb2_buffer *vb` param.
  - `dma_buf_ops.attach`: dropped its `struct device *dev` param
    (`dbuf_attach->dev` carries the same info; the vendor function never
    actually used the `dev` arg it was handed, so this was a pure
    signature trim, no logic change).
  - `dma_buf_ops.kmap`/`.kmap_atomic`: **removed outright**, no
    replacement (dma-buf callers are expected to use `.vmap` instead).
    Dropped both.
  - `dma_buf_ops.vmap`: signature changed from "returns `void *`" to
    "fills in a `struct iosys_map *map`, returns `int` status"; gained a
    mandatory paired `.vunmap`. Ported using `iosys_map_set_vaddr()`;
    added a no-op `.vunmap` (buf->vaddr's lifetime is owned by
    `cma_alloc()`/`cma_free()`, not by vmap/vunmap). Added `#include
    <linux/iosys-map.h>`. Confirmed nothing else in this driver's own
    code calls `dma_buf_vmap()`/`dma_buf_kmap()` on buffers it exports
    (only `.mmap`/`.get_dmabuf`/`.vaddr` are exercised via
    `v4l2_interface`/`fw_lib`), so this is a pure API-shape port, not a
    behavior change for this driver's own use.
- `struct vb2_vmarea_handler.refcount` (in `media/videobuf2-memops.h`)
  changed type from `atomic_t *` to `refcount_t *` (the "checked"
  refcounter type that replaced raw atomic_t-as-a-refcount patterns
  kernel-wide). Changed `vb2_cmalloc_buf.refcount` to match (it's handed
  to `handler.refcount` by pointer). This needed care, not a blind
  find/replace: `atomic_inc()` in `vb2_cmalloc_alloc()` (establishing the
  buffer's *initial* refcount on a just-`kzalloc()`'d, i.e. zeroed,
  struct) became `refcount_set(&buf->refcount, 1)`, **not**
  `refcount_inc()` -- `refcount_inc()` on an already-zero `refcount_t` is
  treated as a use-after-free by design and refuses to increment (would
  have silently left every buffer's refcount at 0). The *other*
  `atomic_inc()`, in `vb2_cmalloc_get_dmabuf()` (bumping an
  already-nonzero refcount for the dmabuf export reference), correctly
  became a plain `refcount_inc()`. `atomic_dec_and_test()` ->
  `refcount_dec_and_test()`, `atomic_read()` -> `refcount_read()`.
- `dma_alloc_from_contiguous()` gained a trailing `bool no_warn` param
  (moot now, see above -- function's gone from this file, noting for
  completeness of the diff history).
- `vb2_create_framevec()` **dropped its 3rd `write` argument entirely**
  (used to select `FOLL_WRITE` when pinning user pages for a
  `DMA_FROM_DEVICE` userptr buffer). Checked `media/videobuf2-memops.h` --
  no replacement parameter exists, it's just `(start, length)` now.
  **NOT VERIFIED**: what write-access policy the new implementation
  defaults to internally. Low risk in practice since this driver's own
  MMAP-based buffer flow doesn't exercise the USERPTR path, but worth a
  second look if userptr-type buffers are ever used against this driver.
- `vm_map_ram()` dropped its trailing `pgprot_t` arg (always maps
  `PAGE_KERNEL` now).
- `ioremap_nocache()` -> `ioremap()` (one call site, in
  `vb2_cmalloc_get_userptr()`'s non-contiguous-pfns fallback path).
- Added `MODULE_IMPORT_NS(DMA_BUF);`. Confirmed by a real modpost error,
  not a guess: `"uses symbol dma_buf_export from namespace DMA_BUF, but
  does not import it"`. dma-buf exports were namespaced at some point in
  this kernel's history; without the import, modpost hard-fails.

### `src/fw/acamera_math.h`
`#define array_size(a) (sizeof(a)/sizeof(a[0]))` collides with
`<linux/overflow.h>`'s own `array_size(a, b) = size_mul(a, b)` (2-arg,
different semantics, pulled in transitively via `<linux/string.h>` et al.
on 6.1). Renamed the vendor macro to `ACAMERA_ARRAY_SIZE` rather than
relying on preprocessor redefinition (which also depends on include
order -- not guaranteed to be consistent across ~130k lines and 192
files). Updated its two call sites (`src/fw_lib/cmos_fsm.c`,
`src/fw_lib/cmos_func.c`).

### `src/fw_lib/ae_manual_func.c`, `src/fw_lib/awb_manual_func.c`
Both call `copy_from_user()`. `ae_manual_func.c` had no uaccess include at
all (added `<linux/uaccess.h>`). `awb_manual_func.c` had `<asm/uaccess.h>`
already, which on this arch/kernel only gets you `raw_copy_from_user()`
-- the checked `copy_from_user()` wrapper lives in `<linux/uaccess.h>`
specifically (added that alongside the existing include rather than
replacing it, to minimize the diff).

### `src/fw_lib/cmos_fsm.c`, `src/fw_lib/cmos_func.c`
Only change: `array_size(...)` -> `ACAMERA_ARRAY_SIZE(...)`, see
`acamera_math.h` above.

### `src/platform/system_am_sc.c`
- Dropped dead `#include <linux/dma-contiguous.h>` (unused in this file).
- `write_to_file()`: identical `get_fs()`/`set_fs()`/`vfs_write()` ->
  `kernel_write()` rewrite as `main_kernel_juno_v4l2.c`'s function of the
  same name (these are two independent copies of near-identical code in
  the vendor tree, not a shared helper -- ported both). Same pre-existing
  `if (!fp)` -> `IS_ERR(fp)` bug fix applied here too.
- `ioremap_nocache()` -> `ioremap()` (one call site, in
  `am_sc_parse_dt()`).

### `src/platform/system_dma.c`
Added `#include <linux/pgtable.h>` before `#include <asm/uaccess.h>`.
Without it: `arch/arm64/include/asm/mmu.h` (pulled in transitively via
arm64's `asm/uaccess.h`) references `pgprot_t` in a function prototype,
and nothing had defined that typedef yet at that point in this file's
include order -- fails with `unknown type name 'pgprot_t'`. Worked by
accident on 4.9 via some other header pulling it in first; needed an
explicit include on 6.1. Also `ioremap_nocache()` -> `ioremap()` (one call
site, in `system_dma_sg_device_setup()`).

### `imx415/imx415.c`
**No changes.** Compiles clean verbatim as vendored from mainline v6.3.

Warnings left as-is (per the task brief: "warnings are fine and
expected, don't chase them all")
-----------------------------------------------------------------------
- `-Wcast-function-type` on every `*_get_fsm_common()` function across
  essentially all of `src/fw_lib/*_intf.c` (cmos, general, AE, AF, AWB,
  gamma, sharpening, sbuf, metadata, noise_reduction, crop, color_matrix,
  matrix_yuv, monitor, dma_writer -- ~15 files): the vendor's FSM
  dispatch table casts each FSM's specific `proc_event` function pointer
  type to a common `FUN_PTR_PROC_EVENT` signature. This is original
  vendor design (type-erasure via function pointer cast for a
  poor-man's-vtable pattern), present unchanged since the 4.9 source;
  newer GCC just warns about it now where older GCC didn't. Not a
  porting bug, not touched.
- `-Wimplicit-fallthrough` in `src/fw_lib/sensor_init.c` and
  `src/fw_lib/color_matrix_fsm.c`: pre-existing vendor `switch` statements
  without explicit fallthrough annotations. Not touched.
- The `array_size`/`array_size(a,b)` macro-redefinition warning that
  originally showed up during the very first build attempt is now gone
  entirely (fixed by the rename in `acamera_math.h`, not just silenced).

Confirmed-but-not-relevant to this build
-----------------------------------------
`subdev/iq` (and `subdev/lens`, `subdev/sensor`) are **not** hard
link-time dependencies of `iv009_isp.ko` -- confirmed by both the
Makefile source lists (v4l2_dev/Makefile's `FW_SRC_OBJ` never references
anything under `subdev/`) and a grep across the vendor `v4l2_dev/` tree
for any `#include` or symbol reference into `subdev/iq`. They're
self-contained separate modules for the legacy raw-sensor-driver path
this port intentionally isn't using (see the `V4L2_drv.c`
generic-subdev-bridge approach described in the task brief). Nothing
under `subdev/` was ported, built, or vendored into this `build/` tree.

Open items for the next session, ranked
-----------------------------------------
1. **DT wiring** (blocks the driver actually binding to anything): add an
   OF-graph `port`/`endpoint` under the `isp@ff140000` node, with a
   `remote-endpoint` pointing at the imx415 i2c client's own endpoint.
   Without it, `main_kernel_juno_v4l2.c`'s notifier registration finds
   zero subdevs and the driver just sits loaded-but-idle (see that file's
   changelog entry above for the full explanation of why the old
   "match-anything" trick isn't available anymore).
2. **DMA coherency verification** (correctness risk, not a build risk):
   confirm whether this device ends up `dma-coherent` in its eventual DT
   node, and if so/not, whether `dma_alloc_coherent()`'s returned vaddr
   is actually in the kernel's linear map on this SoC+kernel combination
   -- `isp-vb2-cmalloc.c`'s `cma_alloc()` and `isp-v4l2.c`'s
   `isp_cma_alloc()` both assume it is (inherited assumption from the
   vendor's original `dma_alloc_from_contiguous()`-based code, which had
   the same assumption baked in, just via a different, now-unusable API).
   Get this wrong and `virt_to_phys()` on the returned buffer silently
   returns a bogus address that the ISP's DMA engine would be programmed
   with -- a data-corruption bug, not a crash, so it could be hard to
   spot without deliberately checking.
3. **`vb2_create_framevec()`'s dropped write-direction argument**: minor,
   only affects the USERPTR buffer-type path (not exercised by this
   driver's own normal MMAP flow) -- see the `isp-vb2-cmalloc.c` entry
   above.
