#
# SPDX-License-Identifier: GPL-2.0
#
# Forward-ported (4.9 -> 6.1.68) out-of-tree build for the Amlogic/ARM
# ACamera ISP driver (iv009_isp). See PORT_NOTES.md in the parent build/
# directory for the full changelog.
#
# Module boundary preserved from the vendor tree's v4l2_dev/Makefile:
# this is the only module needed for the V4L2_drv.c generic-subdev-bridge
# integration path (subdev/iq, subdev/lens, subdev/sensor are legacy
# raw-sensor modules, not linked into this one -- see PORT_NOTES.md).
#

PWD := $(CURDIR)
KDIR ?= /usr/src/linux-headers-$(shell uname -r)

obj-m += iv009_isp.o

FW_SRC_OBJ := app/main_kernel_juno_v4l2.o \
			  app/v4l2_interface/isp-vb2-cmalloc.o \
			  app/v4l2_interface/isp-vb2.o \
			  app/v4l2_interface/fw-interface.o \
			  app/v4l2_interface/isp-v4l2-stream.o \
			  app/v4l2_interface/isp-v4l2.o \
			  app/v4l2_interface/isp-v4l2-ctrl.o \
			  app/main_firmware.o \
			  app/control/acamera_connection.o \
			  app/control/acamera_chardev.o \
			  app/control/application_command_api.o \
			  src/platform/system_stdlib.o \
			  src/platform/system_hdmi.o \
			  src/platform/system_timer.o \
			  src/platform/system_log.o \
			  src/platform/system_spinlock.o \
			  src/platform/system_semaphore.o \
			  src/platform/system_interrupts.o \
			  src/platform/system_am_sc.o \
			  src/platform/system_am_adap.o \
			  src/platform/system_am_mipi.o \
			  src/platform/system_hw_io.o \
			  src/platform/system_spi.o \
			  src/platform/system_chardev.o \
			  src/platform/system_dma.o \
			  src/platform/system_profiler.o \
			  src/platform/system_control.o \
			  src/platform/system_i2c.o \
			  src/platform/system_sw_io.o \
			  src/fw_lib/metadata_func.o \
			  src/fw_lib/general_func.o \
			  src/fw_lib/sensor_fsm.o \
			  src/fw_lib/monitor_fsm.o \
			  src/fw_lib/fsm_param.o \
			  src/fw_lib/acamera.o \
			  src/fw_lib/af_manual_fsm.o \
			  src/fw_lib/cmos_intf.o \
			  src/fw_lib/matrix_yuv_fsm.o \
			  src/fw_lib/general_intf.o \
			  src/fw_lib/dma_writer.o \
			  src/fw_lib/iridix8_manual_func.o \
			  src/fw_lib/metadata_fsm.o \
			  src/fw_lib/gamma_manual_fsm.o \
			  src/fw_lib/acamera_sbus_isp.o \
			  src/fw_lib/sensor_func.o \
			  src/fw_lib/acamera_sbus.o \
			  src/fw_lib/ae_manual_func.o \
			  src/fw_lib/sensor_intf.o \
			  src/fw_lib/ae_manual_intf.o \
			  src/fw_lib/monitor_func.o \
			  src/fw_lib/acamera_command_api.o \
			  src/fw_lib/acamera_calibrations.o \
			  src/fw_lib/dma_writer_func.o \
			  src/fw_lib/fsm_util.o \
			  src/fw_lib/general_fsm.o \
			  src/fw_lib/acamera_sbus_isp_sw.o \
			  src/fw_lib/iridix8_manual_fsm.o \
			  src/fw_lib/acamera_sbus_i2c.o \
			  src/fw_lib/color_matrix_func.o \
			  src/fw_lib/af_manual_intf.o \
			  src/fw_lib/awb_manual_fsm.o \
			  src/fw_lib/noise_reduction_fsm.o \
			  src/fw_lib/acamera_math.o \
			  src/fw_lib/matrix_yuv_func.o \
			  src/fw_lib/acamera_logger.o \
			  src/fw_lib/sensor_init.o \
			  src/fw_lib/gamma_manual_intf.o \
			  src/fw_lib/sharpening_intf.o \
			  src/fw_lib/awb_manual_intf.o \
			  src/fw_lib/acamera_command_api_impl.o \
			  src/fw_lib/cmos_func.o \
			  src/fw_lib/acamera_event_queue.o \
			  src/fw_lib/sbuf_intf.o \
			  src/fw_lib/metadata_intf.o \
			  src/fw_lib/crop_fsm.o \
			  src/fw_lib/noise_reduction_intf.o \
			  src/fw_lib/acamera_ctrl_channel_k2u.o \
			  src/fw_lib/ae_manual_fsm.o \
			  src/fw_lib/cmos_fsm.o \
			  src/fw_lib/sbuf_fsm.o \
			  src/fw_lib/monitor_intf.o \
			  src/fw_lib/sharpening_fsm.o \
			  src/fw_lib/acamera_fsm_mgr.o \
			  src/fw_lib/matrix_yuv_intf.o \
			  src/fw_lib/awb_manual_func.o \
			  src/fw_lib/af_manual_func.o \
			  src/fw_lib/acamera_fw.o \
			  src/fw_lib/sharpening_func.o \
			  src/fw_lib/iridix8_manual_intf.o \
			  src/fw_lib/color_matrix_fsm.o \
			  src/fw_lib/sbuf_func.o \
			  src/fw_lib/gamma_manual_func.o \
			  src/fw_lib/dma_writer_fsm.o \
			  src/fw_lib/dma_writer_intf.o \
			  src/fw_lib/crop_func.o \
			  src/fw_lib/noise_reduction_func.o \
			  src/fw_lib/crop_intf.o \
			  src/fw_lib/acamera_loop_buf.o \
			  src/fw_lib/color_matrix_intf.o \
			  src/fw_lib/acamera_sbus_spi.o \
			  src/calibration/acamera_get_calibrations_v4l2.o \
			  src/calibration/acamera_calibrations_static_linear_dummy.o \
			  src/calibration/acamera_calibrations_dynamic_linear_dummy.o \
			  src/calibration/acamera_calibrations_static_linear_imx415.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx415.o \
			  src/driver/sensor/sensor_bus_config.o \
			  src/driver/sensor/V4L2_drv.o \
			  src/driver/sensor/ACameraDefault.o \
			  src/driver/lens/v4l2_vcm.o \
			  src/driver/lens/lens_init.o

iv009_isp-objs += $(FW_SRC_OBJ)

# NOTE: use $(src), not $(PWD)/shell-pwd -- the Makefile is `include`d by
# Kbuild while its cwd is $(KDIR), so a shell-evaluated $(PWD) silently
# resolves to the kernel tree instead of this module dir (found the hard
# way: "acamera_firmware_config.h: No such file" even though it's right
# here in inc/). $(src) is kbuild's own pointer to this directory.
ccflags-y := -I$(src)/app -I$(src)/inc -I$(src)/app/control -I$(src)/inc/api \
			 -I$(src)/inc/isp -I$(src)/inc/sys -I$(src)/src/platform \
			 -I$(src)/src/fw -I$(src)/src/fw_lib -I$(src)/src/calibration \
			 -I$(src)/src/driver/sensor -I$(src)/src/driver/lens

ccflags-y += -Wno-declaration-after-statement

# This headers package's top Makefile builds NOSTDINC_FLAGS as plain
# "-nostdinc" with no "-isystem <gcc's own include dir>" (stock kernel
# Makefiles normally add that alongside -nostdinc so freestanding headers
# like stdarg.h/stddef.h are still reachable under -nostdinc). Without it,
# any TU that does `#include <stdarg.h>` (e.g. system_log.h, for the
# va_list-based logger) fails with "No such file or directory". Can't
# patch the shared headers tree (out of scope / read-only), so add it
# locally to this module's own cflags instead.
ccflags-y += -isystem $(shell $(CC) -print-file-name=include)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
