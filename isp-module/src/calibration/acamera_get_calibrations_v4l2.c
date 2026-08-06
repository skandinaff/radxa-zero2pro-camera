/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#include "acamera_command_api.h"
#include "acamera_firmware_api.h"
#include "acamera_logger.h"
#include "acamera_sensor_api.h"

/*
 * radxa-zero2pro-camera: calibrations are built into this module, not fetched
 * from the vendor's separate soc_iq subdev module.
 *
 * The vendor stack keeps IQ data in its own kernel module (subdev/iq ->
 * iv009_isp_iq.ko), which this port does not build: it exists only to publish
 * a v4l2 subdev named V4L2_SOC_IQ_NAME that soc_iq_get_calibrations() looks up
 * over an ioctl interface. That module ships tuning for imx227/imx290/imx307/
 * imx481/os08a10 -- there is no IMX415 set in it, so pulling it in would buy
 * nothing for this camera anyway.
 *
 * Supplying calibrations is not optional. acamera_firmware_settings.h marks
 * get_calibrations as "must be provided", and _GET_LUT_PTR() in
 * acamera_calibrations.c does not fail gracefully on a missing table -- it
 * enters a deliberate infinite loop ("Going to the infinite loop"), sleeping 3
 * seconds per iteration, which wedges insmod in uninterruptible sleep and
 * makes the module unloadable. Every one of the CALIBRATION_TOTAL_SIZE (123)
 * entries an enabled FSM touches has to be non-NULL.
 *
 * Only the LINEAR mode is wired up: the overlay selects a non-WDR IMX415 mode,
 * and neither calibration set has linear-WDR/native tables to point at.
 *
 *
 * ---------------------------------------------------------------------------
 * WHICH SET, AND HOW TO ROLL BACK
 * ---------------------------------------------------------------------------
 *
 * Two complete sets exist. Both are always compiled; this file picks one.
 *
 *   ACAMERA_CALIBRATION_IMX415 = 1  (default)
 *       acamera_calibrations_{static,dynamic}_linear_imx415.c
 *       Tuned for this camera and for the shot-trainer CV pipeline: linear
 *       gamma, datasheet black level, identity CCM, sharpening and temporal
 *       denoise off, Iridix off. Several tables are still placeholders pending
 *       hardware measurement -- lens shading above all. Every table names its
 *       source in a comment; docs/calibration-imx415.md has the inventory and
 *       the measurement procedures.
 *
 *   ACAMERA_CALIBRATION_IMX415 = 0
 *       acamera_calibrations_{static,dynamic}_linear_dummy.c
 *       ARM's neutral reference tuning, carried over verbatim from the vendor
 *       tree. Not tuned for the IMX415 at all -- colour, black level, lens
 *       shading and noise reduction will all be wrong -- but it is the
 *       known-good configuration this port was brought up on. Roll back to it
 *       by flipping the #define below (one line, nothing else to change: the
 *       dummy .c files are untouched and still in the Makefile).
 *
 * Which set actually loaded is logged at LOG_CRIT on every call, so a boot log
 * settles the question rather than a code read.
 *
 * ---------------------------------------------------------------------------
 * BUILD REQUIREMENT -- READ THIS IF THE LINK FAILS
 * ---------------------------------------------------------------------------
 *
 * The two imx415 .c files must be listed in isp-module/Makefile alongside the
 * dummy ones. Add these two lines next to the existing
 * src/calibration/acamera_calibrations_*_dummy.o entries:
 *
 *     src/calibration/acamera_calibrations_static_linear_imx415.o \
 *     src/calibration/acamera_calibrations_dynamic_linear_imx415.o \
 *
 * Without them the link fails with an undefined reference to
 * get_calibrations_static_linear_imx415. (Setting ACAMERA_CALIBRATION_IMX415 to
 * 0 also makes the build succeed, at the cost of running the untuned set.)
 */

#ifndef ACAMERA_CALIBRATION_IMX415
#define ACAMERA_CALIBRATION_IMX415 1
#endif

extern uint32_t get_calibrations_static_linear_dummy( ACameraCalibrations *c );
extern uint32_t get_calibrations_dynamic_linear_dummy( ACameraCalibrations *c );

#if ACAMERA_CALIBRATION_IMX415
extern uint32_t get_calibrations_static_linear_imx415( ACameraCalibrations *c );
extern uint32_t get_calibrations_dynamic_linear_imx415( ACameraCalibrations *c );
#define CALIBRATION_SET_NAME "imx415"
#define get_calibrations_static_linear get_calibrations_static_linear_imx415
#define get_calibrations_dynamic_linear get_calibrations_dynamic_linear_imx415
#else
#define CALIBRATION_SET_NAME "dummy"
#define get_calibrations_static_linear get_calibrations_static_linear_dummy
#define get_calibrations_dynamic_linear get_calibrations_dynamic_linear_dummy
#endif

uint32_t get_calibrations_v4l2( uint32_t ctx_id, void *sensor_arg, ACameraCalibrations *c )
{
    uint32_t ret = 0;
    int32_t preset = WDR_MODE_LINEAR;

    /* There is no failure path here on purpose. Returning an empty set leaves
     * the LUT pointers NULL, and _GET_LUT_PTR() responds to a NULL LUT by
     * spinning forever, which wedges the loading process in uninterruptible
     * sleep. Whatever happens, hand back a complete set.
     *
     * sensor_arg is NULL when the firmware asks for calibrations before the
     * sensor FSM has a mode to report -- which it does during init, and always
     * will if the sensor bridge failed to attach. Linear is the right default:
     * it is the mode the overlay selects, and the only one either tree has
     * tables for. */
    if ( sensor_arg )
        preset = ( (sensor_mode_t *)sensor_arg )->wdr_mode;
    else
        LOG( LOG_CRIT, "calibration sensor_arg is NULL, assuming linear" );

    if ( preset != WDR_MODE_LINEAR )
        LOG( LOG_CRIT, "No " CALIBRATION_SET_NAME " calibration for wdr_mode %d, using linear", (int)preset );

    ret = get_calibrations_dynamic_linear( c ) +
          get_calibrations_static_linear( c );

    LOG( LOG_CRIT, "Loaded " CALIBRATION_SET_NAME " calibrations, ctx_id:%d wdr_mode:%d ret:%d",
         ctx_id, (int)preset, ret );

    return ret;
}
