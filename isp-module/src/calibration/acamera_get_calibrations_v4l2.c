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
 * radxa-zero2pro-camera: calibrations come from the built-in "dummy" tables,
 * not from the vendor's separate soc_iq subdev module.
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
 * The dummy set is ARM's neutral reference tuning, carried over verbatim from
 * the vendor tree. It is enough to bring the pipeline up and get frames out;
 * it is NOT tuned for the IMX415, so colour, black level, lens shading and
 * noise reduction will all be visibly wrong. Correct output needs a real
 * IMX415 calibration set. See README "Image quality".
 *
 * Only the LINEAR mode is wired up: the overlay selects a non-WDR IMX415 mode,
 * and the vendor's dummy tree has no linear-WDR/native tables to point at.
 */

extern uint32_t get_calibrations_static_linear_dummy( ACameraCalibrations *c );
extern uint32_t get_calibrations_dynamic_linear_dummy( ACameraCalibrations *c );

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
     * it is the mode the overlay selects, and the only one the dummy tree has
     * tables for. */
    if ( sensor_arg )
        preset = ( (sensor_mode_t *)sensor_arg )->wdr_mode;
    else
        LOG( LOG_CRIT, "calibration sensor_arg is NULL, assuming linear" );

    if ( preset != WDR_MODE_LINEAR )
        LOG( LOG_CRIT, "No dummy calibration for wdr_mode %d, using linear", (int)preset );

    ret = get_calibrations_dynamic_linear_dummy( c ) +
          get_calibrations_static_linear_dummy( c );

    LOG( LOG_CRIT, "Loaded dummy calibrations, ctx_id:%d wdr_mode:%d ret:%d",
         ctx_id, (int)preset, ret );

    return ret;
}
