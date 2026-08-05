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

/*
 * radxa-zero2pro-camera: sensor bridge for a *mainline* V4L2 subdev.
 *
 * The stock version of this file drives ARM's "soc_sensor" shim: a vendor
 * subdev exposing ~30 private ioctls (SOC_SENSOR_GET_PRESET_WIDTH,
 * SOC_SENSOR_ALLOC_AGAIN, SOC_SENSOR_SET_PRESET, ...) that this bridge calls
 * through v4l2_subdev_call(core, ioctl). Nothing in mainline implements that
 * interface, so against drivers/media/i2c/imx415.c every one of those calls
 * failed and the bridge never populated sensor_param_t at all. That is what
 * left the firmware with a NULL modes_table (hence a NULL calibration
 * sensor_arg, and a NULL isp_context_seq that oopsed in
 * acamera_load_array_sequence).
 *
 * This version talks to the mainline imx415 subdev the standard way:
 *
 *   geometry / mode  -> v4l2_subdev_call(pad, set_fmt)
 *   exposure, gain   -> V4L2_CID_EXPOSURE / V4L2_CID_ANALOGUE_GAIN
 *   streaming        -> v4l2_subdev_call(video, s_stream)
 *
 * The mode table is static rather than enumerated. Mainline imx415 has a
 * single output geometry (3864x2192 SGBRG10); its "modes" select lane count
 * and lane rate, which are fixed here by the devicetree overlay, so there is
 * exactly one mode to advertise.
 *
 * It also does what no code in this port previously did: program the CSI-2
 * PHY and the MIPI adapter. In the vendor design each sensor driver calls
 * am_mipi_init()/am_adap_init()/am_adap_start() itself from its mode-set path
 * (see IMX307_drv.c sensor_set_iface()); there is no generic layer that does
 * it. Without those calls the receiver front end is never configured and no
 * pixels reach the ISP no matter what the sensor is doing. See
 * sensor_set_iface() below.
 */

#include "acamera_types.h"
#include "acamera_logger.h"
#include "acamera_sensor_api.h"
#include "acamera_command_api.h"
#include "acamera_firmware_config.h"
#include "isp_config_seq.h"

#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>

#include "system_am_adap.h"
#include "system_am_mipi.h"

extern void *acamera_camera_v4l2_get_subdev_by_prefix( const char *prefix );

/*
 * Fixed properties of the IMX415 as this board drives it. The lane count and
 * lane rate must agree with the CSI endpoint in overlays/camera-overlay.dts:
 * data-lanes = <1 2> and link-frequencies = 720000000. That link frequency is
 * the DDR clock, so the per-lane bit rate is twice it -- 1440 Mbps -- which
 * selects mainline imx415's 2-lane/1440 mode (30.019 fps, HMAX 4510).
 */
#define IMX415_SUBDEV_PREFIX "imx415"
#define IMX415_LANES 2
#define IMX415_LANE_RATE_MBPS 1440
#define IMX415_WIDTH 3864
#define IMX415_HEIGHT 2192
#define IMX415_BITS 10
#define IMX415_HMAX 4510 /* pixels per line incl. blanking, 2-lane 1440 mode */
#define IMX415_VMAX 2250 /* 0x08CA, as written by imx415's mode register list */
#define IMX415_FPS_Q8 ( 30 * 256 )

/* imx415 emits MEDIA_BUS_FMT_SGBRG10_1X10. */
#define IMX415_BAYER BAYER_GBRG

/*
 * SHR0 (exposure) must leave a few lines of margin at both ends of the frame;
 * the sensor rejects integration times outside this. 8 lines is the datasheet
 * minimum for the "all-pixel" readout mode.
 */
#define IMX415_INTEGRATION_MIN 8
#define IMX415_INTEGRATION_MARGIN 8

/* imx415 V4L2_CID_ANALOGUE_GAIN is 0..100 in 0.3 dB steps -> 30 dB. */
#define IMX415_AGAIN_MAX_DB 30

#define V4L2_SENSOR_MAXIMUM_PRESETS_NUM 1

static sensor_mode_t supported_modes[V4L2_SENSOR_MAXIMUM_PRESETS_NUM] = {
    {
        .wdr_mode = WDR_MODE_LINEAR,
        .fps = IMX415_FPS_Q8,
        .resolution = {.width = IMX415_WIDTH, .height = IMX415_HEIGHT},
        .exposures = 1,
        .bits = IMX415_BITS,
    },
};

typedef struct _sensor_context_t {
    sensor_param_t param;
    struct v4l2_subdev *sensor_sd;
    uint32_t vmax;
    int32_t again_val;      /* pending V4L2_CID_ANALOGUE_GAIN, 0.3 dB units */
    uint32_t int_time;      /* pending V4L2_CID_EXPOSURE, in lines */
    uint8_t streaming_flg;
} sensor_context_t;


static sensor_context_t s_ctx[FIRMWARE_CONTEXT_NUMBER];
static int ctx_counter = 0;

static const acam_reg_t **p_isp_data = SENSOR_ISP_SEQUENCE_DEFAULT;


static struct v4l2_ctrl *sensor_ctrl( sensor_context_t *p_ctx, uint32_t id )
{
    if ( p_ctx->sensor_sd == NULL || p_ctx->sensor_sd->ctrl_handler == NULL )
        return NULL;
    return v4l2_ctrl_find( p_ctx->sensor_sd->ctrl_handler, id );
}


static int sensor_ctrl_set( sensor_context_t *p_ctx, uint32_t id, int32_t val )
{
    struct v4l2_ctrl *c = sensor_ctrl( p_ctx, id );
    if ( c == NULL ) {
        LOG( LOG_ERR, "sensor has no control 0x%x", id );
        return -EINVAL;
    }
    return v4l2_ctrl_s_ctrl( c, val );
}


static void sensor_print_params( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    sensor_param_t *param = &p_ctx->param;

    LOG( LOG_CRIT, "IMX415 bridge: %dx%d total %dx%d, lines/s %d, exp %d..%d, again_log2_max %d",
         (int)param->active.width, (int)param->active.height,
         (int)param->total.width, (int)param->total.height,
         (int)param->lines_per_second,
         (int)param->integration_time_min, (int)param->integration_time_max,
         (int)param->again_log2_max );
}


/*
 * Configure the CSI-2 PHY and the MIPI adapter for the mode we just programmed
 * into the sensor. Modelled on IMX307_drv.c sensor_set_iface(): in the vendor
 * design this is the sensor driver's job, and skipping it leaves the receive
 * path dead.
 *
 * DIR_MODE sends pixels straight from the adapter into the ISP. The DDR/DOL
 * staging modes exist for WDR frame interleaving, which linear IMX415 does not
 * use, so this path needs no CMA buffer.
 */
static void sensor_set_iface( sensor_mode_t *mode )
{
    am_mipi_info_t mipi_info;
    struct am_adap_info info;

    if ( mode == NULL ) {
        LOG( LOG_ERR, "Error input param" );
        return;
    }

    memset( &mipi_info, 0, sizeof( mipi_info ) );
    memset( &info, 0, sizeof( struct am_adap_info ) );

    mipi_info.fte1_flag = get_fte1_flag();
    mipi_info.lanes = IMX415_LANES;
    /* UI (unit interval) in ns, rounded up, as the vendor code computes it. */
    mipi_info.ui_val = 1000 / IMX415_LANE_RATE_MBPS;
    if ( ( 1000 % IMX415_LANE_RATE_MBPS ) != 0 )
        mipi_info.ui_val += 1;

    am_mipi_init( &mipi_info );

    info.fmt = ( mode->bits == 12 ) ? AM_RAW12 : AM_RAW10;
    info.img.width = mode->resolution.width;
    info.img.height = mode->resolution.height;
    info.path = PATH0;
    info.mode = DIR_MODE;

    am_adap_set_info( &info );
    am_adap_init();
    am_adap_start( 0 );

    LOG( LOG_CRIT, "MIPI/adapter up: %d lanes, ui %d, %dx%d RAW%d, DIR_MODE",
         mipi_info.lanes, mipi_info.ui_val,
         (int)info.img.width, (int)info.img.height, mode->bits );
}


static void sensor_update_parameters( sensor_context_t *p_ctx )
{
    sensor_param_t *param = &p_ctx->param;
    struct v4l2_ctrl *c;

    param->active.width = IMX415_WIDTH;
    param->active.height = IMX415_HEIGHT;
    param->total.width = IMX415_HMAX;
    param->total.height = p_ctx->vmax;
    param->pixels_per_line = IMX415_HMAX;

    /*
     * Antiflicker and the AE FSM work in lines/second. Derive it from the
     * frame geometry rather than a hardcoded pixel clock so it stays correct
     * if VMAX is retuned (which the rolling-shutter work below does).
     */
    param->lines_per_second = ( IMX415_FPS_Q8 * p_ctx->vmax ) >> 8;

    param->integration_time_min = IMX415_INTEGRATION_MIN;
    param->integration_time_max = p_ctx->vmax - IMX415_INTEGRATION_MARGIN;
    param->integration_time_limit = param->integration_time_max;
    param->integration_time_long_max = param->integration_time_max;
    param->day_light_integration_time_max = param->integration_time_max;

    /* Prefer the driver's own advertised exposure range when it has one. */
    c = sensor_ctrl( p_ctx, V4L2_CID_EXPOSURE );
    if ( c != NULL ) {
        param->integration_time_min = c->minimum;
        param->integration_time_max = c->maximum;
        param->integration_time_limit = c->maximum;
        param->integration_time_long_max = c->maximum;
        param->day_light_integration_time_max = c->maximum;
    }

    param->again_accuracy = 1 << LOG2_GAIN_SHIFT;
    param->again_log2_max = ( IMX415_AGAIN_MAX_DB << LOG2_GAIN_SHIFT ) / 20;
    param->dgain_log2_max = 0;

    /*
     * imx415 latches SHR0/gain on the next frame boundary, so the ISP has to
     * hold its exposure decision for two frames before expecting to see it.
     */
    param->integration_time_apply_delay = 2;
    param->isp_exposure_channel_delay = 0;

    param->sensor_exp_number = 1;
    param->modes_table = supported_modes;
    param->modes_num = V4L2_SENSOR_MAXIMUM_PRESETS_NUM;
    param->mode = 0;
    param->bayer = IMX415_BAYER;
    param->sensor_ctx = p_ctx;

    param->isp_context_seq.sequence = p_isp_data;
    param->isp_context_seq.seq_num = SENSOR_ISP_SEQUENCE_DEFAULT_LINEAR;
    /* ARRAY_SIZE, not the vendor's array_size(): the kernel defines
     * array_size(a, b) as an overflow-checked multiply. */
    param->isp_context_seq.seq_table_max = ARRAY_SIZE( seq_table );
}


static int32_t sensor_alloc_analog_gain( void *ctx, int32_t gain )
{
    sensor_context_t *p_ctx = ctx;
    int32_t gain_db;

    if ( p_ctx == NULL )
        return 0;

    if ( gain > p_ctx->param.again_log2_max )
        gain = p_ctx->param.again_log2_max;
    if ( gain < 0 )
        gain = 0;

    /* log2 gain (Q<LOG2_GAIN_SHIFT>) -> dB -> imx415's 0.3 dB steps. */
    gain_db = ( gain * 20 ) >> LOG2_GAIN_SHIFT;
    p_ctx->again_val = ( gain_db * 10 ) / 3;

    /* Report back the gain actually representable, in the ISP's units. */
    return ( ( ( p_ctx->again_val * 3 ) / 10 ) << LOG2_GAIN_SHIFT ) / 20;
}


static int32_t sensor_alloc_digital_gain( void *ctx, int32_t gain )
{
    /* imx415's digital gain is folded into the same analogue gain register
     * ladder by the mainline driver, so there is nothing separate to set. */
    return 0;
}


static void sensor_alloc_integration_time( void *ctx, uint16_t *int_time, uint16_t *int_time_M, uint16_t *int_time_L )
{
    sensor_context_t *p_ctx = ctx;

    if ( p_ctx == NULL || int_time == NULL )
        return;

    if ( *int_time < p_ctx->param.integration_time_min )
        *int_time = p_ctx->param.integration_time_min;
    if ( *int_time > p_ctx->param.integration_time_max )
        *int_time = p_ctx->param.integration_time_max;

    p_ctx->int_time = *int_time;
}


static int32_t sensor_ir_cut_set( void *ctx, int32_t ir_cut_state )
{
    /* No IR-cut actuator wired on this camera module. */
    return 0;
}


static void sensor_update( void *ctx )
{
    sensor_context_t *p_ctx = ctx;

    if ( p_ctx == NULL || p_ctx->sensor_sd == NULL )
        return;

    sensor_ctrl_set( p_ctx, V4L2_CID_EXPOSURE, p_ctx->int_time );
    sensor_ctrl_set( p_ctx, V4L2_CID_ANALOGUE_GAIN, p_ctx->again_val );
}


static void sensor_test_pattern( void *ctx, uint8_t mode )
{
    sensor_context_t *p_ctx = ctx;

    if ( p_ctx == NULL )
        return;

    /* Optional in mainline imx415; ignore quietly if absent. */
    if ( sensor_ctrl( p_ctx, V4L2_CID_TEST_PATTERN ) != NULL )
        sensor_ctrl_set( p_ctx, V4L2_CID_TEST_PATTERN, mode );
}


static uint16_t sensor_get_id( void *ctx )
{
    /* The mainline driver verifies the chip ID during probe; if the subdev
     * exists at all the sensor answered correctly. 0 means "as expected". */
    return 0;
}


static void sensor_set_mode( void *ctx, uint8_t mode )
{
    sensor_context_t *p_ctx = ctx;
    struct v4l2_subdev_format fmt;
    struct v4l2_subdev_state *state;
    int rc;

    if ( p_ctx == NULL ) {
        LOG( LOG_CRIT, "Sensor context pointer is NULL" );
        return;
    }
    if ( p_ctx->sensor_sd == NULL ) {
        LOG( LOG_CRIT, "imx415 subdev pointer is NULL" );
        return;
    }
    if ( mode >= V4L2_SENSOR_MAXIMUM_PRESETS_NUM ) {
        LOG( LOG_CRIT, "Invalid mode %d", mode );
        return;
    }

    memset( &fmt, 0, sizeof( fmt ) );
    fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    fmt.pad = 0;
    fmt.format.width = supported_modes[mode].resolution.width;
    fmt.format.height = supported_modes[mode].resolution.height;
    fmt.format.code = MEDIA_BUS_FMT_SGBRG10_1X10;
    fmt.format.field = V4L2_FIELD_NONE;
    fmt.format.colorspace = V4L2_COLORSPACE_RAW;

    /*
     * set_fmt on an ACTIVE format needs the subdev's state locked. The imx415
     * subdev was registered with v4l2_subdev_init_finalize(), so it has one.
     */
    state = v4l2_subdev_lock_and_get_active_state( p_ctx->sensor_sd );
    rc = v4l2_subdev_call( p_ctx->sensor_sd, pad, set_fmt, state, &fmt );
    if ( state )
        v4l2_subdev_unlock_state( state );

    if ( rc != 0 ) {
        LOG( LOG_CRIT, "Failed to set sensor format. rc = %d", rc );
        return;
    }

    p_ctx->vmax = IMX415_VMAX;

    sensor_update_parameters( p_ctx );
    sensor_print_params( p_ctx );

    /* Bring up the receive path for this geometry before any pixels flow. */
    sensor_set_iface( &supported_modes[mode] );
}


static const sensor_param_t *sensor_get_parameters( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    return (const sensor_param_t *)&p_ctx->param;
}


static void sensor_disable_isp( void *ctx )
{
}


static uint32_t read_register( void *ctx, uint32_t address )
{
    /*
     * Raw sensor register access is a vendor-tool facility (the ACamera
     * control channel exposes it to userspace). Mainline subdevs deliberately
     * do not offer it -- V4L2_CID_* and the pad ops are the whole interface --
     * and the ISP firmware itself never needs it, so it is a no-op here.
     */
    return 0;
}


static void write_register( void *ctx, uint32_t address, uint32_t data )
{
}


static void stop_streaming( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    int rc;

    if ( p_ctx == NULL || p_ctx->sensor_sd == NULL )
        return;
    if ( !p_ctx->streaming_flg )
        return;

    p_ctx->streaming_flg = 0;

    rc = v4l2_subdev_call( p_ctx->sensor_sd, video, s_stream, 0 );
    if ( rc != 0 && rc != -ENOIOCTLCMD )
        LOG( LOG_ERR, "Failed to stop streaming. rc = %d", rc );

    am_adap_deinit();
    am_mipi_deinit();
}


static void start_streaming( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    int rc;

    if ( p_ctx == NULL || p_ctx->sensor_sd == NULL )
        return;
    if ( p_ctx->streaming_flg )
        return;

    rc = v4l2_subdev_call( p_ctx->sensor_sd, video, s_stream, 1 );
    if ( rc != 0 && rc != -ENOIOCTLCMD ) {
        LOG( LOG_CRIT, "Failed to start streaming. rc = %d", rc );
        return;
    }

    p_ctx->streaming_flg = 1;
    LOG( LOG_CRIT, "imx415 streaming on" );
}


void sensor_deinit_v4l2( void *ctx )
{
    sensor_context_t *p_ctx = ctx;

    if ( p_ctx == NULL ) {
        LOG( LOG_CRIT, "Sensor context pointer is NULL" );
        return;
    }

    stop_streaming( p_ctx );

    if ( --ctx_counter < 0 )
        LOG( LOG_CRIT, "Sensor context pointer is negative %d", ctx_counter );
}

//--------------------Initialization------------------------------------------------------------
void sensor_init_v4l2( void **ctx, sensor_control_t *ctrl )
{
    if ( ctx_counter >= FIRMWARE_CONTEXT_NUMBER ) {
        LOG( LOG_ERR, "Attempt to initialize more sensor instances than was configured. Sensor initialization failed." );
        *ctx = NULL;
        return;
    }

    {
        sensor_context_t *p_ctx = &s_ctx[ctx_counter];

        ctrl->alloc_analog_gain = sensor_alloc_analog_gain;
        ctrl->alloc_digital_gain = sensor_alloc_digital_gain;
        ctrl->alloc_integration_time = sensor_alloc_integration_time;
        ctrl->sensor_update = sensor_update;
        ctrl->sensor_test_pattern = sensor_test_pattern;
        ctrl->set_mode = sensor_set_mode;
        ctrl->get_id = sensor_get_id;
        ctrl->get_parameters = sensor_get_parameters;
        ctrl->disable_sensor_isp = sensor_disable_isp;
        ctrl->read_sensor_register = read_register;
        ctrl->write_sensor_register = write_register;
        ctrl->start_streaming = start_streaming;
        ctrl->stop_streaming = stop_streaming;
        ctrl->ir_cut_set = sensor_ir_cut_set;

        p_ctx->vmax = IMX415_VMAX;
        p_ctx->streaming_flg = 0;
        p_ctx->again_val = 0;
        p_ctx->int_time = IMX415_VMAX - IMX415_INTEGRATION_MARGIN;

        *ctx = p_ctx;

        /*
         * Matched on a prefix, not an exact name: an i2c-instantiated subdev
         * is named "<driver> <adapter>-<addr>", e.g. "imx415 3-001a", so the
         * exact-match lookup the soc_sensor path used can never find it.
         */
        p_ctx->sensor_sd = acamera_camera_v4l2_get_subdev_by_prefix( IMX415_SUBDEV_PREFIX );
        if ( p_ctx->sensor_sd == NULL ) {
            LOG( LOG_CRIT, "imx415 subdev not registered yet; sensor bridge not initialized" );
            /*
             * Still publish a valid parameter set. The firmware reads
             * modes_table/isp_context_seq during init and dereferences them
             * without checking, so leaving them NULL turns a missing sensor
             * into a kernel oops instead of a degraded start.
             */
            sensor_update_parameters( p_ctx );
            ctx_counter++;
            return;
        }

        sensor_update_parameters( p_ctx );

        ctx_counter++;
    }
}
