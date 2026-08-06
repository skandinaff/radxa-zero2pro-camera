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

#include "acamera_fw.h"
#if ACAMERA_ISP_PROFILING
#include "acamera_profiler.h"
#endif

#include "acamera_isp_config.h"
#include "acamera_command_api.h"
#include "acamera_isp_core_nomem_settings.h"
#include "acamera_metering_stats_mem_config.h"
#include "system_timer.h"
#include "acamera_logger.h"
#include "acamera_sbus_api.h"
#include "sensor_init.h"
#include "isp_config_seq.h"
#include "system_am_sc.h"

#define NELEM(x) ((int) (sizeof(x) / sizeof((x)[0])))

#if ISP_HAS_FPGA_WRAPPER
#include "acamera_fpga_config.h"
#endif

#if ISP_HAS_META_CB && defined( ISP_HAS_METADATA_FSM )
#include "metadata_api.h"
#endif


static const acam_reg_t **p_isp_data = SENSOR_ISP_SEQUENCE_DEFAULT;

extern void acamera_notify_evt_data_avail( void );
extern void *acamera_get_api_ctx_ptr( void );

void acamera_load_isp_sequence( uintptr_t isp_base, const acam_reg_t **sequence, uint8_t num )
{
    acamera_sbus_t sbus;
    sbus.mask = SBUS_MASK_SAMPLE_32BITS | SBUS_MASK_SAMPLE_16BITS | SBUS_MASK_SAMPLE_8BITS | SBUS_MASK_ADDR_STEP_32BITS | SBUS_MASK_ADDR_32BITS;
    acamera_sbus_init( &sbus, sbus_isp );
    acamera_load_array_sequence( &sbus, isp_base, 0, sequence, num );
}


void acamera_load_sw_sequence( uintptr_t isp_base, const acam_reg_t **sequence, uint8_t num )
{
    acamera_sbus_t sbus;
    sbus.mask = SBUS_MASK_SAMPLE_32BITS | SBUS_MASK_SAMPLE_16BITS | SBUS_MASK_SAMPLE_8BITS | SBUS_MASK_ADDR_STEP_32BITS | SBUS_MASK_ADDR_32BITS;
    acamera_sbus_init( &sbus, sbus_isp_sw );
    acamera_load_array_sequence( &sbus, isp_base, 0, sequence, num );
}


#define IRQ_ID_UNDEFINED 0xFF


/*
 * How many times we will try to reset the ISP out of an error condition before
 * giving up and leaving it stopped. See acamera_fw_error_routine().
 */
#define ACAMERA_FW_ERROR_MAX_RETRY 3

/* File scope, not function scope, so acamera_fw_init() can clear it: every
 * fresh stream start gets a fresh budget of recovery attempts. */
static uint32_t acamera_fw_error_count = 0;

void acamera_fw_init( acamera_context_t *p_ctx )
{

#if ACAMERA_ISP_PROFILING
#if ACAMERA_ISP_PROFILING_INIT
    p_ctx->binit_profiler = 0;
    p_ctx->breport_profiler = 0;
#else
    p_ctx->binit_profiler = 0;
    p_ctx->breport_profiler = 0;
#endif
    p_ctx->start_profiling = 500; //start when gframe == 500
    p_ctx->stop_profiling = 1000; //stop  when gframe == 1000
#endif

    p_ctx->irq_flag = 1;

    p_ctx->fsm_mgr.p_ctx = p_ctx;
    p_ctx->fsm_mgr.ctx_id = p_ctx->context_id;
    p_ctx->fsm_mgr.isp_base = p_ctx->settings.isp_base;
    acamera_fsm_mgr_init( &p_ctx->fsm_mgr );

    p_ctx->irq_flag = 0;
    acamera_fw_interrupts_enable( p_ctx );
    p_ctx->system_state = FW_RUN;

    /* Fresh start: give the error routine a full budget of recovery attempts
     * again (see ACAMERA_FW_ERROR_MAX_RETRY). */
    acamera_fw_error_count = 0;
}

void acamera_fw_deinit( acamera_context_t *p_ctx )
{
    p_ctx->fsm_mgr.p_ctx = p_ctx;
    acamera_fsm_mgr_deinit( &p_ctx->fsm_mgr );
}

/*
 * Stream teardown/restart, called from the V4L2 STREAMOFF/STREAMON path
 * (fw-interface.c) around the sensor's own start/stop.
 *
 * Why these exist -- measured on hardware 2026-08-06. Every STREAMOFF ended
 * with the ISP error routine firing exactly once, on BROKEN_FRAME (irq_mask
 * 0x8), with broken_frame status 3 = "active width mismatch | active height
 * mismatch" and fr_pipeline_busy latched at 1. The order of events in dmesg is
 * unambiguous:
 *
 *   AM_MIPI: am_mipi_deinit:Success mipi deinit
 *   Found error resetting ISP. MASK is 0x8
 *   input_port: mode_status 0 hc_size0 3864 vc_size 2192
 *   monitor: fr_pipeline_busy 1 broken_frame 3 ...
 *   stopping isp failed, timeout. mode_status 0 (want 0), fr_pipeline_busy 1
 *
 * i.e. the *sensor* side was torn down first. fw_intf_stream_stop() issued
 * SENSOR_STREAMING OFF, which runs V4L2_drv.c stop_streaming(): imx415
 * s_stream(0), am_adap_deinit(), am_mipi_deinit(). That cuts the CSI-2 data
 * off in the middle of whatever frame the ISP was receiving. A truncated frame
 * is a broken frame by definition, and the main pipeline -- which is waiting
 * for the rest of it -- keeps fr_pipeline_busy asserted, forever, because the
 * remaining lines are never going to arrive.
 *
 * The error routine then cannot do its job either. Note the register comment
 * on fr_pipeline_busy in acamera_isp_config.h: "global_fsm_reset must be set
 * when this busy signal is low." With the source of pixels already gone, the
 * busy bit can never go low, so the routine times out, correctly refuses to
 * restart, and leaves the ISP with all interrupts masked
 * (ISP_IRQ_DISABLE_ALL_IRQ) and the input port in SAFE_STOP.
 *
 * Nothing in the STREAMON path ever undid that. acamera_fw_init() -- the only
 * thing that unmasks interrupts and clears acamera_fw_error_count -- runs once
 * per acamera_init_context(), i.e. at module load, and sensor_sw_init() (the
 * only writer of SAFE_START) runs only on a preset-mode change. So the second
 * VIDIOC_STREAMON in the life of a module load restarted the sensor happily
 * and then blocked forever: interrupt_mask_vector still 0xffffffff, input port
 * still SAFE_STOP, ISP interrupt count flat. That is what made a reboot
 * necessary between streaming sessions -- not, as previously assumed, wedged
 * hardware. Reading the ISP after a failed teardown shows 0x50 == 0, i.e.
 * fr_pipeline_busy and broken_frame both clear: the global FSM reset the error
 * routine issues does land, and the block is idle and healthy.
 *
 * So, two halves:
 *
 *   quiesce  Stop the ISP input port *before* the sensor, and wait for the
 *            pipeline to drain. SAFE_STOP means "finish the frame you are on,
 *            then stop", so it only works while pixels are still flowing --
 *            which is exactly the window this closes. Once fr_pipeline_busy
 *            reads 0 there is no frame in flight, and dropping MIPI cannot
 *            break one.
 *
 *   rearm    Undo, on every stream start, everything a previous bad teardown
 *            could have latched: the error budget, the interrupt mask and the
 *            input port request. Belt and braces -- with quiesce in place the
 *            error routine should not run at all -- but it is what makes a
 *            session recoverable without a module reload, and it costs four
 *            register writes.
 */

/* One frame at 60 fps is 16.7 ms; 2 lanes at 30 fps is 33.3 ms. Allow well
 * over two worst-case frame periods before declaring the drain failed. */
#define ACAMERA_FW_QUIESCE_TIMEOUT_MS 200

void acamera_fw_stream_quiesce( void )
{
    uint32_t ms;

    /* Ask the input port to stop at the next frame boundary. Deliberately not
     * touching the DMA writer or the interrupt mask first: the writer must
     * stay armed so the in-flight frame can complete, and the FS/FE interrupts
     * are what advance it. */
    acamera_isp_input_port_mode_request_write( 0, ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_STOP );

    for ( ms = 0; ms < ACAMERA_FW_QUIESCE_TIMEOUT_MS; ms++ ) {
        if ( acamera_isp_input_port_mode_status_read( 0 ) == ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_STOP &&
             !acamera_isp_isp_global_monitor_fr_pipeline_busy_read( 0 ) )
            break;

        system_timer_usleep( 1000 );
    }

    if ( ms >= ACAMERA_FW_QUIESCE_TIMEOUT_MS ) {
        LOG( LOG_CRIT, "ISP did not quiesce in %u ms: mode_status %u fr_pipeline_busy %u broken_frame %u. "
                       "Tearing down anyway; stream restart will re-arm.",
             (unsigned int)ACAMERA_FW_QUIESCE_TIMEOUT_MS,
             (unsigned int)acamera_isp_input_port_mode_status_read( 0 ),
             (unsigned int)acamera_isp_isp_global_monitor_fr_pipeline_busy_read( 0 ),
             (unsigned int)acamera_isp_isp_global_monitor_broken_frame_status_read( 0 ) );
    } else {
        LOG( LOG_INFO, "ISP quiesced in %u ms", (unsigned int)ms );
    }

    /* Pipeline is idle (or hopeless). Silence the block so that nothing the
     * sensor/adapter/MIPI teardown does downstream of here can raise a
     * spurious error interrupt.
     *
     * Deliberately *not* clearing the FR DMA writer's frame_write_on bits the
     * way acamera_fw_error_routine() does. Those accessors live in
     * acamera_isp1_config.h and write the software config shadow via
     * system_sw_*, not the hardware; the value only reaches the ISP when the
     * page is DMA'd across at the next frame start. In the error routine there
     * is a next frame start, so clearing it there is meaningful. Here there
     * deliberately is not one -- the input port is stopped and interrupts are
     * about to be masked -- so the write would achieve nothing, and getting its
     * base argument wrong is an oops rather than a no-op: unlike the
     * acamera_isp_config.h / system_hw_* accessors used above, which ignore
     * their base in favour of the single ioremap in system_hw_io.c, these
     * dereference (base + offset) directly. Measured the hard way, 2026-08-06. */
    acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_DISABLE_ALL_IRQ );
}

void acamera_fw_stream_rearm( void )
{
    /* A previous session's error routine may have exhausted the budget. */
    acamera_fw_error_count = 0;

    /* Drop anything latched while we were masked, then unmask. The 0/1 pulse
     * is the same clear sequence acamera_interrupt_handler() uses. */
    acamera_isp_isp_global_interrupt_clear_write( 0, 0 );
    acamera_isp_isp_global_interrupt_clear_write( 0, 1 );
    acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_MASK_VECTOR );

    /* Arm the input port before the sensor starts, not after, so the very
     * first frame is accepted rather than dropped. */
    acamera_isp_input_port_mode_request_write( 0, ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_START );
}

/*
 * Push a manual exposure/gain change through to the sensor.
 *
 * Why this has to exist -- measured 2026-08-06. ARM's ACamera design splits
 * 3A between the kernel and a userspace algorithm daemon: the kernel's AE FSM
 * collects the histogram (ACAMERA_IRQ_AE_STATS -> event_id_ae_stats_ready) and
 * publishes it over the sbuf shared-buffer channel, and userspace computes the
 * new exposure and hands it back via ae_set_new_param(). That function is the
 * *only* caller of fsm_raise_event( event_id_ae_result_ready ), which is the
 * only thing AE_fsm_process_event() handles, and which in turn is what raises
 * event_id_exposure_changed -- the event that makes cmos_fsm re-run
 * cmos_inttime_update() / cmos_analog_gain_update() / cmos_update_exposure_
 * history() and actually program the sensor.
 *
 * This port has no userspace 3A daemon. An event histogram over a 300-frame
 * capture makes the consequence exact:
 *
 *   event_id_ae_stats_ready        299     <- stats collected, every frame
 *   event_id_ae_result_ready         0     <- nobody ever answers
 *   event_id_exposure_changed        0
 *   event_id_sensor_ready            1     <- once, at init
 *
 * So the whole exposure/gain chain runs exactly once, on that single
 * event_id_sensor_ready at init, and then freezes. "Auto" exposure is not a
 * loop, it is a one-shot; and every manual control writes its value into the
 * calibration block (cmos_control_param_t) where nothing ever reads it again.
 * That is why sweeping V4L2_CID_EXPOSURE_ABSOLUTE 16x and V4L2_CID_GAIN 32x
 * both moved mean output by under 0.05 DN.
 *
 * Raising the event by hand after a manual set is the missing half. It costs
 * one event-queue push and reuses the vendor's own apply path, so the manual
 * case -- the one that does not need an algorithm at all -- works exactly as
 * the design intends. Automatic AE still requires the userspace daemon and is
 * out of scope here.
 */
void acamera_fw_exposure_apply( void )
{
    acamera_context_t *p_ctx = (acamera_context_t *)acamera_get_api_ctx_ptr();

    if ( p_ctx == NULL )
        return;

    acamera_fsm_mgr_raise_event( &p_ctx->fsm_mgr, event_id_exposure_changed );
}

/*
 * The vendor code retries forever. On this board that is actively dangerous:
 * when the error is persistent (a FRAME_COLLISION that repeats every frame),
 * the retry loop re-arms a pipeline that is still mid-DMA, and within ~70 ms
 * the resulting memory corruption SIGSEGVs systemd (PID 1) -- which kills all
 * userspace, takes the network with it, and leaves only a physical power cycle
 * as a way back in. Capping the retries turns an unrecoverable board hang into
 * an ordinary failed capture that can actually be observed and debugged.
 *
 * Retry-capping alone is not enough. Confirmed on hardware 2026-08-05: the
 * corruption is not a product of *cascading* resets -- it happened on ISP
 * ERROR #1, before the retry budget was anywhere near exhausted, and this
 * time it was bad enough to reach disk: systemd(1) aborted on a corrupted
 * internal priority queue and the ext4 journal caught a freed-inode bitmap
 * mismatch on the root filesystem (recovered by journal replay on next boot,
 * but that is luck, not a guarantee). Both this crash and the original one
 * logged "stopping isp failed" -- input_port never actually reached
 * SAFE_STOP, fr_pipeline_busy never cleared -- immediately before the
 * corruption. The vendor code did not treat that as a reason to stop: it
 * force-toggled a global FSM reset and then re-armed the pipeline (SAFE_START
 * + re-enabled interrupts + DMA writer still live) over a pipeline that was
 * never actually quiesced. Two fixes:
 *
 *   1) Disable the DMA writer's own write-enable bits *before* attempting
 *      anything else, unconditionally. This is the one step in the whole
 *      routine that can actually stop an in-flight AXI write burst from the
 *      writer landing on a buffer the driver is simultaneously reassigning --
 *      "safe stop" only asks the input port state machine to wind down, it
 *      does not touch the writer.
 *   2) If the stop timeout is hit -- input port never reaches SAFE_STOP, or
 *      the pipeline is still busy -- do not restart, regardless of how many
 *      retries are left. A "successful" forced reset over a wedged pipeline
 *      is not actually a recovery; it is the specific sequence that corrupted
 *      memory both times this has been observed. Treat a failed stop exactly
 *      like the retry-budget-exhausted case: leave the ISP stopped, leave
 *      interrupts masked, leave the DMA writer disabled, and fail the capture
 *      cleanly. The only way out is a full stream restart through
 *      acamera_fw_init(), same as today.
 */
void acamera_fw_error_routine( acamera_context_t *p_ctx, uint32_t irq_mask )
{
    uint32_t error_count;
    uintptr_t isp_base = p_ctx->settings.isp_base;
    uint32_t stopped_cleanly;

    error_count = ++acamera_fw_error_count;

    /* First, before anything else -- including the diagnostic register reads
     * below, which are read-only and safe regardless of ordering. This is the
     * step that actually closes the race: it stops the writer from landing
     * any further bus transactions on whatever buffer it currently thinks is
     * live, independent of whether the input port ever manages to stop. */
    acamera_isp_fr_dma_writer_frame_write_on_write( isp_base, 0 );
    acamera_isp_fr_uv_dma_writer_frame_write_on_write( isp_base, 0 );

    /*
     * Dump the hardware's own view of why it is unhappy, before the reset
     * clears it. The two interesting questions this answers:
     *
     *   wfifo_fail_full   the DMA write FIFO overflowed -- the write side
     *                     could not drain pixels as fast as the pipe produced
     *                     them. That is a *bandwidth* problem.
     *   vc_size/hc_size   what geometry the ISP input port is actually
     *                     configured for. If this disagrees with the number of
     *                     lines the sensor really sends, the pipeline never
     *                     sees end-of-frame, stays busy forever, and the next
     *                     frame start collides with it. That is a *geometry*
     *                     problem.
     *
     * These are mutually exclusive diagnoses and we have been guessing between
     * them, so read them rather than theorise.
     */
    if ( error_count <= ACAMERA_FW_ERROR_MAX_RETRY ) {
        LOG( LOG_CRIT, "ISP ERROR #%u: irq_mask 0x%x", (unsigned int)error_count, (unsigned int)irq_mask );
        LOG( LOG_CRIT, "  input_port: mode_status %u hc_size0 %u vc_size %u",
             (unsigned int)acamera_isp_input_port_mode_status_read( isp_base ),
             (unsigned int)acamera_isp_input_port_hc_size0_read( isp_base ),
             (unsigned int)acamera_isp_input_port_vc_size_read( isp_base ) );
        LOG( LOG_CRIT, "  monitor: fr_pipeline_busy %u broken_frame %u dma_alarms 0x%x max_addr_delay_fr %u",
             (unsigned int)acamera_isp_isp_global_monitor_fr_pipeline_busy_read( isp_base ),
             (unsigned int)acamera_isp_isp_global_monitor_broken_frame_status_read( isp_base ),
             (unsigned int)acamera_isp_isp_global_monitor_dma_alarms_read( isp_base ),
             (unsigned int)acamera_isp_isp_global_monitor_max_address_delay_line_fr_read( isp_base ) );
        LOG( LOG_CRIT, "  fr_y wfifo: fail_full %u fail_empty %u   fr_uv wfifo: fail_full %u fail_empty %u",
             (unsigned int)acamera_isp_isp_global_monitor_fr_y_dma_wfifo_fail_full_read( isp_base ),
             (unsigned int)acamera_isp_isp_global_monitor_fr_y_dma_wfifo_fail_empty_read( isp_base ),
             (unsigned int)acamera_isp_isp_global_monitor_fr_uv_dma_wfifo_fail_full_read( isp_base ),
             (unsigned int)acamera_isp_isp_global_monitor_fr_uv_dma_wfifo_fail_empty_read( isp_base ) );
    }

    //masked all interrupts
    acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_DISABLE_ALL_IRQ );
    //safe stop
    acamera_isp_input_port_mode_request_write( isp_base, ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_STOP );

    // check whether the HW is stopped or not.
    uint32_t count = 0;
    stopped_cleanly = 1;
    while ( acamera_isp_input_port_mode_status_read( isp_base ) != ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_STOP || acamera_isp_isp_global_monitor_fr_pipeline_busy_read( isp_base ) ) {
        //cannot sleep use this delay
        do {
            count++;
        } while ( count % 32 != 0 );

        if ( ( count >> 5 ) > 50 ) {
            /* Say *which* condition never cleared -- "stopping isp failed" on
             * its own does not distinguish a stuck input port from a pipeline
             * that is still draining. */
            LOG( LOG_CRIT, "stopping isp failed, timeout: %u. mode_status %u (want %u), fr_pipeline_busy %u",
                 (unsigned int)count * 1000,
                 (unsigned int)acamera_isp_input_port_mode_status_read( isp_base ),
                 (unsigned int)ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_STOP,
                 (unsigned int)acamera_isp_isp_global_monitor_fr_pipeline_busy_read( isp_base ) );
            stopped_cleanly = 0;
            break;
        }
    }

    acamera_isp_isp_global_global_fsm_reset_write( isp_base, 1 );
    acamera_isp_isp_global_global_fsm_reset_write( isp_base, 0 );

    if ( error_count > ACAMERA_FW_ERROR_MAX_RETRY || !stopped_cleanly ) {
        /*
         * Either the retry budget is exhausted, or the input port never
         * actually reached SAFE_STOP -- confirmed on hardware to be the exact
         * precondition of a memory-corrupting restart (see the block comment
         * above this function). Leave interrupts masked, the input port
         * stopped, and the DMA writer disabled: the pipeline stays quiet and
         * the capture fails cleanly instead of taking the whole system down.
         * Recovering needs a stream restart, which re-runs acamera_fw_init()
         * and clears this counter.
         */
        LOG( LOG_CRIT, "ISP error routine giving up (error_count %u, stopped_cleanly %u) -- "
                       "leaving ISP and DMA writer stopped. Capture will fail; this is "
                       "deliberate, see acamera_fw_error_routine().",
             (unsigned int)error_count, (unsigned int)stopped_cleanly );
        return;
    }

    //return the interrupts
    acamera_isp_isp_global_interrupt_mask_vector_write( 0, ISP_IRQ_MASK_VECTOR );

    acamera_isp_input_port_mode_request_write( isp_base, ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_START );

    /* Deliberately not re-enabling the DMA writer here. dma_writer_pipe_update()
     * (dma_writer.c) already re-enables both write-on bits unconditionally,
     * every frame-start, once it has a valid empty frame and has programmed a
     * fresh bank0_base address for it -- which is exactly the point at which
     * re-enabling is actually safe. Doing it here instead would race ahead of
     * that and re-arm the writer against whatever stale address was left over
     * from the frame that just errored. */

    LOG( LOG_CRIT, "starting isp from error" );
}


void acamera_fw_process( acamera_context_t *p_ctx )
{
#if ACAMERA_ISP_PROFILING
    if ( ( p_ctx->frame >= p_ctx->start_profiling ) && ( !p_ctx->binit_profiler ) ) {
        acamera_profiler_init();
        p_ctx->binit_profiler = 1;
    }
#endif
    if ( ( p_ctx->system_state == FW_RUN ) ) //need to capture on firmware freeze
    {
        // firmware not frozen
        // 0 means handle all the events and then return.
        acamera_fsm_mgr_process_events( &p_ctx->fsm_mgr, 0 );
    }
#if ACAMERA_ISP_PROFILING
    if ( ( p_ctx->frame >= p_ctx->stop_profiling ) && ( !p_ctx->breport_profiler ) ) {
        acamera_profiler_report();
        p_ctx->breport_profiler = 1;
    }
#endif
}

void acamera_fw_raise_event( acamera_context_t *p_ctx, event_id_t event_id )
{ //dma writer events should be passed for the capture on freeze requirement
    if ( p_ctx->stab.global_freeze_firmware == 0 || event_id == event_id_new_frame || event_id == event_id_drop_frame
#if defined( ISP_HAS_DMA_WRITER_FSM )
         || event_id == event_id_frame_buffer_fr_ready || event_id == event_id_frame_buffer_ds_ready || event_id == event_id_frame_buffer_metadata
#endif
#if defined( ISP_HAS_METADATA_FSM )
         || event_id == event_id_metadata_ready || event_id == event_id_metadata_update
#endif
#if defined( ISP_HAS_BSP_TEST_FSM )
         || event_id == event_id_bsp_test_interrupt_finished
#endif
         ) {
        acamera_event_queue_push( &p_ctx->fsm_mgr.event_queue, (int)( event_id ) );

        acamera_notify_evt_data_avail();
    }
}

void acamera_fsm_mgr_raise_event( acamera_fsm_mgr_t *p_fsm_mgr, event_id_t event_id )
{ //dma writer events should be passed for the capture on freeze requirement
    if ( p_fsm_mgr->p_ctx->stab.global_freeze_firmware == 0 || event_id == event_id_new_frame || event_id == event_id_drop_frame
#if defined( ISP_HAS_DMA_WRITER_FSM )
         || event_id == event_id_frame_buffer_fr_ready || event_id == event_id_frame_buffer_ds_ready || event_id == event_id_frame_buffer_metadata
#endif
#if defined( ISP_HAS_BSP_TEST_FSM )
         || event_id == event_id_bsp_test_interrupt_finished
#endif
         ) {
        acamera_event_queue_push( &( p_fsm_mgr->event_queue ), (int)( event_id ) );

        acamera_notify_evt_data_avail();
    }
}


int32_t acamera_update_calibration_set( acamera_context_ptr_t p_ctx )
{
    int32_t result = 0;
    void *sensor_arg = 0;
    if ( p_ctx->settings.get_calibrations != NULL ) {
        {
            const sensor_param_t *param = NULL;
            acamera_fsm_mgr_get_param( &p_ctx->fsm_mgr, FSM_PARAM_GET_SENSOR_PARAM, NULL, 0, &param, sizeof( param ) );

            uint32_t cur_mode = param->mode;
            if ( cur_mode < param->modes_num ) {
                sensor_arg = &( param->modes_table[cur_mode] );
            }
        }
        if ( p_ctx->settings.get_calibrations( p_ctx->context_id, sensor_arg, &p_ctx->acameraCalibrations ) != 0 ) {
            LOG( LOG_CRIT, "Failed to get calibration set for. Fatal error" );
        }

#if defined( ISP_HAS_GENERAL_FSM )
        acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_RELOAD_CALIBRATION, NULL, 0 );
#endif

// Update some FSMs variables which depends on calibration data.
#if defined( ISP_HAS_AE_BALANCED_FSM ) || defined( ISP_HAS_AE_MANUAL_FSM )
        acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_AE_INIT, NULL, 0 );
#endif

#if defined( ISP_HAS_IRIDIX_FSM ) || defined( ISP_HAS_IRIDIX_HIST_FSM ) || defined( ISP_HAS_IRIDIX_MANUAL_FSM )
        acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_IRIDIX_INIT, NULL, 0 );
#endif

#if defined( ISP_HAS_COLOR_MATRIX_FSM )
        acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_CCM_CHANGE, NULL, 0 );
#endif

#if defined( ISP_HAS_SBUF_FSM )
        acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_SBUF_CALIBRATION_UPDATE, NULL, 0 );
#endif
    } else {
        LOG( LOG_CRIT, "Calibration callback is null. Failed to get calibrations" );
        result = -1;
    }

    return result;
}


int32_t acamera_init_calibrations( acamera_context_ptr_t p_ctx )
{
    int32_t result = 0;
    void *sensor_arg = 0;
#ifdef SENSOR_ISP_SEQUENCE_DEFAULT_FULL
    acamera_load_isp_sequence( p_ctx->settings.isp_base, p_ctx->isp_sequence, SENSOR_ISP_SEQUENCE_DEFAULT_FULL );
#endif

    // if "p_ctx->initialized" is 1, that means we are changing the preset and wdr_mode,
    // we need to update the calibration data and update some FSM variables which
    // depends on calibration data.
    if ( p_ctx->initialized == 1 ) {
        acamera_update_calibration_set( p_ctx );
    } else {
        if ( p_ctx->settings.get_calibrations != NULL ) {
            const sensor_param_t *param = NULL;
            acamera_fsm_mgr_get_param( &p_ctx->fsm_mgr, FSM_PARAM_GET_SENSOR_PARAM, NULL, 0, &param, sizeof( param ) );

            uint32_t cur_mode = param->mode;
            if ( cur_mode < param->modes_num ) {
                sensor_arg = &( param->modes_table[cur_mode] );
            }

            if ( p_ctx->settings.get_calibrations( p_ctx->context_id, sensor_arg, &p_ctx->acameraCalibrations ) != 0 ) {
                LOG( LOG_CRIT, "Failed to get calibration set for. Fatal error" );
            }
        } else {
            LOG( LOG_CRIT, "Calibration callback is null. Failed to get calibrations" );
            result = -1;
        }
    }
    return result;
}

int32_t acamera_init_context_seq( acamera_context_t *p_ctx )
{
    int32_t result = 0;

    // if "p_ctx->initialized" is 1, that means we are changing the preset and wdr_mode,
    // we need to update the calibration data and update some FSM variables which
    // depends on calibration data.
    const sensor_param_t *param = NULL;
    result = acamera_fsm_mgr_get_param( &p_ctx->fsm_mgr, FSM_PARAM_GET_SENSOR_PARAM, NULL, 0, &param, sizeof( param ) );
    if (result != 0) {
        LOG(LOG_ERR, "WARNING:get isp context seq failed.\n");
        return 0;
    }
    p_ctx->isp_context_seq.sequence = param->isp_context_seq.sequence;
    p_ctx->isp_context_seq.seq_num = param->isp_context_seq.seq_num;
    LOG(LOG_ERR, "load isp context sequence[%d]\n", param->isp_context_seq.seq_num);

    acamera_load_sw_sequence( p_ctx->settings.isp_base, p_ctx->isp_context_seq.sequence, p_ctx->isp_context_seq.seq_num );

    /*
     * Re-disarm the output DMA writers that the context sequence just armed.
     *
     * settings_context[] ends with a canned FR writer configuration lifted from
     * whatever bench the vendor captured it on: bank0_base 0x05000000,
     * line_offset 0x1e00, wbank_active 1 and frame_write_on 1. Those are a real
     * physical address and a real enable, and this context space is DMA'd
     * straight into ping and pong a few lines later in acamera_init_context().
     *
     * The dma_writer FSM is initialised inside acamera_fw_init(), which runs
     * *before* this function, and its init clears frame_write_on (dma_writer.c
     * dma_writer_init_frame_queue path). So loading the sequence undoes that and
     * leaves a writer enabled against 0x05000000 -- ordinary kernel memory on
     * this board -- from here until dma_writer_pipe_update() reprograms it at
     * STREAMON. The input port is set to SAFE_START at the end of
     * acamera_init_context() and the MIPI adapter has already been started by
     * the sensor's set_mode, so that window is not theoretical.
     *
     * This project has already lost a day to exactly this failure mode once
     * (a bad DMA address from virt_to_phys() on a coherent allocation, which
     * SIGSEGV'd PID 1 and corrupted the filesystem), so the writers stay off
     * until the FSM that owns them turns them on with an address it allocated.
     */
    acamera_isp_fr_dma_writer_frame_write_on_write( p_ctx->settings.isp_base, 0 );
    acamera_isp_fr_uv_dma_writer_frame_write_on_write( p_ctx->settings.isp_base, 0 );
    acamera_isp_fr_dma_writer_bank0_base_write( p_ctx->settings.isp_base, 0 );
    acamera_isp_fr_uv_dma_writer_bank0_base_write( p_ctx->settings.isp_base, 0 );
#if ISP_HAS_DS1
    acamera_isp_ds1_dma_writer_frame_write_on_write( p_ctx->settings.isp_base, 0 );
    acamera_isp_ds1_uv_dma_writer_frame_write_on_write( p_ctx->settings.isp_base, 0 );
    acamera_isp_ds1_dma_writer_bank0_base_write( p_ctx->settings.isp_base, 0 );
    acamera_isp_ds1_uv_dma_writer_bank0_base_write( p_ctx->settings.isp_base, 0 );
#endif

    return result;
}


#if ISP_HAS_META_CB && defined( ISP_HAS_METADATA_FSM )
static void internal_callback_metadata( void *ctx, const firmware_metadata_t *fw_metadata )
{
    acamera_context_ptr_t p_ctx = (acamera_context_ptr_t)ctx;

    if ( p_ctx->settings.callback_meta != NULL ) {
        p_ctx->settings.callback_meta( p_ctx->context_id, fw_metadata );
    }
}
#endif

#if ISP_HAS_RAW_CB && ISP_DMA_RAW_CAPTURE
//static void internal_callback_raw( void* ctx, tframe_t *tframe, const metadata_t *metadata )
static void internal_callback_raw( void *ctx, aframe_t *aframe, const metadata_t *metadata, uint8_t exposures_num )
{
    acamera_context_ptr_t p_ctx = (acamera_context_ptr_t)ctx;

    if ( p_ctx->settings.callback_raw != NULL ) {
        p_ctx->settings.callback_raw( p_ctx->context_id, aframe, metadata, exposures_num );
    }
}
#endif

#if defined( ISP_HAS_DMA_WRITER_FSM )
static void internal_callback_fr( void *ctx, tframe_t *tframe, const metadata_t *metadata )
{
    acamera_context_ptr_t p_ctx = (acamera_context_ptr_t)ctx;

    if ( p_ctx->settings.callback_fr != NULL ) {
        p_ctx->settings.callback_fr( p_ctx->context_id, tframe, metadata );
    }
}
#endif

#if ISP_HAS_DS1 && defined( ISP_HAS_DMA_WRITER_FSM )
// Callback from DS1 output pipe
static void internal_callback_ds1( void *ctx, tframe_t *tframe, const metadata_t *metadata )
{

    acamera_context_ptr_t p_ctx = (acamera_context_ptr_t)ctx;
    if ( p_ctx->settings.callback_ds1 != NULL ) {
        p_ctx->settings.callback_ds1( p_ctx->context_id, tframe, metadata );
    }
}
#endif

#if ISP_HAS_DS2
// Callback from DS2 output pipe
static void external_callback_ds2( void *ctx, tframe_t *tframe, const metadata_t *metadata )
{

    acamera_context_ptr_t p_ctx = (acamera_context_ptr_t)ctx;
    if ( p_ctx->settings.callback_ds2 != NULL ) {
        p_ctx->settings.callback_ds2( p_ctx->context_id, tframe, metadata );
    }
}
#endif

static void configure_all_frame_buffers( acamera_context_ptr_t p_ctx )
{

#if ISP_HAS_WDR_FRAME_BUFFER
    acamera_isp_frame_stitch_frame_buffer_frame_write_on_write( p_ctx->settings.isp_base, 0 );
    aframe_t *frame_stitch_frames = p_ctx->settings.fs_frames;
    uint32_t frame_stitch_frames_num = p_ctx->settings.fs_frames_number;
    if ( frame_stitch_frames != NULL && frame_stitch_frames_num != 0 ) {
        if ( frame_stitch_frames_num == 1 ) {
            LOG( LOG_INFO, "Only one output buffer will be used for frame_stitch." );
            acamera_isp_frame_stitch_frame_buffer_bank0_base_write( p_ctx->settings.isp_base, frame_stitch_frames[0].address );
            acamera_isp_frame_stitch_frame_buffer_bank1_base_write( p_ctx->settings.isp_base, frame_stitch_frames[0].address );
            acamera_isp_frame_stitch_frame_buffer_line_offset_write( p_ctx->settings.isp_base, frame_stitch_frames[0].line_offset );
        } else {
            // double buffering is enabled
            acamera_isp_frame_stitch_frame_buffer_bank0_base_write( p_ctx->settings.isp_base, frame_stitch_frames[0].address );
            acamera_isp_frame_stitch_frame_buffer_bank1_base_write( p_ctx->settings.isp_base, frame_stitch_frames[1].address );
            acamera_isp_frame_stitch_frame_buffer_line_offset_write( p_ctx->settings.isp_base, frame_stitch_frames[0].line_offset );
        }

        acamera_isp_frame_stitch_frame_buffer_frame_write_on_write( p_ctx->settings.isp_base, 1 );
        acamera_isp_frame_stitch_frame_buffer_axi_port_enable_write( p_ctx->settings.isp_base, 1 );

    } else {
        acamera_isp_frame_stitch_frame_buffer_frame_write_on_write( p_ctx->settings.isp_base, 0 );
        acamera_isp_frame_stitch_frame_buffer_axi_port_enable_write( p_ctx->settings.isp_base, 0 );
        LOG( LOG_ERR, "No output buffers for frame_stitch block provided in settings. frame_stitch wdr buffer is disabled" );
    }
#endif


#if ISP_HAS_META_CB && defined( ISP_HAS_METADATA_FSM )
    acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_META_REGISTER_CB, internal_callback_metadata, sizeof( metadata_callback_t ) );
#endif


#if ISP_HAS_RAW_CB && ISP_DMA_RAW_CAPTURE
    dma_raw_capture_regist_callback( p_ctx->p_gfw, internal_callback_raw );
#endif

#if defined( ISP_HAS_DMA_WRITER_FSM )

    fsm_param_dma_pipe_setting_t pipe_fr;

    pipe_fr.pipe_id = dma_fr;
    pipe_fr.buf_array = p_ctx->settings.fr_frames;
    pipe_fr.buf_len = p_ctx->settings.fr_frames_number;
    pipe_fr.callback = internal_callback_fr;
    acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_DMA_PIPE_SETTING, &pipe_fr, sizeof( pipe_fr ) );

    acamera_isp_fr_dma_writer_format_write( p_ctx->settings.isp_base, FW_OUTPUT_FORMAT );
    acamera_isp_fr_uv_dma_writer_format_write( p_ctx->settings.isp_base, FW_OUTPUT_FORMAT_SECONDARY );
#endif


#if ISP_HAS_DS1 && defined( ISP_HAS_DMA_WRITER_FSM )

    fsm_param_dma_pipe_setting_t pipe_ds1;

    pipe_ds1.pipe_id = dma_ds1;
    pipe_ds1.buf_array = p_ctx->settings.ds1_frames;
    pipe_ds1.buf_len = p_ctx->settings.ds1_frames_number;
    pipe_ds1.callback = internal_callback_ds1;
    acamera_fsm_mgr_set_param( &p_ctx->fsm_mgr, FSM_PARAM_SET_DMA_PIPE_SETTING, &pipe_ds1, sizeof( pipe_ds1 ) );

    acamera_isp_ds1_dma_writer_format_write( p_ctx->settings.isp_base, FW_OUTPUT_FORMAT );
    acamera_isp_ds1_uv_dma_writer_format_write( p_ctx->settings.isp_base, FW_OUTPUT_FORMAT_SECONDARY );

#endif

#if ISP_HAS_DS2
    am_sc_set_callback(p_ctx, external_callback_ds2);
#endif

}

void acamera_fw_get_sensor_name(uint32_t *sname)
{
    acamera_command(TSENSOR, SENSOR_NAME, 0, COMMAND_GET, sname);
    if (sname == NULL) {
        LOG(LOG_ERR, "Error input param\n");
    }
}

static void init_stab( acamera_context_ptr_t p_ctx )
{
    p_ctx->stab.global_freeze_firmware = 0;
    p_ctx->stab.global_manual_exposure = 0;

    p_ctx->stab.global_manual_iridix = 0;
    p_ctx->stab.global_manual_sinter = 0;
    p_ctx->stab.global_manual_temper = 0;
    p_ctx->stab.global_manual_awb = 0;
    p_ctx->stab.global_manual_saturation = 0;
    p_ctx->stab.global_manual_auto_level = 0;
    p_ctx->stab.global_manual_frame_stitch = 0;
    p_ctx->stab.global_manual_raw_frontend = 0;
    p_ctx->stab.global_manual_black_level = 0;
    p_ctx->stab.global_manual_shading = 0;
    p_ctx->stab.global_manual_demosaic = 0;
    p_ctx->stab.global_manual_cnr = 0;
    p_ctx->stab.global_manual_sharpen = 0;

    p_ctx->stab.global_exposure = 0;
    p_ctx->stab.global_long_integration_time = 0;
    p_ctx->stab.global_short_integration_time = 0;
    p_ctx->stab.global_manual_exposure_ratio = SYSTEM_MANUAL_EXPOSURE_RATIO_DEFAULT;
    p_ctx->stab.global_exposure_ratio = SYSTEM_EXPOSURE_RATIO_DEFAULT;

    p_ctx->stab.global_maximum_iridix_strength = SYSTEM_MAXIMUM_IRIDIX_STRENGTH_DEFAULT;
    p_ctx->stab.global_minimum_iridix_strength = SYSTEM_MINIMUM_IRIDIX_STRENGTH_DEFAULT;
    p_ctx->stab.global_iridix_strength_target = 0;
    p_ctx->stab.global_sinter_threshold_target = 0;
    p_ctx->stab.global_temper_threshold_target = 0;
    p_ctx->stab.global_awb_red_gain = 256;
    p_ctx->stab.global_awb_blue_gain = 256;
    p_ctx->stab.global_saturation_target = 0;
    p_ctx->stab.global_ae_compensation = SYSTEM_AE_COMPENSATION_DEFAULT;
    p_ctx->stab.global_calibrate_bad_pixels = 0;
}


extern void *get_system_ctx_ptr( void );

#if USER_MODULE

int32_t acamera_init_context( acamera_context_t *p_ctx, acamera_settings *settings, acamera_firmware_t *g_fw )
{
    int32_t result = 0;
    // keep the context pointer for debug purposes
    p_ctx->context_ref = (uint32_t *)p_ctx;
    p_ctx->p_gfw = g_fw;

    // copy settings
    system_memcpy( (void *)&p_ctx->settings, (void *)settings, sizeof( acamera_settings ) );

    // each context is initialized to the default state
    p_ctx->isp_sequence = p_isp_data;

    // reset frame counters
    p_ctx->isp_frame_counter_raw = 0;
    p_ctx->isp_frame_counter = 0;

    acamera_fw_init( p_ctx );

    init_stab( p_ctx );

    p_ctx->initialized = 1;

    return result;
}

#else


int32_t acamera_init_context( acamera_context_t *p_ctx, acamera_settings *settings, acamera_firmware_t *g_fw )
{
    int32_t result = 0;
    // keep the context pointer for debug purposes
    p_ctx->context_ref = (uint32_t *)p_ctx;
    p_ctx->p_gfw = g_fw;
    if ( p_ctx->sw_reg_map.isp_sw_config_map != NULL ) {

        LOG( LOG_INFO, "Allocated memory for config space of size %d bytes", ACAMERA_ISP1_SIZE );
        LOG( LOG_INFO, "Allocated memory for metering of size %d bytes", ACAMERA_METERING_STATS_MEM_SIZE );
        // copy settings
        system_memcpy( (void *)&p_ctx->settings, (void *)settings, sizeof( acamera_settings ) );


        p_ctx->settings.isp_base = (uintptr_t)p_ctx->sw_reg_map.isp_sw_config_map;

        // each context is initialized to the default state
        p_ctx->isp_sequence = p_isp_data;

        acamera_load_isp_sequence( 0, p_ctx->isp_sequence, SENSOR_ISP_SEQUENCE_DEFAULT_SETTINGS );

#if defined( SENSOR_ISP_SEQUENCE_DEFAULT_SETTINGS_FPGA ) && ISP_HAS_FPGA_WRAPPER
        // these settings are loaded only for ARM FPGA demo platform and must be ignored on other systems
        acamera_load_isp_sequence( 0, p_ctx->isp_sequence, SENSOR_ISP_SEQUENCE_DEFAULT_SETTINGS_FPGA );
#endif

#if ISP_DMA_RAW_CAPTURE
        dma_raw_capture_init( g_fw );
#endif

        // reset frame counters
        p_ctx->isp_frame_counter_raw = 0;
        p_ctx->isp_frame_counter = 0;

        acamera_fw_init( p_ctx );

        acamera_init_context_seq(p_ctx);

        configure_all_frame_buffers( p_ctx );

        init_stab( p_ctx );

        // the custom initialization may be required for a context
        if ( p_ctx->settings.custom_initialization != NULL ) {
            p_ctx->settings.custom_initialization( p_ctx->context_id );
        }

        acamera_isp_input_port_mode_request_write( p_ctx->settings.isp_base, ACAMERA_ISP_INPUT_PORT_MODE_REQUEST_SAFE_START );

        p_ctx->initialized = 1;

    } else {
        result = -1;
        LOG( LOG_CRIT, "Failed to allocate memory for ISP config context" );
    }


    return result;
}
#endif

void acamera_deinit_context( acamera_context_t *p_ctx )
{
    acamera_fw_deinit( p_ctx );
}

void acamera_general_interrupt_hanlder( acamera_context_ptr_t p_ctx, uint8_t event )
{
#ifdef CALIBRATION_INTERRUPTS
    uint32_t *interrupt_counter = _GET_UINT_PTR( p_ctx, CALIBRATION_INTERRUPTS );
    interrupt_counter[event]++;
#endif


    p_ctx->irq_flag++;

    if ( event == ACAMERA_IRQ_FRAME_START ) {
        p_ctx->frame++;
    }

    if ( event == ACAMERA_IRQ_FRAME_END ) {
        // Update frame counter
        p_ctx->isp_frame_counter++;
        LOG( LOG_DEBUG, "Meta frame counter = %d", (int)p_ctx->isp_frame_counter );

#if ISP_DMA_RAW_CAPTURE
        p_ctx->isp_frame_counter_raw++;
#endif

// check frame counter sync when there is raw callback
#if ISP_HAS_RAW_CB
        if ( p_ctx->isp_frame_counter_raw != p_ctx->isp_frame_counter ) {
            LOG( LOG_DEBUG, "Sync frame counter : raw = %d, meta = %d",
                 (int)p_ctx->isp_frame_counter_raw, (int)p_ctx->isp_frame_counter );
            p_ctx->isp_frame_counter = p_ctx->isp_frame_counter_raw;
        }
#endif

        acamera_fw_raise_event( p_ctx, event_id_frame_end );

#if defined( ACAMERA_ISP_PROFILING ) && ( ACAMERA_ISP_PROFILING == 1 )
        acamera_profiler_new_frame();
#endif
    }

    if ( ( p_ctx->stab.global_freeze_firmware == 0 )
         || (event == ACAMERA_IRQ_FRAME_DROP_FR) || (event == ACAMERA_IRQ_FRAME_DROP_DS)
#if defined( ISP_HAS_DMA_WRITER_FSM )
         || ( event == ACAMERA_IRQ_FRAME_WRITER_FR ) // process interrupts for frame buffer anyway (otherwise picture will be frozen)
         || ( event == ACAMERA_IRQ_FRAME_WRITER_DS ) // process interrupts for frame buffer anyway (otherwise picture will be frozen)
#endif
#if defined( ISP_HAS_CMOS_FSM )
         || ( event == ACAMERA_IRQ_FRAME_START ) || ( event == ACAMERA_IRQ_FPGA_FRAME_END ) // process interrupts for FS anyway (otherwise exposure will be only short)
#endif
#if defined( ISP_HAS_BSP_TEST_FSM )
         || event == ACAMERA_IRQ_FRAME_END || event == ACAMERA_IRQ_FRAME_START
#endif
         ) {
        // firmware not frozen
        acamera_fsm_mgr_process_interrupt( &p_ctx->fsm_mgr, event );
    }

    p_ctx->irq_flag--;
}
