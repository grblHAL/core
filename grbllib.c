/*
  grbllib.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "hal.h"
#include "nuts_bolts.h"
#include "tool_change.h"
#include "override.h"
#include "protocol.h"
#include "machine_limits.h"
#include "report.h"
#include "state_machine.h"
#include "nvs_buffer.h"
#include "stream.h"
#if NGC_EXPRESSIONS_ENABLE
#include "ngc_expr.h"
#endif
#if ENABLE_BACKLASH_COMPENSATION
#include "motion_control.h"
#endif
#ifdef KINEMATICS_API
#include "kinematics.h"
#endif

#if COREXY
#include "kinematics/corexy.h"
#endif

#if WALL_PLOTTER
#include "kinematics/wall_plotter.h"
#endif

#if DELTA_ROBOT
#include "kinematics/delta.h"
#endif

#if POLAR_ROBOT
#include "kinematics/polar.h"
#endif

static void task_execute (sys_state_t state);

typedef union {
    uint8_t ok;
    struct {
        uint8_t init          :1,
                setup         :1,
                spindle       :1,
                amass         :1,
                pulse_delay   :1,
                unused        :3;
    };
} driver_startup_t;

#ifndef CORE_TASK_POOL_SIZE
#define CORE_TASK_POOL_SIZE 40
#endif

typedef struct core_task {
    uint32_t time;
    foreground_task_ptr fn;
    void *data;
    struct core_task *next;
} core_task_t;

DCRAM system_t sys; //!< System global variable structure.
DCRAM grbl_t grbl;
DCRAM grbl_hal_t hal;

DCRAM static core_task_t task_pool[CORE_TASK_POOL_SIZE];
static driver_startup_t driver = { .ok = 0xFF };
static core_task_t *next_task = NULL, *immediate_task = NULL, *on_booted = NULL, *systick_task = NULL, *last_freed = NULL;
static on_linestate_changed_ptr on_linestate_changed;
static settings_changed_ptr hal_settings_changed;

#ifdef KINEMATICS_API
kinematics_t kinematics;
#endif

__attribute__((weak)) void board_ports_init (void)
{
    // NOOP
}

__attribute__((always_inline)) static inline void task_free (core_task_t *task)
{
    task->fn = NULL;
    task->next = NULL;
    if(last_freed == NULL)
        last_freed = task;
}

__attribute__((always_inline)) static inline core_task_t *task_run (core_task_t *task)
{
    core_task_t *t = task;
    foreground_task_ptr fn = task->fn;
    void *data = task->data;

    task = task->next;
    task_free(t);
    fn(data);

    return task;
}

void dummy_bool_handler (bool arg)
{
    // NOOP
}

void reset_handler (void)
{
    report_init_fns();

    grbl.on_macro_return = NULL;
}

static bool dummy_irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr callback)
{
    return false;
}

static void report_driver_error (void *data)
{
    char msg[40];

    driver.ok = ~driver.ok;
    strcpy(msg, "Fatal: Incompatible driver (");
    strcat(msg, uitoa(driver.ok));
    strcat(msg, ")");

    report_message(msg, Message_Plain);
}

static void auto_realtime_report (void *data);

static void realtime_report_check (void *data)
{
    task_add_delayed(sys.flags.auto_reporting ? auto_realtime_report : realtime_report_check, NULL, settings.report_interval);
}

static void auto_realtime_report (void *data)
{
    if(sys.flags.auto_reporting) {
        system_set_exec_state_flag(EXEC_STATUS_REPORT);
        task_add_delayed(auto_realtime_report, NULL, settings.report_interval);
    } else if(settings.report_interval)
        task_add_delayed(realtime_report_check, NULL, settings.report_interval);
}

// "Wire" homing signals to limit signals, used when max limit inputs not available.
ISR_CODE static home_signals_t ISR_FUNC(get_homing_status)(void)
{
    home_signals_t home;
    limit_signals_t limits = hal.limits.get_state();

    home.a.value = limits.min.value;
    home.b.value = limits.min2.value;

    return home;
}

// "Wire" homing signals to limit signals, used when max limit inputs available.
ISR_CODE static home_signals_t ISR_FUNC(get_homing_status2)(void)
{
    home_signals_t home;
    limit_signals_t source = xbar_get_homing_source(), limits = hal.limits.get_state();

    home.a.value = (limits.min.value & source.min.mask) | (limits.max.value & source.max.mask);
    home.b.value = (limits.min2.value & source.min2.mask) | (limits.max2.value & source.max2.mask);

    return home;
}

static void output_welcome_message (void *data)
{
    grbl.report.init_message(hal.stream.write);
}

static void onLinestateChanged (serial_linestate_t state)
{
    if(state.dtr) {
        task_delete(output_welcome_message, NULL);
        task_add_delayed(output_welcome_message, NULL, 200);
    }

    if(on_linestate_changed)
        on_linestate_changed(state);
}

static void print_pos_msg (void *data)
{
    hal.stream.write("grblHAL: power on self-test (POS) failed!" ASCII_EOL);

    if(on_booted) do {
    } while((on_booted = task_run(on_booted)));
}

static void onPosFailure (serial_linestate_t state)
{
    if(state.dtr) // delay a bit to let the USB stack come up
        task_add_delayed(print_pos_msg, NULL, 50);
}

static bool onProbeToolsetter (tool_data_t *tool, coord_data_t *position, bool at_g59_3, bool on)
{
    bool ok = false;

    if(at_g59_3 && settings.probe.toolsetter_auto_select)
        ok = hal.probe.select(on ? Probe_Toolsetter : Probe_Default);

    return ok;
}

static void tool_changed (tool_data_t *tool)
{
    if(settings.flags.tool_persistent && tool->tool_id != settings.tool_id) {
        settings.tool_id = tool->tool_id;
        settings_write_global();
    }
}

static void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    hal_settings_changed(settings, changed);

    if(grbl.on_settings_changed)
        grbl.on_settings_changed(settings, changed);
}

static atc_status_t atc_get_state (void)
{
    return hal.driver_cap.atc ? ATC_Online : ATC_None;
}

// main entry point

int grbl_enter (void)
{
    assert(NVS_ADDR_PARAMETERS + N_CoordinateSystems * (sizeof(coord_data_t) + NVS_CRC_BYTES) < NVS_ADDR_STARTUP_BLOCK);
    assert(NVS_ADDR_STARTUP_BLOCK + N_STARTUP_LINE * (sizeof(stored_line_t) + NVS_CRC_BYTES) < NVS_ADDR_BUILD_INFO);

    bool looping = true;

    memset(&sys, 0, sizeof(system_t));
    memset(&task_pool, 0, sizeof(task_pool));

    // Clear all and set some core function pointers
    memset(&grbl, 0, sizeof(grbl_t));
    grbl.on_execute_realtime = grbl.on_execute_delay = task_execute;
    grbl.enqueue_gcode = protocol_enqueue_gcode;
    grbl.enqueue_realtime_command = stream_enqueue_realtime_command;
    grbl.on_report_options = dummy_bool_handler;
    grbl.on_report_command_help = system_command_help;
    grbl.on_get_alarms = alarms_get_details;
    grbl.on_get_errors = errors_get_details;
    grbl.on_get_settings = settings_get_details;
    grbl.on_tool_changed = tool_changed;
#if NGC_EXPRESSIONS_ENABLE
    grbl.on_process_gcode_comment = ngc_process_comment;
#endif

    // Clear all and set some HAL function pointers
    memset(&hal, 0, sizeof(grbl_hal_t));
    hal.version = HAL_VERSION; // Update when signatures and/or contract is changed - driver_init() should fail
    hal.driver_reset = reset_handler;
    hal.irq_enable = dummy_handler;
    hal.irq_disable = dummy_handler;
    hal.irq_claim = dummy_irq_claim;
    hal.nvs.size = GRBL_NVS_SIZE;
    hal.step_us_min = 2.0f;
    hal.coolant_cap.flood = On;
    hal.limits.interrupt_callback = limit_interrupt_handler;
    hal.control.interrupt_callback = control_interrupt_handler;
    hal.stepper.interrupt_callback = stepper_driver_interrupt_handler;
    hal.stream_blocking_callback = stream_tx_blocking;
    hal.tool.atc_get_state = atc_get_state;
    hal.signals_pullup_disable_cap.value = (uint16_t)-1;

    sys.cold_start = true;

    limits_init();

    settings_clear();
    report_init_fns();

#ifdef KINEMATICS_API
    memset(&kinematics, 0, sizeof(kinematics_t));
#endif

    driver.init = driver_init();

#if NVSDATA_BUFFER_ENABLE
    nvs_buffer_alloc(); // Allocate memory block for NVS buffer
#endif

#ifdef DEBUGOUT
    debug_stream_init();
#endif

#if COMPATIBILITY_LEVEL > 0
    hal.stream.suspend_read = NULL;
#endif

#ifdef NO_SAFETY_DOOR_SUPPORT
    hal.signals_cap.safety_door_ajar = Off;
#endif

#if COREXY
    corexy_init();
#endif

#if WALL_PLOTTER
    wall_plotter_init();
#endif

#if DELTA_ROBOT
    delta_robot_init();
#endif

#if POLAR_ROBOT
    polar_init();
#endif

  #if NVSDATA_BUFFER_ENABLE
    nvs_buffer_init();
  #endif
    settings_init(); // Load settings from non-volatile storage

    memset(sys.position, 0, sizeof(sys.position)); // Clear machine position.

// check and configure driver

#if ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    driver.amass = hal.driver_cap.amass_level >= MAX_AMASS_LEVEL;
    hal.driver_cap.amass_level = MAX_AMASS_LEVEL;
#else
    hal.driver_cap.amass_level = 0;
#endif

#ifdef DEFAULT_STEP_PULSE_DELAY
    driver.pulse_delay = hal.driver_cap.step_pulse_delay;
#endif
/*
#if AXIS_N_SETTINGS > 4
    driver_ok = driver_ok & hal.driver_cap.axes >= AXIS_N_SETTINGS;
#endif
*/
    sys.mpg_mode = false;

    if((sys.ioinit_pending = driver.ok == 0xFF)) {

        hal_settings_changed = hal.settings_changed;
        hal.settings_changed = settings_changed;

        driver.setup = hal.driver_setup(&settings);

        sys.ioinit_pending = false;
    }

    spindle_id_t spindle_id, encoder_spindle;

// Sanity checks
    if(!spindle_get_id(settings.spindle.ref_id, &spindle_id)) {

        spindle_ptrs_t *spindle = spindle_get_hal(spindle_get_default(), SpindleHAL_Raw);

        spindle_id = spindle ? spindle->id : 0;
        settings.spindle.ref_id = spindle ? spindle->ref_id : DEFAULT_SPINDLE;
    }

    if(!spindle_get_id(settings.spindle.encoder_spindle, &encoder_spindle))
        settings.spindle.encoder_spindle = settings.spindle.ref_id;
//

    if((driver.spindle = spindle_select(spindle_id))) {
        spindle_ptrs_t *spindle = spindle_get(0);
        driver.spindle = spindle->get_pwm == NULL || spindle->update_pwm != NULL;
    } else
        driver.spindle = spindle_select(spindle_add_null());

    if(!hal.driver_cap.sd_card)
        settings.macro_atc_flags.error_on_no_macro = Off;

    if(driver.ok != 0xFF) {
        sys.alarm = Alarm_SelftestFailed;
        task_run_on_startup(report_driver_error, NULL);
    }

    hal.stepper.enable(settings.steppers.energize, true);

    spindle_all_off();
    hal.coolant.set_state((coolant_state_t){0});

    if(hal.get_position)
        hal.get_position(&sys.position); // TODO: restore on abort when returns true?

#if ENABLE_BACKLASH_COMPENSATION
    mc_backlash_init((axes_signals_t){AXES_BITMASK});
#endif

    sys.driver_started = sys.alarm != Alarm_SelftestFailed;

    // "Wire" homing switches to limit switches if not provided by the driver.
    if(hal.homing.get_state == NULL || settings.homing.flags.use_limit_switches)
        hal.homing.get_state = hal.limits_cap.max.mask ? get_homing_status2 : get_homing_status;

    if(settings.report_interval)
        task_add_delayed(auto_realtime_report, NULL, settings.report_interval);

    if(hal.driver_cap.sd_card || hal.driver_cap.littlefs) {
        fs_options_t fs_options = { .hierarchical_listing = On };
        fs_options.lfs_hidden = hal.driver_cap.littlefs;
        fs_options.sd_mount_on_boot = hal.driver_cap.sd_card;
        setting_remove_elements(Setting_FSOptions, fs_options.mask);
    }

    if(hal.stream.state.linestate_event && !hal.stream.state.passthru) {
        on_linestate_changed = hal.stream.on_linestate_changed;
        hal.stream.on_linestate_changed = onLinestateChanged;
    }

    if(grbl.on_probe_toolsetter == NULL && hal.driver_cap.toolsetter && hal.probe.select)
        grbl.on_probe_toolsetter = onProbeToolsetter;

    if(hal.driver_cap.probe && hal.signals_cap.probe_disconnected)
        task_run_on_startup(probe_connected_event, hal.control.get_state().probe_disconnected ? NULL : (void *)1);

    // Initialization loop upon power-up or a system abort. For the latter, all processes
    // will return to this loop to be cleanly re-initialized.
    while(looping) {

        spindle_num_t spindle_num = N_SYS_SPINDLE;

        // Reset report entry points
        report_init_fns();

        overrides_t override;

        memcpy(&override, &sys.override, sizeof(overrides_t));

        if(!sys.position_lost || settings.homing.flags.keep_on_reset)
            memset(&sys, 0, offsetof(system_t, homed)); // Clear system variables except alarm & homed status.
        else
            memset(&sys, 0, offsetof(system_t, alarm)); // Clear system variables except state & alarm.

        sys.var5399 = -2;                               // Clear last M66 result
        sys.override.feed_rate = sys.cold_start || !settings.flags.keep_feed_override_on_reset ? DEFAULT_FEED_OVERRIDE : override.feed_rate;
        sys.override.rapid_rate = sys.cold_start || !settings.flags.keep_rapids_override_on_reset ? DEFAULT_RAPID_OVERRIDE : override.rapid_rate;
        do {
            if(spindle_is_enabled(--spindle_num))
                spindle_get(spindle_num)->param->override_pct = DEFAULT_SPINDLE_RPM_OVERRIDE; // Set to 100%
        } while(spindle_num);
        sys.flags.auto_reporting = settings.report_interval != 0;

        if(settings.parking.flags.enabled)
            sys.override.control.parking_disable = settings.parking.flags.deactivate_upon_init;

        flush_override_buffers();

        // Reset primary systems.
        hal.stream.reset_read_buffer();                 // Clear input stream buffer
        gc_init(settings.flags.keep_offsets_on_reset);  // Set g-code parser to default state
        hal.limits.enable(settings.limits.flags.hard_enabled, (axes_signals_t){0});
        plan_reset();                                   // Clear block buffer and planner variables
        st_reset();                                     // Clear stepper subsystem variables.
        limits_set_homing_axes();                       // Set axes to be homed from settings.
        system_init_switches();                         // Set switches from inputs.

        // Sync cleared gcode and planner positions to current system position.
        sync_position();

        if(hal.stepper.disable_motors)
            hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);

        if(!hal.driver_cap.atc)
            tc_init();

        // Print welcome message. Indicates an initialization has occurred at power-up or with a reset.
        grbl.report.init_message(hal.stream.write_all);

        if(!settings.flags.no_unlock_after_estop && state_get() == STATE_ESTOP)
            state_set(STATE_ALARM);

        if(hal.driver_cap.mpg_mode)
            protocol_enqueue_realtime_command(sys.mpg_mode ? CMD_STATUS_REPORT_ALL : CMD_STATUS_REPORT);

        // Start main loop. Processes program inputs and executes them.
        if(!(looping = protocol_main_loop()))
            looping = hal.driver_release == NULL || hal.driver_release();

        sys.cold_start = false;
    }

    nvs_buffer_free();

    return 0;
}

__attribute__((always_inline)) static inline core_task_t *task_alloc (void)
{
    core_task_t *task = NULL;
    uint_fast8_t idx = CORE_TASK_POOL_SIZE;

    if(last_freed) {
        task = last_freed;
        last_freed = NULL;
    } else do {
        if(task_pool[--idx].fn == NULL)
            task = &task_pool[idx];
    } while(task == NULL && idx);

    return task;
}

static void task_execute (sys_state_t state)
{
    static uint32_t last_ms = 0;

    core_task_t *task;

    if(immediate_task && sys.driver_started) {

        hal.irq_disable();
        if((task = immediate_task))
            immediate_task = NULL;
        hal.irq_enable();

        if(task) do {
        } while((task = task_run(task)));
    }

    uint32_t now = hal.get_elapsed_ticks();
    if(now == last_ms || next_task == systick_task)
        return;

    last_ms = now;

    if((task = systick_task)) do {
        task->fn(task->data);
    } while((task = task->next));

    while((task = next_task) && (int32_t)(task->time - now) <= 0) {

        hal.irq_disable();

        if(task == next_task)
            next_task = task->next;
        else {
            core_task_t *t;
            if((t = next_task)) {
                while(t->next && t->next != task)
                    t = t->next;
                if(t->next && t->next == task)
                    t->next = task->next;
            }
        }

        hal.irq_enable();

        void *data = task->data;
        foreground_task_ptr fn = task->fn;
        task_free(task);

        fn(data);
    }
}

ISR_CODE bool ISR_FUNC(task_add_delayed)(foreground_task_ptr fn, void *data, uint32_t delay_ms)
{
    core_task_t *task = NULL;

    hal.irq_disable();

    if(fn && (task = task_alloc())) {

        task->time = hal.get_elapsed_ticks() + delay_ms;
        task->fn = fn;
        task->data = data;
        task->next = NULL;

        if(next_task == NULL)
            next_task = task;
        else if((int32_t)(task->time - next_task->time) <= 0) {
            task->next = next_task;
            next_task = task;
        } else {
            core_task_t *t = next_task;
            while(t) {
                if(t->next == NULL || (int32_t)(task->time - t->next->time) < 0) {
                    task->next = t->next;
                    t->next = task;
                    break;
                }
                t = t->next;
            }
        }
    }

    hal.irq_enable();

    return task != NULL;
}

ISR_CODE void task_delete (foreground_task_ptr fn, void *data)
{
    core_task_t *task, *prev = NULL;

    hal.irq_disable();

    if((task = next_task)) do {
        if(fn == task->fn && (data == NULL || data == task->data)) {
            if(prev)
                prev->next = task->next;
            else
                next_task = task->next;
            task_free(task);
            break;
        }
        prev = task;
    } while((task = task->next));

    hal.irq_enable();
}

ISR_CODE bool ISR_FUNC(task_add_systick)(foreground_task_ptr fn, void *data)
{
    core_task_t *task = NULL;

    hal.irq_disable();

    if(fn && (task = task_alloc())) {

        task->fn = fn;
        task->data = data;
        task->next = NULL;

        if(systick_task == NULL)
            systick_task = task;
        else {
            core_task_t *t = systick_task;
            while(t->next)
                t = t->next;
            t->next = task;
        }
    }

    hal.irq_enable();

    return task != NULL;
}

void task_delete_systick (foreground_task_ptr fn, void *data)
{
    core_task_t *task, *prev = NULL;

    hal.irq_disable();

    if((task = systick_task)) do {
        if(fn == task->fn && data == task->data) {
            if(prev)
                prev->next = task->next;
            else
                systick_task = task->next;
            task_free(task);
            break;
        }
        prev = task;
    } while((task = task->next));

    hal.irq_enable();
}

/*! \brief Enqueue a function to be called once by the foreground process.
\param fn pointer to a \a foreground_task_ptr type of function.
\param data pointer to data to be passed to the callee.
\returns true if successful, false otherwise.
*/
ISR_CODE bool ISR_FUNC(task_add_immediate)(foreground_task_ptr fn, void *data)
{
    core_task_t *task = NULL;

    hal.irq_disable();

    if(fn && (task = task_alloc())) {

        task->fn = fn;
        task->data = data;
        task->next = NULL;

        if(immediate_task == NULL)
            immediate_task = task;
        else {
            core_task_t *t = immediate_task;
            while(t->next)
                t = t->next;
            t->next = task;
        }
    }

    hal.irq_enable();

    return task != NULL;
}

/*! \brief Enqueue a function to be called once by the foreground process after the boot sequence is completed.
\param fn pointer to a \a foreground_task_ptr type of function.
\param data pointer to data to be passed to the callee.
\returns true if successful, false otherwise.
*/
ISR_CODE bool ISR_FUNC(task_run_on_startup)(foreground_task_ptr fn, void *data)
{
    if(sys.cold_start) {

        core_task_t *task = NULL;

        hal.irq_disable();

        if(fn && (task = task_alloc())) {

            task->fn = fn;
            task->data = data;
            task->next = NULL;

            if(on_booted == NULL)
                on_booted = task;
            else {
                core_task_t *t = on_booted;
                while(t->next)
                    t = t->next;
                t->next = task;
            }
        }

        hal.irq_enable();

        return task != NULL;

    } else
        return task_add_immediate(fn, data); // TODO: for now, to be removed...
}

// for core use only, called once from protocol.c on cold start
void task_execute_on_startup (void)
{
    if(!sys.driver_started) {

        // Clear task queues except startup warnings.

        core_task_t *task, *prev = NULL;

        if((task = on_booted)) do {
            if(!(task->fn == report_warning)) {
                if(prev)
                    prev->next = task->next;
                else {
                    prev = NULL;
                    on_booted = task->next;
                }
                task_free(task);
            } else
                prev = task;
        } while((task = prev ? prev->next : on_booted));

        while(next_task)
            task_delete(next_task->fn, NULL);

        while(systick_task)
            task_delete_systick(systick_task->fn, NULL);
    }

    if(on_booted && (sys.driver_started || !hal.stream.state.linestate_event)) do {
    } while((on_booted = task_run(on_booted)));

    if(!sys.driver_started) {

        if(hal.stream.state.linestate_event)
            hal.stream.on_linestate_changed = onPosFailure;

        while(true)
            grbl.on_execute_realtime(state_get());
    }
}
