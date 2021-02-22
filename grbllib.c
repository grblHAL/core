/*
  grbllib.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "hal.h"
#include "nuts_bolts.h"
#include "tool_change.h"
#include "override.h"
#include "protocol.h"
#include "limits.h"
#include "report.h"
#include "state_machine.h"
#include "nvs_buffer.h"
#ifdef ENABLE_BACKLASH_COMPENSATION
#include "motion_control.h"
#endif
#ifdef KINEMATICS_API
#include "kinematics.h"
#endif

#ifdef COREXY
#include "corexy.h"
#endif

#ifdef WALL_PLOTTER
#include "wall_plotter.h"
#endif

// Declare system global variable structure
system_t sys = {0};
grbl_t grbl;
grbl_hal_t hal;

// called from stream drivers while tx is blocking, return false to terminate

static bool stream_tx_blocking (void)
{
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.

    grbl.on_execute_realtime(state_get());

    return !(sys.rt_exec_state & EXEC_RESET);
}

#ifdef KINEMATICS_API

kinematics_t kinematics;

// called from mc_line() to segment lines if not overridden, default implementation for pass-through
static bool kinematics_segment_line (float *target, plan_line_data_t *pl_data, bool init)
{
    static uint_fast8_t iterations;

    if(init)
        iterations = 2;
    else
        iterations--;

    return iterations != 0;
}

#endif

#ifdef DEBUGOUT
static void debug_out (bool on)
{
    // NOOP
}
#endif

void dummy_bool_handler (bool arg)
{
    // NOOP
}

// main entry point

int grbl_enter (void)
{
#ifdef N_TOOLS
    assert(NVS_ADDR_GLOBAL + sizeof(settings_t) + NVS_CRC_BYTES < NVS_ADDR_TOOL_TABLE);
#else
    assert(NVS_ADDR_GLOBAL + sizeof(settings_t) + NVS_CRC_BYTES < NVS_ADDR_PARAMETERS);
#endif
    assert(NVS_ADDR_PARAMETERS + N_CoordinateSystems * (sizeof(coord_data_t) + NVS_CRC_BYTES) < NVS_ADDR_STARTUP_BLOCK);
    assert(NVS_ADDR_STARTUP_BLOCK + N_STARTUP_LINE * (sizeof(stored_line_t) + NVS_CRC_BYTES) < NVS_ADDR_BUILD_INFO);

    bool looping = true, driver_ok;

    // Clear all and set some core function pointers
    memset(&grbl, 0, sizeof(grbl_t));
    grbl.on_execute_realtime = protocol_execute_noop;
    grbl.protocol_enqueue_gcode = protocol_enqueue_gcode;
    grbl.on_report_options = dummy_bool_handler;

    // Clear all and set some HAL function pointers
    memset(&hal, 0, sizeof(grbl_hal_t));
    hal.version = HAL_VERSION; // Update when signatures and/or contract is changed - driver_init() should fail
    hal.driver_reset = dummy_handler;
    hal.irq_enable = dummy_handler;
    hal.irq_disable = dummy_handler;
    hal.nvs.size = GRBL_NVS_SIZE;
    hal.stream.enqueue_realtime_command = protocol_enqueue_realtime_command;
    hal.limits.interrupt_callback = limit_interrupt_handler;
    hal.control.interrupt_callback = control_interrupt_handler;
    hal.stepper.interrupt_callback = stepper_driver_interrupt_handler;
    hal.stream_blocking_callback = stream_tx_blocking;
    hal.signals_cap.reset = hal.signals_cap.feed_hold = hal.signals_cap.cycle_start = On;

    sys.cold_start = true;

#ifdef BUFFER_NVSDATA
    nvs_buffer_alloc(); // Allocate memory block for NVS buffer
#endif

    report_init_fns();

#ifdef KINEMATICS_API
    memset(&kinematics, 0, sizeof(kinematics_t));

    kinematics.segment_line = kinematics_segment_line; // default to no segmentation
#endif

#ifdef DEBUGOUT
    hal.debug_out = debug_out; // must be overridden by driver to have any effect
#endif
    driver_ok = driver_init();

#if COMPATIBILITY_LEVEL > 0
    hal.stream.suspend_read = NULL;
#endif

#ifndef ENABLE_SAFETY_DOOR_INPUT_PIN
    hal.signals_cap.safety_door_ajar = Off;
#else
    driver_ok &= hal.signals_cap.safety_door_ajar;
#endif

  #ifdef BUFFER_NVSDATA
    nvs_buffer_init();
  #endif
    settings_init(); // Load Grbl settings from non-volatile storage

    memset(sys.position, 0, sizeof(sys.position)); // Clear machine position.

// check and configure driver

#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    driver_ok = driver_ok && hal.driver_cap.amass_level >= MAX_AMASS_LEVEL;
    hal.driver_cap.amass_level = MAX_AMASS_LEVEL;
#else
    hal.driver_cap.amass_level = 0;
#endif

#ifdef DEFAULT_STEP_PULSE_DELAY
    driver_ok = driver_ok & hal.driver_cap.step_pulse_delay;
#endif
/*
#if AXIS_N_SETTINGS > 4
    driver_ok = driver_ok & hal.driver_cap.axes >= AXIS_N_SETTINGS;
#endif
*/
    sys.mpg_mode = false;

    driver_ok = driver_ok && hal.driver_setup(&settings);

#ifdef ENABLE_SPINDLE_LINEARIZATION
    driver_ok = driver_ok && hal.driver_cap.spindle_pwm_linearization;
#endif

#ifdef SPINDLE_PWM_DIRECT
    driver_ok = driver_ok && hal.spindle.get_pwm != NULL && hal.spindle.update_pwm != NULL;
#endif

    if(!driver_ok) {
        hal.stream.write("grblHAL: incompatible driver" ASCII_EOL);
        while(true);
    }

    if(hal.get_position)
        hal.get_position(&sys.position); // TODO:  restore on abort when returns true?

#ifdef COREXY
    corexy_init();
#endif

#ifdef WALL_PLOTTER
    wall_plotter_init();
#endif

    sys.driver_started = true;

    // "Wire" homing switches to limit switches if not provided by the driver.
    if(hal.homing.get_state == NULL)
        hal.homing.get_state = hal.limits.get_state;

    // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
    // will return to this loop to be cleanly re-initialized.
    while(looping) {

        // Reset report entry points
        report_init_fns();

        if(!sys.position_lost || settings.homing.flags.keep_on_reset)
            memset(&sys, 0, offsetof(system_t, homed)); // Clear system variables except alarm & homed status.
        else
            memset(&sys, 0, offsetof(system_t, alarm)); // Clear system variables except state & alarm.

        sys.override.feed_rate = DEFAULT_FEED_OVERRIDE;          // Set to 100%
        sys.override.rapid_rate = DEFAULT_RAPID_OVERRIDE;        // Set to 100%
        sys.override.spindle_rpm = DEFAULT_SPINDLE_RPM_OVERRIDE; // Set to 100%

        if(settings.parking.flags.enabled)
            sys.override.control.parking_disable = settings.parking.flags.deactivate_upon_init;

        flush_override_buffers();

        // Reset Grbl primary systems.
        hal.stream.reset_read_buffer(); // Clear input stream buffer
        gc_init();                      // Set g-code parser to default state
        hal.limits.enable(settings.limits.flags.hard_enabled, false);
        plan_reset();                   // Clear block buffer and planner variables
        st_reset();                     // Clear stepper subsystem variables.
        limits_set_homing_axes();       // Set axes to be homed from settings.
#ifdef ENABLE_BACKLASH_COMPENSATION
        mc_backlash_init();             // Init backlash configuration.
#endif
        // Sync cleared gcode and planner positions to current system position.
        sync_position();

        if(hal.stepper.disable_motors)
            hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);

        if(!hal.driver_cap.atc)
            tc_init();

        // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
        report_init_message();

        if(state_get() == STATE_ESTOP)
            state_set(STATE_ALARM);

        if(hal.driver_cap.mpg_mode)
            hal.stream.enqueue_realtime_command(sys.mpg_mode ? CMD_STATUS_REPORT_ALL : CMD_STATUS_REPORT);

        // Start Grbl main loop. Processes program inputs and executes them.
        if(!(looping = protocol_main_loop()))
            looping = hal.driver_release == NULL || hal.driver_release();

        sys.cold_start = false;
    }

    return 0;
}
