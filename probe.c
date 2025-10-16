/*
  probe.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Part of grblHAL

  Copyright (c) 2025 Terje Io

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

#include "hal.h"

typedef struct {
    probe_id_t probe_id;
    probe_flags_t flags;
    probeflags_t inverted_bit;
    uint8_t port;
    void *input;
    get_probe_input_ptr get_input;
} probe_t;

static probe_state_t probe_state = { .connected = On };
static probe_t probes[N_PROBE_MAX], *probe = NULL;

static void probe_irq_handler (uint8_t port, bool state)
{
    if(probe_state.is_probing) {
        probe_state.triggered = On;
        if(!probe_state.was_probing) {
            probe_state.was_probing = true;
        }
    } else {
        control_signals_t signals = { .mask = hal.control.get_state().mask };
        signals.probe_triggered = On;
        hal.control.interrupt_callback(signals);
    }
}

// Toggle probe connected status.
static void probe_connected_toggle (void)
{
    if(!probe_state.is_probing) {
        if((probe->flags.connected = probe_state.connected = !probe_state.connected)) {
            if(probe->flags.watchable && settings.probe.enable_protection)
                probe->flags.guarded = probe_state.irq_enabled = ioport_enable_irq(probe->port, IRQ_Mode_Change, probe_irq_handler);
        } else if(probe->flags.guarded && ioport_enable_irq(probe->port, IRQ_Mode_None, probe_irq_handler))
            probe->flags.guarded = probe_state.irq_enabled = Off;

        if(settings.probe.enable_protection)
            system_add_rt_report(Report_ProbeProtect);
    }
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probe_configure (bool is_probe_away, bool probing)
{
    bool invert = !!(settings.probe.value & probe->inverted_bit.value);

    probe_state.inverted = is_probe_away ? !invert : invert;

    if(probing) {
        if(probe->flags.latchable) {
            probe_state.is_probing = probe_state.was_probing = Off;
            probe_state.triggered = hal.probe.get_state().triggered;
            probe_state.irq_mode = probe_state.triggered ? IRQ_Mode_None : (probe_state.inverted ? IRQ_Mode_Falling : IRQ_Mode_Rising);
        }
    } else
        probe_state.irq_mode = probe->flags.connected && probe->flags.watchable && settings.probe.enable_protection ? IRQ_Mode_Change : IRQ_Mode_None;

    if(ioport_enable_irq(probe->port, probe_state.irq_mode, probe_irq_handler)) {
        probe_state.irq_enabled = probe_state.irq_mode != IRQ_Mode_None;
        probe->flags.guarded = !probing && probe_state.irq_enabled;
    }

    if(settings.probe.enable_protection)
        system_add_rt_report(Report_ProbeProtect);

    if(!probe_state.irq_enabled)
        probe_state.triggered = Off;

    probe_state.is_probing = probing;
}

static bool probe_select (probe_id_t probe_id)
{
    uint_fast8_t idx = 0;
    probe_t *selected_probe = NULL;

    do {
        if(probes[idx].probe_id == probe_id && probes[idx].get_input)
            selected_probe = &probes[idx];
    } while(selected_probe == NULL && ++idx < N_PROBE_MAX);

    if(!probe_state.is_probing && selected_probe && selected_probe != probe) {

        if(probe_state.irq_mode != IRQ_Mode_None)
            ioport_enable_irq(probe->port, IRQ_Mode_None, probe_irq_handler);

        probe = selected_probe;
        hal.probe.configure(false, false);
    }

    return probe == selected_probe;
}

static probe_flags_t probe_get_caps (probe_id_t probe_id)
{
    uint_fast8_t idx = 0;
    probe_t *probe = NULL;

    do {
        if(probes[idx].probe_id == probe_id && probes[idx].get_input)
            probe = &probes[idx];
    } while(probe == NULL && ++idx < N_PROBE_MAX);

    return probe ? probe->flags : (probe_flags_t){0};
}

static bool is_triggered (probe_id_t probe_id)
{
    uint_fast8_t idx = 0;
    probe_t *probe = NULL;

    do {
        if(probes[idx].probe_id == probe_id && probes[idx].get_input)
            probe = &probes[idx];
    } while(probe == NULL && ++idx < N_PROBE_MAX);

    return !!probe && probe->get_input(probe->input) ^ !!(settings.probe.value & probe->inverted_bit.value);
}

// Returns the probe connected and triggered pin states.
static probe_state_t get_state (void)
{
    probe_state_t state = { .value = probe_state.value };

    state.probe_id  = probe->probe_id;
    state.connected = probe->flags.connected;

    if(probe_state.is_probing && probe_state.irq_enabled)
        state.triggered = probe_state.triggered;
    else
        state.triggered = probe->get_input(probe->input) ^ probe_state.inverted;

    return state;
}

bool probe_add (probe_id_t probe_id, uint8_t port, pin_irq_mode_t irq_mode, void *input, get_probe_input_ptr get_input)
{
    static uint_fast8_t n_probes = 0;

    if(get_input == NULL || n_probes >= N_PROBE_MAX)
        return false;

    bool can_latch;

    if(!(can_latch = (irq_mode & IRQ_Mode_RisingFalling) == IRQ_Mode_RisingFalling))
        hal.signals_cap.probe_triggered = Off;
    else if(n_probes == 0)
        hal.signals_cap.probe_triggered = On;

    probes[n_probes].probe_id = probe_id;
    probes[n_probes].port = port;
    probes[n_probes].flags.available = On;
    probes[n_probes].flags.connected = probe_state.connected;
    probes[n_probes].flags.latchable = can_latch;
    probes[n_probes].flags.watchable = !!(irq_mode & IRQ_Mode_Change);
    probes[n_probes].input = input;
    probes[n_probes].get_input = get_input;

    switch(probe_id) {

        case Probe_Toolsetter:
            probes[n_probes].inverted_bit.invert_toolsetter_input = On;
            break;

        case Probe_2:
            probes[n_probes].inverted_bit.invert_probe2_input = On;
            break;

        default: // Probe_Default
            probes[n_probes].inverted_bit.invert_probe_pin = On;
            break;
    }

    hal.driver_cap.probe_pull_up = On;
    hal.probe.configure = probe_configure;
    hal.probe.connected_toggle = probe_connected_toggle;
    hal.probe.get_caps = probe_get_caps;
    hal.probe.get_state = get_state;
    hal.probe.is_triggered = is_triggered;

    if(probe == NULL || probe_id == Probe_Default)
        probe = &probes[n_probes];

    if(++n_probes == 2)
        hal.probe.select = probe_select;

    return true;
}

void probe_connected_event (void *data)
{
    if(hal.probe.connected_toggle) {
        if((uint32_t)data == 2)
            hal.probe.connected_toggle();
        else {
            probe_state_t state = hal.probe.get_state();

            if(state.probe_id == Probe_Default && hal.driver_cap.probe && state.connected != !!data)
                hal.probe.connected_toggle();
        }
    }
}
