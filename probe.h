/*
  probe.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

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

#ifndef _PROBE_H_
#define _PROBE_H_

#include "crossbar.h"

#define N_PROBE_MAX 3

// Values that define the probing state machine.

typedef bool (*get_probe_input_ptr)(void *input);

typedef enum {
    Probing_Off = 0, //!< 0
    Probing_Active   //!< 1
} probing_state_t;

typedef enum {
    Probe_Default = 0, //!< 0
    Probe_Toolsetter,  //!< 1
    Probe_2            //!< 2
} probe_id_t;

typedef union {
    uint16_t value;
    struct {
        uint16_t triggered     :1, //!< Set to true when probe or toolsetter is triggered.
                 connected     :1, //!< Set to true when probe is connected. Always set to true if the driver does not have a probe connected input.
                 inverted      :1, //!< For driver use
                 is_probing    :1, //!< For driver use
                 was_probing   :1, //!< For driver use
                 irq_enabled   :1, //!< For driver use
                 irq_mode      :5, //!< pin_irq_mode_t - for driver use
                 unused        :3,
                 probe_id      :2; //!< Id of active probe (for now).
    };
} probe_state_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t available     :1, //!< Set to true when probe input is available.
                connected     :1, //!< Set to true when probe is connected. Always set to true if the driver does not have a probe connected input.
                latchable     :1, //!< Set to true when probe input supports change rising/falling.
                watchable     :1, //!< Set to true when probe input supports change interrupt.
                guarded       :1, //!< Set to true when probe protection is enabled.
                unassigned    :3;
    };
} probe_flags_t;

void probe_connected_event (void *data);
bool probe_add (probe_id_t probe_id, uint8_t port, pin_irq_mode_t irq_mode, void *input, get_probe_input_ptr get_input);

#endif // _PROBE_H_
