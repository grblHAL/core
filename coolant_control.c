/*
  coolant_control.c - coolant control methods

  Part of grblHAL

  Copyright (c) 2016-2023 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "protocol.h"
#include "coolant_control.h"
#include "state_machine.h"

// Main program only. Immediately sets flood coolant running state and also mist coolant,
// if enabled. Also sets a flag to report an update to a coolant state.
// Called by coolant toggle override, parking restore, parking retract, sleep mode, g-code
// parser program end, and g-code parser coolant_sync().
void coolant_set_state (coolant_state_t mode)
{
    if (!ABORTED) { // Block during abort.
        hal.coolant.set_state(mode);
        system_add_rt_report(Report_Coolant); // Set to report change immediately
    }
}

// G-code parser entry-point for setting coolant state. Forces a planner buffer sync and bails
// if an abort or check-mode is active.
bool coolant_sync (coolant_state_t mode)
{
    bool ok = true;
    if (state_get() != STATE_CHECK_MODE) {
        if((ok = protocol_buffer_synchronize())) // Ensure coolant changes state when specified in program.
            coolant_set_state(mode);
    }

    return ok;
}
