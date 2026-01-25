/*
  crossbar.h - signal crossbar functions

  Part of grblHAL

  Copyright (c) 2023-2025 Terje Io

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

static limit_signals_t home_source = {0};

axes_signals_t xbar_fn_to_axismask (pin_function_t fn)
{
    axes_signals_t mask = {0};

    switch(fn) {

        case Output_StepperEnable:
            mask.bits = AXES_BITMASK;
            break;

        case Output_StepperEnableXY:
            mask.x = mask.y = On;
            break;

#if defined(A_AXIS) || defined(B_AXIS)
        case Output_StepperEnableAB:
#ifdef A_AXIS
            mask.a = On;
#endif
#ifdef B_AXIS
            mask.b = On;
#endif
            break;
#endif

        case Input_LimitX:
        case Input_LimitX_Max:
        case Input_LimitX_2:
        case Input_HomeX:
        case Input_MotorFaultX:
        case Input_MotorFaultX_2:
        case Output_StepperEnableX:
            mask.x = On;
            break;

        case Input_LimitY:
        case Input_LimitY_Max:
        case Input_LimitY_2:
        case Input_HomeY:
        case Input_MotorFaultY:
        case Input_MotorFaultY_2:
        case Output_StepperEnableY:
            mask.y = On;
            break;

        case Input_LimitZ:
        case Input_LimitZ_Max:
        case Input_LimitZ_2:
        case Input_HomeZ:
        case Input_MotorFaultZ:
        case Input_MotorFaultZ_2:
        case Output_StepperEnableZ:
            mask.z = On;
            break;

#ifdef A_AXIS
        case Input_LimitA:
        case Input_LimitA_Max:
        case Input_HomeA:
        case Input_MotorFaultA:
        case Output_StepperEnableA:
            mask.a = On;
            break;
#endif
#ifdef B_AXIS
        case Input_LimitB:
        case Input_LimitB_Max:
        case Input_HomeB:
        case Input_MotorFaultB:
        case Output_StepperEnableB:
            mask.b = On;
            break;
#endif
#ifdef C_AXIS
        case Input_LimitC:
        case Input_LimitC_Max:
        case Input_HomeC:
        case Input_MotorFaultC:
        case Output_StepperEnableC:
            mask.c = On;
            break;
#endif
#ifdef U_AXIS
        case Input_LimitU:
        case Input_LimitU_Max:
        case Input_HomeU:
        case Input_MotorFaultU:
        case Output_StepperEnableU:
            mask.u = On;
            break;
#endif
#ifdef V_AXIS
        case Input_LimitV:
        case Input_LimitV_Max:
        case Input_HomeV:
        case Input_MotorFaultV:
        case Output_StepperEnableV:
            mask.v = On;
            break;
#endif
#ifdef W_AXIS
        case Input_LimitW:
        case Input_LimitW_Max:
        case Input_HomeW:
        case Input_MotorFaultW:
        case Output_StepperEnableW:
            mask.w = On;
            break;
#endif
        default:
            break;
    }

    return mask;
}

// Sets limit signals used by homing when home signals are not available.
// For internal use, called by settings.c when homing direction mask is changed.
void xbar_set_homing_source (void)
{
    if(hal.home_cap.a.mask == 0) {
        home_source.max.mask = hal.limits_cap.max.mask & ((~settings.homing.dir_mask.mask) & AXES_BITMASK);
        home_source.min.mask = (~home_source.max.mask) & AXES_BITMASK;
        home_source.max2.mask = hal.limits_cap.max2.mask & ((~settings.homing.dir_mask.mask) & AXES_BITMASK);
        home_source.min2.mask = (~home_source.max2.mask) & AXES_BITMASK;
    }
}

// Returns limit signals used by homing when home signals are not available.
ISR_CODE limit_signals_t xbar_get_homing_source (void)
{
    return home_source;
}

// Returns limit signals used by homing cycle when home signals are not available.
limit_signals_t xbar_get_homing_source_from_cycle (axes_signals_t homing_cycle)
{
    limit_signals_t source = home_source;

    if(hal.home_cap.a.mask == 0) {
        source.min.mask &= homing_cycle.mask;
        source.min2.mask &= homing_cycle.mask;
        source.min.mask |= source.min2.mask;
        source.max.mask &= homing_cycle.mask;
        source.max2.mask &= homing_cycle.mask;
        source.max.mask |= source.max2.mask;
    }

    return source;
}

const char *xbar_fn_to_pinname (pin_function_t fn)
{
    const char *name = NULL;
    uint_fast8_t idx = sizeof(pin_names) / sizeof(pin_name_t);

    do {
        if(pin_names[--idx].function == fn)
            name = pin_names[idx].name;
    } while(idx && !name);

    return name ? name : "N/A";
}

// Only returns description for UART groups
const char *xbar_group_to_description ( pin_group_t group)
{
    return group >= PinGroup_UART && group <= PinGroup_UART4 ? (const char * const[]){ "UART1", "UART2", "UART3", "UART4" }[group - PinGroup_UART] : NULL;
}

control_signals_t xbar_fn_to_signals_mask (pin_function_t fn)
{
    control_signals_t signals;

    signals.mask = fn >= Input_Probe ? 0 : 1 << (uint32_t)fn;

    return signals;
}

const char *xbar_resolution_to_string (pin_cap_t cap)
{
    return !cap.analog || cap.pwm || cap.servo_pwm ? "?" : ((const char * const[]){"4", "8", "10", "12", "14", "16", "18", "20", "24", "32"})[cap.resolution];
}
