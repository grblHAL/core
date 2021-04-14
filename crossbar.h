/*
  crossbar.h - signal crossbar definitions

  Not used by the core.

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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

#ifndef _CROSSBAR_H_
#define _CROSSBAR_H_

typedef enum {
    Input_Probe = 0,
    Input_Reset,
    Input_FeedHold,
    Input_CycleStart,
    Input_SafetyDoor,
    Input_LimitsOverride,
    Input_EStop,
    Input_ModeSelect,
    Input_LimitX,
    Input_LimitX_Max,
    Input_LimitY,
    Input_LimitY_Max,
    Input_LimitZ,
    Input_LimitZ_Max,
    Input_LimitA,
    Input_LimitA_Max,
    Input_LimitB,
    Input_LimitB_Max,
    Input_LimitC,
    Input_LimitC_Max,
    Input_KeypadStrobe,
    Input_QEI_A,
    Input_QEI_B,
    Input_QEI_Select,
    Input_QEI_Index,
    Input_SpindleIndex,
    Input_Aux0,
    Input_Aux1,
    Input_Aux2,
    Input_Aux3
} pin_function_t;

typedef enum {
    IRQ_Mode_None    = 0b00,
    IRQ_Mode_Change  = 0b01,
    IRQ_Mode_Rising  = 0b10,
    IRQ_Mode_Falling = 0b11
} pin_irq_mode_t;

typedef enum {
    PinGroup_Control       = (1<<0),
    PinGroup_Limit         = (1<<1),
    PinGroup_Probe         = (1<<2),
    PinGroup_Keypad        = (1<<3),
    PinGroup_MPG           = (1<<4),
    PinGroup_QEI           = (1<<5),
    PinGroup_QEI_Select    = (1<<6),
    PinGroup_SpindlePulse  = (1<<7),
    PinGroup_SpindleIndex  = (1<<8),
    PinGroup_AuxInput      = (1<<9),
    PinGroup_AuxOutput     = (1<<10)
} pin_group_t;

typedef bool (*xbar_get_value_ptr)(void);
typedef void (*xbar_set_value_ptr)(bool on);
typedef void (*xbar_event_ptr)(bool on);
typedef void (*xbar_config_ptr)(void *cfg_data);

typedef struct {
    uint32_t function;
    void *port;
    uint8_t bit;
    xbar_config_ptr config;
    xbar_get_value_ptr get_value;
    xbar_set_value_ptr set_value;
    xbar_event_ptr on_event;
} xbar_t;

#endif
