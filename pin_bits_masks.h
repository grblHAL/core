/*
  pin_bits_masks.h - for adding bit definitions and masks

  NOTE: This file is not used by the core, it may be used by drivers

  Part of grblHAL

  Copyright (c) 2021-2024 Terje Io

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

// Control input signals

// Define the CONTROL_PORT symbol as a shorthand in the *_map.h file if all control inputs share the same port.
#ifdef CONTROL_PORT

#ifndef RESET_PORT
#define RESET_PORT CONTROL_PORT
#endif
#ifndef FEED_HOLD_PORT
#define FEED_HOLD_PORT CONTROL_PORT
#endif
#ifndef CYCLE_START_PORT
#define CYCLE_START_PORT CONTROL_PORT
#endif
#ifndef ESTOP_PORT
#define ESTOP_PORT CONTROL_PORT
#endif
#ifndef PROBE_DISCONNECT_PORT
#define PROBE_DISCONNECT_PORT CONTROL_PORT
#endif
#ifndef STOP_DISABLE_PORT
#define STOP_DISABLE_PORT CONTROL_PORT
#endif
#ifndef BLOCK_DELETE_PORT
#define BLOCK_DELETE_PORT CONTROL_PORT
#endif
#ifndef SINGLE_BLOCK_PORT
#define SINGLE_BLOCK_PORT CONTROL_PORT
#endif
#ifndef MOTOR_FAULT_PORT
#define MOTOR_FAULT_PORT CONTROL_PORT
#endif
#ifndef MOTOR_WARNING_PORT
#define MOTOR_WARNING_PORT CONTROL_PORT
#endif
#ifndef LIMITS_OVERRIDE_PORT
#define LIMITS_OVERRIDE_PORT CONTROL_PORT
#endif
#if SAFETY_DOOR_ENABLE && !defined(SAFETY_DOOR_PORT)
#define SAFETY_DOOR_PORT CONTROL_PORT
#endif

#endif // CONTROL_PORT

#ifndef RESET_BIT
#ifdef RESET_PIN
#define RESET_BIT       (1<<RESET_PIN)
#else
#define RESET_BIT       0
#endif
#endif

#ifndef FEED_HOLD_BIT
#ifdef FEED_HOLD_PIN
#define FEED_HOLD_BIT   (1<<FEED_HOLD_PIN)
#else
#define FEED_HOLD_BIT   0
#endif
#endif

#ifndef CYCLE_START_BIT
#ifdef CYCLE_START_PIN
#define CYCLE_START_BIT (1<<CYCLE_START_PIN)
#else
#define CYCLE_START_BIT 0
#endif
#endif

#ifndef ESTOP_BIT
#ifdef ESTOP_PIN
#define ESTOP_BIT (1<<ESTOP_PIN)
#else
#define ESTOP_BIT 0
#endif
#endif

// Optional control signals

#ifndef SAFETY_DOOR_BIT
#if defined(SAFETY_DOOR_PIN) && !SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_BIT (1<<SAFETY_DOOR_PIN)
#else
#define SAFETY_DOOR_BIT 0
#if !defined(SAFETY_DOOR_PIN) && SAFETY_DOOR_ENABLE
#warning "SAFETY_DOOR_PIN is not defined!"
#endif
#endif
#endif

#ifndef MOTOR_FAULT_BIT
#if defined(MOTOR_FAULT_PIN) && !MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_BIT (1<<MOTOR_FAULT_PIN)
#else
#define MOTOR_FAULT_BIT 0
#if !defined(MOTOR_FAULT_PIN) && MOTOR_FAULT_ENABLE
#warning "MOTOR_FAULT_PIN is not defined!"
#endif
#endif
#endif

#ifndef MOTOR_WARNING_BIT
#if defined(MOTOR_WARNING_PIN) && !MOTOR_WARNING_ENABLE
#define MOTOR_WARNING_BIT (1<<MOTOR_WARNING_PIN)
#else
#define MOTOR_WARNING_BIT 0
#if !defined(MOTOR_WARNING_PIN) && MOTOR_WARNING_ENABLE
#warning "MOTOR_WARNING_PIN is not defined!"
#endif
#endif
#endif

#ifndef PROBE_DISCONNECT_BIT
#if defined(PROBE_DISCONNECT_PIN) && !PROBE_DISCONNECT_ENABLE
#define PROBE_DISCONNECT_BIT (1<<PROBE_DISCONNECT_PIN)
#else
#define PROBE_DISCONNECT_BIT 0
#endif
#endif

#ifndef STOP_DISABLE_BIT
#if defined(STOP_DISABLE_PIN) && !STOP_DISABLE_ENABLE
#define STOP_DISABLE_BIT (1<<STOP_DISABLE_PIN)
#else
#define STOP_DISABLE_BIT 0
#endif
#endif

#ifndef BLOCK_DELETE_BIT
#if defined(BLOCK_DELETE_PIN) && !BLOCK_DELETE_ENABLE
#define BLOCK_DELETE_BIT (1<<BLOCK_DELETE_PIN)
#else
#define BLOCK_DELETE_BIT 0
#endif
#endif

#ifndef SINGLE_BLOCK_BIT
#if defined(SINGLE_BLOCK_PIN) && !SINGLE_BLOCK_ENABLE
#define SINGLE_BLOCK_BIT (1<<SINGLE_BLOCK_PIN)
#else
#define SINGLE_BLOCK_BIT 0
#endif
#endif

#ifndef LIMITS_OVERRIDE_BIT
#if defined(LIMITS_OVERRIDE_PIN) && !LIMITS_OVERRIDE_ENABLE
#define LIMITS_OVERRIDE_BIT (1<<LIMITS_OVERRIDE_PIN)
#else
#define LIMITS_OVERRIDE_BIT 0
#endif
#endif

#if SAFETY_DOOR_ENABLE || MOTOR_FAULT_ENABLE || MOTOR_WARNING_ENABLE || PROBE_DISCONNECT_ENABLE || \
    STOP_DISABLE_ENABLE || BLOCK_DELETE_ENABLE || SINGLE_BLOCK_ENABLE || LIMITS_OVERRIDE_ENABLE || defined __DOXYGEN__

#define AUX_CONTROLS_ENABLED 1

#if PROBE_DISCONNECT_ENABLE || STOP_DISABLE_ENABLE || BLOCK_DELETE_ENABLE || SINGLE_BLOCK_ENABLE || LIMITS_OVERRIDE_ENABLE
#define AUX_CONTROLS_SCAN    3 // start index for scanned inputs
#else
#define AUX_CONTROLS_SCAN    0
#endif

#if SAFETY_DOOR_ENABLE || MOTOR_FAULT_ENABLE || MOTOR_WARNING_ENABLE
#define AUX_CONTROLS_XMAP    1
#else
#define AUX_CONTROLS_XMAP    0
#endif

static aux_ctrl_t aux_ctrl[] = {
    { .enabled = SAFETY_DOOR_ENABLE, .port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .safety_door_ajar = On }, .function = Input_SafetyDoor },
    { .enabled = MOTOR_FAULT_ENABLE, .port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .motor_fault = On }, .function = Input_MotorFault },
    { .enabled = MOTOR_WARNING_ENABLE, .port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .motor_warning = On }, .function = Input_MotorWarning },
    { .enabled = PROBE_DISCONNECT_ENABLE, .port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .motor_fault = On }, .function = Input_ProbeDisconnect },
    { .enabled = STOP_DISABLE_ENABLE, .port = 0xFF, .irq_mode = IRQ_Mode_Change, .cap = { .stop_disable = On }, .function = Input_StopDisable },
    { .enabled = BLOCK_DELETE_ENABLE, .port = 0xFF, .irq_mode = IRQ_Mode_Change, .cap = { .block_delete = On }, .function = Input_BlockDelete },
    { .enabled = SINGLE_BLOCK_ENABLE, .port = 0xFF, .irq_mode = IRQ_Mode_Change, .cap = { .single_block = On }, .function = Input_SingleBlock },
    { .enabled = LIMITS_OVERRIDE_ENABLE, .port = 0xFF, .irq_mode = IRQ_Mode_None, .cap = { .limits_override = On }, .function = Input_LimitsOverride }
};

#else
#define AUX_CONTROLS_ENABLED 0
#define AUX_CONTROLS_SCAN    0
#endif

//

#ifndef CONTROL_MASK
#if SAFETY_DOOR_ENABLE
#define CONTROL_MASK (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|ESTOP_BIT|PROBE_DISCONNECT_BIT|STOP_DISABLE_BIT|BLOCK_DELETE_BIT|SINGLE_BLOCK_BIT|MOTOR_FAULT_BIT|MOTOR_WARNING_BIT|LIMITS_OVERRIDE_BIT|SAFETY_DOOR_BIT)
#define CONTROL_MASK_SUM (RESET_BIT+FEED_HOLD_BIT+CYCLE_START_BIT+ESTOP_BIT+PROBE_DISCONNECT_BIT+STOP_DISABLE_BIT+BLOCK_DELETE_BIT+SINGLE_BLOCK_BIT+MOTOR_FAULT_BIT+MOTOR_WARNING_BIT+LIMITS_OVERRIDE_BIT+SAFETY_DOOR_BIT)
#else
#define CONTROL_MASK (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|ESTOP_BIT|PROBE_DISCONNECT_BIT|STOP_DISABLE_BIT|BLOCK_DELETE_BIT|SINGLE_BLOCK_BIT|MOTOR_FAULT_BIT|MOTOR_WARNING_BIT|LIMITS_OVERRIDE_BIT)
#define CONTROL_MASK_SUM (RESET_BIT+FEED_HOLD_BIT+CYCLE_START_BIT+ESTOP_BIT+PROBE_DISCONNECT_BIT+STOP_DISABLE_BIT+BLOCK_DELETE_BIT+SINGLE_BLOCK_BIT+MOTOR_FAULT_BIT+MOTOR_WARNING_BIT+LIMITS_OVERRIDE_BIT)
#endif
#endif

// Probe input signal

#if defined(PROBE_PIN) && !defined(PROBE_BIT)
#define PROBE_BIT (1<<PROBE_PIN)
#endif

// Output Signals

#if defined(SPINDLE_ENABLE_PIN) && !defined(SPINDLE_ENABLE_BIT)
#define SPINDLE_ENABLE_BIT (1<<SPINDLE_ENABLE_PIN)
#endif
#if defined(SPINDLE_DIRECTION_PIN) && !defined(SPINDLE_DIRECTION_BIT)
#define SPINDLE_DIRECTION_BIT (1<<SPINDLE_DIRECTION_PIN)
#endif

#if defined(COOLANT_FLOOD_PIN) && !defined(COOLANT_FLOOD_BIT)
#define COOLANT_FLOOD_BIT (1<<COOLANT_FLOOD_PIN)
#endif
#if defined(COOLANT_MIST_PIN) && !defined(COOLANT_MIST_BIT)
#define COOLANT_MIST_BIT (1<<COOLANT_MIST_PIN)
#endif

#if defined(RTS_PIN) && !defined(RTS_BIT)
#define RTS_BIT (1<<RTS_PIN)
#endif

// IRQ enabled input singnals

#ifndef MPG_MODE_PIN
#define MPG_MODE_BIT 0
#elif !defined(MPG_MODE_BIT)
#define MPG_MODE_BIT (1<<MPG_MODE_PIN)
#endif

#ifndef I2C_STROBE_PIN
#define I2C_STROBE_BIT 0
#elif !defined(I2C_STROBE_BIT)
#define I2C_STROBE_BIT (1<<I2C_STROBE_PIN)
#endif

#ifndef SPI_IRQ_PIN
#define SPI_IRQ_BIT 0
#elif !defined(SPI_IRQ_BIT)
#define SPI_IRQ_BIT (1<<SPI_IRQ_PIN)
#endif

#if SPINDLE_SYNC_ENABLE
  #if defined(SPINDLE_INDEX_PIN) && defined(SPINDLE_PULSE_PIN)
    #ifndef SPINDLE_INDEX_BIT
      #define SPINDLE_INDEX_BIT (1<<SPINDLE_INDEX_PIN)
    #endif
    #ifndef SPINDLE_PULSE_BIT
      #define SPINDLE_PULSE_BIT (1<<SPINDLE_PULSE_PIN)
    #endif
  #else
    #define SPINDLE_INDEX_BIT 0
    #define SPINDLE_PULSE_BIT 0
    #error "Spindle sync requires SPINDLE_PULSE_PIN and SPINDLE_INDEX_PIN defined in the board map!"
  #endif
#else
  #define SPINDLE_INDEX_BIT 0
  #define SPINDLE_PULSE_BIT 0
#endif

#if QEI_ENABLE
#ifndef QEI_A_BIT
#define QEI_A_BIT (1<<QEI_A_PIN)
#endif
#ifndef QEI_B_BIT
#define QEI_B_BIT (1<<QEI_B_PIN)
#endif
#ifndef QEI_SELECT_PIN
#define QEI_SELECT_BIT 0
#elif !defined(QEI_SELECT_BIT)
#define QEI_SELECT_BIT (1<<QEI_SELECT_PIN)
#endif
#else
#define QEI_A_BIT 0
#define QEI_B_BIT 0
#define QEI_SELECT_BIT 0
#endif

#ifndef DEVICES_IRQ_MASK
#define DEVICES_IRQ_MASK (MPG_MODE_BIT|I2C_STROBE_BIT|SPI_IRQ_BIT|SPINDLE_INDEX_BIT|QEI_A_BIT|QEI_B_BIT|QEI_SELECT_BIT)
#define DEVICES_IRQ_MASK_SUM (MPG_MODE_BIT+I2C_STROBE_BIT+SPI_IRQ_BIT+SPINDLE_INDEX_BIT+QEI_A_BIT+QEI_B_BIT+QEI_SELECT_BIT)
#endif

// Auxillary input signals

#ifdef AUXINPUT0_PIN
#define AUXINPUT0_BIT (1<<AUXINPUT0_PIN)
#else
#define AUXINPUT0_BIT 0
#endif
#ifdef AUXINPUT1_PIN
#define AUXINPUT1_BIT (1<<AUXINPUT1_PIN)
#else
#define AUXINPUT1_BIT 0
#endif
#ifdef AUXINPUT2_PIN
#define AUXINPUT2_BIT (1<<AUXINPUT2_PIN)
#else
#define AUXINPUT2_BIT 0
#endif
#ifdef AUXINPUT3_PIN
#define AUXINPUT3_BIT (1<<AUXINPUT3_PIN)
#else
#define AUXINPUT3_BIT 0
#endif
#ifdef AUXINPUT4_PIN
#define AUXINPUT4_BIT (1<<AUXINPUT4_PIN)
#else
#define AUXINPUT4_BIT 0
#endif
#ifdef AUXINPUT5_PIN
#define AUXINPUT5_BIT (1<<AUXINPUT5_PIN)
#else
#define AUXINPUT5_BIT 0
#endif
#ifdef AUXINPUT6_PIN
#define AUXINPUT6_BIT (1<<AUXINPUT6_PIN)
#else
#define AUXINPUT6_BIT 0
#endif
#ifdef AUXINPUT7_PIN
#define AUXINPUT7_BIT (1<<AUXINPUT7_PIN)
#else
#define AUXINPUT7_BIT 0
#endif

#ifndef AUXINPUT_MASK
#define AUXINPUT_MASK (AUXINPUT0_BIT|AUXINPUT1_BIT|AUXINPUT2_BIT|AUXINPUT3_BIT|AUXINPUT4_BIT|AUXINPUT5_BIT|AUXINPUT6_BIT|AUXINPUT7_BIT)
#define AUXINPUT_MASK_SUM (AUXINPUT0_BIT+AUXINPUT1_BIT+AUXINPUT2_BIT+AUXINPUT3_BIT+AUXINPUT4_BIT+AUXINPUT5_BIT+AUXINPUT6_BIT+AUXINPUT7_BIT)
#endif

/*EOF*/
