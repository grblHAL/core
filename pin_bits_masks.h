/*
  pin_bits_masks.h - for adding bit definitions and masks

  NOTE: This file is not used by the core, it may be used by drivers

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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

#include "platform.h"

// Sanity checks

#if PROBE_ENABLE && !defined(PROBE_PIN)
#error "Probe input is not supported in this configuration!"
#endif

#if SAFETY_DOOR_ENABLE && !defined(SAFETY_DOOR_PIN)
#error "Safety door input is not supported in this configuration!"
#endif

#if MOTOR_FAULT_ENABLE && !defined(MOTOR_FAULT_PIN)
#error "Motor fault input is not supported in this configuration!"
#endif

#if MOTOR_WARNING_ENABLE && !defined(MOTOR_WARNING_PIN)
#error "Motor warning input is not supported in this configuration!"
#endif

#if I2C_STROBE_ENABLE && !defined(I2C_STROBE_PIN)
#error "I2C keypad/strobe is not supported in this configuration!"
#endif

#if MPG_ENABLE == 1 && !defined(MPG_MODE_PIN)
#error "MPG mode input is not supported in this configuration!"
#endif

#if QEI_SELECT_ENABLE && !defined(QEI_SELECT_PIN)
#error "Encoder select input is not supported in this configuration!"
#endif

#define EXPANDER_PORT 1

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

#ifndef SD_DETECT_BIT
#ifdef SD_DETECT_PIN
#define SD_DETECT_BIT (1<<SD_DETECT_PIN)
#else
#define SD_DETECT_BIT 0
#endif
#endif

#ifndef CONTROL_ENABLE
#define CONTROL_ENABLE 0
#endif

static aux_ctrl_t aux_ctrl[] = {
// The following pins are bound explicitly to aux input pins.
#if (CONTROL_ENABLE & CONTROL_ESTOP) && defined(RESET_PIN)
 #ifndef RESET_PORT
  #define RESET_PORT 0
 #endif
    { .function = Input_EStop,           .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .e_stop = On }, .pin = RESET_PIN, .port = (void *)RESET_PORT },
#elif (CONTROL_ENABLE & CONTROL_RESET) && defined(RESET_PIN)
 #ifndef RESET_PORT
  #define RESET_PORT 0
 #endif
    { .function = Input_Reset,           .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .reset = On }, .pin = RESET_PIN, .port = (void *)RESET_PORT },
#endif
#if (CONTROL_ENABLE & CONTROL_FEED_HOLD) && defined(FEED_HOLD_PIN)
 #ifndef FEED_HOLD_PORT
  #define FEED_HOLD_PORT 0
 #endif
    { .function = Input_FeedHold,        .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .feed_hold = On }, .pin = FEED_HOLD_PIN, .port = (void *)FEED_HOLD_PORT },
#endif
#if (CONTROL_ENABLE & CONTROL_CYCLE_START) && defined(CYCLE_START_PIN)
 #ifndef CYCLE_START_PORT
  #define CYCLE_START_PORT 0
 #endif
    { .function = Input_CycleStart,      .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .cycle_start = On }, .pin = CYCLE_START_PIN, .port = (void *)CYCLE_START_PORT },
#endif
#if SAFETY_DOOR_ENABLE && defined(SAFETY_DOOR_PIN)
 #ifndef SAFETY_DOOR_PORT
  #define SAFETY_DOOR_PORT 0
 #endif
    { .function = Input_SafetyDoor,      .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .safety_door_ajar = On }, .pin = SAFETY_DOOR_PIN, .port = (void *)SAFETY_DOOR_PORT },
#endif
#if MOTOR_FAULT_ENABLE && defined(MOTOR_FAULT_PIN)
 #ifndef MOTOR_FAULT_PORT
  #define MOTOR_FAULT_PORT 0
 #endif
    { .function = Input_MotorFault,      .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .motor_fault = On }, .pin = MOTOR_FAULT_PIN, .port = (void *)MOTOR_FAULT_PORT },
#endif
#if MOTOR_WARNING_ENABLE && defined(MOTOR_WARNING_PIN)
 #ifndef MOTOR_WARNING_PORT
  #define MOTOR_WARNING_PORT 0
 #endif
    { .function = Input_MotorWarning,    .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .motor_fault = On }, .pin = MOTOR_WARNING_PIN, .port = (void *)MOTOR_WARNING_PORT },
#endif

#if I2C_STROBE_ENABLE && defined(I2C_STROBE_PIN)
 #ifndef I2C_STROBE_PORT
  #define I2C_STROBE_PORT 0
 #endif
    { .function = Input_I2CStrobe,       .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_Change,        .cap = { .value = 0 }, .pin = I2C_STROBE_PIN, .port = (void *)I2C_STROBE_PORT },
#endif
#if MPG_ENABLE == 1 && defined(MPG_MODE_PIN)
 #ifndef MPG_MODE_PORT
  #define MPG_MODE_PORT 0
 #endif
     { .function = Input_MPGSelect,      .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_Change,        .cap = { .value = 0 }, .pin = MPG_MODE_PIN, .port = (void *)MPG_MODE_PORT },
#endif
#if QEI_SELECT_ENABLE && defined(QEI_SELECT_PIN)
 #ifndef QEI_SELECT_PORT
  #define QEI_SELECT_PORT 0
 #endif
    { .function = Input_QEI_Select,      .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .value = 0 }, .pin = QEI_SELECT_PIN, .port = (void *)QEI_SELECT_PORT },
#endif

// Probe pins can be bound explicitly and can be "degraded" to not interrupt capable.
#if PROBE_ENABLE && defined(PROBE_PIN)
 #ifndef PROBE_PORT
  #define PROBE_PORT 0
 #endif
    { .function = Input_Probe,           .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .value = 0 }, .pin = PROBE_PIN, .port = (void *)PROBE_PORT },
#endif
#if PROBE2_ENABLE && defined(PROBE2_PIN)
 #ifndef PROBE2_PORT
  #define PROBE2_PORT 0
 #endif
    { .function = Input_Probe2,          .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .value = 0 }, .pin = PROBE2_PIN, .port = (void *)PROBE2_PORT },
#endif
#if TOOLSETTER_ENABLE && defined(TOOLSETTER_PIN)
 #ifndef TOOLSETTER_PORT
  #define TOOLSETTER_PORT 0
 #endif
    { .function = Input_Toolsetter,      .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .value = 0 }, .pin = TOOLSETTER_PIN, .port = (void *)TOOLSETTER_PORT },
#endif

// The following pins are allocated from remaining aux inputs pool
#if TOOLSETTER_ENABLE && !defined(TOOLSETTER_PIN)
    { .function = Input_Toolsetter,      .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .value = 0 }, .pin = 0xFE, .port = NULL },
#endif
#if PROBE2_ENABLE && !defined(PROBE2_PIN)
    { .function = Input_Probe2,          .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .value = 0 }, .pin = 0xFE, .port = NULL },
#endif
#if LIMITS_OVERRIDE_ENABLE
    { .function = Input_LimitsOverride,  .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_None,          .cap = { .limits_override = On }, .pin = 0xFF, .port = NULL },
#endif
#if STOP_DISABLE_ENABLE
    { .function = Input_StopDisable,     .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_Change,        .cap = { .stop_disable = On }, .pin = 0xFF, .port = NULL },
#endif
#if BLOCK_DELETE_ENABLE
    { .function = Input_BlockDelete,     .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_Change,        .cap = { .block_delete = On }, .pin = 0xFF, .port = NULL },
#endif
#if SINGLE_BLOCK_ENABLE
    { .function = Input_SingleBlock,     .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_Change,        .cap = { .single_block = On }, .pin = 0xFF, .port = NULL },
#endif
#if PROBE_DISCONNECT_ENABLE
    { .function = Input_ProbeDisconnect, .aux_port = IOPORT_UNASSIGNED, .irq_mode = IRQ_Mode_RisingFalling, .cap = { .probe_disconnected = On }, .pin = 0xFF, .port = NULL },
#endif
};

static inline bool aux_ctrl_is_probe (pin_function_t function)
{
    return function == Input_Probe || function == Input_Probe2 || function == Input_Toolsetter;
}

#ifdef STM32_PLATFORM

static inline aux_ctrl_t *aux_ctrl_get_fn (void *port, uint8_t pin)
{
    aux_ctrl_t *ctrl_pin = NULL;

    if(sizeof(aux_ctrl) / sizeof(aux_ctrl_t)) {
        uint_fast8_t idx;
        for(idx = 0; ctrl_pin == NULL && aux_ctrl[idx].pin != 0xFF && idx < sizeof(aux_ctrl) / sizeof(aux_ctrl_t); idx++) {
            if(aux_ctrl[idx].pin == pin && aux_ctrl[idx].port == port)
                ctrl_pin = &aux_ctrl[idx];
        }
    }

    return ctrl_pin;
}

#endif

static inline aux_ctrl_t *aux_ctrl_remap_explicit (void *port, uint8_t pin, uint8_t aux_port, void *input)
{
    aux_ctrl_t *ctrl_pin = NULL;

    if(sizeof(aux_ctrl) / sizeof(aux_ctrl_t)) {

        uint_fast8_t idx;

        for(idx = 0; ctrl_pin == NULL && idx < sizeof(aux_ctrl) / sizeof(aux_ctrl_t) && aux_ctrl[idx].pin != 0xFF; idx++) {
            if(aux_ctrl[idx].pin == pin && aux_ctrl[idx].port == port) {
                ctrl_pin = &aux_ctrl[idx];
                ctrl_pin->aux_port = aux_port;
                ctrl_pin->input = input;
                break;
            }
        }
    }

    return ctrl_pin;
}

static inline aux_ctrl_t *aux_ctrl_get_pin (uint8_t aux_port)
{
    aux_ctrl_t *ctrl_pin = NULL;

    uint_fast8_t idx = sizeof(aux_ctrl) / sizeof(aux_ctrl_t);

    if(idx) do {
        if(aux_ctrl[--idx].aux_port == aux_port)
            ctrl_pin = &aux_ctrl[idx];
    } while(idx && ctrl_pin == NULL);

    return ctrl_pin;
}

static inline void aux_ctrl_irq_enable (settings_t *settings, ioport_interrupt_callback_ptr aux_irq_handler)
{
    uint_fast8_t idx = sizeof(aux_ctrl) / sizeof(aux_ctrl_t);

    if(idx) do {
        if(aux_ctrl[--idx].aux_port != 0xFF && aux_ctrl[idx].irq_mode != IRQ_Mode_None) {
            if(!aux_ctrl_is_probe(aux_ctrl[idx].function)) {
                pin_irq_mode_t irq_mode;
                if((irq_mode = aux_ctrl[idx].irq_mode) & IRQ_Mode_RisingFalling)
                    irq_mode = (settings->control_invert.mask & aux_ctrl[idx].cap.mask) ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                hal.port.register_interrupt_handler(aux_ctrl[idx].aux_port, irq_mode, aux_irq_handler);
            }
        }
    } while(idx);
}

typedef bool (*aux_claim_explicit_ptr)(aux_ctrl_t *aux_ctrl);

static bool aux_ctrl_claim_port (xbar_t *properties, uint8_t port, void *data)
{
    if(ioport_claim(Port_Digital, Port_Input, &port, NULL)) {
        ((aux_ctrl_t *)data)->aux_port = port;
        ioport_set_function(properties, ((aux_ctrl_t *)data)->function, &((aux_ctrl_t *)data)->cap);
    }

    return ((aux_ctrl_t *)data)->aux_port != IOPORT_UNASSIGNED;
}

static bool aux_ctrl_find_port (xbar_t *properties, uint8_t port, void *data)
{
    ((aux_ctrl_t *)data)->aux_port = port;

    return true;
}

static inline void aux_ctrl_claim_ports (aux_claim_explicit_ptr aux_claim_explicit, ioports_enumerate_callback_ptr aux_claim)
{
    uint_fast8_t idx;

    if(aux_claim == NULL)
        aux_claim = aux_ctrl_claim_port;

    for(idx = 0; idx < sizeof(aux_ctrl) / sizeof(aux_ctrl_t); idx++) {

        pin_cap_t cap = { .irq_mode = aux_ctrl[idx].irq_mode, .claimable = On };

        if(aux_ctrl[idx].pin == 0xFE) // Toolsetter and Probe2
            ioports_enumerate(Port_Digital, Port_Input, cap, aux_ctrl_find_port, (void *)&aux_ctrl[idx]);
#ifdef STM32_PLATFORM
        if(aux_ctrl[idx].irq_mode == IRQ_Mode_None && !(aux_ctrl[idx].function == Input_Probe || aux_ctrl[idx].function == Input_LimitsOverride))
            continue;
#endif
        if(aux_ctrl[idx].pin == 0xFF) {
            if(ioports_enumerate(Port_Digital, Port_Input, cap, aux_claim, (void *)&aux_ctrl[idx]))
                hal.signals_cap.mask |= aux_ctrl[idx].cap.mask;
        } else if(aux_ctrl[idx].aux_port != IOPORT_UNASSIGNED)
            aux_claim_explicit(&aux_ctrl[idx]);
    }
}

static inline control_signals_t aux_ctrl_scan_status (control_signals_t signals)
{
#if PROBE_DISCONNECT_ENABLE || STOP_DISABLE_ENABLE || BLOCK_DELETE_ENABLE || SINGLE_BLOCK_ENABLE || LIMITS_OVERRIDE_ENABLE

    uint_fast8_t idx =  sizeof(aux_ctrl) / sizeof(aux_ctrl_t);

    if(idx) do {
        if(aux_ctrl[--idx].pin != 0xFF)
            break;
        if(aux_ctrl[idx].aux_port != IOPORT_UNASSIGNED) {
            signals.mask &= ~aux_ctrl[idx].cap.mask;
  #ifdef GRBL_ESP32 // Snowflake guru workaround
            if(hal.port.wait_on_input(Port_Digital, aux_ctrl[idx].aux_port, WaitMode_Immediate, FZERO) == 1)
                signals.mask |= aux_ctrl[idx].cap.mask;
  #else
            if(hal.port.wait_on_input(Port_Digital, aux_ctrl[idx].aux_port, WaitMode_Immediate, 0.0f) == 1)
                signals.mask |= aux_ctrl[idx].cap.mask;
  #endif
        }
    } while(idx);

#endif

    return signals;
}

// The following pins are bound explicitly to aux output pins
static aux_ctrl_out_t aux_ctrl_out[] = {
#if defined(ESP_PLATFORM) || defined(RP2040) // for now
#if defined(STEPPERS_ENABLE_PIN) && STEPPERS_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnable,  .aux_port = IOPORT_UNASSIGNED,  .pin = STEPPERS_ENABLE_PIN, .port = (void *)STEPPERS_ENABLE_PORT },
#endif
#if defined(X_ENABLE_PIN) && X_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableX, .aux_port = IOPORT_UNASSIGNED,  .pin = X_ENABLE_PIN,   .port = (void *)X_ENABLE_PORT },
#endif
#if defined(X2_ENABLE_PIN) && X2_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableX2, .aux_port = IOPORT_UNASSIGNED, .pin = X2_ENABLE_PIN,  .port = (void *)X2_ENABLE_PORT },
#endif
#if defined(Y_ENABLE_PIN) && Y_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableY,  .aux_port = IOPORT_UNASSIGNED, .pin = Y_ENABLE_PIN,   .port = (void *)Y_ENABLE_PORT },
#endif
#if defined(Y2_ENABLE_PIN) && Y2_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableY2, .aux_port = IOPORT_UNASSIGNED, .pin = Y2_ENABLE_PIN,  .port = (void *)Y2_ENABLE_PORT },
#endif
#if defined(XY_ENABLE_PIN) && XY_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableXY, .aux_port = IOPORT_UNASSIGNED, .pin = XY_ENABLE_PIN,  .port = (void *)XY_ENABLE_PORT },
#endif
#if defined(Z_ENABLE_PIN) && Z_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableZ,  .aux_port = IOPORT_UNASSIGNED, .pin = Z_ENABLE_PIN,   .port = (void *)Z_ENABLE_PORT },
#endif
#if defined(Z2_ENABLE_PIN) && Z2_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableZ2, .aux_port = IOPORT_UNASSIGNED, .pin = Z2_ENABLE_PIN,  .port = (void *)Z2_ENABLE_PORT },
#endif
#if defined(A_ENABLE_PIN) && A_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableA,  .aux_port = IOPORT_UNASSIGNED, .pin = A_ENABLE_PIN,   .port = (void *)A_ENABLE_PORT },
#endif
#if defined(B_ENABLE_PIN) && B_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableB,  .aux_port = IOPORT_UNASSIGNED, .pin = B_ENABLE_PIN,   .port = (void *)B_ENABLE_PORT },
#endif
#if defined(C_ENABLE_PIN) && C_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableC,  .aux_port = IOPORT_UNASSIGNED, .pin = C_ENABLE_PIN,   .port = (void *)C_ENABLE_PORT },
#endif
#if defined(U_ENABLE_PIN) && U_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableU,  .aux_port = IOPORT_UNASSIGNED, .pin = U_ENABLE_PIN,   .port = (void *)U_ENABLE_PORT },
#endif
#if defined(V_ENABLE_PIN) && AV_ENABLE_PORT == EXPANDER_PORT
    { .function = Output_StepperEnableV,  .aux_port = IOPORT_UNASSIGNED, .pin = V_ENABLE_PIN,   .port = (void *)V_ENABLE_PORT },
#endif
#endif //
#ifdef SPINDLE_ENABLE_PIN
 #ifndef SPINDLE_ENABLE_PORT
  #define SPINDLE_ENABLE_PORT 0
 #endif
    { .function = Output_SpindleOn,    .aux_port = IOPORT_UNASSIGNED, .pin = SPINDLE_ENABLE_PIN,     .port = (void *)SPINDLE_ENABLE_PORT },
#endif
#ifdef SPINDLE_PWM_PIN
 #ifndef SPINDLE_PWM_PORT
  #define SPINDLE_PWM_PORT 0
 #endif
    { .function = Output_SpindlePWM,   .aux_port = IOPORT_UNASSIGNED, .pin = SPINDLE_PWM_PIN,        .port = (void *)SPINDLE_PWM_PORT },
#endif
#ifdef SPINDLE_DIRECTION_PIN
 #ifndef SPINDLE_DIRECTION_PORT
  #define SPINDLE_DIRECTION_PORT 0
 #endif
    { .function = Output_SpindleDir,   .aux_port = IOPORT_UNASSIGNED, .pin = SPINDLE_DIRECTION_PIN,  .port = (void *)SPINDLE_DIRECTION_PORT },
#endif

#ifdef SPINDLE1_ENABLE_PIN
 #ifndef SPINDLE1_ENABLE_PORT
  #define SPINDLE1_ENABLE_PORT 0
 #endif
    { .function = Output_Spindle1On,   .aux_port = IOPORT_UNASSIGNED, .pin = SPINDLE1_ENABLE_PIN,    .port = (void *)SPINDLE1_ENABLE_PORT },
#endif
#ifdef SPINDLE1_PWM_PIN
 #ifndef SPINDLE1_PWM_PORT
  #define SPINDLE1_PWM_PORT 0
 #endif
    { .function = Output_Spindle1PWM,  .aux_port = IOPORT_UNASSIGNED, .pin = SPINDLE1_PWM_PIN,       .port = (void *)SPINDLE1_PWM_PORT },
#endif
#ifdef SPINDLE1_DIRECTION_PIN
 #ifndef SPINDLE1_DIRECTION_PORT
  #define SPINDLE1_DIRECTION_PORT 0
 #endif
    { .function = Output_Spindle1Dir,  .aux_port = IOPORT_UNASSIGNED, .pin = SPINDLE1_DIRECTION_PIN, .port = (void *)SPINDLE1_DIRECTION_PORT },
#endif

#ifdef COOLANT_FLOOD_PIN
 #ifndef COOLANT_FLOOD_PORT
  #define COOLANT_FLOOD_PORT 0
 #endif
    { .function = Output_CoolantFlood, .aux_port = IOPORT_UNASSIGNED, .pin = COOLANT_FLOOD_PIN,      .port = (void *)COOLANT_FLOOD_PORT },
#endif
#ifdef COOLANT_MIST_PIN
 #ifndef COOLANT_MIST_PORT
  #define COOLANT_MIST_PORT 0
 #endif
    { .function = Output_CoolantMist,  .aux_port = IOPORT_UNASSIGNED, .pin = COOLANT_MIST_PIN,       .port = (void *)COOLANT_MIST_PORT },
#endif

#ifdef COPROC_RESET_PIN
 #ifndef COPROC_RESET_PORT
  #define COPROC_RESET_PORT 0
 #endif
    { .function = Output_CoProc_Reset, .aux_port = IOPORT_UNASSIGNED, .pin = COPROC_RESET_PIN,       .port = (void *)COPROC_RESET_PORT },
#endif
#ifdef COPROC_BOOT0_PIN
 #ifndef COPROC_BOOT0_PORT
  #define COPROC_BOOT0_PORT 0
 #endif
    { .function = Output_CoProc_Boot0, .aux_port = IOPORT_UNASSIGNED, .pin = COPROC_BOOT0_PIN,       .port = (void *)COPROC_BOOT0_PORT },
#endif
#if defined(SPI_RST_PIN) && defined(RP2040)
 #if SPI_RST_PORT == EXPANDER_PORT
    { .function = Output_SPIRST,       .aux_port = IOPORT_UNASSIGNED, .pin = SPI_RST_PIN,            .port = (void *)SPI_RST_PORT },
 #endif
#endif
};

static inline aux_ctrl_out_t *aux_out_remap_explicit (void *port, uint8_t pin, uint8_t aux_port, void *output)
{
    aux_ctrl_out_t *ctrl_pin = NULL;

    uint_fast8_t idx = sizeof(aux_ctrl_out) / sizeof(aux_ctrl_out_t);

    if(idx) do {
        idx--;
        if(aux_ctrl_out[idx].port == port && aux_ctrl_out[idx].pin == pin) {
            ctrl_pin = &aux_ctrl_out[idx];
            ctrl_pin->aux_port = aux_port;
            ctrl_pin->output = output;
        }
    } while(idx && ctrl_pin == NULL);

    return ctrl_pin;
}

typedef bool (*aux_claim_explicit_out_ptr)(aux_ctrl_out_t *aux_ctrl);

static bool aux_ctrl_claim_out_port (xbar_t *properties, uint8_t port, void *data)
{
    if(((aux_ctrl_out_t *)data)->port == (void *)EXPANDER_PORT) {
        if(((aux_ctrl_out_t *)data)->pin == properties->pin && properties->set_value)
            ((aux_ctrl_out_t *)data)->aux_port = port;
    } else if(ioport_claim(Port_Digital, Port_Output, &port, xbar_fn_to_pinname(((aux_ctrl_out_t *)data)->function)))
        ((aux_ctrl_out_t *)data)->aux_port = port;

    return ((aux_ctrl_out_t *)data)->aux_port != 0xFF;
}

static inline void aux_ctrl_claim_out_ports (aux_claim_explicit_out_ptr aux_claim_explicit, ioports_enumerate_callback_ptr aux_claim)
{
    uint_fast8_t idx;

    if(aux_claim == NULL)
        aux_claim = aux_ctrl_claim_out_port;

    for(idx = 0; idx < sizeof(aux_ctrl_out) / sizeof(aux_ctrl_out_t); idx++) {
        if(aux_ctrl_out[idx].port == (void *)EXPANDER_PORT) {
            if(ioports_enumerate(Port_Digital, Port_Output, (pin_cap_t){ .external = On, .claimable = On }, aux_claim, &aux_ctrl_out[idx])) {
                if((aux_ctrl_out[idx].output = ioport_claim(Port_Digital, Port_Output, &aux_ctrl_out[idx].aux_port, NULL /*xbar_fn_to_pinname(aux_ctrl_out[idx].function)*/))) {
                    ioport_set_function((xbar_t *)aux_ctrl_out[idx].output, aux_ctrl_out[idx].function, NULL);
                        // TODO: else set description?
                    aux_claim_explicit(&aux_ctrl_out[idx]);
                }
            }
        } else if(aux_ctrl_out[idx].pin == 0xFF) {
            if(ioports_enumerate(Port_Digital, Port_Output, (pin_cap_t){ .claimable = On }, aux_claim, &aux_ctrl_out[idx]))
                aux_claim_explicit(&aux_ctrl_out[idx]);
        } else if(aux_ctrl_out[idx].aux_port != 0xFF)
            aux_claim_explicit(&aux_ctrl_out[idx]);
    }
}

// Output Signals

#if defined(SPINDLE_ENABLE_PIN) && !defined(SPINDLE_ENABLE_BIT)
#define SPINDLE_ENABLE_BIT (1<<SPINDLE_ENABLE_PIN)
#endif
#if defined(SPINDLE_DIRECTION_PIN) && !defined(SPINDLE_DIRECTION_BIT)
#define SPINDLE_DIRECTION_BIT (1<<SPINDLE_DIRECTION_PIN)
#endif

#if defined(SPINDLE1_ENABLE_PIN) && !defined(SPINDLE1_ENABLE_BIT)
#define SPINDLE1_ENABLE_BIT (1<<SPINDLE1_ENABLE_PIN)
#endif
#if defined(SPINDLE1_DIRECTION_PIN) && !defined(SPINDLE1_DIRECTION_BIT)
#define SPINDLE1_DIRECTION_BIT (1<<SPINDLE1_DIRECTION_PIN)
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

#if QEI_ENABLE
#ifndef QEI_A_BIT
#define QEI_A_BIT (1<<QEI_A_PIN)
#endif
#ifndef QEI_B_BIT
#define QEI_B_BIT (1<<QEI_B_PIN)
#endif
#else
#define QEI_A_BIT 0
#define QEI_B_BIT 0
#endif

#ifndef QEI_SELECT_BIT
#define QEI_SELECT_BIT 0
#endif
#ifndef MPG_MODE_BIT
#define MPG_MODE_BIT 0
#endif
#ifndef I2C_STROBE_BIT
#define I2C_STROBE_BIT 0
#endif

// Do NOT #define PROBE_BIT 0 here!

#if SPINDLE_ENCODER_ENABLE
#ifndef SPINDLE_PULSE_PIN
#error "Spindle encoder requires at least SPINDLE_PULSE_PIN defined in the board map!"
#endif
#if !defined(SPINDLE_PULSE_BIT) && defined(SPINDLE_PULSE_PIN)
#define SPINDLE_PULSE_BIT (1<<SPINDLE_PULSE_PIN)
#endif
#if !defined(SPINDLE_INDEX_BIT) && defined(SPINDLE_INDEX_PIN)
#define SPINDLE_INDEX_BIT (1<<SPINDLE_INDEX_PIN)
#endif
#endif

#ifndef SPINDLE_INDEX_BIT
#define SPINDLE_INDEX_BIT 0
#endif
#ifndef SPINDLE_PULSE_BIT
#define SPINDLE_PULSE_BIT 0
#endif

#if SPINDLE_SYNC_ENABLE && (SPINDLE_INDEX_BIT + SPINDLE_PULSE_BIT) == 0
#error "Spindle sync requires SPINDLE_PULSE_PIN and SPINDLE_INDEX_PIN defined in the board map!"
#endif

#ifndef SPI_IRQ_PIN
#define SPI_IRQ_BIT 0
#elif !defined(SPI_IRQ_BIT)
#define SPI_IRQ_BIT (1<<SPI_IRQ_PIN)
#endif

#ifndef DEVICES_IRQ_MASK
#define DEVICES_IRQ_MASK (SPI_IRQ_BIT|SPINDLE_INDEX_BIT|QEI_A_BIT|QEI_B_BIT|SD_DETECT_BIT)
#define DEVICES_IRQ_MASK_SUM (SPI_IRQ_BIT+SPINDLE_INDEX_BIT+QEI_A_BIT+QEI_B_BIT+SD_DETECT_BIT)
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
#ifdef AUXINPUT8_PIN
#define AUXINPUT8_BIT (1<<AUXINPUT8_PIN)
#else
#define AUXINPUT8_BIT 0
#endif
#ifdef AUXINPUT9_PIN
#define AUXINPUT9_BIT (1<<AUXINPUT9_PIN)
#else
#define AUXINPUT9_BIT 0
#endif
#ifdef AUXINPUT10_PIN
#define AUXINPUT10_BIT (1<<AUXINPUT10_PIN)
#else
#define AUXINPUT10_BIT 0
#endif
#ifdef AUXINPUT11_PIN
#define AUXINPUT11_BIT (1<<AUXINPUT11_PIN)
#else
#define AUXINPUT11_BIT 0
#endif
#ifdef AUXINPUT12_PIN
#define AUXINPUT12_BIT (1<<AUXINPUT12_PIN)
#else
#define AUXINPUT12_BIT 0
#endif
#ifdef AUXINPUT13_PIN
#define AUXINPUT13_BIT (1<<AUXINPUT13_PIN)
#else
#define AUXINPUT13_BIT 0
#endif
#ifdef AUXINPUT14_PIN
#define AUXINPUT14_BIT (1<<AUXINPUT14_PIN)
#else
#define AUXINPUT14_BIT 0
#endif
#ifdef AUXINPUT15_PIN
#define AUXINPUT15_BIT (1<<AUXINPUT15_PIN)
#else
#define AUXINPUT15_BIT 0
#endif

#ifndef AUXINPUT_MASK
#define AUXINPUT_MASK (AUXINPUT0_BIT|AUXINPUT1_BIT|AUXINPUT2_BIT|AUXINPUT3_BIT|AUXINPUT4_BIT|AUXINPUT5_BIT|AUXINPUT6_BIT|AUXINPUT7_BIT|\
                       AUXINPUT8_BIT|AUXINPUT9_BIT|AUXINPUT10_BIT|AUXINPUT11_BIT|AUXINPUT12_BIT|AUXINPUT13_BIT|AUXINPUT4_BIT|AUXINPUT15_BIT)
#define AUXINPUT_MASK_SUM (AUXINPUT0_BIT+AUXINPUT1_BIT+AUXINPUT2_BIT+AUXINPUT3_BIT+AUXINPUT4_BIT+AUXINPUT5_BIT+AUXINPUT6_BIT+AUXINPUT7_BIT+\
                           AUXINPUT8_BIT+AUXINPUT9_BIT+AUXINPUT10_BIT+AUXINPUT11_BIT+AUXINPUT12_BIT+AUXINPUT13_BIT+AUXINPUT4_BIT+AUXINPUT15_BIT)
#endif

/*EOF*/
