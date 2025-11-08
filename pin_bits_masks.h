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

#define a_cap(pin) .cap.pin

#if defined(ESP_PLATFORM) || defined(RP2040) || defined(__IMXRT1062__)
#define add_aux_input(fn, aux, irq, signal_bit) { .function = fn, .irq_mode = irq, .signal.value = signal_bit, .port = IOPORT_UNASSIGNED, .gpio.pin = aux##_PIN },
#else
#define add_aux_input(fn, aux, irq, signal_bit) { .function = fn, .irq_mode = irq, .signal.value = signal_bit, .port = IOPORT_UNASSIGNED, .gpio.port = (void *)aux##_PORT, .gpio.pin = aux##_PIN },
#endif
#if defined(__IMXRT1062__) || defined(ESP_PLATFORM)
#define add_aux_output(fn, aux) { .function = fn, .port = IOPORT_UNASSIGNED, .gpio.pin = aux##_PIN },
#else
#define add_aux_output(fn, aux) { .function = fn, .port = IOPORT_UNASSIGNED, .gpio.port = (void *)aux##_PORT, .gpio.pin = aux##_PIN },
#endif
#define add_aux_input_scan(fn, irq, signal_bit) { .function = fn, .irq_mode = irq, .signal.value = signal_bit, .port = IOPORT_UNASSIGNED, .gpio.pin = 0xFF, .scan = On },
#define add_aux_input_no_signal(fn, irq) { .function = fn, .irq_mode = irq, .port = IOPORT_UNASSIGNED, .gpio.pin = 0xFE },
#define add_aux_output_exp(fn, aux) { .function = fn, .port = IOPORT_UNASSIGNED, .gpio.port = (void *)aux##_PORT, .gpio.pin = aux##_PIN },

static aux_ctrl_t aux_ctrl[] = {
// The following pins are bound explicitly to aux input pins.
#ifdef RESET_PIN
  #if (CONTROL_ENABLE & CONTROL_ESTOP)
    add_aux_input(Input_EStop, RESET, IRQ_Mode_RisingFalling, SIGNALS_ESTOP_BIT)
  #elif (CONTROL_ENABLE & CONTROL_RESET)
    add_aux_input(Input_Reset, RESET, IRQ_Mode_RisingFalling, SIGNALS_RESET_BIT)
  #endif
#endif
#if (CONTROL_ENABLE & CONTROL_FEED_HOLD) && defined(FEED_HOLD_PIN)
    add_aux_input(Input_FeedHold, FEED_HOLD, IRQ_Mode_RisingFalling, SIGNALS_FEEDHOLD_BIT)
#endif
#if (CONTROL_ENABLE & CONTROL_CYCLE_START) && defined(CYCLE_START_PIN)
    add_aux_input(Input_CycleStart, CYCLE_START, IRQ_Mode_RisingFalling, SIGNALS_CYCLESTART_BIT)
#endif
#if SAFETY_DOOR_ENABLE && defined(SAFETY_DOOR_PIN)
    add_aux_input(Input_SafetyDoor, SAFETY_DOOR, IRQ_Mode_RisingFalling, SIGNALS_SAFETYDOOR_BIT)
#endif
#if MOTOR_FAULT_ENABLE && defined(MOTOR_FAULT_PIN)
    add_aux_input(Input_MotorFault, MOTOR_FAULT, IRQ_Mode_RisingFalling, SIGNALS_MOTOR_FAULT_BIT)
#endif
#if MOTOR_WARNING_ENABLE && defined(MOTOR_WARNING_PIN)
    add_aux_input(Input_MotorWarning, MOTOR_WARNING, IRQ_Mode_RisingFalling, SIGNALS_MOTOR_WARNING_BIT)
#endif
#if I2C_STROBE_ENABLE && defined(I2C_STROBE_PIN)
    add_aux_input(Input_I2CStrobe, I2C_STROBE, IRQ_Mode_Change, 0)
#endif
#if MPG_ENABLE == 1 && defined(MPG_MODE_PIN)
    add_aux_input(Input_MPGSelect, MPG_MODE, IRQ_Mode_Change, 0)
#endif
#if QEI_SELECT_ENABLE && defined(QEI_SELECT_PIN)
    add_aux_input(Input_QEI_Select, QEI_SELECT, IRQ_Mode_RisingFalling, 0)
#endif
// Probe pins can be bound explicitly and can be "degraded" to not interrupt capable.
#if PROBE_ENABLE && defined(PROBE_PIN)
    add_aux_input(Input_Probe, PROBE, IRQ_Mode_RisingFalling, 0)
#endif
#if PROBE2_ENABLE && defined(PROBE2_PIN)
    add_aux_input(Input_Probe2, PROBE2, IRQ_Mode_RisingFalling, 0)
#endif
#if TOOLSETTER_ENABLE && defined(TOOLSETTER_PIN)
    add_aux_input(Input_Toolsetter, TOOLSETTER, IRQ_Mode_RisingFalling, 0)
#endif

// The following pins are allocated from remaining aux inputs pool
#if TOOLSETTER_ENABLE && !defined(TOOLSETTER_PIN)
    add_aux_input_no_signal(Input_Toolsetter, IRQ_Mode_RisingFalling)
#endif
#if PROBE2_ENABLE && !defined(PROBE2_PIN)
    add_aux_input_no_signal(Input_Probe2, IRQ_Mode_RisingFalling)
#endif
#if TLS_OVERTRAVEL_ENABLE
    add_aux_input_scan(Input_ToolsetterOvertravel, IRQ_Mode_Change, SIGNALS_TLS_OVERTRAVEL_BIT)
#endif
#if LIMITS_OVERRIDE_ENABLE
    add_aux_input_scan(Input_LimitsOverride, IRQ_Mode_None, SIGNALS_LIMITS_OVERRIDE_BIT)
#endif
#if STOP_DISABLE_ENABLE
    add_aux_input_scan(Input_StopDisable, IRQ_Mode_Change, SIGNALS_STOPDISABLE_BIT)
#endif
#if BLOCK_DELETE_ENABLE
    add_aux_input_scan(Input_BlockDelete, IRQ_Mode_Change, SIGNALS_BLOCKDELETE_BIT)
#endif
#if SINGLE_BLOCK_ENABLE
    add_aux_input_scan(Input_SingleBlock, IRQ_Mode_Change, SIGNALS_SINGLE_BLOCK_BIT)
#endif
#if PROBE_DISCONNECT_ENABLE
    add_aux_input_scan(Input_ProbeDisconnect, IRQ_Mode_Change, SIGNALS_PROBE_CONNECTED_BIT)
#endif
};

static inline bool aux_ctrl_is_probe (pin_function_t function)
{
    return function == Input_Probe || function == Input_Probe2 || function == Input_Toolsetter;
}

#ifdef STM32_PLATFORM

static inline aux_ctrl_t *aux_ctrl_get_fn (aux_gpio_t gpio)
{
    aux_ctrl_t *ctrl_pin = NULL;

    if(sizeof(aux_ctrl) / sizeof(aux_ctrl_t)) {
        uint_fast8_t idx;
        for(idx = 0; ctrl_pin == NULL && aux_ctrl[idx].gpio.pin != 0xFF && idx < sizeof(aux_ctrl) / sizeof(aux_ctrl_t); idx++) {
            if(aux_ctrl[idx].gpio.pin == gpio.pin && aux_ctrl[idx].gpio.port == gpio.port)
                ctrl_pin = &aux_ctrl[idx];
        }
    }

    return ctrl_pin;
}

#endif // STM32_PLATFORM

static inline xbar_t *aux_ctrl_claim_port (aux_ctrl_t *aux_ctrl)
{
    xbar_t *pin = NULL;

    if(aux_ctrl) {
        if(aux_ctrl->port != IOPORT_UNASSIGNED && (pin = ioport_claim(Port_Digital, Port_Input, &aux_ctrl->port, NULL))) {

            aux_ctrl->gpio.port = pin->port;
            aux_ctrl->gpio.pin = pin->pin;

            ioport_set_function(pin, aux_ctrl->function, &aux_ctrl->signal);
        } else
            aux_ctrl->port = IOPORT_UNASSIGNED;
    }

    return pin;
}

static inline aux_ctrl_t *aux_ctrl_remap_explicit (aux_gpio_t gpio, uint8_t port, void *input)
{
    int_fast8_t idx;
    aux_ctrl_t *ctrl_pin = NULL;

    if(sizeof(aux_ctrl) / sizeof(aux_ctrl_t)) {
        for(idx = 0; ctrl_pin == NULL && idx < sizeof(aux_ctrl) / sizeof(aux_ctrl_t) && aux_ctrl[idx].gpio.pin != 0xFF; idx++) {
            if(aux_ctrl[idx].gpio.pin == gpio.pin && aux_ctrl[idx].gpio.port == gpio.port) {
                ctrl_pin = &aux_ctrl[idx];
                ctrl_pin->port = port;
                ctrl_pin->input = input;
                break;
            }
        }
    }

    return ctrl_pin;
}

static inline aux_ctrl_t *aux_ctrl_in_get (uint8_t port)
{
    aux_ctrl_t *ctrl_pin = NULL;

    uint_fast8_t idx = sizeof(aux_ctrl) / sizeof(aux_ctrl_t);

    if(idx) do {
        if(aux_ctrl[--idx].port == port)
            ctrl_pin = &aux_ctrl[idx];
    } while(idx && ctrl_pin == NULL);

    return ctrl_pin;
}

static inline void aux_ctrl_irq_enable (settings_t *settings, ioport_interrupt_callback_ptr aux_irq_handler)
{
    uint_fast8_t idx = sizeof(aux_ctrl) / sizeof(aux_ctrl_t);

    if(idx) do {
        if(aux_ctrl[--idx].port != 0xFF && aux_ctrl[idx].irq_mode != IRQ_Mode_None) {
            if(!aux_ctrl_is_probe(aux_ctrl[idx].function)) {
                pin_irq_mode_t irq_mode;
                if((irq_mode = aux_ctrl[idx].irq_mode) & IRQ_Mode_RisingFalling)
                    irq_mode = (settings->control_invert.mask & aux_ctrl[idx].signal.mask) ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                hal.port.register_interrupt_handler(aux_ctrl[idx].port, irq_mode, aux_irq_handler);
            }
        }
    } while(idx);
}

typedef bool (*aux_claim_explicit_ptr)(aux_ctrl_t *aux_ctrl);

// Default/internal functions for aux_ctrl_claim_ports()

static bool __claim_in_port (xbar_t *properties, uint8_t port, void *data)
{
    if(ioport_claim(Port_Digital, Port_Input, &port, NULL)) {
        ((aux_ctrl_t *)data)->port = port;
        ((aux_ctrl_t *)data)->gpio.port = properties->port;
        ((aux_ctrl_t *)data)->gpio.pin = properties->pin;
        ioport_set_function(properties, ((aux_ctrl_t *)data)->function, &((aux_ctrl_t *)data)->signal);
    }

    return ((aux_ctrl_t *)data)->port != IOPORT_UNASSIGNED;
}

static bool __find_in_port (xbar_t *properties, uint8_t port, void *data)
{
    ((aux_ctrl_t *)data)->port = port;

    return true;
}

// --

static inline void aux_ctrl_claim_ports (aux_claim_explicit_ptr aux_claim_explicit, ioports_enumerate_callback_ptr aux_claim)
{
    uint_fast8_t idx;

    if(aux_claim == NULL)
        aux_claim = __claim_in_port;

    for(idx = 0; idx < sizeof(aux_ctrl) / sizeof(aux_ctrl_t); idx++) {

        if(aux_ctrl[idx].port != IOPORT_UNASSIGNED)
            aux_claim_explicit(&aux_ctrl[idx]);

        else {

            pin_cap_t cap = { .irq_mode = aux_ctrl[idx].irq_mode, .claimable = On };

            if(aux_ctrl[idx].gpio.pin == 0xFE) // Toolsetter and Probe2
                ioports_enumerate(Port_Digital, Port_Input, cap, __find_in_port, (void *)&aux_ctrl[idx]);
#ifdef STM32_PLATFORM
            if(aux_ctrl[idx].irq_mode == IRQ_Mode_None && !(aux_ctrl[idx].function == Input_Probe || aux_ctrl[idx].function == Input_LimitsOverride))
                continue;
#endif
            if(aux_ctrl[idx].gpio.pin == 0xFF) {
                if(ioports_enumerate(Port_Digital, Port_Input, cap, aux_claim, (void *)&aux_ctrl[idx]))
                    hal.signals_cap.mask |= aux_ctrl[idx].signal.mask;
            }
        }
    }
}

static inline control_signals_t aux_ctrl_scan_status (control_signals_t signals)
{
#if PROBE_DISCONNECT_ENABLE || STOP_DISABLE_ENABLE || BLOCK_DELETE_ENABLE || SINGLE_BLOCK_ENABLE || LIMITS_OVERRIDE_ENABLE

    uint_fast8_t idx =  sizeof(aux_ctrl) / sizeof(aux_ctrl_t);

    if(idx) do {
        if(!aux_ctrl[--idx].scan)
            break;
        signals.mask &= ~aux_ctrl[idx].signal.mask;
        if(aux_ctrl[idx].port != IOPORT_UNASSIGNED) {
  #ifdef GRBL_ESP32 // Snowflake guru workaround
            if(hal.port.wait_on_input(Port_Digital, aux_ctrl[idx].port, WaitMode_Immediate, FZERO) == 1)
                signals.mask |= aux_ctrl[idx].signal.mask;
  #else
            if(hal.port.wait_on_input(Port_Digital, aux_ctrl[idx].port, WaitMode_Immediate, 0.0f) == 1)
                signals.mask |= aux_ctrl[idx].signal.mask;
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
    add_aux_output_exp(Output_StepperEnable, STEPPERS_ENABLE)
#endif
#if defined(X_ENABLE_PIN) && X_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableX, X_ENABLE)
#endif
#if defined(X2_ENABLE_PIN) && X2_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableX2, X2_ENABLE)
#endif
#if defined(Y_ENABLE_PIN) && Y_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableY, Y_ENABLE)
#endif
#if defined(Y2_ENABLE_PIN) && Y2_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableY2, Y2_ENABLE)
#endif
#if defined(XY_ENABLE_PIN) && XY_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableXY, XY_ENABLE)
#endif
#if defined(Z_ENABLE_PIN) && Z_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableZ, Z_ENABLE)
#endif
#if defined(Z2_ENABLE_PIN) && Z2_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableZ2, Z2_ENABLE)
#endif
#if defined(A_ENABLE_PIN) && A_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableA, A_ENABLE)
#endif
#if defined(B_ENABLE_PIN) && B_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableB, B_ENABLE)
#endif
#if defined(C_ENABLE_PIN) && C_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableC, C_ENABLE)
#endif
#if defined(U_ENABLE_PIN) && U_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableU, U_ENABLE)
#endif
#if defined(V_ENABLE_PIN) && V_ENABLE_PORT == EXPANDER_PORT
    add_aux_output_exp(Output_StepperEnableV, V_ENABLE)
#endif
#endif //

#ifdef SPINDLE_ENABLE_PIN
    add_aux_output(Output_SpindleOn, SPINDLE_ENABLE)
#endif
#ifdef SPINDLE_PWM_PIN
    add_aux_output(Output_SpindlePWM, SPINDLE_PWM)
#endif
#ifdef SPINDLE_DIRECTION_PIN
    add_aux_output(Output_SpindleDir, SPINDLE_DIRECTION)
#endif
#ifdef SPINDLE1_ENABLE_PIN
    add_aux_output(Output_Spindle1On, SPINDLE1_ENABLE)
#endif
#ifdef SPINDLE1_DIRECTION_PIN
    add_aux_output(Output_Spindle1Dir, SPINDLE1_DIRECTION)
#endif
#ifdef SPINDLE1_PWM_PIN
    add_aux_output(Output_Spindle1PWM, SPINDLE1_PWM)
#endif
#ifdef COOLANT_FLOOD_PIN
    add_aux_output(Output_CoolantFlood, COOLANT_FLOOD)
#endif
#ifdef COOLANT_MIST_PIN
    add_aux_output(Output_CoolantMist, COOLANT_MIST)
#endif
#ifdef COPROC_RESET_PIN
    add_aux_output(Output_CoProc_Reset, COPROC_RESET)
#endif
#ifdef COPROC_BOOT0_PIN
    add_aux_output(Output_CoProc_Boot0, COPROC_BOOT0)
#endif
#if defined(SPI_RST_PIN) && defined(RP2040)
 #ifndef SPI_RST_PORT
  #define SPI_RST_PORT 0
 #endif
    add_aux_output(Output_SPIRST, SPI_RST)
#endif
};

static inline aux_ctrl_out_t *aux_out_remap_explicit (aux_gpio_t gpio, uint8_t port, void *output)
{
    aux_ctrl_out_t *ctrl_pin = NULL;

    uint_fast8_t idx = sizeof(aux_ctrl_out) / sizeof(aux_ctrl_out_t);

    if(idx) do {
        idx--;
        if(aux_ctrl_out[idx].gpio.port == gpio.port && aux_ctrl_out[idx].gpio.pin == gpio.pin) {
            ctrl_pin = &aux_ctrl_out[idx];
            ctrl_pin->port = port;
            ctrl_pin->output = output;
        }
    } while(idx && ctrl_pin == NULL);

    return ctrl_pin;
}

typedef bool (*aux_claim_explicit_out_ptr)(aux_ctrl_out_t *aux_ctrl);

// Default functions for aux_ctrl_claim_out_ports()

static bool __claim_out_port (xbar_t *properties, uint8_t port, void *data)
{
    if(((aux_ctrl_out_t *)data)->gpio.port == (void *)EXPANDER_PORT) {
        if(((aux_ctrl_out_t *)data)->gpio.pin == properties->pin && properties->set_value)
            ((aux_ctrl_out_t *)data)->port = port;
    } else if(ioport_claim(Port_Digital, Port_Output, &port, xbar_fn_to_pinname(((aux_ctrl_out_t *)data)->function)))
        ((aux_ctrl_out_t *)data)->port = port;

    return ((aux_ctrl_out_t *)data)->port != IOPORT_UNASSIGNED;
}

static bool ___claim_out_port_explicit (aux_ctrl_out_t *aux_ctrl)
{
    xbar_t *pin;

    if((pin = ioport_claim(Port_Digital, Port_Output, &aux_ctrl->port, NULL)))
        ioport_set_function(pin, aux_ctrl->function, NULL);
    else
        aux_ctrl->port = IOPORT_UNASSIGNED;

    return aux_ctrl->port != IOPORT_UNASSIGNED;
}

//

static inline void aux_ctrl_claim_out_ports (aux_claim_explicit_out_ptr aux_claim_explicit, ioports_enumerate_callback_ptr aux_claim)
{
    uint_fast8_t idx;

    if(aux_claim == NULL)
        aux_claim = __claim_out_port;

    if(aux_claim_explicit == NULL)
        aux_claim_explicit = ___claim_out_port_explicit;

    for(idx = 0; idx < sizeof(aux_ctrl_out) / sizeof(aux_ctrl_out_t); idx++) {
        if(aux_ctrl_out[idx].gpio.port == (void *)EXPANDER_PORT) {
            if(ioports_enumerate(Port_Digital, Port_Output, (pin_cap_t){ .external = On, .claimable = On }, aux_claim, &aux_ctrl_out[idx])) {
                if((aux_ctrl_out[idx].output = ioport_claim(Port_Digital, Port_Output, &aux_ctrl_out[idx].port, NULL /*xbar_fn_to_pinname(aux_ctrl_out[idx].function)*/))) {
                    ioport_set_function((xbar_t *)aux_ctrl_out[idx].output, aux_ctrl_out[idx].function, NULL);
                        // TODO: else set description?
                    aux_claim_explicit(&aux_ctrl_out[idx]);
                }
            }
        } else if(aux_ctrl_out[idx].gpio.pin == 0xFF) {
            if(ioports_enumerate(Port_Digital, Port_Output, (pin_cap_t){ .claimable = On }, aux_claim, &aux_ctrl_out[idx]))
                aux_claim_explicit(&aux_ctrl_out[idx]);
        } else if(aux_ctrl_out[idx].port != IOPORT_UNASSIGNED)
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

#if defined(RS485_DIR_PIN) && !defined(RS485_DIR_BIT)
#define RS485_DIR_BIT (1<<RS485_DIR_PIN)
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

#if SPINDLE_ENCODER_ENABLE && (SPINDLE_INDEX_BIT + SPINDLE_PULSE_BIT) == 0
#error "Spindle encoder requires SPINDLE_PULSE_PIN and SPINDLE_INDEX_PIN defined in the board map!"
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
