/*
  pin_bits_masks.h - for adding bit definitions and masks

  NOTE: This file is not used by the core, it may be used by drivers

  Part of grblHAL

  Copyright (c) 2021-2024 Terje Io

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

#ifndef SD_DETECT_BIT
#ifdef SD_DETECT_PIN
#define SD_DETECT_BIT (1<<SD_DETECT_PIN)
#else
#define SD_DETECT_BIT 0
#endif
#endif

// Optional control signals

#ifndef SAFETY_DOOR_BIT
#if defined(SAFETY_DOOR_PIN) && !defined(AUX_DEVICES)
#define SAFETY_DOOR_BIT (1<<SAFETY_DOOR_PIN)
#else
#define SAFETY_DOOR_BIT 0
#endif
#endif

// Optional control signals, assigned to auxiliary input pins

#ifndef MOTOR_FAULT_BIT
#if defined(MOTOR_FAULT_PIN) && !MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_BIT (1<<MOTOR_FAULT_PIN)
#else
#define MOTOR_FAULT_BIT 0
#endif
#endif

#ifndef MOTOR_WARNING_BIT
#if defined(MOTOR_WARNING_PIN) && !MOTOR_WARNING_ENABLE
#define MOTOR_WARNING_BIT (1<<MOTOR_WARNING_PIN)
#else
#define MOTOR_WARNING_BIT 0
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
    STOP_DISABLE_ENABLE || BLOCK_DELETE_ENABLE || SINGLE_BLOCK_ENABLE || LIMITS_OVERRIDE_ENABLE || \
    (defined(AUX_DEVICES) && (PROBE_ENABLE || I2C_STROBE_ENABLE || MPG_ENABLE == 1 || QEI_SELECT_ENABLE)) || defined __DOXYGEN__

#define AUX_CONTROLS_ENABLED 1

#if PROBE_DISCONNECT_ENABLE || STOP_DISABLE_ENABLE || BLOCK_DELETE_ENABLE || SINGLE_BLOCK_ENABLE || LIMITS_OVERRIDE_ENABLE
#define AUX_CONTROLS_SCAN    1
#else
#define AUX_CONTROLS_SCAN    0
#endif

static aux_ctrl_t aux_ctrl[] = {
// The following pins are bound explicitly to aux input pins
#if PROBE_ENABLE && defined(PROBE_PIN) && defined(AUX_DEVICES)
#ifdef PROBE_PORT
    { .function = Input_Probe, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .value = 0 }, .pin = PROBE_PIN, .port = PROBE_PORT },
#else
    { .function = Input_Probe, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .value = 0 }, .pin = PROBE_PIN, .port = NULL },
#endif
#endif
#if SAFETY_DOOR_ENABLE && defined(SAFETY_DOOR_PIN)
#ifdef SAFETY_DOOR_PORT
    { .function = Input_SafetyDoor, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .safety_door_ajar = On }, .pin = SAFETY_DOOR_PIN, .port = SAFETY_DOOR_PORT },
#else
    { .function = Input_SafetyDoor, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .safety_door_ajar = On }, .pin = SAFETY_DOOR_PIN, .port = NULL },
#endif
#endif
#if MOTOR_FAULT_ENABLE && defined(MOTOR_FAULT_PIN)
#ifdef MOTOR_FAULT_PORT
    { .function = Input_MotorFault, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .motor_fault = On }, .pin = MOTOR_FAULT_PIN, .port = MOTOR_FAULT_PORT },
#else
    { .function = Input_MotorFault, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .motor_fault = On }, .pin = MOTOR_FAULT_PIN, .port = NULL },
#endif
#if MOTOR_WARNING_ENABLE && defined(MOTOR_WARNING_PIN)
#ifdef MOTOR_WARNING_PORT
    { .function = Input_MotorWarning, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .motor_fault = On }, .pin = MOTOR_WARNING_PIN, .port = MOTOR_WARNING_PORT },
#else
    { .function = Input_MotorWarning, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .motor_warning = On }, .pin = MOTOR_WARNING_PIN, .port = NULL },
#endif
#endif
#endif
#if I2C_STROBE_ENABLE && defined(I2C_STROBE_PIN) && defined(AUX_DEVICES)
#ifdef I2C_STROBE_PORT
    { .function = Input_I2CStrobe, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Change), .cap = { .value = 0 }, .pin = I2C_STROBE_PIN, .port = I2C_STROBE_PORT },
#else
    { .function = Input_I2CStrobe, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Change), .cap = { .value = 0 }, .pin = I2C_STROBE_PIN, .port = NULL },
#endif
#endif
#if MPG_ENABLE == 1 && defined(MPG_MODE_PIN) && defined(AUX_DEVICES)
#ifdef MPG_MODE_PORT
    { .function = Input_MPGSelect, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Change), .cap = { .value = 0 }, .pin = MPG_MODE_PIN, .port = MPG_MODE_PORT },
#else
    { .function = Input_MPGSelect, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Change), .cap = { .value = 0 }, .pin = MPG_MODE_PIN, .port = NULL },
#endif
#endif
#if QEI_SELECT_ENABLE && defined(QEI_SELECT_PIN) && defined(AUX_DEVICES)
#ifdef QEI_SELECT_PORT
    { .function = Input_QEI_Select, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .value = 0 }, .pin = QEI_SELECT_PIN, .port = QEI_SELECT_PORT },
#else
    { .function = Input_QEI_Select, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .value = 0 }, .pin = QEI_SELECT_PIN, .port = NULL },
#endif
#endif
// The following pins are allocated from remaining aux inputs pool
#if LIMITS_OVERRIDE_ENABLE
    { .function = Input_LimitsOverride, .aux_port = 0xFF, .irq_mode = IRQ_Mode_None, .cap = { .limits_override = On }, .pin = 0xFF, .port = NULL },
#endif
#if STOP_DISABLE_ENABLE
    { .function = Input_StopDisable, .aux_port = 0xFF, .irq_mode = IRQ_Mode_Change, .cap = { .stop_disable = On }, .pin = 0xFF, .port = NULL },
#endif
#if BLOCK_DELETE_ENABLE
    { .function = Input_BlockDelete, .aux_port = 0xFF, .irq_mode = IRQ_Mode_Change, .cap = { .block_delete = On }, .pin = 0xFF, .port = NULL },
#endif
#if SINGLE_BLOCK_ENABLE
    { .function = Input_SingleBlock, .aux_port = 0xFF, .irq_mode = IRQ_Mode_Change, .cap = { .single_block = On }, .pin = 0xFF, .port = NULL },
#endif
#if PROBE_DISCONNECT_ENABLE
    { .function = Input_ProbeDisconnect, .aux_port = 0xFF, .irq_mode = (pin_irq_mode_t)(IRQ_Mode_Rising|IRQ_Mode_Falling), .cap = { .probe_disconnected = On }, .pin = 0xFF, .port = NULL },
#endif
};

static inline aux_ctrl_t *aux_ctrl_remap_explicit (void *port, uint8_t pin, uint8_t aux_port, void *input)
{
    aux_ctrl_t *ctrl_pin = NULL;

    uint_fast8_t idx = sizeof(aux_ctrl) / sizeof(aux_ctrl_t);

    do {
        idx--;
        if(aux_ctrl[idx].port == port && aux_ctrl[idx].pin == pin) {
            ctrl_pin = &aux_ctrl[idx];
            ctrl_pin->aux_port = aux_port;
            ctrl_pin->input = input;
        }
    } while(idx && ctrl_pin == NULL);

    return ctrl_pin;
}

static inline aux_ctrl_t *aux_ctrl_get_pin (uint8_t aux_port)
{
    aux_ctrl_t *ctrl_pin = NULL;

    uint_fast8_t idx = sizeof(aux_ctrl) / sizeof(aux_ctrl_t);

    do {
        if(aux_ctrl[--idx].aux_port == aux_port)
            ctrl_pin = &aux_ctrl[idx];
    } while(idx && ctrl_pin == NULL);

    return ctrl_pin;
}

static inline void aux_ctrl_irq_enable (settings_t *settings, ioport_interrupt_callback_ptr aux_irq_handler)
{
    uint_fast8_t idx = sizeof(aux_ctrl) / sizeof(aux_ctrl_t);

    if(idx) do {
        if(aux_ctrl[--idx].aux_port != 0xFF) {
#if PROBE_ENABLE && defined(PROBE_PIN) && defined(AUX_DEVICES)
            if(aux_ctrl[idx].function == Input_Probe) {
                xbar_t *xbar;
                if((xbar = hal.port.get_pin_info(Port_Digital, Port_Input, aux_ctrl[idx].aux_port))) {
                    gpio_in_config_t cfg;
                    cfg.inverted = settings->probe.invert_probe_pin;
                    cfg.debounce = xbar->mode.debounce;
                    cfg.pull_mode = settings->probe.disable_probe_pullup ? PullMode_None : PullMode_Up;
                    xbar->config(xbar, &cfg, false);
                }
            } else
#endif
            if(aux_ctrl[idx].irq_mode != IRQ_Mode_None) {
                if(aux_ctrl[idx].irq_mode & (IRQ_Mode_Falling|IRQ_Mode_Rising))
                    aux_ctrl[idx].irq_mode = (settings->control_invert.mask & aux_ctrl[idx].cap.mask) ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                hal.port.register_interrupt_handler(aux_ctrl[idx].aux_port, aux_ctrl[idx].irq_mode, aux_irq_handler);
            }
        }
    } while(idx);
}

typedef bool (*aux_claim_explicit_ptr)(aux_ctrl_t *aux_ctrl);

static bool aux_ctrl_claim_port (xbar_t *properties, uint8_t port, void *data)
{
    if(ioport_claim(Port_Digital, Port_Input, &port, xbar_fn_to_pinname(((aux_ctrl_t *)data)->function)))
        ((aux_ctrl_t *)data)->aux_port = port;

    return ((aux_ctrl_t *)data)->aux_port != 0xFF;
}

static inline void aux_ctrl_claim_ports (aux_claim_explicit_ptr aux_claim_explicit, ioports_enumerate_callback_ptr aux_claim)
{
    uint_fast8_t idx;

    if(aux_claim == NULL)
        aux_claim = aux_ctrl_claim_port;

    for(idx = 0; idx < sizeof(aux_ctrl) / sizeof(aux_ctrl_t); idx++) {
        if(aux_ctrl[idx].pin == 0xFF) {
            if(ioports_enumerate(Port_Digital, Port_Input, (pin_cap_t){ .irq_mode = aux_ctrl[idx].irq_mode, .claimable = On }, aux_claim, (void *)&aux_ctrl[idx]))
                hal.signals_cap.mask |= aux_ctrl[idx].cap.mask;
        } else if(aux_ctrl[idx].aux_port != 0xFF)
            aux_claim_explicit(&aux_ctrl[idx]);
    }
}

#if AUX_CONTROLS_SCAN

static inline control_signals_t aux_ctrl_scan_status (control_signals_t signals)
{
    uint_fast8_t idx =  sizeof(aux_ctrl) / sizeof(aux_ctrl_t);

    if(idx) do {
        if(aux_ctrl[--idx].pin != 0xFF)
            break;
        if(aux_ctrl[idx].aux_port != 0xFF) {
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

    return signals;
}

#endif

#else
#define AUX_CONTROLS_ENABLED 0
#define AUX_CONTROLS_SCAN    0
#endif

#if defined(AUX_CONTROLS_OUT) && !defined(AUX_CONTROLS)
#define AUX_CONTROLS AUX_CONTROL_SPINDLE
#elif !defined(AUX_CONTROLS)
#define AUX_CONTROLS 0
#endif

#if AUX_CONTROLS

// The following pins are bound explicitly to aux output pins
static aux_ctrl_out_t aux_ctrl_out[] = {
#if AUX_CONTROLS & AUX_CONTROL_SPINDLE
#ifdef SPINDLE_ENABLE_PIN
 #ifndef SPINDLE_ENABLE_PORT
  #define SPINDLE_ENABLE_PORT NULL
 #endif
    { .function = Output_SpindleOn,    .aux_port = 0xFF, .pin = SPINDLE_ENABLE_PIN,     .port = SPINDLE_ENABLE_PORT },
#endif
#ifdef SPINDLE_PWM_PIN
 #ifndef SPINDLE_PWM_PORT
  #define SPINDLE_PWM_PORT NULL
 #endif
    { .function = Output_SpindlePWM,   .aux_port = 0xFF, .pin = SPINDLE_PWM_PIN,        .port = SPINDLE_PWM_PORT },
#endif
#ifdef SPINDLE_DIRECTION_PIN
 #ifndef SPINDLE_DIRECTION_PORT
  #define SPINDLE_DIRECTION_PORT NULL
 #endif
    { .function = Output_SpindleDir,   .aux_port = 0xFF, .pin = SPINDLE_DIRECTION_PIN,  .port = SPINDLE_DIRECTION_PORT },
#endif

#ifdef SPINDLE1_ENABLE_PIN
 #ifndef SPINDLE1_ENABLE_PORT
  #define SPINDLE1_ENABLE_PORT NULL
 #endif
    { .function = Output_Spindle1On,   .aux_port = 0xFF, .pin = SPINDLE1_ENABLE_PIN,    .port = SPINDLE1_ENABLE_PORT },
#endif
#ifdef SPINDLE1_PWM_PIN
 #ifndef SPINDLE1_PWM_PORT
  #define SPINDLE1_PWM_PORT NULL
 #endif
    { .function = Output_Spindle1PWM,  .aux_port = 0xFF, .pin = SPINDLE1_PWM_PIN,       .port = SPINDLE1_PWM_PORT },
#endif
#ifdef SPINDLE1_DIRECTION_PIN
 #ifndef SPINDLE1_DIRECTION_PORT
  #define SPINDLE1_DIRECTION_PORT NULL
 #endif
    { .function = Output_Spindle1Dir,  .aux_port = 0xFF, .pin = SPINDLE1_DIRECTION_PIN, .port = SPINDLE1_DIRECTION_PORT },
#endif
#endif // SPINDLES

#if AUX_CONTROLS & AUX_CONTROL_COOLANT
#ifdef COOLANT_FLOOD_PIN
 #ifndef COOLANT_FLOOD_PORT
  #define COOLANT_FLOOD_PORT NULL
 #endif
    { .function = Output_CoolantFlood, .aux_port = 0xFF, .pin = COOLANT_FLOOD_PIN,      .port = COOLANT_FLOOD_PORT },
#endif
#ifdef COOLANT_MIST_PIN
 #ifndef COOLANT_MIST_PORT
  #define COOLANT_MIST_PORT NULL
 #endif
    { .function = Output_CoolantMist,  .aux_port = 0xFF, .pin = COOLANT_MIST_PIN,       .port = COOLANT_MIST_PORT },
#endif
#endif // COOLANT

#ifdef COPROC_RESET_PIN
 #ifndef COPROC_RESET_PORT
  #define COPROC_RESET_PORT NULL
 #endif
    { .function = Output_CoProc_Reset, .aux_port = 0xFF, .pin = COPROC_RESET_PIN,       .port = COPROC_RESET_PORT },
#endif
#ifdef COPROC_BOOT0_PIN
 #ifndef COPROC_BOOT0_PORT
  #define COPROC_BOOT0_PORT NULL
 #endif
    { .function = Output_CoProc_Boot0, .aux_port = 0xFF, .pin = COPROC_BOOT0_PIN,       .port = COPROC_BOOT0_PORT },
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
    if(ioport_claim(Port_Digital, Port_Output, &port, xbar_fn_to_pinname(((aux_ctrl_t *)data)->function)))
        ((aux_ctrl_t *)data)->aux_port = port;

    return ((aux_ctrl_t *)data)->aux_port != 0xFF;
}

static inline void aux_ctrl_claim_out_ports (aux_claim_explicit_out_ptr aux_claim_explicit, ioports_enumerate_callback_ptr aux_claim)
{
    uint_fast8_t idx;

    if(aux_claim == NULL)
        aux_claim = aux_ctrl_claim_out_port;

    for(idx = 0; idx < sizeof(aux_ctrl_out) / sizeof(aux_ctrl_out_t); idx++) {
        if(aux_ctrl_out[idx].pin == 0xFF)
            ioports_enumerate(Port_Digital, Port_Output, (pin_cap_t){ .claimable = On }, aux_claim, (void *)&aux_ctrl_out[idx]);
        else if(aux_ctrl_out[idx].aux_port != 0xFF)
            aux_claim_explicit(&aux_ctrl_out[idx]);
    }
}

#endif // AUX_CONTROLS

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

#ifndef AUX_DEVICES

// IRQ capability for the probe input is optional
#if defined(PROBE_PIN) && !defined(PROBE_BIT)
#define PROBE_BIT (1<<PROBE_PIN)
#endif

#if defined(MPG_MODE_PIN) && !defined(MPG_MODE_BIT)
#define MPG_MODE_BIT (1<<MPG_MODE_PIN)
#endif

#if defined(I2C_STROBE_PIN) && !defined(I2C_STROBE_BIT)
#define I2C_STROBE_BIT (1<<I2C_STROBE_PIN)
#endif

#if defined(QEI_SELECT_PIN) && !defined(QEI_SELECT_BIT)
#define QEI_SELECT_BIT (1<<QEI_SELECT_PIN)
#endif

#endif // !AUX_DEVICES

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
#ifdef AUX_DEVICES
#define DEVICES_IRQ_MASK (SPI_IRQ_BIT|SPINDLE_INDEX_BIT|QEI_A_BIT|QEI_B_BIT|SD_DETECT_BIT)
#define DEVICES_IRQ_MASK_SUM (SPI_IRQ_BIT+SPINDLE_INDEX_BIT+QEI_A_BIT+QEI_B_BIT+SD_DETECT_BIT)
#else
#define DEVICES_IRQ_MASK (MPG_MODE_BIT|I2C_STROBE_BIT|QEI_SELECT_BIT|SPI_IRQ_BIT|SPINDLE_INDEX_BIT|QEI_A_BIT|QEI_B_BIT|SD_DETECT_BIT)
#define DEVICES_IRQ_MASK_SUM (MPG_MODE_BIT+I2C_STROBE_BIT+QEI_SELECT_BIT+SPI_IRQ_BIT+SPINDLE_INDEX_BIT+QEI_A_BIT+QEI_B_BIT+SD_DETECT_BIT)
#endif
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
