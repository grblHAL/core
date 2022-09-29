/*
  crossbar.h - signal crossbar definitions

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

#include "nuts_bolts.h"

typedef enum {
    Input_Probe = 0,
    Input_Reset,
    Input_FeedHold,
    Input_CycleStart,
    Input_SafetyDoor,
    Input_LimitsOverride,
    Input_EStop,
    Input_MPGSelect,
    Input_ModeSelect = Input_MPGSelect, // Deprecated
    Input_LimitX,
    Input_LimitX_2,
    Input_LimitX_Max,
    Input_LimitY,
    Input_LimitY_2,
    Input_LimitY_Max,
    Input_LimitZ,
    Input_LimitZ_2,
    Input_LimitZ_Max,
    Input_LimitA,
    Input_LimitA_Max,
    Input_LimitB,
    Input_LimitB_Max,
    Input_LimitC,
    Input_LimitC_Max,
    Input_LimitU,
    Input_LimitU_Max,
    Input_LimitV,
    Input_LimitV_Max,
    Input_MISO,
    Input_RX,
    Input_KeypadStrobe, // To be deprecated?
    Input_I2CStrobe,
    Input_QEI_A,
    Input_QEI_B,
    Input_QEI_Select,
    Input_QEI_Index,
    Input_SpindleIndex,
    Input_SpindlePulse,
    Input_Aux0,
    Input_Aux1,
    Input_Aux2,
    Input_Aux3,
    Input_Aux4,
    Input_Aux5,
    Input_Aux6,
    Input_Aux7,
    Input_MotorWarning,
    Input_MotorFault,
    Outputs,
    Output_StepX = Outputs,
    Output_StepX_2,
    Output_StepY,
    Output_StepY_2,
    Output_StepZ,
    Output_StepZ_2,
    Output_StepA,
    Output_StepB,
    Output_StepC,
    Output_StepU,
    Output_StepV,
    Output_DirX,
    Output_DirX_2,
    Output_DirY,
    Output_DirY_2,
    Output_DirZ,
    Output_DirZ_2,
    Output_DirA,
    Output_DirB,
    Output_DirC,
    Output_DirU,
    Output_DirV,
    Output_MotorChipSelect,
    Output_MotorChipSelectX,
    Output_MotorChipSelectY,
    Output_MotorChipSelectZ,
    Output_MotorChipSelectM3,
    Output_MotorChipSelectM4,
    Output_MotorChipSelectM5,
    Output_MotorChipSelectM6,
    Output_MotorChipSelectM7,
    Output_StepperPower,
    Output_StepperEnable,
    Output_StepperEnableX,
    Output_StepperEnableY,
    Output_StepperEnableZ,
    Output_StepperEnableA,
    Output_StepperEnableB,
    Output_StepperEnableU,
    Output_StepperEnableV,
    Output_StepperEnableC,
    Output_StepperEnableXY,
    Output_StepperEnableAB,
    Output_SpindleOn,
    Output_SpindleDir,
    Output_SpindlePWM,
    Output_CoolantMist,
    Output_CoolantFlood,
    Output_TX,
    Output_RTS,
    Output_SCK,
    Output_MOSI,
    Output_SdCardCS,
    Output_Aux0,
    Output_Aux1,
    Output_Aux2,
    Output_Aux3,
    Output_Aux4,
    Output_Aux5,
    Output_Aux6,
    Output_Aux7,
    Bidirectional,
    Bidirectional_SDA = Bidirectional,
    Bidirectional_MotorUARTX,
    Bidirectional_MotorUARTY,
    Bidirectional_MotorUARTZ,
    Bidirectional_MotorUARTM3,
    Bidirectional_MotorUARTM4,
    Bidirectional_MotorUARTM5,
    Bidirectional_MotorUARTM6,
    Bidirectional_MotorUARTM7
} pin_function_t;

#define PIN_ISINPUT(pin) (pin < Outputs)
#define PIN_ISOUTPUT(pin) (pin >= Outputs && pin < Bidirectional)
#define PIN_ISBIDIRECTIONAL(pin) (pin >= Bidirectional)

typedef struct {
    pin_function_t function;
    const char *name;
} pin_name_t;

PROGMEM static const pin_name_t pin_names[] = {
   { .function = Input_Probe,               .name = "Probe" },
   { .function = Input_Reset,               .name = "Reset" },
   { .function = Input_FeedHold,            .name = "Feed hold" },
   { .function = Input_CycleStart,          .name = "Cycle start" },
   { .function = Input_SafetyDoor,          .name = "Safety door" },
   { .function = Input_LimitsOverride,      .name = "Limits override" },
   { .function = Input_EStop,               .name = "Emergency stop" },
   { .function = Input_MPGSelect,           .name = "MPG mode select" },
   { .function = Input_LimitX,              .name = "X limit min" },
   { .function = Input_LimitX_2,            .name = "X limit min 2" },
   { .function = Input_LimitX_Max,          .name = "X limit max" },
   { .function = Input_LimitY,              .name = "Y limit min" },
   { .function = Input_LimitY_2,            .name = "Y limit min 2" },
   { .function = Input_LimitY_Max,          .name = "Y limit max" },
   { .function = Input_LimitZ,              .name = "Z limit min" },
   { .function = Input_LimitZ_2,            .name = "Z limit min 2" },
   { .function = Input_LimitZ_Max,          .name = "Z limit max" },
   { .function = Input_MISO,                .name = "MISO" },
   { .function = Input_RX,                  .name = "RX" },
   { .function = Input_KeypadStrobe,        .name = "Keypad strobe" },
   { .function = Input_I2CStrobe,           .name = "I2C strobe" },
   { .function = Input_QEI_A,               .name = "QEI A" },
   { .function = Input_QEI_B,               .name = "QEI B" },
   { .function = Input_QEI_Select,          .name = "QEI select" },
   { .function = Input_QEI_Index,           .name = "QEI index" },
   { .function = Input_SpindleIndex,        .name = "Spindle index" },
   { .function = Input_SpindlePulse,        .name = "Spindle pulse" },
   { .function = Input_MotorWarning,        .name = "Motor warning" },
   { .function = Input_MotorFault,          .name = "Motor fault" },
   { .function = Input_Aux0,                .name = "Aux input 0" },
   { .function = Input_Aux1,                .name = "Aux input 1" },
   { .function = Input_Aux2,                .name = "Aux input 2" },
   { .function = Input_Aux3,                .name = "Aux input 3" },
   { .function = Input_Aux4,                .name = "Aux input 4" },
   { .function = Input_Aux5,                .name = "Aux input 5" },
   { .function = Input_Aux6,                .name = "Aux input 6" },
   { .function = Input_Aux7,                .name = "Aux input 7" },
   { .function = Output_StepX,              .name = "X step" },
   { .function = Output_StepX_2,            .name = "X2 step" },
   { .function = Output_StepY,              .name = "Y step" },
   { .function = Output_StepY_2,            .name = "Y2 step" },
   { .function = Output_StepZ,              .name = "Z step" },
   { .function = Output_StepZ_2,            .name = "Z2 step" },
   { .function = Output_DirX,               .name = "X dir" },
   { .function = Output_DirX_2,             .name = "X2 dir" },
   { .function = Output_DirY,               .name = "Y dir" },
   { .function = Output_DirY_2,             .name = "Y2 dir" },
   { .function = Output_DirZ,               .name = "Z dir" },
   { .function = Output_DirZ_2,             .name = "Z2 dir" },
   { .function = Output_StepperPower,       .name = "Stepper power" },
   { .function = Output_StepperEnable,      .name = "Steppers enable" },
   { .function = Output_StepperEnableX,     .name = "X enable" },
   { .function = Output_StepperEnableY,     .name = "Y enable" },
   { .function = Output_StepperEnableZ,     .name = "Z enable" },
   { .function = Output_StepperEnableXY,    .name = "XY enable" },
#ifdef A_AXIS
   { .function = Output_StepA,              .name = "A step" },
   { .function = Output_DirA,               .name = "A dir" },
   { .function = Output_StepperEnableA,     .name = "A enable" },
   { .function = Input_LimitA,              .name = "A limit min" },
   { .function = Input_LimitA_Max,          .name = "A limit max" },
#endif
#ifdef B_AXIS
   { .function = Output_StepB,              .name = "B step" },
   { .function = Output_DirB,               .name = "B dir" },
   { .function = Output_StepperEnableB,     .name = "B enable" },
   { .function = Output_StepperEnableAB,    .name = "AB enable" },
   { .function = Input_LimitB,              .name = "B limit min" },
   { .function = Input_LimitB_Max,          .name = "B limit max" },
#endif
#ifdef C_AXIS
   { .function = Output_StepC,              .name = "C step" },
   { .function = Output_DirC,               .name = "C dir" },
   { .function = Output_StepperEnableC,     .name = "C enable" },
   { .function = Input_LimitC,              .name = "C limit min" },
   { .function = Input_LimitC_Max,          .name = "C limit max" },
#endif
#ifdef U_AXIS
   { .function = Output_StepU,              .name = "U step" },
   { .function = Output_DirU,               .name = "U dir" },
   { .function = Output_StepperEnableU,     .name = "U enable" },
   { .function = Input_LimitU,              .name = "U limit min" },
   { .function = Input_LimitU_Max,          .name = "U limit max" },
#endif
#ifdef V_AXIS
   { .function = Output_StepV,              .name = "V step" },
   { .function = Output_DirV,               .name = "V dir" },
   { .function = Output_StepperEnableV,     .name = "V enable" },
   { .function = Input_LimitV,              .name = "V limit min" },
   { .function = Input_LimitV_Max,          .name = "V limit max" },
#endif
   { .function = Output_MotorChipSelect,    .name = "Motor CS" },
   { .function = Output_MotorChipSelectX,   .name = "Motor CSX" },
   { .function = Output_MotorChipSelectY,   .name = "Motor CSY" },
   { .function = Output_MotorChipSelectZ,   .name = "Motor CSZ" },
   { .function = Output_MotorChipSelectM3,  .name = "Motor CSM3" },
   { .function = Output_MotorChipSelectM4,  .name = "Motor CSM4" },
   { .function = Output_MotorChipSelectM5,  .name = "Motor CSM5" },
   { .function = Output_MotorChipSelectM6,  .name = "Motor CSM6" },
   { .function = Output_MotorChipSelectM7,  .name = "Motor CSM7" },
   { .function = Output_SpindleOn,          .name = "Spindle on" },
   { .function = Output_SpindleDir,         .name = "Spindle direction" },
   { .function = Output_SpindlePWM,         .name = "Spindle PWM" },
   { .function = Output_CoolantMist,        .name = "Mist" },
   { .function = Output_CoolantFlood,       .name = "Flood" },
   { .function = Output_TX,                 .name = "TX" },
   { .function = Output_RTS,                .name = "RTS" },
   { .function = Output_SCK,                .name = "SCK" },
   { .function = Output_MOSI,               .name = "MOSI" },
   { .function = Output_SdCardCS,           .name = "SD card CS" },
   { .function = Output_Aux0,               .name = "Aux out 0" },
   { .function = Output_Aux1,               .name = "Aux out 1" },
   { .function = Output_Aux2,               .name = "Aux out 2" },
   { .function = Output_Aux3,               .name = "Aux out 3" },
   { .function = Output_Aux4,               .name = "Aux out 4" },
   { .function = Output_Aux5,               .name = "Aux out 5" },
   { .function = Output_Aux6,               .name = "Aux out 6" },
   { .function = Output_Aux7,               .name = "Aux out 7" },
   { .function = Bidirectional_SDA,         .name = "SDA" },
   { .function = Bidirectional_MotorUARTX,  .name = "UART X" },
   { .function = Bidirectional_MotorUARTY,  .name = "UART Y" },
   { .function = Bidirectional_MotorUARTZ,  .name = "UART Z" },
   { .function = Bidirectional_MotorUARTM3, .name = "UART M3" },
   { .function = Bidirectional_MotorUARTM4, .name = "UART M4" },
   { .function = Bidirectional_MotorUARTM5, .name = "UART M5" },
   { .function = Bidirectional_MotorUARTM6, .name = "UART M6" },
   { .function = Bidirectional_MotorUARTM7, .name = "UART M7" }
};

typedef enum {
    PinGroup_SpindleControl = 0,
    PinGroup_SpindlePWM,
    PinGroup_Coolant,
    PinGroup_SpindlePulse,
    PinGroup_SpindleIndex,
    PinGroup_StepperPower,
    PinGroup_StepperEnable,
    PinGroup_StepperStep,
    PinGroup_StepperDir,
    PinGroup_AuxOutput,
    PinGroup_SdCard,
    PinGroup_MotorChipSelect,
    PinGroup_MotorUART,
    PinGroup_I2C,
    PinGroup_SPI,
    PinGroup_UART,
    PinGroup_UART1 = PinGroup_UART,
    PinGroup_UART2,
    PinGroup_UART3,
    PinGroup_UART4,
    PinGroup_USB,
// Interrupt capable pins that may have debounce processing enabled
    PinGroup_Control       = (1<<8),
    PinGroup_Limit         = (1<<9),
    PinGroup_Probe         = (1<<10),
    PinGroup_Keypad        = (1<<11),
    PinGroup_MPG           = (1<<12),
    PinGroup_QEI           = (1<<13),
    PinGroup_QEI_Select    = (1<<14),
    PinGroup_QEI_Index     = (1<<15),
    PinGroup_Motor_Warning = (1<<16),
    PinGroup_Motor_Fault   = (1<<17),
    PinGroup_AuxInput      = (1<<18)
} pin_group_t;

//! Pin interrupt modes, may be or'ed when reporting pin capability.
typedef enum {
    IRQ_Mode_None    = 0b00000, //!< 0b00000 (0x00)
    IRQ_Mode_Rising  = 0b00001, //!< 0b00001 (0x01)
    IRQ_Mode_Falling = 0b00010, //!< 0b00010 (0x02)
    IRQ_Mode_Change  = 0b00100, //!< 0b00100 (0x04)
    IRQ_Mode_Edges   = 0b00111, //!< 0b00111 (0x07) - only used to report port capability.
    IRQ_Mode_High    = 0b01000, //!< 0b01000 (0x08)
    IRQ_Mode_Low     = 0b10000, //!< 0b10000 (0x10)
    IRQ_Mode_All     = 0b11111  //!< 0b11111 (0x1F) - only used to report port capability.
} pin_irq_mode_t;

typedef enum {
    IRQ_I2C_Strobe = 0
} irq_type_t;

typedef bool (*irq_callback_ptr)(uint_fast8_t id, bool level);

typedef struct driver_irq_handler {
    irq_type_t type;
    irq_callback_ptr callback;
    struct driver_irq_handler *next;
} driver_irq_handler_t;

//! Pin pullup and pulldown modes, may be or'ed when reporting pin capability.
typedef enum {
    PullMode_None    = 0b00, //!< 0b00 (0x00)
    PullMode_Up      = 0b01, //!< 0b01 (0x01)
    PullMode_Down    = 0b10, //!< 0b10 (0x02)
    PullMode_UpDown  = 0b11  //!< 0b11 (0x03) - only used to report port capability.
} pull_mode_t;

#define PINMODE_NONE     (0)
#define PINMODE_OUTPUT   (1U<<1)
#ifndef __LPC17XX__
#define PINMODE_OD       (1U<<2)
#endif
#define PINMODE_PULLUP   (PullMode_Up<<3)
#define PINMODE_PULLDOWN (PullMode_Down<<3)
#define PINMODE_REMAP    (1U<<10)

typedef union {
    uint16_t mask;
    struct {
        uint16_t input      :1,
                 output     :1,
                 open_drain :1,
                 pull_mode  :2,
                 irq_mode   :5,
                 pwm        :1,
                 analog     :1,
                 peripheral :1,
                 claimed    :1,
                 remapped   :1,
                 can_remap  :1;
    };
} pin_mode_t;

typedef bool (*xbar_get_value_ptr)(void);
typedef void (*xbar_set_value_ptr)(bool on);
typedef void (*xbar_event_ptr)(bool on);
typedef void (*xbar_config_ptr)(void *cfg_data);

typedef struct {
    pin_function_t function;
    pin_group_t group;
    void *port;
    const char *description;
    uint_fast8_t pin;
    uint32_t bit;
    pin_mode_t mode;
    pin_mode_t cap;
    xbar_config_ptr config;
    xbar_get_value_ptr get_value;
    xbar_set_value_ptr set_value;
    xbar_event_ptr on_event;
} xbar_t;

typedef struct {
    pin_function_t function;
    pin_group_t group;
    void *port;
    uint_fast8_t pin;
    pin_mode_t mode;
    const char *description;
} periph_pin_t;

typedef struct periph_signal {
    periph_pin_t pin;
    struct periph_signal *next;
} periph_signal_t;

#endif
