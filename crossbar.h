/*
  crossbar.h - signal crossbar definitions

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

#ifndef _CROSSBAR_H_
#define _CROSSBAR_H_

#include "nuts_bolts.h"

typedef enum {
// NOTE: the sequence of the following enums MUST match the control_signals_t layout
    Input_Reset = 0,
    Input_FeedHold,
    Input_CycleStart,
    Input_SafetyDoor,
    Input_BlockDelete,
    Input_StopDisable,
    Input_EStop,
    Input_ProbeDisconnect,
    Input_MotorFault,
    Input_MotorWarning,
    Input_LimitsOverride,
    Input_SingleBlock,
    Input_Unassigned,
    Input_ProbeOvertravel,
    Input_Probe,
// end control_signals_t sequence
    Input_Toolsetter,
    Input_ToolsetterOvertravel,
    Input_MPGSelect,
    Input_ModeSelect = Input_MPGSelect, // Deprecated
    Input_LimitX,
    Input_LimitX_2,
    Input_LimitX_Max,
    Input_HomeX,
    Input_HomeX_2,
    Input_LimitY,
    Input_LimitY_2,
    Input_LimitY_Max,
    Input_HomeY,
    Input_HomeY_2,
    Input_LimitZ,
    Input_LimitZ_2,
    Input_LimitZ_Max,
    Input_HomeZ,
    Input_HomeZ_2,
    Input_LimitA,
    Input_LimitA_Max,
    Input_HomeA,
    Input_LimitB,
    Input_LimitB_Max,
    Input_HomeB,
    Input_LimitC,
    Input_LimitC_Max,
    Input_HomeC,
    Input_LimitU,
    Input_LimitU_Max,
    Input_HomeU,
    Input_LimitV,
    Input_LimitV_Max,
    Input_HomeV,
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
    Input_Aux8,
    Input_Aux9,
    Input_Aux10,
    Input_Aux11,
    Input_AuxMax = Input_Aux11,
    Input_Analog_Aux0,
    Input_Analog_Aux1,
    Input_Analog_Aux2,
    Input_Analog_Aux3,
    Input_Analog_Aux4,
    Input_Analog_Aux5,
    Input_Analog_Aux6,
    Input_Analog_Aux7,
    Input_Analog_AuxMax = Input_Analog_Aux7,
// Output pins
    Output_StepX,
    Outputs = Output_StepX,
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
    Output_Spindle1On,
    Output_Spindle1Dir,
    Output_Spindle1PWM,
    Output_CoolantMist,
    Output_CoolantFlood,
    Output_Aux0,
    Output_Aux1,
    Output_Aux2,
    Output_Aux3,
    Output_Aux4,
    Output_Aux5,
    Output_Aux6,
    Output_Aux7,
    Output_Aux8,
    Output_Aux9,
    Output_Aux10,
    Output_Aux11,
    Output_Aux12,
    Output_Aux13,
    Output_Aux14,
    Output_Aux15,
    Output_AuxMax = Output_Aux15,
    Output_Analog_Aux0,
    Output_Analog_Aux1,
    Output_Analog_Aux2,
    Output_Analog_Aux3,
    Output_Analog_Aux4,
    Output_Analog_Aux5,
    Output_Analog_Aux6,
    Output_Analog_Aux7,
    Output_Analog_AuxMax = Output_Analog_Aux7,
    Output_LED,
    Output_LED_R,
    Output_LED_G,
    Output_LED_B,
    Output_LED_W,
    Output_LED_Adressable,
    Output_LED0_Adressable = Output_LED_Adressable,
    Output_LED1_Adressable,
    Output_CoProc_Reset,
    Output_CoProc_Boot0,
// Multipin peripherals
    Input_MISO,
    Multipin = Input_MISO,
    Output_MOSI,
    Output_SPICLK,
    Output_SPICS,
    Output_FlashCS,
    Output_SdCardCS,
    Input_SdCardDetect,
    Output_SPIRST,
    Input_SPIIRQ,
    Output_SCK,
    Output_I2CSCK = Output_SCK,
    Bidirectional_SDA,
    Bidirectional_I2CSDA = Bidirectional_SDA,
    Input_KeypadStrobe, // To be deprecated? Use Input_I2CStrobe instead.
    Input_I2CStrobe,
    Input_RX,
    Output_TX,
    Output_RTS,
    Input_QEI_A,
    Input_QEI_B,
    Input_QEI_Select,
    Input_QEI_Index,
// Single pin bidirectional peripherals
    Bidirectional_MotorUARTX,
    Bidirectional = Bidirectional_MotorUARTX,
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
    { .function = Input_Reset,                .name = "Reset" },
    { .function = Input_FeedHold,             .name = "Feed hold" },
    { .function = Input_CycleStart,           .name = "Cycle start" },
    { .function = Input_SafetyDoor,           .name = "Safety door" },
    { .function = Input_BlockDelete,          .name = "Block delete" },
    { .function = Input_StopDisable,          .name = "Stop disable" },
    { .function = Input_EStop,                .name = "Emergency stop" },
    { .function = Input_ProbeDisconnect,      .name = "Probe disconnect" },
    { .function = Input_MotorFault,           .name = "Motor fault" },
    { .function = Input_MotorWarning,         .name = "Motor warning" },
    { .function = Input_LimitsOverride,       .name = "Limits override" },
    { .function = Input_SingleBlock,          .name = "Single block" },
    { .function = Input_ProbeOvertravel,      .name = "Probe overtravel" },
    { .function = Input_Probe,                .name = "Probe" },
    { .function = Input_Toolsetter,           .name = "Toolsetter" },
    { .function = Input_ToolsetterOvertravel, .name = "Toolsetter overtravel" },
    { .function = Input_MPGSelect,            .name = "MPG mode select" },
    { .function = Input_LimitX,               .name = "X limit min" },
    { .function = Input_LimitX_2,             .name = "X limit min 2" },
    { .function = Input_LimitX_Max,           .name = "X limit max" },
    { .function = Input_HomeX,                .name = "X home" },
    { .function = Input_HomeX_2,              .name = "X home 2" },
    { .function = Input_LimitY,               .name = "Y limit min" },
    { .function = Input_LimitY_2,             .name = "Y limit min 2" },
    { .function = Input_LimitY_Max,           .name = "Y limit max" },
    { .function = Input_HomeY,                .name = "Y home" },
    { .function = Input_HomeY_2,              .name = "Y home 2" },
    { .function = Input_LimitZ,               .name = "Z limit min" },
    { .function = Input_LimitZ_2,             .name = "Z limit min 2" },
    { .function = Input_LimitZ_Max,           .name = "Z limit max" },
    { .function = Input_HomeZ,                .name = "Z home" },
    { .function = Input_HomeZ_2,              .name = "Z home 2" },
#ifndef NO_SETTINGS_DESCRIPTIONS
    { .function = Input_SpindleIndex,         .name = "Spindle index" },
    { .function = Input_SpindlePulse,         .name = "Spindle pulse" },
    { .function = Input_Aux0,                 .name = "Aux in 0" },
    { .function = Input_Aux1,                 .name = "Aux in 1" },
    { .function = Input_Aux2,                 .name = "Aux in 2" },
    { .function = Input_Aux3,                 .name = "Aux in 3" },
    { .function = Input_Aux4,                 .name = "Aux in 4" },
    { .function = Input_Aux5,                 .name = "Aux in 5" },
    { .function = Input_Aux6,                 .name = "Aux in 6" },
    { .function = Input_Aux7,                 .name = "Aux in 7" },
    { .function = Input_Aux8,                 .name = "Aux in 8" },
    { .function = Input_Aux9,                 .name = "Aux in 9" },
    { .function = Input_Aux10,                .name = "Aux in 10" },
    { .function = Input_Aux11,                .name = "Aux in 11" },
    { .function = Input_Analog_Aux0,          .name = "Aux analog in 0" },
    { .function = Input_Analog_Aux1,          .name = "Aux analog in 1" },
    { .function = Input_Analog_Aux2,          .name = "Aux analog in 2" },
    { .function = Input_Analog_Aux3,          .name = "Aux analog in 3" },
    { .function = Input_Analog_Aux4,          .name = "Aux analog in 4" },
    { .function = Input_Analog_Aux5,          .name = "Aux analog in 5" },
    { .function = Input_Analog_Aux6,          .name = "Aux analog in 6" },
    { .function = Input_Analog_Aux7,          .name = "Aux analog in 7" },
#endif
    { .function = Output_StepX,               .name = "X step" },
    { .function = Output_StepX_2,             .name = "X2 step" },
    { .function = Output_StepY,               .name = "Y step" },
    { .function = Output_StepY_2,             .name = "Y2 step" },
    { .function = Output_StepZ,               .name = "Z step" },
    { .function = Output_StepZ_2,             .name = "Z2 step" },
    { .function = Output_DirX,                .name = "X dir" },
    { .function = Output_DirX_2,              .name = "X2 dir" },
    { .function = Output_DirY,                .name = "Y dir" },
    { .function = Output_DirY_2,              .name = "Y2 dir" },
    { .function = Output_DirZ,                .name = "Z dir" },
    { .function = Output_DirZ_2,              .name = "Z2 dir" },
    { .function = Output_StepperPower,        .name = "Stepper power" },
    { .function = Output_StepperEnable,       .name = "Steppers enable" },
    { .function = Output_StepperEnableX,      .name = "X enable" },
    { .function = Output_StepperEnableY,      .name = "Y enable" },
    { .function = Output_StepperEnableZ,      .name = "Z enable" },
    { .function = Output_StepperEnableXY,     .name = "XY enable" },
#ifdef A_AXIS
    { .function = Output_StepA,               .name = "A step" },
    { .function = Output_DirA,                .name = "A dir" },
    { .function = Output_StepperEnableA,      .name = "A enable" },
    { .function = Input_LimitA,               .name = "A limit min" },
    { .function = Input_LimitA_Max,           .name = "A limit max" },
    { .function = Input_HomeA,                .name = "A home" },
#endif
#ifdef B_AXIS
    { .function = Output_StepB,               .name = "B step" },
    { .function = Output_DirB,                .name = "B dir" },
    { .function = Output_StepperEnableB,      .name = "B enable" },
    { .function = Output_StepperEnableAB,     .name = "AB enable" },
    { .function = Input_LimitB,               .name = "B limit min" },
    { .function = Input_LimitB_Max,           .name = "B limit max" },
    { .function = Input_HomeB,                .name = "B home" },
#endif
#ifdef C_AXIS
    { .function = Output_StepC,               .name = "C step" },
    { .function = Output_DirC,                .name = "C dir" },
    { .function = Output_StepperEnableC,      .name = "C enable" },
    { .function = Input_LimitC,               .name = "C limit min" },
    { .function = Input_LimitC_Max,           .name = "C limit max" },
    { .function = Input_HomeC,                .name = "C home" },
#endif
#ifdef U_AXIS
    { .function = Output_StepU,               .name = "U step" },
    { .function = Output_DirU,                .name = "U dir" },
    { .function = Output_StepperEnableU,      .name = "U enable" },
    { .function = Input_LimitU,               .name = "U limit min" },
    { .function = Input_LimitU_Max,           .name = "U limit max" },
    { .function = Input_HomeU,                .name = "U home" },
#endif
#ifdef V_AXIS
    { .function = Output_StepV,               .name = "V step" },
    { .function = Output_DirV,                .name = "V dir" },
    { .function = Output_StepperEnableV,      .name = "V enable" },
    { .function = Input_LimitV,               .name = "V limit min" },
    { .function = Input_LimitV_Max,           .name = "V limit max" },
    { .function = Input_HomeV,                .name = "V home" },
#endif
#ifndef NO_SETTINGS_DESCRIPTIONS
    { .function = Output_MotorChipSelect,     .name = "Motor CS" },
    { .function = Output_MotorChipSelectX,    .name = "Motor CSX" },
    { .function = Output_MotorChipSelectY,    .name = "Motor CSY" },
    { .function = Output_MotorChipSelectZ,    .name = "Motor CSZ" },
    { .function = Output_MotorChipSelectM3,   .name = "Motor CSM3" },
    { .function = Output_MotorChipSelectM4,   .name = "Motor CSM4" },
    { .function = Output_MotorChipSelectM5,   .name = "Motor CSM5" },
    { .function = Output_MotorChipSelectM6,   .name = "Motor CSM6" },
    { .function = Output_MotorChipSelectM7,   .name = "Motor CSM7" },
    { .function = Output_SpindleOn,           .name = "Spindle on" },
    { .function = Output_SpindleDir,          .name = "Spindle direction" },
    { .function = Output_SpindlePWM,          .name = "Spindle PWM" },
    { .function = Output_Spindle1On,          .name = "Spindle 2 on" },
    { .function = Output_Spindle1Dir,         .name = "Spindle 2 direction" },
    { .function = Output_Spindle1PWM,         .name = "Spindle 2 PWM" },
    { .function = Output_CoolantMist,         .name = "Mist" },
    { .function = Output_CoolantFlood,        .name = "Flood" },
    { .function = Output_Aux0,                .name = "Aux out 0" },
    { .function = Output_Aux1,                .name = "Aux out 1" },
    { .function = Output_Aux2,                .name = "Aux out 2" },
    { .function = Output_Aux3,                .name = "Aux out 3" },
    { .function = Output_Aux4,                .name = "Aux out 4" },
    { .function = Output_Aux5,                .name = "Aux out 5" },
    { .function = Output_Aux6,                .name = "Aux out 6" },
    { .function = Output_Aux7,                .name = "Aux out 7" },
    { .function = Output_Aux8,                .name = "Aux out 8" },
    { .function = Output_Aux9,                .name = "Aux out 9" },
    { .function = Output_Aux10,               .name = "Aux out 10" },
    { .function = Output_Aux11,               .name = "Aux out 11" },
    { .function = Output_Aux12,               .name = "Aux out 12" },
    { .function = Output_Aux13,               .name = "Aux out 13" },
    { .function = Output_Aux14,               .name = "Aux out 14" },
    { .function = Output_Aux15,               .name = "Aux out 15" },
    { .function = Output_Analog_Aux0,         .name = "Aux analog out 0" },
    { .function = Output_Analog_Aux1,         .name = "Aux analog out 1" },
    { .function = Output_Analog_Aux2,         .name = "Aux analog out 2" },
    { .function = Output_Analog_Aux3,         .name = "Aux analog out 3" },
    { .function = Output_Analog_Aux4,         .name = "Aux analog out 4" },
    { .function = Output_Analog_Aux5,         .name = "Aux analog out 5" },
    { .function = Output_Analog_Aux6,         .name = "Aux analog out 6" },
    { .function = Output_Analog_Aux7,         .name = "Aux analog out 7" },
    { .function = Output_LED,                 .name = "LED" },
    { .function = Output_LED_R,               .name = "LED R" },
    { .function = Output_LED_G,               .name = "LED G" },
    { .function = Output_LED_B,               .name = "LED B" },
    { .function = Output_LED_W,               .name = "LED W" },
    { .function = Output_LED_Adressable,      .name = "LED adressable" },
    { .function = Output_LED1_Adressable,     .name = "LED adressable 1" },
    { .function = Output_CoProc_Reset,        .name = "CoProc Reset" },
    { .function = Output_CoProc_Boot0,        .name = "CoProc Boot0" },
    { .function = Input_MISO,                 .name = "MISO" },
    { .function = Output_MOSI,                .name = "MOSI" },
    { .function = Output_SPICLK,              .name = "SPI CLK" },
    { .function = Output_SPICS,               .name = "SPI CS" },
    { .function = Output_FlashCS,             .name = "Flash CS" },
    { .function = Output_SdCardCS,            .name = "SD card CS" },
    { .function = Input_SdCardDetect,         .name = "SD card detect" },
    { .function = Output_SPIRST,              .name = "SPI reset" },
    { .function = Input_SPIIRQ,               .name = "SPI IRQ" },
    { .function = Output_I2CSCK,              .name = "I2C SCK" },
    { .function = Bidirectional_SDA,          .name = "I2C SDA" },
    { .function = Input_KeypadStrobe,         .name = "Keypad strobe" },
    { .function = Input_I2CStrobe,            .name = "I2C strobe" },
    { .function = Input_RX,                   .name = "RX" },
    { .function = Output_TX,                  .name = "TX" },
    { .function = Output_RTS,                 .name = "RTS" },
    { .function = Input_QEI_A,                .name = "QEI A" },
    { .function = Input_QEI_B,                .name = "QEI B" },
    { .function = Input_QEI_Select,           .name = "QEI select" },
    { .function = Input_QEI_Index,            .name = "QEI index" },
    { .function = Bidirectional_MotorUARTX,   .name = "UART X" },
    { .function = Bidirectional_MotorUARTY,   .name = "UART Y" },
    { .function = Bidirectional_MotorUARTZ,   .name = "UART Z" },
    { .function = Bidirectional_MotorUARTM3,  .name = "UART M3" },
    { .function = Bidirectional_MotorUARTM4,  .name = "UART M4" },
    { .function = Bidirectional_MotorUARTM5,  .name = "UART M5" },
    { .function = Bidirectional_MotorUARTM6,  .name = "UART M6" },
    { .function = Bidirectional_MotorUARTM7,  .name = "UART M7" }
#endif
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
    PinGroup_AuxInputAnalog,
    PinGroup_AuxOutputAnalog,
    PinGroup_MotorChipSelect,
    PinGroup_MotorUART,
    PinGroup_I2C,
    PinGroup_SPI,
    PinGroup_UART1,
    PinGroup_UART = PinGroup_UART1,
    PinGroup_UART2,
    PinGroup_UART3,
    PinGroup_UART4,
    PinGroup_USB,
    PinGroup_CAN,
    PinGroup_LED,
    PinGroup_Home,
// Interrupt capable pins that may have debounce processing enabled
    PinGroup_Control       = (1<<8),
    PinGroup_Limit         = (1<<9),
    PinGroup_LimitMax      = (1<<10),
    PinGroup_Probe         = (1<<11),
    PinGroup_Keypad        = (1<<12),
    PinGroup_MPG           = (1<<13),
    PinGroup_QEI           = (1<<14),
    PinGroup_QEI_Select    = (1<<15),
    PinGroup_QEI_Index     = (1<<16),
    PinGroup_Motor_Warning = (1<<17),
    PinGroup_Motor_Fault   = (1<<18),
    PinGroup_SdCard        = (1<<19),
    PinGroup_AuxInput      = (1<<20)
} pin_group_t;

//! Pin interrupt modes, may be or'ed when reporting pin capability.
typedef enum {
    IRQ_Mode_None          = 0b00000, //!< 0b00000 (0x00)
    IRQ_Mode_Rising        = 0b00001, //!< 0b00001 (0x01)
    IRQ_Mode_Falling       = 0b00010, //!< 0b00010 (0x02)
    IRQ_Mode_RisingFalling = 0b00011, //!< 0b00011 (0x03) - only used to report port capability.
    IRQ_Mode_Change        = 0b00100, //!< 0b00100 (0x04)
    IRQ_Mode_Edges         = 0b00111, //!< 0b00111 (0x07) - only used to report port capability.
    IRQ_Mode_High          = 0b01000, //!< 0b01000 (0x08)
    IRQ_Mode_Low           = 0b10000, //!< 0b10000 (0x10)
    IRQ_Mode_All           = 0b11111  //!< 0b11111 (0x1F) - only used to report port capability.
} pin_irq_mode_t;

typedef enum {
    IRQ_I2C_Strobe = 0,
    IRQ_SPI
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

#define PINMODE_NONE        (0)
#define PINMODE_OUTPUT      (1U<<1)
#ifndef __LPC17XX__
#define PINMODE_OD          (1U<<2)
#endif
#define PINMODE_PULLUP      (PullMode_Up<<3)
#define PINMODE_PULLDOWN    (PullMode_Down<<3)
#define PINMODE_ANALOG      (1U<<11)
#define PINMODE_PWM         (1U<<12)
#define PINMODE_PWM_SERVO   (1U<<13)

typedef union {
    uint16_t mask;
    struct {
        uint16_t input      :1,
                 output     :1,
                 open_drain :1,
                 pull_mode  :2,
                 irq_mode   :5,
                 invert     :1,
                 analog     :1,
                 pwm        :1,
                 servo_pwm  :1,
                 claimable  :1,
                 debounce   :1;
    };
} pin_cap_t;

typedef union {
    uint16_t mask;
    struct {
        uint16_t input      :1,
                 output     :1,
                 open_drain :1,
                 pull_mode  :2,
                 irq_mode   :5,
                 inverted   :1,
                 analog     :1,
                 pwm        :1,
                 servo_pwm  :1,
                 claimed    :1,
                 debounce   :1;
    };
} pin_mode_t;

#define XBAR_SET_CAP(cap, mode) { cap.mask = mode.mask; cap.claimable = !mode.claimed; }
#define XBAR_SET_DIN_INFO(pin, pin_id, src, cfg_fn, get_val_fn) { \
  pin.id = pin_id; \
  pin.mode = src.mode; \
  pin.cap = src.cap; \
  pin.cap.invert = On; \
  pin.cap.claimable = !src.mode.claimed; \
  pin.function = src.id; \
  pin.group = src.group; \
  pin.pin = src.pin; \
  pin.port = (void *)src.port; \
  pin.description = src.description; \
  pin.config = cfg_fn; \
  pin.get_value = get_val_fn; \
}
#define XBAR_SET_DOUT_INFO(pin, pin_id, src, cfg_fn, get_val_fn) { \
  pin.id = pin_id; \
  pin.mode = src.mode; \
  pin.cap.mask = src.mode.mask; \
  pin.cap.invert = On; \
  pin.cap.claimable = !src.mode.claimed; \
  pin.function = src.id; \
  pin.group = src.group; \
  pin.pin = src.pin; \
  pin.port = (void *)src.port; \
  pin.description = src.description; \
  pin.config = cfg_fn; \
  pin.get_value = get_val_fn; \
}

//! /a cfg_data argument to /a xbar_config_ptr for gpio input pins
typedef struct {
    bool inverted;
    bool debounce;
    pull_mode_t pull_mode;
} gpio_in_config_t;

//! /a cfg_data argument to /a xbar_config_ptr for gpio output pins
typedef struct {
    bool inverted;
    bool open_drain;
    bool pwm;
} gpio_out_config_t;

//! /a cfg_data argument to /a xbar_config_ptr for PWM pins
typedef struct {
    float freq_hz;   //
    float min;
    float max;
    float off_value; // percent of period
    float min_value; // percent of period
    float max_value; // percent of period
    bool invert;
    bool servo_mode;
} pwm_config_t;

typedef union
{
    pwm_config_t *pwm_config;
    gpio_in_config_t *gpio_in_config;
    gpio_out_config_t *gpio_out_config;
} xbar_cfg_ptr_t __attribute__ ((__transparent_union__));

struct xbar;

typedef float (*xbar_get_value_ptr)(struct xbar *pin);
typedef void (*xbar_set_value_ptr)(struct xbar *pin, float value);
typedef void (*xbar_event_ptr)(bool on);
typedef bool (*xbar_config_ptr)(struct xbar *pin, xbar_cfg_ptr_t cfg_data, bool persistent);

typedef struct {
    pin_function_t function;
    uint8_t aux_port;
    pin_irq_mode_t irq_mode;
    control_signals_t cap;
    uint8_t pin;
    void *port;
    void *input;
} aux_ctrl_t;

typedef struct {
    pin_function_t function;
    uint8_t aux_port;
    uint8_t pin;
    void *port;
    void *output;
} aux_ctrl_out_t;

typedef struct xbar {
    uint8_t id;                     //!< Pin id.
    pin_function_t function;        //!< Pin function.
    pin_group_t group;              //!< Pin group.
    void *port;                     //!< Optional pointer to the underlying peripheral or pin specific data.
    const char *description;        //!< Optional pointer to description string.
    uint_fast8_t pin;               //!< Pin number.
    pin_cap_t cap;                  //!< Pin capabilities.
    pin_mode_t mode;                //!< Current pin configuration.
    xbar_config_ptr config;         //!< Optional pointer to function for configuring the port.
    xbar_get_value_ptr get_value;   //!< Optional pointer to function to get current port value.
    xbar_set_value_ptr set_value;   //!< Optional pointer to function to set port value.
    xbar_event_ptr on_event;        //!< Not used - might be removed.
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

typedef union {
    uint8_t mask;
    struct {
        uint8_t limits      :1,
                aux_inputs  :1,
                safety_door :1,
                qei_select  :1,
                unassigned  :4;
    };
} pin_debounce_t;

void xbar_set_homing_source (void);
limit_signals_t xbar_get_homing_source (void);
limit_signals_t xbar_get_homing_source_from_cycle (axes_signals_t homing_cycle);
axes_signals_t xbar_fn_to_axismask (pin_function_t id);
const char *xbar_fn_to_pinname (pin_function_t id);
control_signals_t xbar_fn_to_signals_mask (pin_function_t id);

#endif
