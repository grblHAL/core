/*
  driver_opts2.h - for preprocessing options from my_machine.h, compiler symbols or from the board map file

  NOTE: This file is not used by the core, it may be included by drivers after the map file is included

  Part of grblHAL

  Copyright (c) 2024 Terje Io

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

//
// NOTE: do NOT change options here - edit the driver specific my_machine.h instead!
//

#if DRIVER_SPINDLE_ENABLE && !defined(SPINDLE_ENABLE_PIN)
#warning "Selected spindle is not supported!"
#undef DRIVER_SPINDLE_ENABLE
#define DRIVER_SPINDLE_ENABLE 0
#endif

#if DRIVER_SPINDLE_DIR_ENABLE && !defined(SPINDLE_DIRECTION_PIN)
#warning "Selected spindle is not fully supported - no direction output!"
#undef DRIVER_SPINDLE_DIR_ENABLE
#define DRIVER_SPINDLE_DIR_ENABLE 0
#endif

#if DRIVER_SPINDLE_PWM_ENABLE && (!DRIVER_SPINDLE_ENABLE || !defined(SPINDLE_PWM_PIN))
#warning "Selected spindle is not supported!"
#undef DRIVER_SPINDLE_PWM_ENABLE
#define DRIVER_SPINDLE_PWM_ENABLE 0
#endif

#if DRIVER_SPINDLE_PWM1_ENABLE && (!DRIVER_SPINDLE1_ENABLE || !defined(SPINDLE1_PWM_PIN))
#warning "Selected spindle 1 is not supported!"
#undef DRIVER_SPINDLE_PWM1_ENABLE
#define DRIVER_SPINDLE_PWM1_ENABLE 0
#endif

#if MPG_ENABLE == 1 && !defined(MPG_MODE_PIN)
#error "MPG_MODE_PIN must be defined!"
#endif

#if KEYPAD_ENABLE == 1 && !defined(I2C_STROBE_PORT)
#error Keypad plugin not supported!
#elif I2C_STROBE_ENABLE && !defined(I2C_STROBE_PORT)
#error I2C strobe not supported!
#endif

#if EEPROM_ENABLE == 0
#define FLASH_ENABLE 1
#else
#define FLASH_ENABLE 0
#endif

#if TRINAMIC_ENABLE
#include "motors/trinamic.h"
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#if TRINAMIC_UART_ENABLE == 1 && !defined(TRINAMIC_STREAM)
#define TRINAMIC_STREAM 1
#endif
#endif

#if USB_SERIAL_CDC && defined(SERIAL_PORT)
#define SP0 1
#else
#define SP0 0
#endif

#ifdef SERIAL1_PORT
#define SP1 1
#else
#define SP1 0
#endif

#ifdef SERIAL2_PORT
#define SP2 1
#else
#define SP2 0
#endif

#if MODBUS_ENABLE & MODBUS_RTU_ENABLED
#define MODBUS_TEST 1
#else
#define MODBUS_TEST 0
#endif

#if TRINAMIC_ENABLE && TRINAMIC_UART_ENABLE == 1
#define TRINAMIC_TEST 1
#else
#define TRINAMIC_TEST 0
#endif

#if KEYPAD_ENABLE == 2 && MPG_ENABLE == 0
#define KEYPAD_TEST 1
#else
#define KEYPAD_TEST 0
#endif

#if (MODBUS_TEST + KEYPAD_TEST + (MPG_ENABLE ? 1 : 0) + TRINAMIC_TEST + (BLUETOOTH_ENABLE == 2 ? 1 : 0)) > (SP0 + SP1 + SP2)
#error "Too many options that uses a serial port are enabled!"
#endif

#undef SP0
#undef SP1
#undef SP2
#undef MODBUS_TEST
#undef KEYPAD_TEST
#undef TRINAMIC_TEST

#if KEYPAD_ENABLE == 2 && !defined(KEYPAD_STREAM)
#if USB_SERIAL_CDC
#define KEYPAD_STREAM     0
#else
#define KEYPAD_STREAM     1
#endif
#if (MODBUS_ENABLE & MODBUS_RTU_ENABLED) && defined(MODBUS_RTU_STREAM) && MODBUS_RTU_STREAM == MPG_STREAM
#undef KEYPAD_STREAM
#define KEYPAD_STREAM (MODBUS_RTU_STREAM + 1)
#endif
#endif

#if MPG_ENABLE && !defined(MPG_STREAM)
#if USB_SERIAL_CDC
#define MPG_STREAM          0
#else
#define MPG_STREAM          1
#endif
#if (MODBUS_ENABLE & MODBUS_RTU_ENABLED) && defined(MODBUS_RTU_STREAM) && MODBUS_RTU_STREAM == MPG_STREAM
#undef MPG_STREAM
#define MPG_STREAM (MODBUS_RTU_STREAM + 1)
#endif
#endif

#ifndef COPROC_STREAM
#if USB_SERIAL_CDC
#define COPROC_STREAM     0
#else
#define COPROC_STREAM     1
#endif
#endif

/**/
