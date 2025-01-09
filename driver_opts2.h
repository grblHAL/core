/*
  driver_opts2.h - for preprocessing options from my_machine.h, compiler symbols or from the board map file

  NOTE: This file is not used by the core, it may be included by drivers after the map file is included

  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io

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

#if (DRIVER_SPINDLE_ENABLE & SPINDLE_ENA) && !defined(SPINDLE_ENABLE_PIN)
#warning "Selected spindle is not supported!"
#endif

#if (DRIVER_SPINDLE_ENABLE & SPINDLE_DIR) && !defined(SPINDLE_DIRECTION_PIN)
#warning "Selected spindle is not fully supported - no direction output!"
#endif

#if (DRIVER_SPINDLE_ENABLE & SPINDLE_PWM) && !defined(SPINDLE_PWM_PIN)
#warning "Selected spindle is not supported!"
#endif

#if (DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM) && !defined(SPINDLE_PWM_PIN)
#warning "Selected spindle 1 is not supported!"
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

#if TRINAMIC_SPI_ENABLE == 1 || TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 2660 || TRINAMIC_ENABLE == 5160

#undef TRINAMIC_SPI_ENABLE
#define TRINAMIC_SPI_40BIT     (1<<1)
#define TRINAMIC_SPI_20BIT     (1<<2)
#define TRINAMIC_SPI_CS_SINGLE (1<<3)

#if TRINAMIC_ENABLE == 2660
#ifdef MOTOR_CS_PORT
#define TRINAMIC_SPI_ENABLE (TRINAMIC_SPI_20BIT|TRINAMIC_SPI_CS_SINGLE)
#else
#define TRINAMIC_SPI_ENABLE TRINAMIC_SPI_20BIT
#endif
#else
#ifdef MOTOR_CS_PIN
#define TRINAMIC_SPI_ENABLE (TRINAMIC_SPI_40BIT|TRINAMIC_SPI_CS_SINGLE)
#else
#define TRINAMIC_SPI_ENABLE TRINAMIC_SPI_40BIT
#endif
#endif
#endif // TRINAMIC_SPI_ENABLE ...

#endif // TRINAMIC_ENABLE

#ifndef TRINAMIC_I2C
#define TRINAMIC_I2C        0
#endif
#if TRINAMIC_ENABLE && TRINAMIC_I2C
#define TRINAMIC_MOTOR_ENABLE 1
#else
#define TRINAMIC_MOTOR_ENABLE 0
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

#if defined(COPROC_RESET_PIN) && defined(COPROC_BOOT0_PIN)
#define COPROC_PASSTHRU 1
#else
#define COPROC_PASSTHRU 0
#endif

#ifndef COPROC_STREAM
#if USB_SERIAL_CDC
#define COPROC_STREAM     0
#else
#define COPROC_STREAM     1
#endif
#endif

/**/
