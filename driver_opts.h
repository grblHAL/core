/*
  driver_opts.h - for preprocessing options from my_machine.h or compiler symbols

  NOTE: This file is not used by the core, it may be used by drivers

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

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

#pragma once

#include "hal.h"
#include "nuts_bolts.h"

#ifdef OPTS_POSTPROCESSING
#define NO_OPTS_POST 0
#else
#define NO_OPTS_POST 1
#endif

#ifndef X_GANGED
#define X_GANGED            0
#endif
#ifndef X_AUTO_SQUARE
#define X_AUTO_SQUARE       0
#endif
#ifndef Y_GANGED
#define Y_GANGED            0
#endif
#ifndef Y_AUTO_SQUARE
#define Y_AUTO_SQUARE       0
#endif
#ifndef Z_GANGED
#define Z_GANGED            0
#endif
#ifndef Z_AUTO_SQUARE
#define Z_AUTO_SQUARE       0
#endif
#ifndef X_GANGED_LIM_MAX
#define X_GANGED_LIM_MAX    0
#endif
#ifndef Y_GANGED_LIM_MAX
#define Y_GANGED_LIM_MAX    0
#endif
#ifndef Z_GANGED_LIM_MAX
#define Z_GANGED_LIM_MAX    0
#endif

#if X_AUTO_SQUARE && !X_GANGED
#undef X_GANGED
#define X_GANGED 1
#endif
#if Y_AUTO_SQUARE && !Y_GANGED
#undef Y_GANGED
#define Y_GANGED 1
#endif
#if Z_AUTO_SQUARE && !Z_GANGED
#undef Z_GANGED
#define Z_GANGED 1
#endif

#define N_GANGED (X_GANGED + Y_GANGED + Z_GANGED)
#define N_AUTO_SQUARED (X_AUTO_SQUARE + Y_AUTO_SQUARE + Z_AUTO_SQUARE)
#define N_ABC_MOTORS (N_ABC_AXIS + N_GANGED)

#ifndef PROBE_ENABLE
#define PROBE_ENABLE        1
#endif

#ifndef NEOPIXELS_ENABLE
#define NEOPIXELS_ENABLE    0
#endif

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC      0 // for UART comms
#endif
#ifndef USB_SERIAL_WAIT
#define USB_SERIAL_WAIT     0
#endif

#if USB_SERIAL_CDC == 0 && !defined(SERIAL_STREAM)
#define SERIAL_STREAM       0
#endif

#ifndef MACROS_ENABLE
#define MACROS_ENABLE       0
#endif

#ifndef KEYPAD_ENABLE
#define KEYPAD_ENABLE       0
#endif

#if KEYPAD_ENABLE == 1
#ifdef I2C_STROBE_ENABLE
#undef I2C_STROBE_ENABLE
#endif
#define I2C_STROBE_ENABLE 1
#elif NO_OPTS_POST && KEYPAD_ENABLE == 2 && !defined(KEYPAD_STREAM)
#if USB_SERIAL_CDC
#define KEYPAD_STREAM     0
#else
#define KEYPAD_STREAM     1
#endif
#endif

#ifndef I2C_STROBE_ENABLE
#define I2C_STROBE_ENABLE   0
#endif

#ifndef MPG_ENABLE
#define MPG_ENABLE          0
#endif

#if NO_OPTS_POST && MPG_ENABLE && !defined(MPG_STREAM)
#if USB_SERIAL_CDC
#define MPG_STREAM          0
#else
#define MPG_STREAM          1
#endif
#endif

#if DISPLAY_ENABLE == 2
#ifdef I2C_ENABLE
#undef I2C_ENABLE
#endif
#define I2C_ENABLE 1
#endif

#ifndef EEPROM_ENABLE
#define EEPROM_ENABLE       0
#endif
#ifndef EEPROM_IS_FRAM
#define EEPROM_IS_FRAM      0
#endif

#ifndef I2C_ENABLE
#if EEPROM_ENABLE || KEYPAD_ENABLE == 1 || DISPLAY_ENABLE == 1 || DISPLAY_ENABLE == 2 || I2C_STROBE_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
#define I2C_ENABLE 1
#else
#define I2C_ENABLE 0
#endif
#endif

#ifndef SPINDLE_SYNC_ENABLE
#define SPINDLE_SYNC_ENABLE     0
#endif

#ifndef SPINDLE_ENCODER_ENABLE
#if SPINDLE_SYNC_ENABLE
#define SPINDLE_ENCODER_ENABLE  1
#else
#define SPINDLE_ENCODER_ENABLE  0
#endif
#endif

#ifndef TRINAMIC_ENABLE
  #define TRINAMIC_ENABLE   0
#endif
#if TRINAMIC_ENABLE == 2209
  #ifndef TRINAMIC_UART_ENABLE
    #define TRINAMIC_UART_ENABLE 1
  #endif
  #if NO_OPTS_POST &&  !defined(TRINAMIC_STREAM) && TRINAMIC_UART_ENABLE == 1
    #define TRINAMIC_STREAM 1
  #endif
#else
  #define TRINAMIC_UART_ENABLE 0
#endif
#if (TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 2660 || TRINAMIC_ENABLE == 5160)
  #if !defined(TRINAMIC_SPI_ENABLE)
    #define TRINAMIC_SPI_ENABLE  1
  #endif
#else
  #define TRINAMIC_SPI_ENABLE  0
#endif
#ifndef TRINAMIC_DEV
#define TRINAMIC_DEV        0
#endif

#if NO_OPTS_POST
#ifndef TRINAMIC_I2C
#define TRINAMIC_I2C        0
#endif
#if TRINAMIC_ENABLE && TRINAMIC_I2C
#define TRINAMIC_MOTOR_ENABLE 1
#else
#define TRINAMIC_MOTOR_ENABLE 0
#endif
#endif

#ifndef LITTLEFS_ENABLE
#define LITTLEFS_ENABLE     0
#endif

#ifndef PWM_RAMPED
#define PWM_RAMPED          0
#endif
#ifndef PLASMA_ENABLE
#define PLASMA_ENABLE       0
#elif PLASMA_ENABLE
#if defined(STEP_INJECT_ENABLE)
#undef STEP_INJECT_ENABLE
#endif
#define STEP_INJECT_ENABLE  1
#endif
#ifndef PPI_ENABLE
#define PPI_ENABLE          0
#endif

#if EMBROIDERY_ENABLE
#if defined(SDCARD_ENABLE) && SDCARD_ENABLE == 0
#undef SDCARD_ENABLE
#endif
#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE       1
#endif
#endif

#ifndef ESTOP_ENABLE
  #if COMPATIBILITY_LEVEL <= 1
    #define ESTOP_ENABLE    1
  #else
    #define ESTOP_ENABLE    0
  #endif
#elif ESTOP_ENABLE && COMPATIBILITY_LEVEL > 1
  #warning "Enabling ESTOP may not work with all senders!"
#endif

#define AUX_CONTROL_SPINDLE 0b0001
#define AUX_CONTROL_COOLANT 0b0010
#define AUX_CONTROL_DEVICES 0b0100
#define AUX_CONTROL_INPUTS  0b1000

// Control signals, keep in sync with control_signals_t
#define CONTROL_RESET       0b0000001
#define CONTROL_FEEDHOLD    0b0000010
#define CONTROL_CYCLESTART  0b0000100
#define CONTROL_SAFETYDOOR  0b0001000
#define CONTROL_BLOCKDELETE 0b0010000
#define CONTROL_STOPDISABLE 0b0100000
#define CONTROL_ESTOP       0b1000000

#ifndef CONTROL_ENABLE
#if ESTOP_ENABLE
#define CONTROL_ENABLE      (CONTROL_FEEDHOLD|CONTROL_CYCLESTART|CONTROL_ESTOP)
#else
#define CONTROL_ENABLE      (CONTROL_RESET|CONTROL_FEEDHOLD|CONTROL_CYCLESTART)
#endif
#endif

// Coolant signals, keep in sync with coolant_state_t
#define COOLANT_FLOOD       0b01
#define COOLANT_MIST        0b10

#ifndef COOLANT_ENABLE
#define COOLANT_ENABLE      (COOLANT_FLOOD|COOLANT_MIST)
#endif

// Spindle signals
#define SPINDLE_ENA         0b001
#define SPINDLE_PWM         0b010
#define SPINDLE_DIR         0b100

#ifndef SPINDLE0_ENABLE
#define SPINDLE0_ENABLE DEFAULT_SPINDLE
#endif

#ifndef SPINDLE1_ENABLE
#define SPINDLE1_ENABLE     0
#elif SPINDLE1_ENABLE == -1 || SPINDLE1_ENABLE == SPINDLE_ALL || SPINDLE1_ENABLE == SPINDLE_ALL_VFD
#warning "SPINDLE1_ENABLE cannot be set to -1, SPINDLE_ALL or SPINDLE_ALL_VFD"
#undef SPINDLE1_ENABLE
#define SPINDLE1_ENABLE     0
#endif

#ifndef SPINDLE2_ENABLE
#define SPINDLE2_ENABLE     0
#elif SPINDLE2_ENABLE == -1 || SPINDLE2_ENABLE == SPINDLE_ALL || SPINDLE2_ENABLE == SPINDLE_ALL_VFD
#warning "SPINDLE2_ENABLE cannot be set to -1, SPINDLE_ALL or SPINDLE_ALL_VFD"
#undef SPINDLE2_ENABLE
#define SPINDLE2_ENABLE     0
#endif

#ifndef SPINDLE3_ENABLE
#define SPINDLE3_ENABLE     0
#elif SPINDLE3_ENABLE == -1 || SPINDLE3_ENABLE == SPINDLE_ALL || SPINDLE3_ENABLE == SPINDLE_ALL_VFD
#warning "SPINDLE3_ENABLE cannot be set to -1, SPINDLE_ALL or SPINDLE_ALL_VFD"
#undef SPINDLE1_ENABLE
#define SPINDLE1_ENABLE     0
#endif

#if SPINDLE0_ENABLE == -1 || SPINDLE0_ENABLE == SPINDLE_ALL
#define SPINDLE_ENABLE (SPINDLE_ALL|(1<<SPINDLE1_ENABLE)|(1<<SPINDLE2_ENABLE)|(1<<SPINDLE3_ENABLE))
#elif SPINDLE0_ENABLE == SPINDLE_ALL_VFD
#define SPINDLE_ENABLE (SPINDLE_ALL_VFD|SPINDLE_ALL|(1<<SPINDLE1_ENABLE)|(1<<SPINDLE2_ENABLE)|(1<<SPINDLE3_ENABLE))
#else
#define SPINDLE_ENABLE ((1<<SPINDLE0_ENABLE)|(1<<SPINDLE1_ENABLE)|(1<<SPINDLE2_ENABLE)|(1<<SPINDLE3_ENABLE))
#endif

// Driver spindle 0

#if SPINDLE_ENABLE & (1<<SPINDLE_PWM0)
#define DRIVER_SPINDLE_ENABLE (SPINDLE_ENA|SPINDLE_PWM|SPINDLE_DIR)
#elif SPINDLE_ENABLE & (1<<SPINDLE_PWM0_NODIR)
#define DRIVER_SPINDLE_ENABLE (SPINDLE_ENA|SPINDLE_PWM)
#elif SPINDLE_ENABLE & (1<<SPINDLE_ONOFF0)
#define DRIVER_SPINDLE_ENABLE SPINDLE_ENA
#elif SPINDLE_ENABLE & (1<<SPINDLE_ONOFF0_DIR)
#define DRIVER_SPINDLE_ENABLE (SPINDLE_ENA|SPINDLE_DIR)
#else
#define DRIVER_SPINDLE_ENABLE       0
#endif

#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define DRIVER_SPINDLE_DIR_ENABLE   1 // Deprecated
#else
#define DRIVER_SPINDLE_DIR_ENABLE   0 // Deprecated
#endif

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define DRIVER_SPINDLE_PWM_ENABLE  1 // Deprecated
#define DRIVER_SPINDLE_NAME "PWM"
#else
#define DRIVER_SPINDLE_PWM_ENABLE  0 // Deprecated
#if DRIVER_SPINDLE_ENABLE
#define DRIVER_SPINDLE_NAME "Basic"
#endif
#endif

// Driver spindle 1

#if SPINDLE_ENABLE & ((1<<SPINDLE_PWM1)|(1<<SPINDLE_PWM1_NODIR)|(1<<SPINDLE_ONOFF1)|(1<<SPINDLE_ONOFF1_DIR))
#if N_SPINDLE > 1
#if SPINDLE_ENABLE & (1<<SPINDLE_PWM1)
#define DRIVER_SPINDLE1_ENABLE (SPINDLE_ENA|SPINDLE_PWM|SPINDLE_DIR)
#elif SPINDLE_ENABLE & (1<<SPINDLE_PWM1_NODIR)
#define DRIVER_SPINDLE1_ENABLE (SPINDLE_ENA|SPINDLE_PWM)
#elif SPINDLE_ENABLE & (1<<SPINDLE_ONOFF1)
#define DRIVER_SPINDLE1_ENABLE SPINDLE_ENA
#elif SPINDLE_ENABLE & (1<<SPINDLE_ONOFF1_DIR)
#define DRIVER_SPINDLE1_ENABLE (SPINDLE_ENA|SPINDLE_DIR)
#endif
#else
#warning "Configure N_SPINDLE > 1 in grbl/config.h when enabling second driver spindle!"
#endif
#else
#define DRIVER_SPINDLE1_ENABLE       0
#endif

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
#define DRIVER_SPINDLE1_DIR_ENABLE   1 // Deprecated
#else
#define DRIVER_SPINDLE1_DIR_ENABLE   0 // Deprecated
#endif

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
#define DRIVER_SPINDLE1_PWM_ENABLE  1 // Deprecated
#define DRIVER_SPINDLE1_NAME "PWM2"
#else
#define DRIVER_SPINDLE1_PWM_ENABLE  0 // Deprecated
#if DRIVER_SPINDLE1_ENABLE
#define DRIVER_SPINDLE1_NAME "Basic2"
#endif
#endif

//

#ifndef VFD_ENABLE
  #if SPINDLE_ENABLE & SPINDLE_ALL_VFD
    #define VFD_ENABLE 1
  #else
    #define VFD_ENABLE 0
  #endif
#endif

#define MODBUS_RTU_ENABLED     0b001
#define MODBUS_RTU_DIR_ENABLED 0b010
#define MODBUS_TCP_ENABLED     0b100

#if MODBUS_ENABLE == 2
#undef MODBUS_ENABLE
#define MODBUS_ENABLE 0b011
#endif

#ifndef MODBUS_ENABLE
#if VFD_ENABLE
#define MODBUS_ENABLE       1
#else
#define MODBUS_ENABLE       0
#endif
#endif

#ifndef STEP_INJECT_ENABLE
  #if SPINDLE_ENABLE & (1<<SPINDLE_STEPPER)
    #define STEP_INJECT_ENABLE 1
  #else
    #define STEP_INJECT_ENABLE 0
  #endif
#endif

#ifndef QEI_ENABLE
#define QEI_ENABLE          0
#endif
#ifndef QEI_SELECT_ENABLE
#define QEI_SELECT_ENABLE   0
#endif
#ifndef ODOMETER_ENABLE
#define ODOMETER_ENABLE     0
#endif
#ifndef OPENPNP_ENABLE
#define OPENPNP_ENABLE      0
#endif
#ifndef FANS_ENABLE
#define FANS_ENABLE         0
#endif
#ifndef LIMITS_OVERRIDE_ENABLE
#define LIMITS_OVERRIDE_ENABLE  0
#endif

#ifndef BLUETOOTH_ENABLE
#define BLUETOOTH_ENABLE    0
#endif
#if BLUETOOTH_ENABLE
#ifndef BLUETOOTH_DEVICE
#define BLUETOOTH_DEVICE    "grblHAL"
#endif
#ifndef BLUETOOTH_SERVICE
#define BLUETOOTH_SERVICE   "grblHAL Serial Port" // Minimum 8 characters, or blank for open
#endif
#endif

// Optional control signals

#ifndef SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_ENABLE  0
#endif
#ifndef PROBE_DISCONNECT_ENABLE
#define PROBE_DISCONNECT_ENABLE 0
#endif
#ifndef STOP_DISABLE_ENABLE
#define STOP_DISABLE_ENABLE 0
#endif
#ifndef BLOCK_DELETE_ENABLE
#define BLOCK_DELETE_ENABLE 0
#endif
#ifndef SINGLE_BLOCK_ENABLE
#define SINGLE_BLOCK_ENABLE 0
#endif
#ifndef MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_ENABLE 0
#endif
#ifndef MOTOR_WARNING_ENABLE
#define MOTOR_WARNING_ENABLE 0
#endif
#ifndef LIMITS_OVERRIDE_ENABLE
#define LIMITS_OVERRIDE_ENABLE 0
#endif

#if SAFETY_DOOR_ENABLE && defined(NO_SAFETY_DOOR_SUPPORT)
#error "Driver does not support safety door functionality!"
#endif

//

#ifndef WIFI_ENABLE
#define WIFI_ENABLE         0
#endif

#ifndef WEBUI_ENABLE
#define WEBUI_ENABLE        0
#endif

#ifndef WEBUI_AUTH_ENABLE
#define WEBUI_AUTH_ENABLE   0
#endif

#ifndef WEBUI_INFLASH
#define WEBUI_INFLASH       0
#endif

#if WEBUI_ENABLE

#if !WIFI_ENABLE
#ifdef ETHERNET_ENABLE
#undef ETHERNET_ENABLE
#endif
#define ETHERNET_ENABLE     1
#endif
#ifdef HTTP_ENABLE
#undef HTTP_ENABLE
#endif
#define HTTP_ENABLE         1
#ifdef WEBSOCKET_ENABLE
#undef WEBSOCKET_ENABLE
#endif
#define WEBSOCKET_ENABLE    1
#if defined(SDCARD_ENABLE) && SDCARD_ENABLE == 0
#undef SDCARD_ENABLE
#endif
#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE       1
#endif
#endif

#ifndef ETHERNET_ENABLE
#define ETHERNET_ENABLE     0
#endif
#ifndef TELNET_ENABLE
#define TELNET_ENABLE       0
#endif
#ifndef HTTP_ENABLE
#define HTTP_ENABLE         0
#endif
#ifndef WEBSOCKET_ENABLE
#define WEBSOCKET_ENABLE    0
#endif
#ifndef FTP_ENABLE
#define FTP_ENABLE          0
#elif !(SDCARD_ENABLE || LITTLEFS_ENABLE)
#undef FTP_ENABLE
#define FTP_ENABLE          0
#endif
#ifndef MDNS_ENABLE
#define MDNS_ENABLE         0
#endif
#ifndef SSDP_ENABLE
#define SSDP_ENABLE         0
#endif
#if SSDP_ENABLE && !HTTP_ENABLE
#undef HTTP_ENABLE
#define HTTP_ENABLE         1
#endif
#ifndef MQTT_ENABLE
#define MQTT_ENABLE         0
#endif

#if ETHERNET_ENABLE || WIFI_ENABLE
#ifndef NETWORK_HOSTNAME
#define NETWORK_HOSTNAME        "grblHAL"
#endif
#ifndef NETWORK_IPMODE
#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
#endif
#ifndef NETWORK_IP
#define NETWORK_IP              "192.168.5.1"
#endif
#ifndef NETWORK_GATEWAY
#define NETWORK_GATEWAY         "192.168.5.1"
#endif
#ifndef NETWORK_MASK
#define NETWORK_MASK            "255.255.255.0"
#endif
#ifndef NETWORK_FTP_PORT
#define NETWORK_FTP_PORT        21
#endif
#ifndef NETWORK_TELNET_PORT
#define NETWORK_TELNET_PORT     23
#endif
#ifndef NETWORK_HTTP_PORT
#define NETWORK_HTTP_PORT       80
#endif
#ifndef NETWORK_MODBUS_PORT
#define NETWORK_MODBUS_PORT     502
#endif
#ifndef NETWORK_MQTT_PORT
#define NETWORK_MQTT_PORT       1883
#endif
#ifndef NETWORK_WEBSOCKET_PORT
#if HTTP_ENABLE
#define NETWORK_WEBSOCKET_PORT  81
#else
#define NETWORK_WEBSOCKET_PORT  80
#endif
#endif
#if NETWORK_IPMODE < 0 || NETWORK_IPMODE > 2
#error "Invalid IP mode selected!"
#endif
#if HTTP_ENABLE && NETWORK_WEBSOCKET_PORT == NETWORK_HTTP_PORT
#warning "HTTP and WebSocket protocols cannot share the same port!"
#endif
#endif // ETHERNET_ENABLE || WIFI_ENABLE

#if WIFI_ENABLE

#ifndef NETWORK_STA_SSID
#define NETWORK_STA_SSID         ""
#endif
#ifndef NETWORK_STA_PASSWORD
#define NETWORK_STA_PASSWORD     ""
#endif
#ifndef NETWORK_STA_HOSTNAME
#define NETWORK_STA_HOSTNAME     "grblHAL"
#endif
#ifndef NETWORK_STA_IPMODE
#define NETWORK_STA_IPMODE       1 // DHCP
#endif
#ifndef NETWORK_STA_IP
#define NETWORK_STA_IP           "192.168.5.1"
#endif
#ifndef NETWORK_STA_GATEWAY
#define NETWORK_STA_GATEWAY      "192.168.5.1"
#endif
#ifndef NETWORK_STA_MASK
#define NETWORK_STA_MASK         "255.255.255.0"
#endif

#if WIFI_SOFTAP > 0
#ifndef NETWORK_AP_SSID
#define NETWORK_AP_SSID         "grblHAL_AP"
#endif
#ifndef NETWORK_AP_PASSWORD
#define NETWORK_AP_PASSWORD     "grblHALpwd"
#endif
#ifndef NETWORK_AP_HOSTNAME
#define NETWORK_AP_HOSTNAME     "grblHAL_AP"
#endif
#ifndef NETWORK_AP_IPMODE
#define NETWORK_AP_IPMODE       0 // static
#endif
#ifndef NETWORK_AP_IP
#define NETWORK_AP_IP           "192.168.5.1"
#endif
#ifndef NETWORK_AP_GATEWAY
#define NETWORK_AP_GATEWAY      "192.168.5.1"
#endif
#ifndef NETWORK_AP_MASK
#define NETWORK_AP_MASK         "255.255.255.0"
#endif
#endif // WIFI_SOFTAP
#endif // WIFI_ENABLE

#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE       0
#endif

#if LITTLEFS_ENABLE == 2 && SDCARD_ENABLE
#undef LITTLEFS_ENABLE
#define LITTLEFS_ENABLE     1
#endif

#if LITTLEFS_ENABLE
#if LITTLEFS_ENABLE == 2
#define LITTLEFS_MOUNT_DIR "/"
#else
#define LITTLEFS_MOUNT_DIR "/littlefs"
#endif
#endif

#ifndef SPI_ENABLE
#if SDCARD_ENABLE || TRINAMIC_SPI_ENABLE
#define SPI_ENABLE 1
#else
#define SPI_ENABLE 0
#endif
#endif

/*EOF*/
