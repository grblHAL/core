/*
  driver_opts.h - for preprocessing options from my_machine.h or compiler symbols

  NOTE: This file is not used by the core, it may be used by drivers

  Part of grblHAL

  Copyright (c) 2020-2023 Terje Io

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

//
// NOTE: do NOT change options here - edit the driver specific my_machine.h instead!
//

#pragma once

#include "hal.h"
#include "nuts_bolts.h"

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

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC      0 // for UART comms
#endif
#ifndef USB_SERIAL_WAIT
#define USB_SERIAL_WAIT     0
#endif

#ifndef KEYPAD_ENABLE
#define KEYPAD_ENABLE       0
#endif

#ifndef MACROS_ENABLE
#define MACROS_ENABLE       0
#endif

#ifndef MPG_ENABLE
#define MPG_ENABLE          0
#endif

#if MPG_ENABLE == 1 && KEYPAD_ENABLE == 2
#define MPG_MODE            2
#elif MPG_ENABLE
#define MPG_MODE            1
#else
#define MPG_MODE            0
#endif

#if MPG_ENABLE && !defined(MPG_STREAM)
#if USB_SERIAL_CDC
#define MPG_STREAM          0
#else
#define MPG_STREAM          1
#endif
#endif

#if KEYPAD_ENABLE == 1
#ifdef I2C_STROBE_ENABLE
#undef I2C_STROBE_ENABLE
#endif
#define I2C_STROBE_ENABLE 1
#elif KEYPAD_ENABLE == 2 && !defined(MPG_STREAM)
#ifndef KEYPAD_STREAM
#if USB_SERIAL_CDC
#define KEYPAD_STREAM     0
#else
#define KEYPAD_STREAM     1
#endif
#endif
#endif
#ifndef I2C_STROBE_ENABLE
#define I2C_STROBE_ENABLE   0
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
#define SPINDLE_SYNC_ENABLE 0
#endif

#ifndef TRINAMIC_ENABLE
  #define TRINAMIC_ENABLE   0
#endif
#if TRINAMIC_ENABLE == 2209
  #if !defined(TRINAMIC_UART_ENABLE)
    #define TRINAMIC_UART_ENABLE 1
  #endif
#else
  #define TRINAMIC_UART_ENABLE 0
#endif
#if (TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160)
  #if !defined(TRINAMIC_SPI_ENABLE)
    #define TRINAMIC_SPI_ENABLE  1
  #endif
#else
  #define TRINAMIC_SPI_ENABLE  0
#endif
#ifndef TRINAMIC_I2C
#define TRINAMIC_I2C        0
#endif
#if TRINAMIC_ENABLE && TRINAMIC_I2C
#define TRINAMIC_MOTOR_ENABLE 1
#else
#define TRINAMIC_MOTOR_ENABLE 0
#endif
#ifndef TRINAMIC_DEV
#define TRINAMIC_DEV        0
#endif

#ifndef LITTLEFS_ENABLE
#define LITTLEFS_ENABLE     0
#endif

#ifndef PWM_RAMPED
#define PWM_RAMPED          0
#endif
#ifndef PLASMA_ENABLE
#define PLASMA_ENABLE       0
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

#ifndef VFD_SPINDLE
#if VFD_ENABLE
#define VFD_SPINDLE         1
#else
#define VFD_SPINDLE         0
#endif
#endif

#define MODBUS_RTU_ENABLED     0b001
#define MODBUS_RTU_DIR_ENABLED 0b010
#define MODBUS_TCP_ENABLED     0b100

#if MODBUS_ENABLE == 2
#undef MOBUS_ENABLE
#define MOBUS_ENABLE 0b011
#endif

#ifndef MODBUS_ENABLE
#if VFD_ENABLE
#define MODBUS_ENABLE       1
#else
#define MODBUS_ENABLE       0
#endif
#endif

#if !VFD_SPINDLE || N_SPINDLE > 1
#define DRIVER_SPINDLE_ENABLE  1
#else
#define DRIVER_SPINDLE_ENABLE  0
#endif

#ifndef QEI_ENABLE
#define QEI_ENABLE          0
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

#ifndef SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_ENABLE  0
#endif

#if SAFETY_DOOR_ENABLE && defined(NO_SAFETY_DOOR_SUPPORT)
#error "Driver does not support safety door functionality!"
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

#if WEBUI_ENABLE && !defined(ESP_PLATFORM) 

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
#elif !SDCARD_ENABLE
#undef FTP_ENABLE
#define FTP_ENABLE          0
#endif
#ifndef MDNS_ENABLE
#define MDNS_ENABLE         0
#endif
#ifndef SSDP_ENABLE
#define SSDP_ENABLE         0
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
#define NETWORK_AP_PASSWORD     "grblHAL"
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

#ifndef SPI_ENABLE
#if SDCARD_ENABLE || TRINAMIC_SPI_ENABLE
#define SPI_ENABLE 1
#else
#define SPI_ENABLE 0
#endif
#endif

/*EOF*/
