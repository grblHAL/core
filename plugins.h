/*
  plugins.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Some data structures and function declarations for plugins that require driver code

  These are NOT referenced in the core grbl code

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

#ifndef _PLUGINS_H_
#define _PLUGINS_H_

#include <stdint.h>
#include <stdbool.h>

#include "nvs.h"

// Jogging

typedef struct {
    float fast_speed;
    float slow_speed;
    float step_speed;
    float fast_distance;
    float slow_distance;
    float step_distance;
} jog_settings_t;

// Networking

typedef enum {
    IpMode_Static = 0,
    IpMode_DHCP,
    IpMode_AutoIP
} ip_mode_t;

typedef union {
    uint8_t mask;
    struct {
        uint8_t telnet     :1,
                websocket  :1,
                http       :1,
                ftp        :1,
                dns        :1,
                mdns       :1,
                ssdp       :1,
                webdav     :1;
    };
} network_services_t;

typedef char ssid_t[65];
typedef uint8_t bssid_t[6];
typedef char username_t[33];
typedef char password_t[33];
typedef char hostname_t[33];
typedef char uri_t[65];

typedef struct {
    char ip[16];
    uint16_t port;
    username_t user;
    password_t password;
} mqtt_settings_t;

typedef struct {
    char ip[16];
    char gateway[16];
    char mask[16];
    hostname_t hostname;
    uint16_t telnet_port;
    uint16_t websocket_port;
    uint16_t http_port;
    uint16_t ftp_port;
    ip_mode_t ip_mode;
    network_services_t services;
#if MQTT_ENABLE
    mqtt_settings_t mqtt;
#endif
#if defined(_WIZCHIP_) || defined(HAS_MAC_SETTING)
    uint8_t mac[6];
#endif
} network_settings_t;

typedef enum {
    WiFiMode_NULL = 0,
    WiFiMode_STA,
    WiFiMode_AP,
    WiFiMode_APSTA
} grbl_wifi_mode_t;

#ifdef WIN32
#undef interface
#endif

typedef struct {
    const char *interface;
    bool is_ethernet;
    bool link_up;
    uint16_t mbps;
    char mac[18];
    char mqtt_client_id[18];
    grbl_wifi_mode_t wifi_mode;
    network_settings_t status;
} network_info_t;

typedef struct {
    ssid_t ssid;
    bssid_t bssid;
    password_t password;
    char country[4];
    uint8_t channel;
    network_settings_t network;
} wifi_ap_settings_t;

typedef struct {
    ssid_t ssid;
    password_t password;
    network_settings_t network;
} wifi_sta_settings_t;

typedef struct {
    char device_name[33];
    char service_name[33];
} bluetooth_settings_t;

typedef struct {
    uint32_t baud_rate;
    uint32_t rx_timeout;
} modbus_settings_t;

#define MODBUS_TCP_SETTINGS_INCREMENT 5

typedef enum {
    Setting_ModbusIpAddress = 0,
    Setting_ModbusPort      = 1,
    Setting_ModbusId        = 2
} modbus_tcp_setting_id_t;

typedef struct {
    char ip[16];
    uint16_t port;
    uint8_t id;
} modbus_tcp_settings_t;

// Quadrature encoder interface

typedef enum {
    Encoder_Universal = 0,
    Encoder_FeedRate,
    Encoder_RapidRate,
    Encoder_Spindle_RPM,
    Encoder_MPG,
    Encoder_MPG_X,
    Encoder_MPG_Y,
    Encoder_MPG_Z,
    Encoder_MPG_A,
    Encoder_MPG_B,
    Encoder_MPG_C,
    Encoder_Spindle_Position
} encoder_mode_t;

typedef enum {
    Setting_EncoderMode = 0,
    Setting_EncoderCPR = 1,     //!< Count Per Revolution.
    Setting_EncoderCPD = 2,     //!< Count Per Detent.
    Setting_EncoderDblClickWindow = 3 // ms
} encoder_setting_id_t;

typedef union {
    uint8_t events;
    struct {
        uint8_t position_changed  :1,
                direction_changed :1,
                click             :1,
                dbl_click         :1,
                long_click        :1,
                index_pulse       :1,
                unused            :2;
    };
} encoder_event_t;

typedef union {
    uint8_t flags;
    uint8_t value;
    struct {
        uint8_t single_count_per_detent :1;
    };
} encoder_flags_t;

typedef struct {
    encoder_mode_t mode;
    uint32_t cpr;               //!< Count per revolution.
    uint32_t cpd;               //!< Count per detent.
    uint32_t dbl_click_window;  //!< ms.
    encoder_flags_t flags;
} encoder_settings_t;

typedef struct {
    encoder_mode_t mode;
    uint_fast8_t id;
    uint_fast8_t axis;          //!< Axis index for MPG encoders, 0xFF for others.
    int32_t position;
    uint32_t velocity;
    encoder_event_t event;
    encoder_settings_t *settings;
} encoder_t;

// DISPLAYS:

// Interfaces
#define DISPLAY_I2C             (1<<0) //!< 1
#define DISPLAY_SPI             (1<<1) //!< 2
#define DISPLAY_UART            (1<<2) //!< 4

// Plugins
#define DISPLAY_I2C_INTERFACE   ((1<<3)|DISPLAY_I2C) //!< 9  - @grblhal
#define DISPLAY_I2C_LEDS        ((1<<4)|DISPLAY_I2C) //!< 17 - @grblhal
#define DISPLAY_I2C_OLED_1      ((1<<5)|DISPLAY_I2C) //!< 33 - @luc-github

// Driver chips

#define DISPLAY_DRIVER_SH1106  1
#define DISPLAY_DRIVER_SSD1306 2
#define DISPLAY_DRIVER_SSD1331 3
#define DISPLAY_DRIVER_ILI9340 4
#define DISPLAY_DRIVER_ILI9341 5
#define DISPLAY_DRIVER_ILI9486 6

// I2C interface:

typedef void (*keycode_callback_ptr)(const char c);

typedef uint_fast16_t i2c_address_t;

typedef struct {
    i2c_address_t address;
    union {
        struct  {
            uint8_t cmd_bytes;
            uint16_t cmd;
        };
        struct  {
            uint8_t word_addr_bytes;
            uint16_t word_addr;
        };
    };
    uint_fast16_t count;
    bool no_block;
    uint8_t *data;
} i2c_transfer_t;

typedef union {
    uint8_t ok;
    struct {
        uint8_t started         :1,
                tx_non_blocking :1,
                tx_dma          :1,
                unassigned      :5;
    };
} i2c_cap_t;

extern i2c_cap_t i2c_start (void);
extern bool i2c_probe (i2c_address_t i2c_address);
extern bool i2c_send (i2c_address_t i2c_address, uint8_t *data, size_t size, bool block);
extern bool i2c_receive (i2c_address_t i2cAddr, uint8_t *buf, size_t size, bool block);
extern bool i2c_get_keycode (i2c_address_t i2c_address, keycode_callback_ptr callback);
extern bool i2c_transfer (i2c_transfer_t *i2c, bool read);

// SPI interface:

/*
typedef void (*spi_cs_ptr)(const char c);

typedef struct {
    spi_cs_ptr cs;
    uint32_t f_clk;
} spi_cfg_t;

typedef union {
    uint8_t ok;
    struct {
        uint8_t started         :1,
                tx_non_blocking :1,
                tx_dma          :1,
                unassigned      :5;
    };
} spi_cap_t;

extern spi_cap_t spi_start (spi_cfg_t *cfg);
extern uint32_t spi_set_speed (uint32_t prescaler);
extern uint8_t spi_get_byte (void);
extern uint8_t spi_put_byte (uint8_t byte);
extern void spi_write (uint8_t *data, size_t size);
extern void spi_read (uint8_t *data, size_t size);
extern void spi_write (uint8_t *data, size_t size);
*/

// EEPROM/FRAM:

typedef i2c_transfer_t nvs_transfer_t;
#define i2c_nvs_transfer(i2c, rd) i2c_transfer(i2c, rd)

typedef bool nvs_transfer_result_t;
#define NVS_TransferResult_OK true

#endif
