/*
  crossbar.h - signal crossbar definitions

  Not used by the core.

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

typedef bool (*xbar_get_value_ptr)(void);
typedef void (*xbar_set_value_ptr)(bool on);
typedef void (*xbar_event_ptr)(bool on);
typedef void (*xbar_config_ptr)(void *cfg_data);

typedef struct {
    uint32_t function;
    void *port;
    uint8_t bit;
    xbar_config_ptr config;
    xbar_get_value_ptr get_value;
    xbar_set_value_ptr set_value;
    xbar_event_ptr on_event;
} xbar_t;

#endif
