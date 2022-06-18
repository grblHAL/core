/*
  plugins_init.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Calls the init function of enabled plugins, may be included at the end of the drivers driver_init() inmplementation.

  These are NOT referenced in the core grbl code

  Part of grblHAL

  Copyright (c) 2021-2022 Terje Io

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

#if TRINAMIC_ENABLE
    extern bool trinamic_init (void);
    trinamic_init();
#endif

#if MODBUS_ENABLE
    extern void modbus_init (void);
    modbus_init();
#endif

#if VFD_ENABLE
    extern void vfd_init (void);
    vfd_init();
#endif

#if N_SPINDLE > 1
    extern void spindle_select_init(void);
    spindle_select_init();
#endif

#ifndef GRBL_ESP32 // ESP32 has its own bluetooth_init
#if BLUETOOTH_ENABLE
    extern void bluetooth_init (void);
    bluetooth_init();
#endif
#endif

#if KEYPAD_ENABLE
    extern bool keypad_init (void);
    keypad_init();
#endif

#if LASER_COOLANT_ENABLE
    extern void laser_coolant_init (void);
    laser_coolant_init();
#endif

#if FANS_ENABLE
    extern void fans_init (void);
    fans_init();
#endif

#if OPENPNP_ENABLE
    extern void openpnp_init (void);
    openpnp_init();
#endif

// ESP32 has its own webui_init
#ifndef GRBL_ESP32

#if WEBUI_ENABLE
    extern void webui_init (void);
    webui_init();
#endif

#endif

    my_plugin_init();

// Third party plugin definitions.
// The code for these has to be downloaded from the source and placed in the same folder as driver.c
// Note: Third party plugins may have more than one implementation, there is no "owner" of plugins listed here.
//       It is also guaranteed that there will be no implementation in the grblHAL repository of these.

#if ATC_ENABLE
    extern void atc_init (void);
    atc_init();
#endif

#if PROBE_RELAY_ENABLE
    extern void probe_relay_init (void);
    probe_relay_init();
#endif

#if STATUS_LIGHT_ENABLE
    extern void status_light_init (void);
    status_light_init();
#endif

// End third party plugin definitions.

#if ODOMETER_ENABLE
    extern void odometer_init (void);
    odometer_init(); // NOTE: this *must* be last plugin to be initialized as it claims storage at the end of NVS.
#endif

/*EOF*/
