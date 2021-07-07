/*
  plugins_init.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Calls the init function of enabled plugins, may be included at the end of the drivers driver_init() inmplementation.

  These are NOT referenced in the core grbl code

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

#if PLASMA_ENABLE
    extern void plasma_init (void);
    plasma_init();
#endif

#if TRINAMIC_ENABLE
    extern void trinamic_init (void);
    trinamic_init();
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

    my_plugin_init();

#if ODOMETER_ENABLE
    extern void odometer_init (void);
    odometer_init(); // NOTE: this *must* be last plugin to be initialized as it claims storage at the end of NVS.
#endif

/*EOF*/
