/*
  expanders_init.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Calls the init function of enabled expanders, this file is typically included early in driver.h
  io_expanders_init() should be called at the end of the drivers driver_init() implementation,
  just before the driver claims ports.

  These are NOT referenced in the core grbl code

  Part of grblHAL

  Copyright (c) 2025 Terje Io

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

#pragma once

extern void board_ports_init (void); // default is a weak function

// I2C expanders

#if PCA9654E_ENABLE || MCP3221_ENABLE || MCP4725_ENABLE

#if defined(I2C_ENABLE) && !I2C_ENABLE
#undef I2C_ENABLE
#endif

#ifndef I2C_ENABLE
#define I2C_ENABLE 1
#endif

#if MCP3221_ENABLE
extern void mcp3221_init (void);
#endif

#if MCP4725_ENABLE
extern void mcp4725_init (void);
#endif

#if PCA9654E_ENABLE
extern void pca9654e_init(void);
#endif

// Third party I2C expander plugins goes after this line

#endif // I2C expanders

// SPI expanders

//

// ModBus expanders

#if PICOHAL_IO_ENABLE || R4SLS08_ENABLE

#if !defined(MODBUS_ENABLE) || !(MODBUS_ENABLE & MODBUS_RTU_ENABLED)
#error "Enabled IO expander(s) require Modbus RTU!"
#endif

#if R4SLS08_ENABLE
extern void r4sls08_init (void);
#endif

// Third party Modbus expander plugins goes after this line

#if PICOHAL_IO_ENABLE
extern void picohal_io_init (void);
#endif

#endif // ModBus expanders

// CANBus expanders

//

// Other expanders

#if THCAD2_ENABLE
    extern void thcad2_init (void);
#endif

#if FNC_EXPANDER_ENABLE
    void fnc_expander_init (void);
#endif

//

static inline void io_expanders_init (void)
{
    board_ports_init(); // can be implemented by board specific code

#if MCP3221_ENABLE
    mcp3221_init();
#endif

#if MCP4725_ENABLE
    mcp4725_init();
#endif

#if R4SLS08_ENABLE
    r4sls08_init();
#endif

#if PCA9654E_ENABLE
    pca9654e_init();
#endif

#if PICOHAL_IO_ENABLE
    picohal_io_init();
#endif

#if FNC_EXPANDER_ENABLE
    fnc_expander_init();
#endif

#if THCAD2_ENABLE
    thcad2_init();
#endif
}
