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

// I2C expanders

#if PCA9654E_ENABLE || MCP3221_ENABLE

#if defined(I2C_ENABLE) && !I2C_ENABLE
#undef I2C_ENABLE
#endif

#ifndef I2C_ENABLE
#define I2C_ENABLE 1
#endif

#if MCP3221_ENABLE
extern void mcp3221_init (void);
#endif

#if PCA9654E_ENABLE
extern void pca9654e_init(void);
#endif

// Third party I2C expander plugins goes after this line

#endif // I2C expanders

// SPI expanders

//

// ModBus expanders



#if PICOHAL_IO_ENABLE

#ifndef MODBUS_ENABLE
#error "Modbus must be enabled to use the Picohal IO expander!"
#undef PICOHAL_IO_ENABLE
#endif

extern void picohal_io_init (void);
#endif

// CANBus expanders

//

// Other expanders

//

static inline void io_expanders_init (void)
{
#if MCP3221_ENABLE
    mcp3221_init();
#endif

#if PCA9654E_ENABLE
    pca9654e_init();
#endif

#if PICOHAL_IO_ENABLE
    picohal_io_init();
#endif

}
