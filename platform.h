/*
  platform.h - platform specific definitions

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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

#if defined(STM32F103xB) || defined(STM32F103xE)
#define STM32_F1_PLATFORM
#endif

#if defined(STM32F303xC)
#define STM32_F3_PLATFORM
#endif

#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F407xx) || defined(STM32F411xE) || \
    defined(STM32F412Vx) || defined(STM32F429xx) || defined(STM32F446xx)
#define STM32_F4_PLATFORM
#endif

#if defined(STM32F756xx) || defined(STM32F765xx)
#define STM32_F7_PLATFORM
#endif

#if defined(STM32H743xx) || defined(STM32H723xx)
#define STM32_H7_PLATFORM
#endif

#if defined(STM32_F1_PLATFORM) || defined(STM32_F3_PLATFORM) || defined(STM32_F4_PLATFORM) || defined(STM32_F7_PLATFORM) || defined(STM32_H7_PLATFORM)
#define STM32_PLATFORM
#endif

#if defined(STM32_PLATFORM) || defined(__LPC17XX__) || defined(__IMXRT1062__) || defined(__SAM3X8E__)
#define UINT32FMT "%lu"
#define UINT32SFMT "lu"
#else
#define UINT32FMT "%u"
#define UINT32SFMT "u"
#endif
