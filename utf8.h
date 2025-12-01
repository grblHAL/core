/*
  utf8.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Part of grblHAL

  Copyright (c) 2025 Terje Io

  utf32_to_utf8() is Copyright 2025 Kang-Che Sung, see license in utf8.c.

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

uint16_t utf32_to_utf8 (uint8_t *buffer, uint32_t code_point);
