/*
  utf8.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Part of grblHAL

  Copyright (c) 2025 Terje Io

  utf32_to_utf8() is Copyright 2025 Kang-Che Sung, see license below.

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

#include <stdint.h>

/*
static enqueue_realtime_command_ptr utf8_base_handler = NULL;

int32_t utf8_decode (const io_stream_t *stream)
{
    static int32_t count = 0, utf8_c = SERIAL_NO_DATA;

    if((count && (stream->get_rx_buffer_count()) < count) || (utf8_c = stream->read()) == SERIAL_NO_DATA)
        return SERIAL_NO_DATA;

    if(utf8_c & 0b11100000) {
        count = (c & 0b11100000) == 0b11100000 ? (c & 0b00110000) >> 4 : 1;
        if(c & (0b01000000 >> count)) {
            count = 0;
            utf8_c = 0xFFFD;
        } else
            utf8_c = c & (0b00111111 >> count);
    }

    if(count) do {

        int32_t c;

        if(((c = stream->read())& 0b11000000) != 0b10000000) {
            count = 1;
            utf8_c = 0xFFFD;
        } else {
            utf8_c = (utf8_c << 6) | (c & 0b00111111);
        }
    } while(--count);

    return utf8_c;
}

ISR_CODE bool ISR_FUNC(utf8_insert)(uint8_t c)
{
    static int32_t count = 0, utf8_c = 0;

    if((c & 0b11000000) == 0b11000000)
        count = (utf8_c & 0b11100000) == 0b11100000 ? (utf8_c & 0b00110000) >> 4 : 1;
    else if(count)
        count--;

    return count ? false : utf8_base_handler(c);
}

bool stream_utf8_enable (const io_stream_t *stream, bool enable)
{
    if(enable) {
        if(utf8_base_handler == NULL)
            utf8_base_handler = hal.stream.set_enqueue_rt_handler(utf8_insert);
    } else {
        if(utf8_base_handler) {
             hal.stream.set_enqueue_rt_handler(utf8_base_handler);
             utf8_base_handler = NULL;
        }
    }

    return true;
}
*/

/*
<https://gitlab.com/-/snippets/3718423>

Copyright 2025 Kang-Che Sung <explorer09 @ gmail.com>

MIT License (MIT/Expat)

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/* SPDX-License-Identifier: MIT */

uint16_t utf32_to_utf8 (uint8_t *buffer, uint32_t code_point)
{
//    assert(code_point <= 0x10FFFF);
//    assert(code_point < 0xD800 || code_point > 0xDFFF);

	if(code_point > 0x10FFFF)
		return 0;

	uint16_t idx, length = 1;
    uint32_t first_byte = code_point, mask;

    if(code_point <= 0x7F) {
        mask = 0b0111111; // We assume ASCII characters appear most frequently.
    } else {
        // Find out how many bytes are needed.
        mask = 0b00111111;
        do {
            length++;
            first_byte >>= 6;
            mask >>= 1;
        } while(first_byte > mask);
    }

    if(length <= 4) {
        buffer[0] = (uint8_t)(first_byte + (~mask << 1));
        for(idx = length - 1; idx > 0; idx--) {
            buffer[idx] = (code_point & 0b00111111) | 0b10000000;
            code_point >>= 6;
        }
    }

    return length;
}
