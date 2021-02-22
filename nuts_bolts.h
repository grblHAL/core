/*
  nuts_bolts.h - Header file for shared definitions, variables, and functions

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef _NUTS_BOLTS_H_
#define _NUTS_BOLTS_H_

#include "grbl.h"

#ifndef true
#define false 0
#define true 1
#endif

#define Off 0
#define On 1

#define SOME_LARGE_VALUE 1.0E+38f
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define TAN_30 0.57735f         // Used for threading calculations (60 degree inserts)
#define RADDEG 0.0174532925f    // Radians per degree

#define ABORTED (sys.abort || sys.cancel)

// Convert character to uppercase
#define CAPS(c) ((c >= 'a' && c <= 'z') ? c & 0x5F : c)

#ifndef STM32F103xB
#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif
#endif

// Axis array index values. Must start with 0 and be continuous.
#define X_AXIS 0 // Axis indexing value.
#define Y_AXIS 1
#define Z_AXIS 2
#define X_AXIS_BIT bit(X_AXIS)
#define Y_AXIS_BIT bit(Y_AXIS)
#define Z_AXIS_BIT bit(Z_AXIS)
#if N_AXIS > 3
#define A_AXIS 3
#define A_AXIS_BIT bit(A_AXIS)
#endif
#if N_AXIS > 4
#define B_AXIS 4
#define B_AXIS_BIT bit(B_AXIS)
#endif
#if N_AXIS == 6
#define C_AXIS 5
#define C_AXIS_BIT bit(C_AXIS)
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|A_AXIS_BIT|B_AXIS_BIT|C_AXIS_BIT)
#endif

#if N_AXIS == 3
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)
#elif N_AXIS == 4
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|A_AXIS_BIT)
#elif N_AXIS == 5
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|A_AXIS_BIT|B_AXIS_BIT)
#endif

extern char const *const axis_letter[];

typedef union {
    uint8_t mask;
    uint8_t value;
    struct {
        uint8_t x :1,
                y :1,
                z :1,
                a :1,
                b :1,
                c :1;
    };
} axes_signals_t;

#pragma pack(push, 1)

typedef struct {
    axes_signals_t min;
    axes_signals_t max;
    axes_signals_t min2;
    axes_signals_t max2;
} limit_signals_t;

#pragma pack(pop)

typedef enum {
    DelayMode_Dwell = 0,
    DelayMode_SysSuspend
} delaymode_t;

 // Delay struct, currently not used by core - may be used by drivers
typedef struct {
    volatile uint32_t ms;
    void (*callback)(void);
} delay_t;

// Conversions
#define MM_PER_INCH (25.40f)
#define INCH_PER_MM (0.0393701f)

#define MAX_INT_DIGITS 8 // Maximum number of digits in int32 (and float)
#define STRLEN_COORDVALUE (MAX_INT_DIGITS + N_DECIMAL_COORDVALUE_INCH + 1) // 8.4 format - excluding terminating null

// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
#define isequal_position_vector(a,b) !memcmp(a, b, sizeof(coord_data_t))

// Bit field and masking macros
#ifndef bit
#define bit(n) (1UL << n)
#endif
#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define BIT_SET(x, bit, v) { if (v) { x |= (bit); } else { x &= ~(bit); } }

#define bit_istrue(x, mask) ((x & (mask)) != 0)
#define bit_isfalse(x, mask) ((x & (mask)) == 0)

// Converts an uint32 variable to string.
char *uitoa (uint32_t n);

// Converts a float variable to string with the specified number of decimal places.
char *ftoa (float n, uint8_t decimal_places);

// Returns true if float value is a whole number (integer)
bool isintf (float value);

// Read a floating point value from a string. Line points to the input buffer, char_counter
// is the indexer pointing to the current character of the line, while float_ptr is
// a pointer to the result variable. Returns true when it succeeds
bool read_float(char *line, uint_fast8_t *char_counter, float *float_ptr);

// Non-blocking delay function used for general operation and suspend features.
void delay_sec(float seconds, delaymode_t mode);

float convert_delta_vector_to_unit_vector(float *vector);

// calculate checksum byte for data
uint8_t calc_checksum (uint8_t *data, uint32_t size);

char *strcaps (char *s);

void dummy_handler (void);

#endif
