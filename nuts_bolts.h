/*
  nuts_bolts.h - Header file for shared definitions, variables, and functions

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef _NUTS_BOLTS_H_
#define _NUTS_BOLTS_H_

#include "grbl.h"
#include "errors.h"

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

#define TOLERANCE_EQUAL 0.0001f

#define RADDEG  0.01745329251994329577f // Radians per degree
#define DEGRAD 57.29577951308232087680f // Degrees per radians
#define SQRT3   1.73205080756887729353f
#define SIN120  0.86602540378443864676f
#define COS120 -0.5f
#define TAN60   1.73205080756887729353f
#define SIN30   0.5f
#define TAN30   0.57735026918962576451f
#define TAN30_2 0.28867513459481288225f

#define ABORTED (sys.abort || sys.cancel)

// Convert character to uppercase
#define CAPS(c) ((c >= 'a' && c <= 'z') ? (c & 0x5F) : c)
#define LCAPS(c) ((c >= 'A' && c <= 'Z') ? (c | 0x20) : c)

#if !(defined(STM32F103xB) || defined(STM32F303xC))
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
#if N_AXIS > 5
#define C_AXIS 5
#define C_AXIS_BIT bit(C_AXIS)
#endif
#if N_AXIS > 6
#define U_AXIS 6
#define U_AXIS_BIT bit(U_AXIS)
#endif
#if N_AXIS == 8
#define V_AXIS 7
#define V_AXIS_BIT bit(V_AXIS)
#endif

#if N_AXIS == 3
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)
#elif N_AXIS == 4
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|A_AXIS_BIT)
#elif N_AXIS == 5
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|A_AXIS_BIT|B_AXIS_BIT)
#elif N_AXIS == 6
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|A_AXIS_BIT|B_AXIS_BIT|C_AXIS_BIT)
#elif N_AXIS == 7
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|A_AXIS_BIT|B_AXIS_BIT|C_AXIS_BIT|U_AXIS_BIT)
#else
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|A_AXIS_BIT|B_AXIS_BIT|C_AXIS_BIT|U_AXIS_BIT|V_AXIS_BIT)
#endif

#ifdef V_AXIS
#define N_ABC_AXIS 5
#elif defined(U_AXIS)
#define N_ABC_AXIS 4
#elif defined(C_AXIS)
#define N_ABC_AXIS 3
#elif defined(B_AXIS)
#define N_ABC_AXIS 2
#elif defined(A_AXIS)
#define N_ABC_AXIS 1
#else
#define N_ABC_AXIS 0
#endif

extern char const *const axis_letter[];

typedef union {
    uint8_t mask;
    uint8_t bits;
    uint8_t value;
    struct {
        uint8_t x :1,
                y :1,
                z :1,
                a :1,
                b :1,
                c :1,
                u :1,
                v :1;
    };
} axes_signals_t;

typedef union {
    float values[2];
    struct {
        float x;
        float y;
    };
} point_2d_t;

#pragma pack(push, 1)

//! \brief Limit switches struct, consists of four packed axes_signals_t structs in 32 bits.
typedef struct {
    axes_signals_t min;     //!< Min limit switches status, required.
    axes_signals_t max;     //!< Max limit switches status, optional.
    axes_signals_t min2;    //!< Secondary min limit switch(es) status, required for auto squaring enabled axes.
    axes_signals_t max2;    //!< Secondary max limit switches status, optional (of no practical use?).
} limit_signals_t;

//! \brief Home switches struct, consists of two packed axes_signals_t structs.
typedef struct {
    axes_signals_t a;       //!< Primary home switches status, optional. Limit signals are used for homing if not available.
    axes_signals_t b;       //!< Secondary home switch(es) status, required for auto squaring enabled axes if primary switches are available.
} home_signals_t;

#pragma pack(pop)

typedef enum {
    DelayMode_Dwell = 0,
    DelayMode_SysSuspend
} delaymode_t;

// Conversions
#define MM_PER_INCH (25.40f)
#define INCH_PER_MM (0.0393701f)

#define MAX_INT_DIGITS 9 // Maximum number of digits in int32 (and float)
#define STRLEN_COORDVALUE (MAX_INT_DIGITS + N_DECIMAL_COORDVALUE_INCH + 1) // 8.4 format - excluding terminating null

// Useful macros
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef constrain
#define constrain(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
#endif
#define clear_vector(a) memset(a, 0, sizeof(a))
#define isequal_position_vector(a, b) !memcmp(a, b, sizeof(coord_data_t))
#define is0_position_vector(a) !memcmp(a, &((coord_data_t){0}), sizeof(coord_data_t))

// Bit field and masking macros
#ifndef bit
#define bit(n) (1UL << (n))
#endif
#define bit_true(x, mask) (x) |= (mask)
#define bit_false(x, mask) (x) &= ~(mask)
#define BIT_SET(x, bit, v) { if (v) { x |= (bit); } else { x &= ~(bit); } }

#define bit_istrue(x, mask) ((x & (mask)) != 0)
#define bit_isfalse(x, mask) ((x & (mask)) == 0)

// Converts an uint32 variable to string.
char *uitoa (uint32_t n);

// Converts a float variable to string with the specified number of decimal places.
char *ftoa (float n, uint8_t decimal_places);

// Trim trailing zeros and possibly decimal point
char *trim_float (char *s);

// Returns true if float value is a whole number (integer)
bool isintf (float value);

status_code_t read_uint (char *line, uint_fast8_t *char_counter, uint32_t *uint_ptr);

// Read a floating point value from a string. Line points to the input buffer, char_counter
// is the indexer pointing to the current character of the line, while float_ptr is
// a pointer to the result variable. Returns true when it succeeds
bool read_float (char *line, uint_fast8_t *char_counter, float *float_ptr);

// Non-blocking delay function used for general operation and suspend features.
bool delay_sec (float seconds, delaymode_t mode);

float convert_delta_vector_to_unit_vector(float *vector);

// parse ISO8601 datetime
struct tm *get_datetime (const char *s);

char *strcaps (char *s);

uint_fast8_t bit_count (uint32_t bits);

void dummy_handler (void);

#ifdef _WIN32

static int ffs (int i)
{
    int idx = 0;

    while(i) {
        idx++;
        if(i & 1)
            break;
        i >>= 1;
    }

    return idx;
}

#endif // _WIN32

#endif
