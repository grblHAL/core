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
#define IS_AXIS_LETTER(c) (AXIS3_LETTER == c || AXIS4_LETTER == c || AXIS5_LETTER == c || AXIS6_LETTER == c || AXIS7_LETTER == c)
#define IS_AXIS_LETTER_VALID(c) (c == 'A' || c == 'B' || c == 'C' || c == 'U' || c == 'V' || c == 'W')
//#define AXIS_LETTER_FN_IDX(c) ((c >= 'X' && c <= 'Z') ? ((c - 'X') << 1) : ((c >= 'A' && c <= 'C') ? c - 'A' + 6 : ((c >= 'U' && c <= 'W') ? c - 'U' + 9 : -1)))

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
#define AXIS3_IDX 3
#define AXIS3_BIT bit(AXIS3_IDX)
#endif
#if N_AXIS > 4
#define AXIS4_IDX 4
#define AXIS4_BIT bit(AXIS4_IDX)
#endif
#if N_AXIS > 5
#define AXIS5_IDX 5
#define AXIS5_BIT bit(AXIS5_IDX)
#endif
#if N_AXIS > 6
#define AXIS6_IDX 6
#define AXIS6_BIT bit(AXIS6_IDX)
#endif
#if N_AXIS == 8
#define AXIS7_IDX 7
#define AXIS7_BIT bit(AXIS7_IDX)
#endif

#if N_AXIS == 3
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)
#elif N_AXIS == 4
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|AXIS3_BIT)
#elif N_AXIS == 5
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|AXIS3_BIT|AXIS4_BIT)
#elif N_AXIS == 6
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|AXIS3_BIT|AXIS4_BIT|AXIS5_BIT)
#elif N_AXIS == 7
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|AXIS3_BIT|AXIS4_BIT|AXIS5_BIT|AXIS6_BIT)
#else
#define AXES_BITMASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT|AXIS3_BIT|AXIS4_BIT|AXIS5_BIT|AXIS6_BIT|AXIS7_BIT)
#endif

#ifdef AXIS7_IDX
#define N_ABC_AXIS 5
#elif defined(AXIS6_IDX)
#define N_ABC_AXIS 4
#elif defined(AXIS5_IDX)
#define N_ABC_AXIS 3
#elif defined(AXIS4_IDX)
#define N_ABC_AXIS 2
#elif defined(AXIS3_IDX)
#define N_ABC_AXIS 1
#else
#define N_ABC_AXIS 0
#endif

#define AXIS0_LETTER 'X'
#define AXIS1_LETTER 'Y'
#define AXIS2_LETTER 'Z'

#if N_AXIS > 3
  #if AXIS_REMAP_ABC2UVW
    #define AXIS3_LETTER 'U'
  #elif !defined(AXIS3_LETTER)
    #define AXIS3_LETTER 'A'
  #elif !IS_AXIS_LETTER_VALID(AXIS3_LETTER)
    #error "Illegal axis letter assigned!"
  #endif
#else
    #define AXIS3_LETTER 0
#endif

#if N_AXIS > 4
  #if AXIS_REMAP_ABC2UVW
    #define AXIS4_LETTER 'V'
  #elif !defined(AXIS4_LETTER)
    #define AXIS4_LETTER 'B'
  #elif !IS_AXIS_LETTER_VALID(AXIS4_LETTER)
    #error "Illegal axis letter assigned!"
  #endif
#else
  #define AXIS4_LETTER 0
#endif

#if N_AXIS > 5
  #if AXIS_REMAP_ABC2UVW
    #define AXIS5_LETTER 'W'
  #elif !defined(AXIS5_LETTER)
    #define AXIS5_LETTER 'C'
  #elif !IS_AXIS_LETTER_VALID(AXIS5_LETTER)
    #error "Illegal axis letter assigned!"
  #endif
#else
  #define AXIS5_LETTER 0
#endif

#if N_AXIS > 6
  #if !defined(AXIS6_LETTER)
    #define AXIS6_LETTER 'U'
  #elif !IS_AXIS_LETTER_VALID(AXIS6_LETTER)
    #error "Illegal axis letter assigned!"
  #endif
#else
  #define AXIS6_LETTER 0
#endif

#if N_AXIS > 7
  #if !defined(AXIS7_LETTER)
    #define AXIS7_LETTER 'V'
  #elif !IS_AXIS_LETTER_VALID(AXIS7_LETTER)
    #error "Illegal axis letter assigned!"
  #endif
#else
  #define AXIS7_LETTER 0
#endif

#if N_AXIS == 4
#define AXIS_LETTER_TO_IDX(c) AXIS3_IDX
#elif N_AXIS == 5
#define AXIS_LETTER_TO_IDX(c) \
(c == AXIS3_LETTER ? AXIS3_IDX : \
(c == AXIS4_LETTER ? AXIS4_IDX : -1))
#elif N_AXIS == 6
#define AXIS_LETTER_TO_IDX(c) \
(c == AXIS3_LETTER ? AXIS3_IDX : \
(c == AXIS4_LETTER ? AXIS4_IDX : \
(c == AXIS5_LETTER ? AXIS5_IDX : -1)))
#elif N_AXIS == 7
#define AXIS_LETTER_TO_IDX(c) \
(c == AXIS3_LETTER ? AXIS3_IDX : \
(c == AXIS4_LETTER ? AXIS4_IDX : \
(c == AXIS5_LETTER ? AXIS5_IDX : \
(c == AXIS6_LETTER ? AXIS6_IDX : -1))))
#elif N_AXIS == 8
#define AXIS_LETTER_TO_IDX(c) \
(c == AXIS3_LETTER ? AXIS3_IDX : \
(c == AXIS4_LETTER ? AXIS4_IDX : \
(c == AXIS5_LETTER ? AXIS5_IDX : \
(c == AXIS6_LETTER ? AXIS6_IDX : \
(c == AXIS7_LETTER ? AXIS7_IDX : -1)))))
#endif

#if IS_AXIS_LETTER('A')
#define A_AXIS AXIS_LETTER_TO_IDX('A')
#define A_AXIS_BIT bit(A_AXIS)
#endif

#if IS_AXIS_LETTER('B')
#define B_AXIS AXIS_LETTER_TO_IDX('B')
#define B_AXIS_BIT bit(B_AXIS)
#endif

#if IS_AXIS_LETTER('C')
#define C_AXIS AXIS_LETTER_TO_IDX('C')
#define C_AXIS_BIT bit(C_AXIS)
#endif

#if !LATHE_UVW_OPTION

#if IS_AXIS_LETTER('U')
#define U_AXIS AXIS_LETTER_TO_IDX('U')
#define U_AXIS_BIT bit(U_AXIS)
#endif

#if IS_AXIS_LETTER('V')
#define V_AXIS AXIS_LETTER_TO_IDX('V')
#define V_AXIS_BIT bit(V_AXIS)
#endif

#if IS_AXIS_LETTER('W')
#define W_AXIS AXIS_LETTER_TO_IDX('W')
#define W_AXIS_BIT bit(W_AXIS)
#endif

#endif

typedef union {
    uint8_t mask;
    uint8_t bits;
    uint8_t value;
    struct {
        uint8_t x :1,
                y :1,
                z :1
#if N_AXIS > 3
  #ifdef A_AXIS
              , a :1
  #endif
  #ifdef B_AXIS
              , b :1
  #endif
  #ifdef C_AXIS
              , c :1
  #endif
  #ifdef U_AXIS
              , u :1
  #endif
  #ifdef V_AXIS
              , v :1
  #endif
  #ifdef W_AXIS
              , w :1
  #endif
#endif
;
    };
    struct {
        uint8_t a0 :1,
                a1 :1,
                a2 :1,
                a3 :1,
                a4 :1,
                a5 :1,
                a6 :1,
                a7 :1;
    };
} axes_signals_t;

//! Coordinate data.
typedef union {
    float values[N_AXIS];
    struct {
        float x;
        float y;
        float z;
#if N_AXIS > 3
  #ifdef A_AXIS
        float a;
  #endif
  #ifdef B_AXIS
        float b;
  #endif
  #ifdef C_AXIS
        float c;
  #endif
  #ifdef U_AXIS
        float u;
  #endif
  #ifdef V_AXIS
        float v;
  #endif
  #ifdef W_AXIS
        float w;
  #endif
#endif
    };
} coord_data_t;

//! Coordinate system data.
typedef struct {
    coord_data_t coord;
#ifdef ROTATION_ENABLE
    float rotation;
#endif
} coord_system_data_t;

typedef union {
    int32_t value[N_AXIS];
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
#if N_AXIS > 3
  #ifdef B_AXIS
        int32_t a;
  #endif
  #ifdef B_AXIS
        int32_t b;
  #endif
  #ifdef C_AXIS
        int32_t c;
  #endif
  #ifdef U_AXIS
        int32_t u;
  #endif
  #ifdef V_AXIS
        int32_t v;
  #endif
  #ifdef W_AXIS
        int32_t w;
  #endif
#endif
    };
} mpos_t;

typedef union {
    float values[2];
    struct {
        float x;
        float y;
    };
} point_2d_t;

typedef union {
    float values[3];
    struct {
        float x;
        float y;
        float z;
    };
} point_3d_t;

//! Axis index to plane assignment.
typedef union {
    uint8_t axis[3];
    struct {
        uint8_t axis_0;
        uint8_t axis_1;
        uint8_t axis_linear;
    };
} plane_t;

#pragma pack(push, 1)

//! \brief Limit switches struct, consists of four packed axes_signals_t structs in 32 bits.
typedef union {
    uint32_t bits;
    struct {
        axes_signals_t min;     //!< Min limit switches status, required.
        axes_signals_t max;     //!< Max limit switches status, optional.
        axes_signals_t min2;    //!< Secondary min limit switch(es) status, required for auto squaring enabled axes.
        axes_signals_t max2;    //!< Secondary max limit switches status, optional (of no practical use?).
    };
} limit_signals_t;

//! \brief Home switches struct, consists of two packed axes_signals_t structs.
typedef struct {
    axes_signals_t a;       //!< Primary home switches status, optional. Limit signals are used for homing if not available.
    axes_signals_t b;       //!< Secondary home switch(es) status, required for auto squaring enabled axes if primary switches are available.
} home_signals_t;

//! \brief Stepper driver states struct.
typedef union {
    uint16_t state;
    home_signals_t details; // Stepper driver signals states.
} stepper_state_t;

//! \brief // Stepper driver warning and fault signal states, consists of two packed stepper_state_t structs in 32 bits.
typedef struct {
    stepper_state_t warning; //!< Stepper drivers warning states.
    stepper_state_t fault;   //!< Stepper drivers fault states.
} stepper_status_t;

#pragma pack(pop)

// NOTE: the pin_function_t enum must be kept in sync with any changes!
typedef union {
    uint16_t bits;
    uint16_t mask;
    uint16_t value;
    struct {
        uint16_t reset                :1,
                 feed_hold            :1,
                 cycle_start          :1,
                 safety_door_ajar     :1,
                 block_delete         :1,
                 stop_disable         :1, //! M1
                 e_stop               :1,
                 probe_disconnected   :1,
                 motor_fault          :1,
                 motor_warning        :1,
                 limits_override      :1,
                 single_block         :1,
                 tls_overtravel       :1, //! used for probe (toolsetter) protection
                 probe_overtravel     :1, //! used for probe protection
                 probe_triggered      :1, //! used for probe protection
                 deasserted           :1; //! this flag is set if signals are deasserted.
    };
} control_signals_t;

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

#define bit_istrue(x, mask) (((x) & (mask)) != 0)
#define bit_isfalse(x, mask) (((x) & (mask)) == 0)

extern char const *const axis_letter[];
extern const coord_data_t null_vector;

// Converts an uint32 variable to string.
char *uitoa (uint32_t n);

// Converts a float variable to string with the specified number of decimal places.
char *ftoa (float n, uint8_t decimal_places);

// Trim trailing zeros and possibly decimal point
char *trim_float (char *s);

// Returns true if float value is a whole number (integer)
bool isintf (float value);

status_code_t read_uint (const char *line, uint_fast8_t *char_counter, uint32_t *uint_ptr);

// Read a floating point value from a string. Line points to the input buffer, char_counter
// is the indexer pointing to the current character of the line, while float_ptr is
// a pointer to the result variable. Returns true when it succeeds
bool read_float (const char *line, uint_fast8_t *char_counter, float *float_ptr);

// Non-blocking delay function used for general operation and suspend features.
bool delay_sec (float seconds, delaymode_t mode);

float convert_delta_vector_to_unit_vector(float *vector);

void rotate (coord_data_t *pt, plane_t plane, float angle /*rad*/);

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
