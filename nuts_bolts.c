/*
  nuts_bolts.c - Shared functions

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

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "protocol.h"
#include "state_machine.h"
#include "nuts_bolts.h"

#ifndef DWELL_TIME_STEP
#define DWELL_TIME_STEP 50 // Integer (1-255) (milliseconds)
#endif

#define MAX_PRECISION 10

static char buf[STRLEN_COORDVALUE + 1];

static const float froundvalues[MAX_PRECISION + 1] =
{
    0.5,                // 0
    0.05,               // 1
    0.005,              // 2
    0.0005,             // 3
    0.00005,            // 4
    0.000005,           // 5
    0.0000005,          // 6
    0.00000005,         // 7
    0.000000005,        // 8
    0.0000000005,       // 9
    0.00000000005       // 10
};

char const *const axis_letter[N_AXIS] = {
    "X",
    "Y",
    "Z"
#if N_AXIS > 3
    ,"A"
#endif
#if N_AXIS > 4
    ,"B"
#endif
#if N_AXIS > 5
    ,"C"
#endif
};

// Converts an uint32 variable to string.
char *uitoa (uint32_t n)
{
    char *bptr = buf + sizeof(buf);

    *--bptr = '\0';

    if (n == 0)
        *--bptr = '0';
    else while (n) {
        *--bptr = '0' + (n % 10);
        n /= 10;
    }

    return bptr;
}

// Convert float to string by immediately converting to integers.
// Number of decimal places, which are tracked by a counter, must be set by the user.
// The integers is then efficiently converted to a string.
char *ftoa (float n, uint8_t decimal_places)
{
    bool isNegative;
    char *bptr = buf + sizeof(buf);

    *--bptr = '\0';

    if ((isNegative = n < 0.0f))
        n = -n;

    n += froundvalues[decimal_places];

    uint32_t a = (uint32_t)n;

    if (decimal_places) {

        n -= (float)a;

        uint_fast8_t decimals = decimal_places;
        while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
            n *= 100.0f;
            decimals -= 2;
        }

        if (decimals)
            n *= 10.0f;

        uint32_t b = (uint32_t)n;

        while(decimal_places--) {
            if(b) {
                *--bptr = (b % 10) + '0'; // Get digit
                b /= 10;
            } else
                *--bptr = '0';
        }
    }

    *--bptr = '.'; // Always add decimal point (TODO: is this really needed?)

    if(a == 0)
        *--bptr = '0';

    else while(a) {
        *--bptr = (a % 10) + '0'; // Get digit
        a /= 10;
    }

    if(isNegative)
        *--bptr = '-';

    return bptr;
}

// Extracts a floating point value from a string. The following code is based loosely on
// the avr-libc strtod() function by Michael Stumpf and Dmitry Xmelkov and many freely
// available conversion method examples, but has been highly optimized for Grbl. For known
// CNC applications, the typical decimal value is expected to be in the range of E0 to E-4.
// Scientific notation is officially not supported by g-code, and the 'E' character may
// be a g-code word on some CNC systems. So, 'E' notation will not be recognized.
// NOTE: Thanks to Radu-Eosif Mihailescu for identifying the issues with using strtod().
bool read_float (char *line, uint_fast8_t *char_counter, float *float_ptr)
{
    char *ptr = line + *char_counter;
    int_fast8_t exp = 0;
    uint_fast8_t ndigit = 0, c;
    uint32_t intval = 0;
    bool isnegative, isdecimal = false;

    // Grab first character and increment pointer. No spaces assumed in line.
    c = *ptr++;

    // Capture initial positive/minus character
    if ((isnegative = (c == '-')) || c == '+')
        c = *ptr++;

    // Extract number into fast integer. Track decimal in terms of exponent value.
    while(c) {
        c -= '0';
        if (c <= 9) {
            ndigit++;
            if (ndigit <= MAX_INT_DIGITS) {
                if (isdecimal)
                    exp--;
                intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
            } else if (!isdecimal)
                exp++;  // Drop overflow digits
        } else if (c == (uint_fast8_t)('.' - '0') && !isdecimal)
            isdecimal = true;
         else
            break;

        c = *ptr++;
    }

    // Return if no digits have been read.
    if (!ndigit)
        return false;

    // Convert integer into floating point.
    float fval = (float)intval;

    // Apply decimal. Should perform no more than two floating point multiplications for the
    // expected range of E0 to E-4.
    if (fval != 0.0f) {
        while (exp <= -2) {
            fval *= 0.01f;
            exp += 2;
        }
        if (exp < 0)
            fval *= 0.1f;
        else if (exp > 0) do {
            fval *= 10.0f;
        } while (--exp > 0);
    }

    // Assign floating point value with correct sign.
    *float_ptr = isnegative ? - fval : fval;
    *char_counter = ptr - line - 1; // Set char_counter to next statement

    return true;
}

// Returns true if float value is a whole number (integer)
bool isintf (float value)
{
    return value != NAN && fabsf(value - truncf(value)) < 0.001f;
}

// Non-blocking delay function used for general operation and suspend features.
void delay_sec (float seconds, delaymode_t mode)
{
    uint_fast16_t i = (uint_fast16_t)ceilf((1000.0f / DWELL_TIME_STEP) * seconds) + 1;

    while (--i && !sys.abort) {
        if (mode == DelayMode_Dwell) {
            protocol_execute_realtime();
        } else { // DelayMode_SysSuspend
          // Execute rt_system() only to avoid nesting suspend loops.
          protocol_exec_rt_system();
          if (state_door_reopened()) // Bail, if safety door reopens.
              return;
        }
        hal.delay_ms(DWELL_TIME_STEP, 0); // Delay DWELL_TIME_STEP increment
    }
}


float convert_delta_vector_to_unit_vector (float *vector)
{
    uint_fast8_t idx = N_AXIS;
    float magnitude = 0.0f, inv_magnitude;

    do {
        if (vector[--idx] != 0.0f)
            magnitude += vector[idx] * vector[idx];
    } while(idx);

    idx = N_AXIS;
    magnitude = sqrtf(magnitude);
    inv_magnitude = 1.0f / magnitude;

    do {
        vector[--idx] *= inv_magnitude;
    } while(idx);

    return magnitude;
}


// calculate checksum byte for data
uint8_t calc_checksum (uint8_t *data, uint32_t size) {

    uint8_t checksum = 0;

    while(size--) {
        checksum = (checksum << 1) | (checksum >> 7);
        checksum += *(data++);
    }

    return checksum;
}

// Remove spaces from and convert string to uppercase (in situ)
char *strcaps (char *s)
{
    char c, *s1 = s, *s2 = s;

    do {
        c = *s1++;
        if(c != ' ')
            *s2++ = CAPS(c);
    } while(c);

    *s2 = '\0';

    return s;
}

void dummy_handler (void)
{
    // NOOP
}
