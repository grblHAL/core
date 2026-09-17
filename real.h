/*
  real.h - wrappers for real type (float or double)

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

#if REAL_IS_DOUBLE

typedef double real_t;

#define powr(a, b) pow(a, b)
#define sqrtr(a) sqrt(a)
#define fabsr(a) fabs(a)
#define sinr(a) sin(a)
#define asinr(a) asin(a)
#define cosr(a) cos(a)
#define acosr(a) acos(a)
#define tanr(a) tan(a)
#define atanr(a) atan(a)
#define atan2r(a, b) atan(a, b)
#define ceilr(a) ceil(a)
#define floorr(a) floor(a)
#define lroundr(a) lround(a)
#define truncr(a) trunc(a)
#define modfr(a, b) modf(a, b)
#define fmodr(a, b) fmod(a, b)
#define isnanr(a) isnan(a)

#else

typedef float real_t;

#define powr(a, b) powf(a, b)
#define sqrtr(a) sqrtf(a)
#define fabsr(a) fabsf(a)
#define sinr(a) sinf(a)
#define asinr(a) asinf(a)
#define cosr(a) cosf(a)
#define acosr(a) acosf(a)
#define tanr(a) tanf(a)
#define atanr(a) atanf(a)
#define atan2r(a, b) atanf(a, b)
#define ceilr(a) ceilf(a)
#define floorr(a) floorf(a)
#define lroundr(a) lroundf(a)
#define truncr(a) truncf(a)
#define modfr(a, b) modff(a, b)
#define fmodr(a, b) fmodf(a, b)
#define isnanr(a) isnanf(a)

#endif
