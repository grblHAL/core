/*
  sleep.h - Sleep methods header file

  Part of grblHAL
  
  Copyright (c) 2016 Sungeun K. Jeon  

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

#ifndef _SLEEP_H_
#define _SLEEP_H_

// Checks running conditions for sleep. If satisfied, enables sleep countdown and executes
// sleep mode upon elapse.
void sleep_check();

#endif
