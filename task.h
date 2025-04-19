/*
  task.h - delayed task handling

  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io

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

#ifndef _CORE_TASK_H_
#define _CORE_TASK_H_

typedef void (*foreground_task_ptr)(void *data);

bool task_add_immediate (foreground_task_ptr fn, void *data);
bool task_add_delayed (foreground_task_ptr fn, void *data, uint32_t delay_ms);
bool task_run_on_startup (foreground_task_ptr fn, void *data);
void task_delete (foreground_task_ptr fn, void *data);
bool task_add_systick (foreground_task_ptr fn, void *data);
void task_delete_systick (foreground_task_ptr fn, void *data);

#endif // _CORE_TASK_H_
