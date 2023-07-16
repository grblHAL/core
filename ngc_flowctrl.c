/*
  ngc_flowctrl.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Program flow control, for filesystem macros

  Part of grblHAL

  Copyright (c) 2023 Terje Io

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

#include "hal.h"

#if NGC_EXPRESSIONS_ENABLE

#include <string.h>

#include "errors.h"
#include "ngc_expr.h"
#include "ngc_params.h"

#ifndef NGC_STACK_DEPTH
#define NGC_STACK_DEPTH 10
#endif

typedef enum {
    NGCFlowCtrl_NoOp = 0,
    NGCFlowCtrl_If,
    NGCFlowCtrl_ElseIf,
    NGCFlowCtrl_Else,
    NGCFlowCtrl_EndIf,
    NGCFlowCtrl_Do,
    NGCFlowCtrl_Continue,
    NGCFlowCtrl_Break,
    NGCFlowCtrl_While,
    NGCFlowCtrl_EndWhile,
    NGCFlowCtrl_Repeat,
    NGCFlowCtrl_EndRepeat,
    NGCFlowCtrl_Return,
    NGCFlowCtrl_RaiseAlarm,
    NGCFlowCtrl_RaiseError
} ngc_cmd_t;

typedef struct {
    uint32_t o_label;
    ngc_cmd_t operation;
    vfs_file_t *file;
    size_t file_pos;
    char *expr;
    uint32_t repeats;
    bool skip;
    bool handled;
    bool brk;
} ngc_stack_entry_t;

static volatile int_fast8_t stack_idx = -1;
static ngc_stack_entry_t stack[NGC_STACK_DEPTH] = {0};

static status_code_t read_command (char *line, uint_fast8_t *pos, ngc_cmd_t *operation)
{
    char c = line[*pos];
    status_code_t status = Status_OK;

    (*pos)++;

    switch(c) {

        case 'A':
            if (!strncmp(line + *pos, "LARM", 4)) {
                *operation = NGCFlowCtrl_RaiseAlarm;
                *pos += 4;
            } else
                status = Status_FlowControlSyntaxError; // Unknown statement name starting with A
            break;

        case 'B':
            if (!strncmp(line + *pos, "REAK", 4)) {
                *operation = NGCFlowCtrl_Break;
                *pos += 4;
            } else
                status = Status_FlowControlSyntaxError; // Unknown statement name starting with B
            break;

        case 'C':
            if (!strncmp(line + *pos, "ONTINUE", 7)) {
                *operation = NGCFlowCtrl_Continue;
                *pos += 7;
            } else
                status = Status_FlowControlSyntaxError; // Unknown statement name starting with C
            break;

        case 'D':
            if (line[*pos] == 'O') {
                *operation = NGCFlowCtrl_Do;
                (*pos)++;
            } else
                status = Status_FlowControlSyntaxError; // Unknown statement name starting with D
            break;

        case 'E':
            if (!strncmp(line + *pos, "LSEIF", 5)) {
                *operation = NGCFlowCtrl_ElseIf;
                *pos += 5;
            } else if (!strncmp(line + *pos, "LSE", 3)) {
                *operation = NGCFlowCtrl_Else;
                *pos += 3;
            } else if (!strncmp(line + *pos, "NDIF", 4)) {
                *operation = NGCFlowCtrl_EndIf;
                *pos += 4;
            } else if (!strncmp(line + *pos, "NDWHILE", 7)) {
                *operation = NGCFlowCtrl_EndWhile;
                *pos += 7;
            } else if (!strncmp(line + *pos, "NDREPEAT", 8)) {
                *operation = NGCFlowCtrl_EndRepeat;
                *pos += 8;
            } else if (!strncmp(line + *pos, "RROR", 4)) {
                *operation = NGCFlowCtrl_RaiseError;
                *pos += 4;
            } else
                status = Status_FlowControlSyntaxError; // Unknown statement name starting with E
            break;

        case 'I':
            if(line[*pos] == 'F') {
                *operation = NGCFlowCtrl_If;
                (*pos)++;
            } else
                *operation = Status_FlowControlSyntaxError; // Unknown statement name starting with F
            break;

        case 'R':
            if (!strncmp(line + *pos, "EPEAT", 5)) {
                *operation = NGCFlowCtrl_Repeat;
                *pos += 5;
            } else if (!strncmp(line + *pos, "ETURN", 5)) {
                *operation = NGCFlowCtrl_Return;
                *pos += 5;
            } else
                *operation = Status_FlowControlSyntaxError; // Unknown statement name starting with R
            break;

        case 'W':
            if (!strncmp(line + *pos, "HILE", 4)) {
                *operation = NGCFlowCtrl_While;
                *pos += 4;
            } else
                status = Status_FlowControlSyntaxError; // Unknown statement name starting with W
            break;

        default:
            status = Status_FlowControlSyntaxError; // Unknown statement
    }

    return status;
}

static status_code_t stack_push (uint32_t o_label, ngc_cmd_t operation)
{
    if(stack_idx < (NGC_STACK_DEPTH - 1)) {
        stack[++stack_idx].o_label = o_label;
        stack[stack_idx].file = hal.stream.file;
        stack[stack_idx].operation = operation;
        return Status_OK;
    }

    return Status_FlowControlStackOverflow;
}

static bool stack_pull (void)
{
    bool ok;

    if((ok = stack_idx >= 0)) {
        if(stack[stack_idx].expr)
            free(stack[stack_idx].expr);
        memset(&stack[stack_idx], 0, sizeof(ngc_stack_entry_t));
        stack_idx--;
    }

    return ok;
}


// Public functions

void ngc_flowctrl_init (void)
{
    while(stack_idx >= 0)
        stack_pull();
}

status_code_t ngc_flowctrl (uint32_t o_label, char *line, uint_fast8_t *pos, bool *skip)
{
    float value;
    bool skipping;
    ngc_cmd_t operation, last_op;

    status_code_t status;

    if((status = read_command(line, pos, &operation)) != Status_OK)
        return status;

    skipping = stack_idx >= 0 && stack[stack_idx].skip;
    last_op = stack_idx >= 0 ? stack[stack_idx].operation : NGCFlowCtrl_NoOp;

    switch(operation) {

        case NGCFlowCtrl_If:
            if(!skipping && (status = ngc_eval_expression(line, pos, &value)) == Status_OK) {
                if((status = stack_push(o_label, operation)) == Status_OK) {
                    stack[stack_idx].skip = value == 0.0f;
                    stack[stack_idx].handled = !stack[stack_idx].skip;
                }
            }
            break;

        case NGCFlowCtrl_ElseIf:
            if(last_op == NGCFlowCtrl_If || last_op == NGCFlowCtrl_ElseIf) {
                if(o_label == stack[stack_idx].o_label &&
                    !(stack[stack_idx].skip = stack[stack_idx].handled) && !stack[stack_idx].handled &&
                      (status = ngc_eval_expression(line, pos, &value)) == Status_OK) {
                    if(!(stack[stack_idx].skip = value == 0.0f)) {
                        stack[stack_idx].operation = operation;
                        stack[stack_idx].handled = true;
                    }
                }
            } else if(!skipping)
                status = Status_FlowControlSyntaxError;
            break;

        case NGCFlowCtrl_Else:
            if(last_op == NGCFlowCtrl_If || last_op == NGCFlowCtrl_ElseIf) {
                if(o_label == stack[stack_idx].o_label) {
                    if(!(stack[stack_idx].skip = stack[stack_idx].handled))
                        stack[stack_idx].operation = operation;
                }
            } else if(!skipping)
                status = Status_FlowControlSyntaxError;
            break;

        case NGCFlowCtrl_EndIf:
            if(last_op == NGCFlowCtrl_If || last_op == NGCFlowCtrl_ElseIf || last_op == NGCFlowCtrl_Else) {
                if(o_label == stack[stack_idx].o_label)
                    stack_pull();
            } else if(!skipping)
                status = Status_FlowControlSyntaxError;
            break;

        case NGCFlowCtrl_Do:
            if(hal.stream.file) {
                if(!skipping && (status = stack_push(o_label, operation)) == Status_OK) {
                    stack[stack_idx].file_pos = vfs_tell(hal.stream.file);
                    stack[stack_idx].skip = false;
                }
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_While:
            if(hal.stream.file) {
                char *expr = line + *pos;
                if(stack[stack_idx].brk) {
                    if(last_op == NGCFlowCtrl_Do && o_label == stack[stack_idx].o_label)
                        stack_pull();
                } else if(!skipping && (status = ngc_eval_expression(line, pos, &value)) == Status_OK) {
                    if(last_op == NGCFlowCtrl_Do) {
                        if(o_label == stack[stack_idx].o_label) {
                            if(value != 0.0f)
                                vfs_seek(stack[stack_idx].file, stack[stack_idx].file_pos);
                            else
                                stack_pull();
                        }
                    } else if((status = stack_push(o_label, operation)) == Status_OK) {
                        if(!(stack[stack_idx].skip = value == 0.0f)) {
                            if((stack[stack_idx].expr = malloc(strlen(expr) + 1))) {
                                strcpy(stack[stack_idx].expr, expr);
                                stack[stack_idx].file = hal.stream.file;
                                stack[stack_idx].file_pos = vfs_tell(hal.stream.file);
                            } else
                                status = Status_FlowControlOutOfMemory;
                        }
                    }
                }
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_EndWhile:
            if(hal.stream.file) {
                if(last_op == NGCFlowCtrl_While) {
                    if(o_label == stack[stack_idx].o_label) {
                        uint_fast8_t pos = 0;
                        if(!stack[stack_idx].skip && (status = ngc_eval_expression(stack[stack_idx].expr, &pos, &value)) == Status_OK) {
                            if(!(stack[stack_idx].skip = value == 0))
                                vfs_seek(stack[stack_idx].file, stack[stack_idx].file_pos);
                        }
                        if(stack[stack_idx].skip)
                            stack_pull();
                    }
                } else
                    status = Status_FlowControlSyntaxError;
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_Repeat:
            if(hal.stream.file) {
                if(!skipping && (status = ngc_eval_expression(line, pos, &value)) == Status_OK) {
                    if((status = stack_push(o_label, operation)) == Status_OK) {
                        if(!(stack[stack_idx].skip = value == 0.0f)) {
                            stack[stack_idx].file = hal.stream.file;
                            stack[stack_idx].file_pos = vfs_tell(hal.stream.file);
                            stack[stack_idx].repeats = (uint32_t)value;
                        }
                    }
                }
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_EndRepeat:
            if(hal.stream.file) {
                if(last_op == NGCFlowCtrl_Repeat) {
                    if(o_label == stack[stack_idx].o_label) {
                        if(stack[stack_idx].repeats && --stack[stack_idx].repeats)
                            vfs_seek(stack[stack_idx].file, stack[stack_idx].file_pos);
                        else
                            stack_pull();
                    }
                } else
                    status = Status_FlowControlSyntaxError;
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_Break:
            if(hal.stream.file) {
                if(!skipping) {
                    while(o_label != stack[stack_idx].o_label && stack_pull());
                    last_op = stack_idx >= 0 ? stack[stack_idx].operation : NGCFlowCtrl_NoOp;
                    if(last_op == NGCFlowCtrl_Do || last_op == NGCFlowCtrl_While || last_op == NGCFlowCtrl_Repeat) {
                        if(o_label == stack[stack_idx].o_label) {
                            stack[stack_idx].repeats = 0;
                            stack[stack_idx].brk = stack[stack_idx].skip = stack[stack_idx].handled = true;
                        }
                    } else
                        status = Status_FlowControlSyntaxError;
                }
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_Continue:
            if(hal.stream.file) {
                if(!skipping) {
                    while(o_label != stack[stack_idx].o_label && stack_pull());
                    if(stack_idx >= 0 && o_label == stack[stack_idx].o_label) switch(stack[stack_idx].operation) {

                        case NGCFlowCtrl_Repeat:
                            if(stack[stack_idx].repeats && --stack[stack_idx].repeats)
                                vfs_seek(stack[stack_idx].file, stack[stack_idx].file_pos);
                            else
                                stack_pull();
                            break;

                        case NGCFlowCtrl_Do:
                            vfs_seek(stack[stack_idx].file, stack[stack_idx].file_pos);
                            break;

                        case NGCFlowCtrl_While:
                            {
                                uint_fast8_t pos = 0;
                                if(!stack[stack_idx].skip && (status = ngc_eval_expression(stack[stack_idx].expr, &pos, &value)) == Status_OK) {
                                    if(!(stack[stack_idx].skip = value == 0))
                                        vfs_seek(stack[stack_idx].file, stack[stack_idx].file_pos);
                                }
                                if(stack[stack_idx].skip) {
                                    if(stack[stack_idx].expr) {
                                        free(stack[stack_idx].expr);
                                        stack[stack_idx].expr = NULL;
                                    }
                                    stack_pull();
                                }
                            }
                            break;

                        default:
                            status = Status_FlowControlSyntaxError;
                            break;
                    } else
                        status = Status_FlowControlSyntaxError;
                }
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_RaiseAlarm:
            if(!skipping && ngc_eval_expression(line, pos, &value) == Status_OK)
                system_raise_alarm((alarm_code_t)value);
            break;

        case NGCFlowCtrl_RaiseError:
            if(!skipping && ngc_eval_expression(line, pos, &value) == Status_OK)
                status = (status_code_t)value;
            break;

        case NGCFlowCtrl_Return:
            if(!skipping && grbl.on_macro_return) {
                vfs_file_t *file = stack[stack_idx].file;
                while(stack_idx >= 0 && file == stack[stack_idx].file)
                    stack_pull();
                if(ngc_eval_expression(line, pos, &value) == Status_OK) {
                    ngc_named_param_set("_value", value);
                    ngc_named_param_set("_value_returned", 1.0f);
                } else
                    ngc_named_param_set("_value_returned", 0.0f);
                grbl.on_macro_return();
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        default:
            status = Status_FlowControlSyntaxError;
    }

    if(status != Status_OK) {
        ngc_flowctrl_init();
        *skip = false;
    } else
        *skip = stack_idx >= 0 && stack[stack_idx].skip;

    return status;
}

#endif // NGC_EXPRESSIONS_ENABLE
