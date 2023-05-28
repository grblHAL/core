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
} ngc_cmd_stack_t;

static ngc_cmd_stack_t stack[10] = {0};
static int_fast8_t stack_index = -1;

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
                status = Status_FlowControlSyntaxError; // Unknown operation name starting with G
            break;

        case 'B':
            if (!strncmp(line + *pos, "REAK", 4)) {
                *operation = NGCFlowCtrl_Break;
                *pos += 4;
            } else
                status = Status_FlowControlSyntaxError; // Unknown operation name starting with G
            break;

        case 'C':
            if (!strncmp(line + *pos, "ONTINUE", 7)) {
                *operation = NGCFlowCtrl_Continue;
                *pos += 7;
            } else
                status = Status_FlowControlSyntaxError; // Unknown operation name starting with G
            break;

        case 'D':
            if (line[*pos] == 'O') {
                *operation = NGCFlowCtrl_Do;
                (*pos)++;
            } else
                status = Status_FlowControlSyntaxError; // Unknown operation name starting with R
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
                status = Status_FlowControlSyntaxError; // Unknown operation name starting with G
            break;

        case 'I':
            if(line[*pos] == 'F') {
                *operation = NGCFlowCtrl_If;
                (*pos)++;
            } else
                *operation = Status_FlowControlSyntaxError;
            break;

        case 'R':
            if (!strncmp(line + *pos, "EPEAT", 5)) {
                *operation = NGCFlowCtrl_Repeat;
                *pos += 5;
            } else if (!strncmp(line + *pos, "ETURN", 5)) {
                *operation = NGCFlowCtrl_Return;
                *pos += 5;
            } else
                *operation = Status_FlowControlSyntaxError;
            break;

        case 'W':
            if (!strncmp(line + *pos, "HILE", 4)) {
                *operation = NGCFlowCtrl_While;
                *pos += 4;
            } else
                status = Status_FlowControlSyntaxError; // Unknown operation name starting with G
            break;

        default:
            status = Status_FlowControlSyntaxError; // Unknown operation name
    }

    return status;
}

void ngc_flowctrl_init (void)
{
    if(stack_index >= 0) do {
        if(stack[stack_index].expr)
            free(stack[stack_index].expr);
    } while(--stack_index != -1);

    memset(stack, 0, sizeof(stack));
}

status_code_t ngc_flowctrl (uint32_t o_label, char *line, uint_fast8_t *pos, bool *skip)
{
    float value;
    bool skipping;
    ngc_cmd_t operation, last_op;

    status_code_t status;

    if((status = read_command(line, pos, &operation)) != Status_OK)
        return status;

    skipping = stack_index >= 0 && stack[stack_index].skip;
    last_op = stack_index >= 0 ? stack[stack_index].operation : NGCFlowCtrl_NoOp;

    switch(operation) {

        case NGCFlowCtrl_If:
            if(!skipping && (status = ngc_eval_expression(line, pos, &value)) == Status_OK) {
                stack[++stack_index].o_label = o_label;
                stack[stack_index].operation = operation;
                stack[stack_index].o_label = o_label;
                stack[stack_index].file = sys.macro_file;
                stack[stack_index].skip = value == 0.0f;
                stack[stack_index].handled = !stack[stack_index].skip;
            }
            break;

        case NGCFlowCtrl_ElseIf:
            if(last_op == NGCFlowCtrl_If || last_op == NGCFlowCtrl_ElseIf) {
                if(o_label == stack[stack_index].o_label &&
                    !(stack[stack_index].skip = stack[stack_index].handled) && !stack[stack_index].handled &&
                      (status = ngc_eval_expression(line, pos, &value)) == Status_OK) {
                    if(!(stack[stack_index].skip = value == 0.0f)) {
                        stack[stack_index].operation = operation;
                        stack[stack_index].handled = true;
                    }
                }
            } else
                status = Status_FlowControlSyntaxError;
            break;

        case NGCFlowCtrl_Else:
            if(last_op == NGCFlowCtrl_If || last_op == NGCFlowCtrl_ElseIf) {
                if(o_label == stack[stack_index].o_label) {
                    if(!(stack[stack_index].skip = stack[stack_index].handled))
                        stack[stack_index].operation = operation;
                }
            } else
                status = Status_FlowControlSyntaxError;
            break;

        case NGCFlowCtrl_EndIf:
            if(last_op == NGCFlowCtrl_If || last_op == NGCFlowCtrl_ElseIf || last_op == NGCFlowCtrl_Else) {
                if(o_label == stack[stack_index].o_label)
                    stack_index--;
            } else
                status = Status_FlowControlSyntaxError;
            break;

        case NGCFlowCtrl_Do:
            if(sys.macro_file) {
                if(!skipping) {
                    stack[++stack_index].operation = operation;
                    stack[stack_index].o_label = o_label;
                    stack[stack_index].file = sys.macro_file;
                    stack[stack_index].file_pos = vfs_tell(sys.macro_file);
                    stack[stack_index].skip = false;
                }
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_While:
            if(sys.macro_file) {
                char *expr = line + *pos;
                if(!skipping && (status = ngc_eval_expression(line, pos, &value)) == Status_OK) {
                    if(last_op == NGCFlowCtrl_Do) {
                        if(o_label == stack[stack_index].o_label) {
                            if(value != 0.0f)
                                vfs_seek(sys.macro_file, stack[stack_index].file_pos);
                            else
                                stack_index--;
                        }
                    } else {
                        stack[++stack_index].operation = operation;
                        stack[stack_index].o_label = o_label;
                        if(!(stack[stack_index].skip = value == 0.0f)) {
                            if((stack[stack_index].expr = malloc(strlen(expr) + 1))) {
                                strcpy(stack[stack_index].expr, expr);
                                stack[stack_index].file = sys.macro_file;
                                stack[stack_index].file_pos = vfs_tell(sys.macro_file);
                            } else
                                status = Status_FlowControlOutOfMemory;
                        }
                    }
                }
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_EndWhile:
            if(sys.macro_file) {
                if(last_op == NGCFlowCtrl_While) {
                    if(o_label == stack[stack_index].o_label) {
                        uint_fast8_t pos = 0;
                        if(!stack[stack_index].skip && (status = ngc_eval_expression(stack[stack_index].expr, &pos, &value)) == Status_OK) {
                            if(!(stack[stack_index].skip = value == 0))
                                vfs_seek(sys.macro_file, stack[stack_index].file_pos);
                        }
                        if(stack[stack_index].skip) {
                            if(stack[stack_index].expr) {
                                free(stack[stack_index].expr);
                                stack[stack_index].expr = NULL;
                            }
                            stack_index--;
                        }
                    }
                } else
                    status = Status_FlowControlSyntaxError;
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_Repeat:
            if(sys.macro_file) {
                if(!skipping && (status = ngc_eval_expression(line, pos, &value)) == Status_OK) {
                    stack[++stack_index].operation = operation;
                    stack[stack_index].o_label = o_label;
                    if(!(stack[stack_index].skip = value == 0.0f)) {
                        stack[stack_index].file = sys.macro_file;
                        stack[stack_index].file_pos = vfs_tell(sys.macro_file);
                        stack[stack_index].repeats = (uint32_t)value;
                    }
                }
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_EndRepeat:
            if(sys.macro_file) {
                if(last_op == NGCFlowCtrl_Repeat) {
                    if(o_label == stack[stack_index].o_label) {
                        if(stack[stack_index].repeats && --stack[stack_index].repeats)
                            vfs_seek(sys.macro_file, stack[stack_index].file_pos);
                        else
                            stack_index--;
                    }
                } else
                    status = Status_FlowControlSyntaxError;
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_Break:
            if(sys.macro_file) {
                if(last_op == NGCFlowCtrl_Do || last_op == NGCFlowCtrl_While || last_op == NGCFlowCtrl_Repeat) {
                    if(o_label == stack[stack_index].o_label) {
                        stack[stack_index].repeats = 0;
                        stack[stack_index].skip = true;
                    }
                } else
                    status = Status_FlowControlSyntaxError;
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_Continue:
            if(sys.macro_file) {
                if(o_label == stack[stack_index].o_label) switch(last_op) {

                    case NGCFlowCtrl_Repeat:
                        if(stack[stack_index].repeats && --stack[stack_index].repeats)
                            vfs_seek(sys.macro_file, stack[stack_index].file_pos);
                        else
                            stack_index--;
                        break;

                    case NGCFlowCtrl_Do:
                        vfs_seek(sys.macro_file, stack[stack_index].file_pos);
                        break;

                    case NGCFlowCtrl_While:
                        {
                            uint_fast8_t pos = 0;
                            if(!stack[stack_index].skip && (status = ngc_eval_expression(stack[stack_index].expr, &pos, &value)) == Status_OK) {
                                if(!(stack[stack_index].skip = value == 0))
                                    vfs_seek(sys.macro_file, stack[stack_index].file_pos);
                            }
                            if(stack[stack_index].skip) {
                                if(stack[stack_index].expr) {
                                    free(stack[stack_index].expr);
                                    stack[stack_index].expr = NULL;
                                }
                                stack_index--;
                            }
                        }
                        break;

                    default:
                        status = Status_FlowControlSyntaxError;
                        break;
                } else
                    status = Status_FlowControlSyntaxError;
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_RaiseAlarm:
            if(ngc_eval_expression(line, pos, &value) == Status_OK)
                system_raise_alarm((alarm_code_t)value);
            break;

        case NGCFlowCtrl_RaiseError:
            if(ngc_eval_expression(line, pos, &value) == Status_OK)
                status = (status_code_t)value;
            break;

        case NGCFlowCtrl_Return:
            if(!skipping && grbl.on_macro_return) {
                stack_index = -1;
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
        *skip = stack_index >= 0 && stack[stack_index].skip;

    return status;
}

#endif // NGC_EXPRESSIONS_ENABLE
