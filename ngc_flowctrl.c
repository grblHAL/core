/*
  ngc_flowctrl.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Program flow control, for filesystem macros

  Part of grblHAL

  Copyright (c) 2023-2024 Terje Io

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

#include "hal.h"

#if NGC_EXPRESSIONS_ENABLE

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "errors.h"
#include "ngc_expr.h"
#include "ngc_params.h"
#include "stream_file.h"
//#include "string_registers.h"

#ifndef NGC_STACK_DEPTH
#define NGC_STACK_DEPTH 20
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
    NGCFlowCtrl_Sub,
    NGCFlowCtrl_EndSub,
    NGCFlowCtrl_Call,
    NGCFlowCtrl_Return,
    NGCFlowCtrl_RaiseAlarm,
    NGCFlowCtrl_RaiseError
} ngc_cmd_t;

typedef struct ngc_sub {
    uint32_t o_label;
    vfs_file_t *file;
    size_t file_pos;
    struct ngc_sub *next;
} ngc_sub_t;

typedef struct {
    uint32_t o_label;
    ngc_cmd_t operation;
    ngc_sub_t *sub;
    vfs_file_t *file;
    size_t file_pos;
    char *expr;
    uint32_t repeats;
    bool skip;
    bool handled;
    bool brk;
} ngc_stack_entry_t;

static volatile int_fast8_t stack_idx = -1;
static bool skip_sub = false;
static ngc_sub_t *subs = NULL, *exec_sub = NULL;
static ngc_stack_entry_t stack[NGC_STACK_DEPTH] = {0};
static on_gcode_message_ptr on_gcode_comment;

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
            } else if (!strncmp(line + *pos, "ALL", 3)) {
                *operation = NGCFlowCtrl_Call;
                *pos += 3;
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
            } else if (!strncmp(line + *pos, "NDSUB", 5)) {
                *operation = NGCFlowCtrl_EndSub;
                *pos += 5;
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
                status = Status_FlowControlSyntaxError; // Unknown statement name starting with F
            break;

        case 'R':
            if (!strncmp(line + *pos, "EPEAT", 5)) {
                *operation = NGCFlowCtrl_Repeat;
                *pos += 5;
            } else if (!strncmp(line + *pos, "ETURN", 5)) {
                *operation = NGCFlowCtrl_Return;
                *pos += 5;
            } else
                status = Status_FlowControlSyntaxError; // Unknown statement name starting with R
            break;

        case 'S':
            if (!strncmp(line + *pos, "UB", 2)) {
                *operation = NGCFlowCtrl_Sub;
                *pos += 2;
            } else
                status = Status_FlowControlSyntaxError; // Unknown statement name starting with S
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

// Returns the last called named sub with reference count
static ngc_sub_t *get_refcount (uint32_t *refcount)
{
    ngc_sub_t *sub, *last = NULL;
    uint32_t o_label = 0;

    if((sub = subs)) do {
        if(sub->o_label > NGC_MAX_PARAM_ID)
            o_label = sub->o_label;
    } while((sub = sub->next));

    if((sub = subs)) do {
        if(sub->o_label == o_label) {
            last = sub;
            (*refcount)++;
        }
    } while((sub = sub->next));

    return last;
}

static ngc_sub_t *add_sub (uint32_t o_label, vfs_file_t *file)
{
    ngc_sub_t *sub;

    if((sub = malloc(sizeof(ngc_sub_t))) != NULL) {
        sub->o_label = o_label;
        sub->file = file;
        sub->file_pos = vfs_tell(file);
        sub->next = NULL;
        if(subs == NULL)
            subs = sub;
        else {
            ngc_sub_t *last = subs;
            while(last->next)
                last = last->next;
            last->next = sub;
        }
    }

    return sub;
}

static void clear_subs (vfs_file_t *file)
{
    ngc_sub_t *current = subs, *prev = NULL, *next;

    subs = NULL;

    while(current) {
        next = current->next;
        if(file == NULL || file == current->file) {
            free(current);
            if(prev)
                prev->next = next;
        } else {
            if(subs == NULL)
                subs = current;
            prev = current;
        }
        current = next;
    }
}

static status_code_t stack_push (uint32_t o_label, ngc_cmd_t operation)
{
    if(stack_idx < (NGC_STACK_DEPTH - 1) && (operation != NGCFlowCtrl_Call || ngc_call_push(&stack[stack_idx + 1]))) {
        stack[++stack_idx].o_label = o_label;
        stack[stack_idx].file = hal.stream.file;
        stack[stack_idx].operation = operation;
        stack[stack_idx].sub = exec_sub;
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
        if(stack[stack_idx].operation == NGCFlowCtrl_Call)
            ngc_call_pop();
        memset(&stack[stack_idx], 0, sizeof(ngc_stack_entry_t));
        stack_idx--;
    }

    return ok;
}

static void stack_unwind_sub (uint32_t o_label)
{
    while(stack_idx >= 0 && stack[stack_idx].o_label != o_label)
        stack_pull();

    if(stack_idx >= 0) {
        if(o_label > NGC_MAX_PARAM_ID) {
            uint32_t count = 0;
            if(stack[stack_idx].sub == get_refcount(&count) && count == 1)
                ngc_string_param_delete((ngc_string_id_t)o_label);
            clear_subs(stack[stack_idx].file);
            stream_redirect_close(stack[stack_idx].file);
        } else
            vfs_seek(stack[stack_idx].file, stack[stack_idx].file_pos);

        stack_pull();
    }

    exec_sub = stack_idx >= 0 ? stack[stack_idx].sub : NULL;
}

// Public functions

void ngc_flowctrl_unwind_stack (vfs_file_t *file)
{
    clear_subs(file);
    while(stack_idx >= 0 && stack[stack_idx].file == file)
        stack_pull();
}

static status_code_t onGcodeComment (char *comment)
{
    uint_fast8_t pos = 6;
    status_code_t status = Status_OK;

    if(!strncasecmp(comment, "ABORT,", 6)) {
        char *msg = NULL;
        *comment = '!';
        msg = grbl.on_process_gcode_comment(comment);
        if(msg == NULL)
            msg = ngc_substitute_parameters(comment + pos);
        if(msg) {
            report_message(msg, Message_Error);
            free(msg);
        }
        status = Status_UserException;
    } else if(on_gcode_comment)
        status = on_gcode_comment(comment);

    return status;
}

void ngc_flowctrl_init (void)
{
    static bool init_ok = false;

    if(!init_ok) {
        init_ok = true;
        on_gcode_comment = grbl.on_gcode_comment;
        grbl.on_gcode_comment = onGcodeComment;
    }

    clear_subs(NULL);
    while(stack_idx >= 0)
        stack_pull();
}

// NOTE: onNamedSubError will be called recursively for each
// redirected file by the grbl.report.status_message() call.
static status_code_t onNamedSubError (status_code_t status)
{
    static bool closing = false;

    if(stack_idx >= 0) {

        ngc_sub_t *sub;
        uint32_t o_label = 0;

        if((sub = subs)) do {
            if(sub->file == hal.stream.file && sub->o_label > NGC_MAX_PARAM_ID)
                o_label = sub->o_label;
        } while(o_label == 0 && (sub = sub->next));

        if(sub) {

            if(!closing) {
                char *name, msg[100];
                closing = true;
                if((name = ngc_string_param_get((ngc_string_id_t)o_label))) {
                    sprintf(msg, "error %d in named sub %s.macro", (uint8_t)status, name);
                    report_message(msg, Message_Warning);
                }
            }

            stack_unwind_sub(o_label);
            status = grbl.report.status_message(status);
        }

        closing = false;
        ngc_flowctrl_init();
    }

    return status;
}

static status_code_t onNamedSubEOF (vfs_file_t *file, status_code_t status)
{
    if(stack_idx >= 0 && stack[stack_idx].file == file) {
        stream_redirect_close(stack[stack_idx].file);
        ngc_flowctrl_unwind_stack(stack[stack_idx].file);
    }

    return status;
}

status_code_t ngc_flowctrl (uint32_t o_label, char *line, uint_fast8_t *pos, bool *skip)
{
    float value;
    bool skipping;
    ngc_cmd_t operation, last_op;

    status_code_t status;

    if((status = read_command(line, pos, &operation)) != Status_OK)
        return status;

    skipping = skip_sub || (stack_idx >= 0 && stack[stack_idx].skip);
    last_op = stack_idx >= 0 ? stack[stack_idx].operation : (skip_sub ? NGCFlowCtrl_Sub : NGCFlowCtrl_NoOp);

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
                if(stack_idx >= 0 && stack[stack_idx].brk) {
                    if(last_op == NGCFlowCtrl_Do && o_label == stack[stack_idx].o_label)
                        stack_pull();
                } else if(!skipping && (status = ngc_eval_expression(line, pos, &value)) == Status_OK) {
                    if(last_op == NGCFlowCtrl_Do && o_label == stack[stack_idx].o_label) {
                        if(value != 0.0f)
                            vfs_seek(stack[stack_idx].file, stack[stack_idx].file_pos);
                        else
                            stack_pull();
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
                        if(!skipping) {
                            uint_fast8_t pos = 0;
                            if(!stack[stack_idx].skip && (status = ngc_eval_expression(stack[stack_idx].expr, &pos, &value)) == Status_OK) {
                                if(!(stack[stack_idx].skip = value == 0.0f))
                                    vfs_seek(stack[stack_idx].file, stack[stack_idx].file_pos);
                            }
                        }
                        if(stack[stack_idx].skip)
                            stack_pull();
                    }
                } else if(!skipping)
                    status = Status_FlowControlSyntaxError;
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_Repeat:
            if(hal.stream.file) {
                if(!skipping && (status = ngc_eval_expression(line, pos, &value)) == Status_OK) {
                    if((status = stack_push(o_label, operation)) == Status_OK) {
                        value = nearbyintf(value);
                        if(!(stack[stack_idx].skip = value <= 0.0f)) {
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
                        if(!skipping && stack[stack_idx].repeats && --stack[stack_idx].repeats)
                            vfs_seek(stack[stack_idx].file, stack[stack_idx].file_pos);
                        else
                            stack_pull();
                    }
                } else if(!skipping)
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
                                stack[stack_idx].skip = true;
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

        case NGCFlowCtrl_Sub:
            if(hal.stream.file) {
                ngc_sub_t *sub;
                if(o_label > NGC_MAX_PARAM_ID) {

                    if((sub = subs)) do {
                        if(sub->o_label == o_label && sub->file == hal.stream.file)
                            break;
                    } while(sub->next && (sub = sub->next));

                    if(sub == NULL || sub->o_label != o_label)
                        status = Status_FlowControlSyntaxError;
                } else if(!(skip_sub = (sub = add_sub(o_label, hal.stream.file)) != NULL))
                    status = Status_FlowControlOutOfMemory;
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_EndSub:
            if(hal.stream.file) {
                if(!skip_sub) {
                    stack_unwind_sub(o_label);
                    if(ngc_eval_expression(line, pos, &value) == Status_OK) {
                        ngc_named_param_set("_value", value);
                        ngc_named_param_set("_value_returned", 1.0f);
                    } else
                        ngc_named_param_set("_value_returned", 0.0f);
                }
                skip_sub = false;
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_Call:

            if(hal.stream.file || o_label > NGC_MAX_PARAM_ID) {

                if(!skipping) {

                    ngc_sub_t *sub = NULL;

                    if(o_label > NGC_MAX_PARAM_ID) {

                        char *subname;
                        if((subname = ngc_string_param_get((ngc_string_id_t)o_label))) {
                            char filename[60];
                            vfs_file_t *file;
#if LITTLEFS_ENABLE == 1
                            sprintf(filename, "/littlefs/%s.macro", subname);

                            if((file = stream_redirect_read(filename, onNamedSubError, onNamedSubEOF)) == NULL) {
                                sprintf(filename, "/%s.macro", subname);
                                file = stream_redirect_read(filename, onNamedSubError, onNamedSubEOF);
                            }
#else
                            sprintf(filename, "/%s.macro", subname);
                            file = stream_redirect_read(filename, onNamedSubError, onNamedSubEOF);
#endif
                            if(file) {
                                if((sub = add_sub(o_label, file)) == NULL)
                                    status = Status_FlowControlOutOfMemory;
                            } else
                                status = Status_FileOpenFailed;
                       }
                    } else if((sub = subs)) do {
                        if(sub->o_label == o_label && sub->file == hal.stream.file)
                            break;
                    } while((sub = sub->next));

                    if(sub == NULL)
                        status = Status_FlowControlSyntaxError;
                    else {

                        float params[30];
                        ngc_param_id_t param_id = 1;

                        while(line[*pos] && status == Status_OK && param_id <= 30) {
                            status = ngc_eval_expression(line, pos, &params[param_id - 1]);
                            param_id++;
                        }

                        if(status == Status_OK && param_id < 30) do {
                            ngc_param_get(param_id, &params[param_id - 1]);
                        } while(++param_id <= 30);

                        if(status == Status_OK && (status = stack_push(o_label, operation)) == Status_OK) {

                            stack[stack_idx].sub = exec_sub = sub;
                            stack[stack_idx].file = hal.stream.file;
                            stack[stack_idx].file_pos = vfs_tell(hal.stream.file);
                            stack[stack_idx].repeats = 1;

                            for(param_id = 1; param_id <= 30; param_id++) {
                                if(params[param_id - 1] != 0.0f) {
                                    if(!ngc_param_set(param_id, params[param_id - 1]))
                                        status = Status_FlowControlOutOfMemory;
                                }
                            }

                            if(status == Status_OK) {
                                ngc_named_param_set("_value", 0.0f);
                                ngc_named_param_set("_value_returned", 0.0f);
                                vfs_seek(sub->file, sub->file_pos);
                            }
                        }
                    }
                }
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        case NGCFlowCtrl_Return:
            if(hal.stream.file) {
                if(!skipping) {

                    bool g65_return = false;

                    if(exec_sub)
                        stack_unwind_sub(o_label);
                    else if((g65_return = !!grbl.on_macro_return))
                        ngc_flowctrl_unwind_stack(stack[stack_idx].file);

                    if(ngc_eval_expression(line, pos, &value) == Status_OK) {
                        ngc_named_param_set("_value", value);
                        ngc_named_param_set("_value_returned", 1.0f);
                    } else
                        ngc_named_param_set("_value_returned", 0.0f);

                    if(g65_return)
                        grbl.on_macro_return();
                }
            } else
                status = Status_FlowControlNotExecutingMacro;
            break;

        default:
            status = Status_FlowControlSyntaxError;
    }

    if(status != Status_OK) {
        ngc_flowctrl_init();
        *skip = false;
        if(settings.flags.ngc_debug_out)
            report_message(line, Message_Plain);
    } else
        *skip = skip_sub || (stack_idx >= 0 && stack[stack_idx].skip);

    return status;
}

#endif // NGC_EXPRESSIONS_ENABLE
