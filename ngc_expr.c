// ngc_expr.c - derived from:

/********************************************************************
* Description: interp_execute.cc
*
*   Derived from a work by Thomas Kramer
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

/* Modified by Terje Io for grblHAL */


#include "nuts_bolts.h"

#if NGC_EXPRESSIONS_ENABLE

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "errors.h"
#include "ngc_expr.h"
#include "ngc_params.h"

#define MAX_STACK 7

typedef enum {
    NGCBinaryOp_NoOp = 0,
    NGCBinaryOp_DividedBy,
    NGCBinaryOp_Modulo,
    NGCBinaryOp_Power,
    NGCBinaryOp_Times,
    NGCBinaryOp_Binary2 = NGCBinaryOp_Times,
    NGCBinaryOp_And2,
    NGCBinaryOp_ExclusiveOR,
    NGCBinaryOp_Minus,
    NGCBinaryOp_NotExclusiveOR,
    NGCBinaryOp_Plus,
    NGCBinaryOp_RightBracket,
    NGCBinaryOp_RelationalFirst,
    NGCBinaryOp_LT = NGCBinaryOp_RelationalFirst,
    NGCBinaryOp_EQ,
    NGCBinaryOp_NE,
    NGCBinaryOp_LE,
    NGCBinaryOp_GE,
    NGCBinaryOp_GT,
    NGCBinaryOp_RelationalLast = NGCBinaryOp_GT,
} ngc_binary_op_t;

typedef enum {
    NGCUnaryOp_ABS = 1,
    NGCUnaryOp_ACOS,
    NGCUnaryOp_ASIN,
    NGCUnaryOp_ATAN,
    NGCUnaryOp_COS,
    NGCUnaryOp_EXP,
    NGCUnaryOp_FIX,
    NGCUnaryOp_FUP,
    NGCUnaryOp_LN,
    NGCUnaryOp_Round,
    NGCUnaryOp_SIN,
    NGCUnaryOp_SQRT,
    NGCUnaryOp_TAN,
    NGCUnaryOp_Exists,  // Not implemented
} ngc_unary_op_t;

/*! \brief Executes the operations: /, MOD, ** (POW), *.

\param lhs pointer to the left hand side operand and result.
\param operation \ref ngc_binary_op_t enum value.
\param rhs pointer to the right hand side operand.
\returns #Status_OK enum value if processed without error, appropriate \ref status_code_t enum value if not.
*/
static status_code_t execute_binary1 (float *lhs, ngc_binary_op_t operation, float *rhs)
{
    status_code_t status = Status_OK;

    switch (operation) {

        case NGCBinaryOp_DividedBy:
            if(*rhs == 0.0f || *rhs == -0.0f)
                status = Status_ExpressionDivideByZero; // Attempt to divide by zero
            else
                *lhs = *lhs / *rhs;
            break;

        case NGCBinaryOp_Modulo: // always calculates a positive answer
            *lhs = fmodf(*lhs, *rhs);
            if(*lhs < 0.0f)
                *lhs = *lhs + fabsf(*rhs);
            break;

        case NGCBinaryOp_Power:
            if(*lhs < 0.0f && floorf(*rhs) != *rhs)
                status = Status_ExpressionInvalidArgument; // Attempt to raise negative value to non-integer power
            else
                *lhs = powf(*lhs, *rhs);
            break;

        case NGCBinaryOp_Times:
            *lhs = *lhs * *rhs;
            break;

        default:
            status = Status_ExpressionUknownOp;
    }

    return status;
}

/*! \brief Executes the operations: +, -, AND, OR, XOR, EQ, NE, LT, LE, GT, GE
The RS274/NGC manual does not say what
the calculated value of the logical operations should be. This
function calculates either 1.0 (meaning true) or 0.0 (meaning false).
Any non-zero input value is taken as meaning true, and only 0.0 means false.

\param lhs pointer to the left hand side operand and result.
\param operation \ref ngc_binary_op_t enum value.
\param rhs pointer to the right hand side operand.
\returns #Status_OK enum value if processed without error, appropriate \ref status_code_t enum value if not.
*/
static status_code_t execute_binary2 (float *lhs, ngc_binary_op_t operation, float *rhs)
{
    switch(operation) {

        case NGCBinaryOp_And2:
            *lhs = ((*lhs == 0.0f) || (*rhs == 0.0f)) ? 0.0f : 1.0f;
            break;

        case NGCBinaryOp_ExclusiveOR:
            *lhs = (((*lhs == 0.0f) && (*rhs != 0.0f)) || ((*lhs != 0.0f) && (*rhs == 0.0f))) ? 1.0f : 0.0f;
            break;

        case NGCBinaryOp_Minus:
            *lhs = (*lhs - *rhs);
            break;

        case NGCBinaryOp_NotExclusiveOR:
            *lhs = ((*lhs != 0.0f) || (*rhs != 0.0f)) ? 1.0f : 0.0f;
            break;

        case NGCBinaryOp_Plus:
            *lhs = (*lhs + *rhs);
            break;

        case NGCBinaryOp_LT:
            *lhs = (*lhs < *rhs) ? 1.0f : 0.0f;
            break;

        case NGCBinaryOp_EQ:
            {
                float diff = *lhs - *rhs;
                diff = (diff < 0.0f) ? -diff : diff;
                *lhs = (diff < TOLERANCE_EQUAL) ? 1.0f : 0.0f;
            }
            break;

        case NGCBinaryOp_NE:
            {
                float diff = *lhs - *rhs;
                diff = (diff < 0.0f) ? -diff : diff;
                *lhs = (diff >= TOLERANCE_EQUAL) ? 1.0f : 0.0f;
            }
            break;

        case NGCBinaryOp_LE:
            *lhs = (*lhs <= *rhs) ? 1.0f : 0.0f;
            break;

        case NGCBinaryOp_GE:
            *lhs = (*lhs >= *rhs) ? 1.0f : 0.0f;
            break;

        case NGCBinaryOp_GT:
            *lhs = (*lhs > *rhs) ? 1.0f : 0.0f;
            break;

        default:
            return Status_ExpressionUknownOp;
    }

    return Status_OK;
}

/*! \brief Executes a binary operation.

This just calls either execute_binary1 or execute_binary2.

\param lhs pointer to the left hand side operand and result.
\param operation \ref ngc_binary_op_t enum value.
\param rhs pointer to the right hand side operand.
\returns #Status_OK enum value if processed without error, appropriate \ref status_code_t enum value if not.
*/
static status_code_t execute_binary (float *lhs, ngc_binary_op_t operation, float *rhs)
{
    if (operation <= NGCBinaryOp_Binary2)
        return execute_binary1(lhs, operation, rhs);

    return execute_binary2(lhs, operation, rhs);
}

/*! \brief Executes an unary operation: ABS, ACOS, ASIN, COS, EXP, FIX, FUP, LN, ROUND, SIN, SQRT, TAN

All angle measures in the input or output are in degrees.

\param operand pointer to the operand.
\param operation \ref ngc_binary_op_t enum value.
\returns #Status_OK enum value if processed without error, appropriate \ref status_code_t enum value if not.
*/
static status_code_t execute_unary (float *operand, ngc_unary_op_t operation)
{
    status_code_t status = Status_OK;

    switch (operation) {

        case NGCUnaryOp_ABS:
            if (*operand < 0.0f)
                *operand = (-1.0f * *operand);
            break;

        case NGCUnaryOp_ACOS:
            if(*operand < -1.0f || *operand > 1.0f)
                status = Status_ExpressionArgumentOutOfRange; // Argument to ACOS out of range
            else
                *operand = acosf(*operand) * DEGRAD;
            break;

        case NGCUnaryOp_ASIN:
            if(*operand < -1.0f || *operand > 1.0f)
                status = Status_ExpressionArgumentOutOfRange; // Argument to ASIN out of range
            else
                *operand = asinf(*operand) * DEGRAD;
            break;

        case NGCUnaryOp_COS:
            *operand = cosf(*operand * RADDEG);
            break;

        case NGCUnaryOp_Exists:
            // do nothing here, result for the EXISTS function is set by read_unary()
            break;

        case NGCUnaryOp_EXP:
            *operand = expf(*operand);
            break;

        case NGCUnaryOp_FIX:
            *operand = floor(*operand);
            break;

        case NGCUnaryOp_FUP:
            *operand = ceilf(*operand);
            break;

        case NGCUnaryOp_LN:
            if(*operand <= 0.0f)
                status = Status_ExpressionArgumentOutOfRange; // Argument to LN out of range
            else
                *operand = logf(*operand);
            break;

        case NGCUnaryOp_Round:
            *operand = (float)((int)(*operand + ((*operand < 0.0f) ? -0.5f : 0.5f)));
            break;

        case NGCUnaryOp_SIN:
            *operand = sinf(*operand * RADDEG);
            break;

        case NGCUnaryOp_SQRT:
            if(*operand < 0.0f)
                status = Status_ExpressionArgumentOutOfRange; // Negative argument to SQRT
            else
                *operand = sqrtf(*operand);
            break;

        case NGCUnaryOp_TAN:
            *operand = tanf(*operand *RADDEG);
            break;

        default:
            status = Status_ExpressionUknownOp;
    }

    return status;
}

/*! \brief Returns an integer representing the precedence level of an operator.

\param operator \ref ngc_binary_op_t enum value.
\returns precedence level.
*/
static uint_fast8_t precedence (ngc_binary_op_t operator)
{
    switch(operator)
    {
        case NGCBinaryOp_RightBracket:
            return 1;

        case NGCBinaryOp_And2:
            case NGCBinaryOp_ExclusiveOR:
            case NGCBinaryOp_NotExclusiveOR:
            return 2;

        case NGCBinaryOp_LT:
        case NGCBinaryOp_EQ:
        case NGCBinaryOp_NE:
        case NGCBinaryOp_LE:
        case NGCBinaryOp_GE:
        case NGCBinaryOp_GT:
            return 3;

        case NGCBinaryOp_Minus:
        case NGCBinaryOp_Plus:
            return 4;

        case NGCBinaryOp_NoOp:
        case NGCBinaryOp_DividedBy:
        case NGCBinaryOp_Modulo:
        case NGCBinaryOp_Times:
            return 5;

        case NGCBinaryOp_Power:
            return 6;

        default:
            break;
    }

    return 0;   // should never happen
}

/*! \brief Reads a binary operation out of the line
starting at the index given by the pos offset. If a valid one is found, the
value of operation is set to the symbolic value for that operation.

\param line pointer to RS274/NGC code (block).
\param pos offset into line where expression starts.
\param operation pointer to \ref ngc_binary_op_t enum value.
\returns #Status_OK enum value if processed without error, appropriate \ref status_code_t enum value if not.
*/
static status_code_t read_operation (char *line, uint_fast8_t *pos, ngc_binary_op_t *operation)
{
    char c = line[*pos];
    status_code_t status = Status_OK;

    (*pos)++;

    switch(c) {

        case '+':
            *operation = NGCBinaryOp_Plus;
            break;

        case '-':
            *operation = NGCBinaryOp_Minus;
            break;

        case '/':
            *operation = NGCBinaryOp_DividedBy;
            break;

        case '*':
            if(line[*pos] == '*') {
                *operation = NGCBinaryOp_Power;
                (*pos)++;
            } else
                *operation = NGCBinaryOp_Times;
            break;

        case ']':
            *operation = NGCBinaryOp_RightBracket;
            break;

        case 'A':
            if (!strncmp(line + *pos, "ND", 2)) {
                *operation = NGCBinaryOp_And2;
                *pos += 2;
            } else
                status = Status_ExpressionUknownOp; // Unknown operation name starting with A
        break;

        case 'M':
            if (!strncmp(line + *pos, "OD", 2)) {
                *operation = NGCBinaryOp_Modulo;
                *pos += 2;
            } else
                status = Status_ExpressionUknownOp; // Unknown operation name starting with M
            break;

        case 'R':
            if (line[*pos] == 'R') {
                *operation = NGCBinaryOp_NotExclusiveOR;
                (*pos)++;
            } else
                status = Status_ExpressionUknownOp; // Unknown operation name starting with R
            break;

        case 'X':
            if (!strncmp(line + *pos, "OR", 2)) {
                *operation = NGCBinaryOp_ExclusiveOR;
                *pos += 2;
            } else
                status = Status_ExpressionUknownOp; // Unknown operation name starting with X
        break;

        /* relational operators */
        case 'E':
            if(line[*pos] == 'Q') {
                *operation = NGCBinaryOp_EQ;
                (*pos)++;
            } else
                status = Status_ExpressionUknownOp; // Unknown operation name starting with E
            break;

        case 'N':
            if(line[*pos] == 'E') {
                *operation = NGCBinaryOp_NE;
                (*pos)++;
            } else
                status = Status_ExpressionUknownOp; // Unknown operation name starting with N
            break;

        case 'G':
            if(line[*pos] == 'E') {
                *operation = NGCBinaryOp_GE;
                (*pos)++;
            }
            else if(line[*pos] == 'T') {
                *operation = NGCBinaryOp_GT;
                (*pos)++;
            } else
                status = Status_ExpressionUknownOp; // Unknown operation name starting with G
            break;

        case 'L':
            if(line[*pos] == 'E') {
                *operation = NGCBinaryOp_LE;
                (*pos)++;
            } else if(line[*pos] == 'T') {
                *operation = NGCBinaryOp_LT;
                (*pos)++;
            }
            else
                status = Status_ExpressionUknownOp;; // Unknown operation name starting with L
            break;

//        case '\0':
//            status = Status_ExpressionUknownOp; // No operation name found

        default:
            status = Status_ExpressionUknownOp; // Unknown operation name
    }

    return status;
}

/*! \brief Reads the name of an unary operation out of the line
starting at the index given by the pos offset. If a valid one is found, the
value of operation is set to the symbolic value for that operation.

\param line pointer to RS274/NGC code (block).
\param pos offset into line where expression starts.
\param operation pointer to \ref ngc_unary_op_t enum value.
\returns #Status_OK enum value if processed without error, appropriate \ref status_code_t enum value if not.
*/
static status_code_t read_operation_unary (char *line, uint_fast8_t *pos, ngc_unary_op_t *operation)
{
    char c = line[*pos];
    status_code_t status = Status_OK;

    (*pos)++;

    switch(c) {

        case 'A':
            if(!strncmp(line + *pos, "BS", 2)) {
                *operation = NGCUnaryOp_ABS;
                *pos += 2;
            } else if(!strncmp(line + *pos, "COS", 3)) {
                *operation = NGCUnaryOp_ACOS;
                *pos += 3;
            } else if(!strncmp(line + *pos, "SIN", 3)) {
                *operation = NGCUnaryOp_ASIN;
                *pos += 3;
            } else if(!strncmp(line + *pos, "TAN", 3)) {
                *operation = NGCUnaryOp_ATAN;
                *pos += 3;
            } else
                status = Status_ExpressionUknownOp;
            break;

        case 'C':
            if(!strncmp(line + *pos, "OS", 2)) {
                *operation = NGCUnaryOp_COS;
                *pos += 2;
            } else
                status = Status_ExpressionUknownOp;
            break;

        case 'E':
            if(!strncmp(line + *pos, "XP", 2)) {
                *operation = NGCUnaryOp_EXP;
                *pos += 2;
/*            }  else if(!strncmp(line + *pos, "XISTS", 5)) {
                *operation = NGCUnaryOp_Exists;
                *pos += 5; */
            } else
                status = Status_ExpressionUknownOp;
            break;

        case 'F':
            if(!strncmp(line + *pos, "IX", 2)) {
                *operation = NGCUnaryOp_FIX;
                *pos += 2;
            } else if(!strncmp(line + *pos, "UP", 2)) {
                *operation = NGCUnaryOp_FUP;
                *pos += 2;
            } else
                status = Status_ExpressionUknownOp;
            break;

        case 'L':
            if(line[*pos] == 'N') {
                *operation = NGCUnaryOp_LN;
                (*pos)++;
            } else
                status = Status_ExpressionUknownOp;
            break;

        case 'R':
            if (!strncmp(line + *pos, "OUND", 4)) {
                *operation = NGCUnaryOp_Round;
                *pos += 4;
            } else
                status = Status_ExpressionUknownOp;
            break;

        case 'S':
            if(!strncmp(line + *pos, "IN", 2)) {
                *operation = NGCUnaryOp_SIN;
                *pos += 2;
            } else if(!strncmp((line + *pos), "QRT", 3)) {
                *operation = NGCUnaryOp_SQRT;
                *pos += 3;
            } else
                status = Status_ExpressionUknownOp;
            break;

        case 'T':
            if(!strncmp(line + *pos, "AN", 2)) {
                *operation = NGCUnaryOp_TAN;
                *pos += 2;
            } else
                status = Status_ExpressionUknownOp;
            break;

        default:
            status = Status_ExpressionUknownOp;
    }

    return status;
}

/*! \brief Reads the value out of a parameter of the line, starting at the
index given by the pos offset.

According to the RS274/NGC manual [NCMS, p. 62], the characters following
# may be any "parameter expression". Thus, the following are legal
and mean the same thing (the value of the parameter whose number is
stored in parameter 2):
  ##2
  #[#2]

Parameter setting is done in parallel, not sequentially. For example
if #1 is 5 before the line "#1=10 #2=#1" is read, then after the line
is is executed, #1 is 10 and #2 is 5. If parameter setting were done
sequentially, the value of #2 would be 10 after the line was executed.

\param line pointer to RS274/NGC code (block).
\param pos offset into line where expression starts.
\param value pointer to float where result is to be stored.
\returns #Status_OK enum value if processed without error, appropriate \ref status_code_t enum value if not.
*/
static status_code_t read_parameter (char *line, uint_fast8_t *pos, float *value, bool check)
{
    status_code_t status = Status_BadNumberFormat;

    if(*(line + *pos) == '#') {

        (*pos)++;

        if(*(line + *pos) == '<') {

            (*pos)++;
            char *param = line + *pos, *arg = line + *pos;

            while(*arg && *arg != '>')
                arg++;

            *pos += arg - param + 1;

            if(*arg == '>') {
                *arg = '\0';
                if(ngc_named_param_get(param, value))
                    status = Status_OK;
                *arg = '>';
            }

        } else if(read_float(line, pos, value)) {
            if(ngc_param_get((ngc_param_id_t)*value, value))
                status = Status_OK;
        }
    }

    return status;
}

/*! \brief Reads a slash and the second argument to the ATAN function,
starting at the index given by the pos offset. Then it computes the value
of the ATAN operation applied to the two arguments.

\param line pointer to RS274/NGC code (block).
\param pos offset into line where expression starts.
\param value pointer to float where result is to be stored.
\returns #Status_OK enum value if processed without error, appropriate \ref status_code_t enum value if not.
*/
static status_code_t read_atan (char *line, uint_fast8_t *pos, float *value)
{
    float argument2;

    if(line[*pos] != '/')
        return Status_ExpressionSyntaxError; // Slash missing after first ATAN argument

    (*pos)++;

    if(line[*pos] != '[')
        return Status_ExpressionSyntaxError; // Left bracket missing after slash with ATAN;

    status_code_t status;

    if((status = ngc_eval_expression(line, pos, &argument2)) == Status_OK)
        *value = atan2f(*value, argument2) * DEGRAD;  /* value in radians, convert to degrees */

    return status;
}

/*! \brief Reads the value out of an unary operation of the line, starting at the
index given by the pos offset. The ATAN operation is
handled specially because it is followed by two arguments.

\param line pointer to RS274/NGC code (block).
\param pos offset into line where expression starts.
\param value pointer to float where result is to be stored.
\returns #Status_OK enum value if processed without error, appropriate \ref status_code_t enum value if not.
*/
static status_code_t read_unary (char *line, uint_fast8_t *pos, float *value)
{
    ngc_unary_op_t operation;
    status_code_t status;

    if((status = read_operation_unary(line, pos, &operation)) == Status_OK) {

        if(line[*pos] != '[')
            status = Status_ExpressionSyntaxError; // Left bracket missing after unary operation name
        else {

            /*
            if (operation == NGCUnaryOp_Exists) {
                CHP(read_bracketed_parameter(line, pos, value, true));
                return Status_OK;
            }
        */
            if((status = ngc_eval_expression(line, pos, value)) == Status_OK) {
                if(operation == NGCUnaryOp_ATAN)
                    status = read_atan(line, pos, value);
                else
                    status = execute_unary(value, operation);
            }
        }
    }

    return status;
}

/*! \brief Reads a real value out of the line, starting at the
index given by the pos offset. The value may be a number, a parameter
value, a unary function, or an expression. It calls one of four
other readers, depending upon the first character.

\param line pointer to RS274/NGC code (block).
\param pos offset into line where expression starts.
\param value pointer to float where result is to be stored.
\returns #Status_OK enum value if processed without error, appropriate \ref status_code_t enum value if not.
*/
static status_code_t read_real_value (char *line, uint_fast8_t *pos, float *value)
{
    char c = line[*pos], c1;

    if(c == '\0')
        return Status_ExpressionSyntaxError; // No characters found when reading real value

    status_code_t status;

    c1 = line[*pos + 1];

    if(c == '[')
        status = ngc_eval_expression(line, pos, value);
    else if(c == '#')
        status = read_parameter(line, pos, value, false);
    else if(c == '+' && c1 && !isdigit(c1) && c1 != '.') {
        (*pos)++;
        status = read_real_value(line, pos, value);
    } else if(c == '-' && c1 && !isdigit(c1) && c1 != '.') {
        (*pos)++;
        status = read_real_value(line, pos, value);
        *value = -*value;
    } else if ((c >= 'A') && (c <= 'Z'))
        status = read_unary(line, pos, value);
    else
        status = (read_float(line, pos, value) ? Status_OK : Status_BadNumberFormat);

    if(isnanf(*value))
        status = Status_ExpressionInvalidResult; // Calculation resulted in 'not a number'
    else if(isinff(*value))
        status = Status_ExpressionInvalidResult; // Calculation resulted in 'not a number'

    return status;
}

/*! \brief Evaluate expression and set result if successful.

\param line pointer to RS274/NGC code (block).
\param pos offset into line where expression starts.
\param value pointer to float where result is to be stored.
\returns #Status_OK enum value if evaluated without error, appropriate \ref status_code_t enum value if not.
*/
status_code_t ngc_eval_expression (char *line, uint_fast8_t *pos, float *value)
{
    float values[MAX_STACK];
    ngc_binary_op_t operators[MAX_STACK];
    uint_fast8_t stack_index = 1;

    if(line[*pos] != '[')
        return Status_GcodeUnsupportedCommand;

    (*pos)++;

    status_code_t status;

    if((status = read_real_value(line, pos, values)) != Status_OK)
        return status;

    if((status = read_operation(line, pos, operators)) != Status_OK)
        return status;

    for(; operators[0] != NGCBinaryOp_RightBracket;) {

        if((status = read_real_value(line, pos, values + stack_index)) != Status_OK)
            return status;

        if((status = read_operation(line, pos, operators + stack_index)) != Status_OK)
            return status;

        if (precedence(operators[stack_index]) > precedence(operators[stack_index - 1]))
            stack_index++;
        else { // precedence of latest operator is <= previous precedence
            for(; precedence(operators[stack_index]) <= precedence(operators[stack_index - 1]);) {

                if((status = execute_binary(values + stack_index - 1, operators[stack_index - 1], values + stack_index)) != Status_OK)
                    return status;

                operators[stack_index - 1] = operators[stack_index];
                if((stack_index > 1) && precedence(operators[stack_index - 1] <= precedence(operators[stack_index - 2])))
                    stack_index--;
                else
                    break;
            }
        }
    }

    *value = values[0];

    return Status_OK;
}

#endif
