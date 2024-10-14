/* ngc_expr.h */

#ifndef _NGC_EXPR_H_
#define _NGC_EXPR_H_

status_code_t ngc_eval_expression (char *line, uint_fast8_t *pos, float *value);
status_code_t ngc_read_real_value (char *line, uint_fast8_t *pos, float *value);
status_code_t ngc_read_parameter (char *line, uint_fast8_t *pos, float *value, bool check);

#endif
