/* ngc_expr.h */

#ifndef _NGC_EXPR_H_
#define _NGC_EXPR_H_

status_code_t ngc_read_name (char *line, uint_fast8_t *pos, char *buffer);
status_code_t ngc_read_real_value (char *line, uint_fast8_t *pos, float *value);
status_code_t ngc_read_integer_value(char *line, uint_fast8_t *pos, int32_t *value);
status_code_t ngc_read_integer_unsigned (char *line, uint_fast8_t *pos, uint32_t *value);
status_code_t ngc_read_parameter (char *line, uint_fast8_t *pos, float *value, bool check);
status_code_t ngc_eval_expression (char *line, uint_fast8_t *pos, float *value);
void ngc_expr_init(void);

#endif
