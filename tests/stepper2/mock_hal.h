#ifndef STEPPER2_TEST_MOCK_HAL_H
#define STEPPER2_TEST_MOCK_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(__TINYC__) && defined(_WIN32)
// The released Windows TinyCC math header uses an unsupported x87 constraint.
static float test_fabsf (float value) { return value < 0.0f ? -value : value; }
static float test_sqrtf (float value) { return (float)sqrt((double)value); }
#define fabsf test_fabsf
#define sqrtf test_sqrtf
#endif

#define FLASHMEM
#define ISR_CODE
#define ISR_FUNC(name) name
#define ST2_DEBUG 0

typedef union { uint32_t bits; uint32_t mask; } axes_signals_t;
typedef enum { Stepper2_Steps, Stepper2_InfiniteSteps, Stepper2_mm } position_t;
typedef void *hal_timer_t;
typedef void (*foreground_task_ptr)(void *);
typedef struct st2_motor st2_motor_t;
typedef struct { float steps_per_mm, acceleration, max_rate; } axis_settings_t;

static struct { axis_settings_t axis[3]; } settings;
static uint64_t clock_us;
static uint32_t output_calls, callback_calls, timer_stops;
static int64_t output_position;
static uint32_t failures, tests;

static void output_step (axes_signals_t axis, axes_signals_t dir)
{
    if(axis.bits != 4)
        failures++;
    output_calls++;
    output_position += dir.bits ? -1 : 1;
}

static uint64_t get_micros (void) { return clock_us; }
static void timer_start (hal_timer_t timer, uint64_t delay) { (void)timer; (void)delay; }
static void timer_stop (hal_timer_t timer) { (void)timer; timer_stops++; }
static void stopped (void *context) { (void)context; }

static bool task_add_delayed (foreground_task_ptr callback, void *context, uint32_t delay)
{
    (void)context;
    if(callback != stopped || delay != 2)
        failures++;
    callback_calls++;
    return true;
}

static struct {
    struct { void (*output_step)(axes_signals_t, axes_signals_t); } stepper;
    struct { void (*start)(hal_timer_t, uint64_t); void (*stop)(hal_timer_t); } timer;
    uint64_t (*get_micros)(void);
} hal = {{ output_step }, { timer_start, timer_stop }, get_micros};

bool st2_motor_stop (st2_motor_t *motor);
#endif
