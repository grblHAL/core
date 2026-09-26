/* Motor stream injection contract. Part of grblHAL, GPLv3 or later. */
#ifndef _STEPPER_INJECTION_H_
#define _STEPPER_INJECTION_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    Injection_Progress = 0,
    Injection_Completed,
    Injection_Cancelled,
    Injection_Fault
} injection_result_t;

/* Delay is relative to the preceding event, in microseconds. A terminal
 * event may have no step (the profile's final no-output transition). */
typedef struct {
    uint32_t delay_us;
    bool step;
    bool last;
} injection_event_t;

typedef struct {
    uint32_t id;
    uint32_t completed_steps;  // cumulative, full pulses only
    injection_result_t result;
} injection_progress_t;

typedef struct {
    uint32_t id;
    uint32_t axis_mask;
    uint32_t direction_mask;
    uint32_t requested_steps;
    void *context;
    /* Called only after reservation of the event slot, under the driver lock.
     * Must not block, allocate, call HAL or invoke client callbacks. */
    injection_event_t (*next)(void *context);
    /* Driver worker context, outside ISR and driver locks. Context must remain
     * alive through cancellation; receiver must validate the motion ID. */
    void (*notify)(void *context, const injection_progress_t *progress);
} injection_motion_t;

typedef struct {
    bool (*supports)(uint32_t axis_mask);
    /* Caller holds lock. False means no generator call and no accepted motion. */
    bool (*submit)(const injection_motion_t *motion);
    /* Caller holds lock. Invalidates pending output, including on reset. */
    void (*cancel)(uint32_t id);
    /* Short, recursive, ISR-safe cross-core critical section. Never wait for
     * completion or invoke a client callback while holding it. */
    void (*lock)(void);
    void (*unlock)(void);
} stepper_injection_t;

#endif
