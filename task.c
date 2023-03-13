#include "core_handlers.h"
/*
typedef struct {
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    on_execute_realtime_ptr fn[RT_QUEUE_SIZE];
} realtime_queue_t;

static realtime_queue_t realtime_queue = {0};

// Enqueue a function to be called once by the
// foreground process, typically enqueued from an interrupt handler.
ISR_CODE bool ISR_FUNC(grbl_task_enqueue)(on_execute_realtime_ptr fn)
{
    bool ok;
    uint_fast8_t bptr = (realtime_queue.head + 1) & (RT_QUEUE_SIZE - 1);    // Get next head pointer

    if((ok = bptr != realtime_queue.tail)) {          // If not buffer full
        realtime_queue.fn[realtime_queue.head] = fn;  // add function pointer to buffer,
        realtime_queue.head = bptr;                   // update pointer and
        system_set_exec_state_flag(EXEC_RT_COMMAND);  // flag it for execute
    }

    return ok;
}

// Enqueue a function to be called once by the
// foreground process, typically enqueued from an interrupt handler.
ISR_CODE bool ISR_FUNC(grbl_task_enqueue_delayed)(on_execute_realtime_ptr fn, uint32_t ms_delay, void *context)
{
    bool ok;
    uint_fast8_t bptr = (realtime_queue.head + 1) & (RT_QUEUE_SIZE - 1);    // Get next head pointer

    if((ok = bptr != realtime_queue.tail)) {          // If not buffer full
        realtime_queue.fn[realtime_queue.head] = fn;  // add function pointer to buffer,
        realtime_queue.head = bptr;                   // update pointer and
        system_set_exec_state_flag(EXEC_RT_COMMAND);  // flag it for execute
    }

    return ok;
}

// Execute enqueued functions.
static void grbl_task_execute (void)
{
    while(realtime_queue.tail != realtime_queue.head) {
        uint_fast8_t bptr = realtime_queue.tail;
        on_execute_realtime_ptr call;
        if((call = realtime_queue.fn[bptr])) {
            realtime_queue.fn[bptr] = NULL;
            call(state_get());
        }
        realtime_queue.tail = (bptr + 1) & (RT_QUEUE_SIZE - 1);
    }

    if(!sys.driver_started)
        while(true);
}
*/
