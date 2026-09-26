static void check (bool condition, const char *message)
{
    if(!condition) {
        if(failures < 12)
            printf("FAIL: %s\n", message);
        failures++;
    }
}

static void configure (st2_motor_t *motor, float spm, float acceleration, bool polling)
{
    memset(motor, 0, sizeof(*motor));
    settings.axis[0] = (axis_settings_t){spm, acceleration * 3600.0f, 6000.0f};
    motor->axis.bits = 4;
    motor->executor.polling = polling;
    motor->executor.step_inject_timer = polling ? NULL : (hal_timer_t)1;
    motor->on_stopped = stopped;
    st2_motor_config(motor, &settings.axis[0]);
    output_calls = callback_calls = timer_stops = 0;
    output_position = 0;
    clock_us = 0;
}

// Fingerprint the entire serviced trace, including the terminal no-output
// transition, for comparison with a fixed pre-refactoring source via --source.
static uint64_t trace_hash = 14695981039346656037ULL;

static void trace_value (uint64_t value)
{
    trace_hash = (trace_hash ^ value) * 1099511628211ULL;
}

static void tick (st2_motor_t *motor)
{
    clock_us += motor->profile.delay;
    if(motor->executor.polling)
        st2_motor_run(motor);
    else
        motor_irq(motor);

    trace_value(clock_us);
    trace_value(motor->profile.state);
    trace_value(motor->profile.delay);
    trace_value(motor->profile.c64);
    trace_value(motor->profile.denom);
    trace_value(motor->profile.step_no);
    trace_value(motor->profile.step_down);
    trace_value(output_calls);
    trace_value(st2_get_position(motor));
    trace_value(callback_calls);
    trace_value(timer_stops);
}

static void finish (st2_motor_t *motor)
{
    unsigned limit = 100000;
    while(st2_motor_running(motor) && --limit)
        tick(motor);
    check(limit != 0, "motion must terminate");
}

static void finite (unsigned count, float spm, float acceleration, float rate, int sign, bool polling, position_t type)
{
    st2_motor_t motor;
    configure(&motor, spm, acceleration, polling);
    float distance = sign * (type == Stepper2_mm ? count / spm : (float)count);
    check(st2_motor_move(&motor, distance, rate, type), "finite move accepted");
    uint64_t previous_interval = 0;
    bool decelerating = false;
    unsigned limit = 100000;
    while(st2_motor_running(&motor) && --limit) {
        uint64_t interval = motor.profile.delay;
        unsigned before = output_calls;
        tick(&motor);
        if(output_calls != before) {
            if(previous_interval && interval > previous_interval)
                decelerating = true;
            if(decelerating)
                check(interval >= previous_interval, "finite ramp does not accelerate again while braking");
            previous_interval = interval;
        }
    }
    check(limit != 0, "finite motion must terminate");
    if(output_calls != count && failures < 12)
        printf("count=%u spm=%.0f a=%.0f rate=%.0f sign=%d poll=%d type=%d got=%u\n",
               count, spm, acceleration, rate, sign, polling, type, output_calls);
    check(output_calls == count, "exact requested output count");
    check(output_position == sign * (int64_t)count, "signed output position");
    check(st2_get_position(&motor) == output_position, "position equals output calls");
    check(callback_calls == ((count == 1 && type == Stepper2_Steps) ? 0u : 1u), "one completion for profiled move");
    check(st2_get_speed(&motor) == 0, "idle speed");
    unsigned before = output_calls;
    motor_irq(&motor); // A late timer callback must not emit a step from Idle.
    check(output_calls == before, "idle callback has no output");
    check(st2_set_position(&motor, 123), "idle position may be reset");
    tests++;
}

static void stop_and_speed (bool infinite, unsigned stop_after, bool polling)
{
    st2_motor_t motor;
    configure(&motor, 400, 100, polling);
    check(st2_motor_move(&motor, infinite ? 1 : 1000, 600,
                        infinite ? Stepper2_InfiniteSteps : Stepper2_Steps), "control move accepted");
    for(unsigned i = 0; i < stop_after; i++)
        tick(&motor);
    check(st2_motor_running(&motor), "moving before stop");
    check(!st2_set_position(&motor, 123), "running position cannot be reset");
    if(infinite && stop_after > 500) {
        st2_motor_set_speed(&motor, 900);
        for(unsigned i = 0; i < 1000; i++) tick(&motor);
        st2_motor_set_speed(&motor, 300);
        for(unsigned i = 0; i < 1000; i++) tick(&motor);
        check(st2_motor_running(&motor), "infinite survives speed changes");
    }
    check(st2_motor_stop(&motor), "stop accepted");
    finish(&motor);
    check(callback_calls == 1, "stop completes once");
    check(output_position == st2_get_position(&motor), "stop position matches output");
    tests++;
}

static void polling_and_reset (void)
{
    st2_motor_t motor;
    configure(&motor, 400, 150, true);
    check(st2_motor_move(&motor, 100, 200, Stepper2_Steps), "polling move accepted");
    clock_us = motor.profile.delay - 1;
    st2_motor_run(&motor);
    check(output_calls == 0, "polling does not emit early");
    clock_us += 100000;
    st2_motor_run(&motor);
    check(output_calls == 1, "late poll emits only one step");
    st2_motor_run(&motor);
    check(output_calls == 1, "polling rebases on serviced time");
    tests++;

    motors = &motor;
    st2_reset();
    check(motor.position_lost && !st2_motor_running(&motor), "active reset loses position and stops");
    motor_irq(&motor);
    check(output_calls == 1 && callback_calls == 0, "reset cannot complete or emit stale motion");
    tests++;
    check(st2_set_position(&motor, 123), "restore idle position");
    st2_reset();
    check(!motor.position_lost && st2_get_position(&motor) == 123, "idle reset retains known position");
    motors = NULL;
    tests++;
}

int main (void)
{
    const float spm[] = {100, 400, 800};
    const float acceleration[] = {10, 100, 150};
    const float rates[] = {50, 160, 200, 600, 1200};
    for(unsigned s = 0; s < 3; s++)
        for(unsigned a = 0; a < 3; a++)
            for(unsigned r = 0; r < 5; r++)
                for(unsigned n = 1; n <= 128; n++)
                    for(int direction = -1; direction <= 1; direction += 2)
                        for(unsigned polling = 0; polling < 2; polling++)
                            for(unsigned type = 0; type < 2; type++)
                                finite(n, spm[s], acceleration[a], rates[r], direction, polling,
                                       type ? Stepper2_mm : Stepper2_Steps);
    for(unsigned n = 255; n <= 4095; n = n * 2 + 1)
        for(unsigned r = 0; r < 5; r++)
            finite(n, 400, 150, rates[r], 1, false, Stepper2_mm);
    for(unsigned polling = 0; polling < 2; polling++)
        for(unsigned infinite = 0; infinite < 2; infinite++) {
            stop_and_speed(infinite, 1, polling);
            stop_and_speed(infinite, 50, polling);
            stop_and_speed(infinite, 600, polling);
        }
    st2_motor_t motor;
    configure(&motor, 400, 100, true);
    check(!st2_motor_move(&motor, 0, 600, Stepper2_mm), "zero distance rejected");
    check(!st2_motor_move(&motor, 1, 0, Stepper2_mm), "zero speed rejected");
    check(output_calls == 0 && callback_calls == 0, "empty request emits nothing");
    tests++;
    configure(&motor, 400, 150, true);
    for(unsigned cut = 0; cut < 100; cut++) {
        check(st2_motor_move(&motor, 0.12f, 200, Stepper2_mm), "repeated correction accepted");
        finish(&motor);
        check(output_calls == (cut + 1) * 48, "repeated corrections have no excess steps");
        check(st2_get_position(&motor) == output_position, "repeated position follows output");
    }
    check(callback_calls == 100, "one completion per repeated correction");
    tests++;
    polling_and_reset();
    printf("%u scenarios, %u failures\n", tests, failures);
    printf("Serviced trace: %016llx\n", (unsigned long long)trace_hash);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
