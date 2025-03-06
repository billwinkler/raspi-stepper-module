#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "../include/delta_robot_config.h"
#include "../include/delta_robot.h"
#include "../include/stepper_control.h"

#define MIN_PHASE_PULSES 10
#define NS_PER_SEC 1000000000LL
#define TIME_SCALE_PRECISION 1000 // Fixed-point precision (3 decimal places)

struct stepper_motor motor_states[MOTOR_COUNT] = {
    { .id = 0, .gpio_step = CONFIG_MOTOR0_STEP_PIN, .gpio_dir = CONFIG_MOTOR0_DIR_PIN },
    { .id = 1, .gpio_step = CONFIG_MOTOR1_STEP_PIN, .gpio_dir = CONFIG_MOTOR1_DIR_PIN },
    { .id = 2, .gpio_step = CONFIG_MOTOR2_STEP_PIN, .gpio_dir = CONFIG_MOTOR2_DIR_PIN }
};

static ktime_t calculate_next_period(struct stepper_motor *state)
{
    unsigned int k = state->pulse_count;
    unsigned int total = state->total_pulses;
    unsigned int accel = state->accel_pulses;
    unsigned int decel = state->decel_pulses;
    long long min_period = NS_PER_SEC / CONFIG_MAX_FREQUENCY;  // 200,000 ns
    long long max_period = NS_PER_SEC / CONFIG_MIN_FREQUENCY;  // 1,000,000 ns
    long long period_accel = min_period;
    long long period_decel = min_period;
    long long period_k;
    unsigned int time_scale = state->time_scale; // Fixed-point scale factor

    if (total <= 1 || k >= total)
        return ktime_set(0, 0);

    if (k < accel && accel > 1) {
        long long delta = (max_period - min_period) * k / (accel - 1);
        period_accel = ((max_period - delta) * time_scale) / TIME_SCALE_PRECISION;
    }
    if (k >= total - decel && decel > 1) {
        unsigned int m = total - 1 - k;
        long long delta = (max_period - min_period) * (decel - 1 - m) / (decel - 1);
        period_decel = ((min_period + delta) * time_scale) / TIME_SCALE_PRECISION;
    }

    period_k = (period_accel > period_decel) ? period_accel : period_decel;
    if (period_k > (max_period * time_scale) / TIME_SCALE_PRECISION)
        period_k = (max_period * time_scale) / TIME_SCALE_PRECISION;

    return ktime_set(0, period_k);
}

static enum hrtimer_restart motor_timer_callback(struct hrtimer *timer)
{
    struct stepper_motor *state = container_of(timer, struct stepper_motor, timer);
    ktime_t next_period;

    if (state->pulse_count >= state->total_pulses || state->abort) {
        gpio_set_value(state->gpio_step, 0);
        printk(KERN_INFO "Motor %d: Motion stopped - pulse_count=%u, total=%u, abort=%d\n",
               state->id, state->pulse_count, state->total_pulses, state->abort);
        return HRTIMER_NORESTART;
    }

    gpio_set_value(state->gpio_step, 1);
    udelay(PULSE_WIDTH_US);
    gpio_set_value(state->gpio_step, 0);

    state->pulse_count++;
    next_period = calculate_next_period(state);

    if (next_period > 0) {
        ktime_t adjusted_delay = ktime_sub(next_period, ktime_set(0, PULSE_WIDTH_US * 1000));
        if (ktime_compare(adjusted_delay, ktime_set(0, 0)) < 0)
            adjusted_delay = ktime_set(0, 0);
        hrtimer_start(timer, adjusted_delay, HRTIMER_MODE_REL);
        return HRTIMER_RESTART;
    }

    return HRTIMER_NORESTART;
}

int stepper_init(void)
{
    int i, ret;

    for (i = 0; i < MOTOR_COUNT; i++) {
        ret = gpio_request(motor_states[i].gpio_step, "step");
        if (ret) {
            printk(KERN_ERR "stepper_control: Failed to request GPIO %d for step\n",
                   motor_states[i].gpio_step);
            goto cleanup;
        }
        gpio_direction_output(motor_states[i].gpio_step, 0);

        ret = gpio_request(motor_states[i].gpio_dir, "dir");
        if (ret) {
            printk(KERN_ERR "stepper_control: Failed to request GPIO %d for direction\n",
                   motor_states[i].gpio_dir);
            gpio_free(motor_states[i].gpio_step);
            goto cleanup;
        }
        gpio_direction_output(motor_states[i].gpio_dir, 0);

        hrtimer_init(&motor_states[i].timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        motor_states[i].timer.function = motor_timer_callback;
        motor_states[i].pulse_count = 0;
        motor_states[i].abort = false;
        motor_states[i].time_scale = TIME_SCALE_PRECISION; // 1.0 in fixed-point (1000)
    }

    printk(KERN_INFO "stepper_control: GPIOs and hrtimers initialized for all motors\n");
    return 0;

cleanup:
    for (i--; i >= 0; i--) {
        gpio_free(motor_states[i].gpio_step);
        gpio_free(motor_states[i].gpio_dir);
    }
    return ret;
}

void stepper_exit(void)
{
    int i;

    for (i = 0; i < MOTOR_COUNT; i++) {
        hrtimer_cancel(&motor_states[i].timer);
        gpio_free(motor_states[i].gpio_step);
        gpio_free(motor_states[i].gpio_dir);
    }
    printk(KERN_INFO "stepper_control: GPIOs freed and timers canceled\n");
}

void start_motor_motion(int motor_id, struct delta_robot_cmd *cmd)
{
    struct stepper_motor *motor;
    int limit_switch_pin;
    unsigned int default_accel;
    unsigned int default_decel;
    unsigned int total;

    if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
        printk(KERN_ERR "stepper_control: Invalid motor id %d\n", motor_id);
        return;
    }

    motor = &motor_states[motor_id];

    motor->direction = cmd->direction;
    motor->total_pulses = cmd->total_pulses;
    motor->pulse_count = 0;
    motor->abort = false;

    default_accel = CONFIG_ACCELERATION_PULSES;
    default_decel = CONFIG_DECELERATION_PULSES;
    total = motor->total_pulses;

    if (total <= 2 * MIN_PHASE_PULSES) {
        motor->accel_pulses = total / 2;
        motor->decel_pulses = total - motor->accel_pulses;
    } else if (total < default_accel + default_decel) {
        motor->accel_pulses = total / 2;
        motor->decel_pulses = total - motor->accel_pulses;
        if (motor->accel_pulses < MIN_PHASE_PULSES) {
            motor->accel_pulses = MIN_PHASE_PULSES;
            motor->decel_pulses = total - MIN_PHASE_PULSES;
        }
        if (motor->decel_pulses < MIN_PHASE_PULSES) {
            motor->decel_pulses = MIN_PHASE_PULSES;
            motor->accel_pulses = total - MIN_PHASE_PULSES;
        }
    } else {
        motor->accel_pulses = default_accel;
        motor->decel_pulses = default_decel;
    }

    switch (motor_id) {
        case 0: limit_switch_pin = CONFIG_LIMIT_SWITCH1_PIN; break;
        case 1: limit_switch_pin = CONFIG_LIMIT_SWITCH2_PIN; break;
        case 2: limit_switch_pin = CONFIG_LIMIT_SWITCH3_PIN; break;
        default: return;
    }

    if (motor->direction == 1 && gpio_get_value(limit_switch_pin) == 0) {
        printk(KERN_WARNING "Motor %d: Motion suppressed - limit switch already triggered\n", motor_id);
        return;
    }

    printk(KERN_INFO "stepper_control: Starting motor %d for %d pulses, accel=%d, decel=%d, direction %d\n",
           motor_id, motor->total_pulses, motor->accel_pulses, motor->decel_pulses, motor->direction);

    gpio_set_value(motor->gpio_dir, motor->direction);
    hrtimer_start(&motor->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
}

int get_motor_step_pin(int motor_id)
{
    if (motor_id < 0 || motor_id >= MOTOR_COUNT) return -1;
    return motor_states[motor_id].gpio_step;
}

static long long estimate_motion_duration(unsigned int total_pulses, unsigned int accel_pulses, unsigned int decel_pulses)
{
    long long min_period = NS_PER_SEC / CONFIG_MAX_FREQUENCY; // 200,000 ns
    long long max_period = NS_PER_SEC / CONFIG_MIN_FREQUENCY; // 1,000,000 ns
    long long duration = 0;
    unsigned int constant_pulses;
    unsigned int k;

    if (total_pulses <= 1) return 0;

    constant_pulses = total_pulses - accel_pulses - decel_pulses;
    if (constant_pulses < 0) constant_pulses = 0;

    if (accel_pulses > 1) {
        k = 0;
        for (; k < accel_pulses; k++) {
            long long delta = (max_period - min_period) * k / (accel_pulses - 1);
            duration += max_period - delta;
        }
    } else if (accel_pulses == 1) {
        duration += max_period;
    }

    duration += constant_pulses * min_period;

    if (decel_pulses > 1) {
        k = 0;
        for (; k < decel_pulses; k++) {
            unsigned int m = decel_pulses - 1 - k;
            long long delta = (max_period - min_period) * m / (decel_pulses - 1);
            duration += min_period + delta;
        }
    } else if (decel_pulses == 1) {
        duration += min_period;
    }

    return duration;
}

void start_synchronized_motion(struct delta_robot_cmd cmds[], int num_cmds)
{
    int i;
    struct hrtimer *timers[MOTOR_COUNT] = {NULL};
    long long max_duration = 0;
    long long current_duration;
    struct stepper_motor *motor;
    int limit_switch_pin;
    unsigned int default_accel;
    unsigned int default_decel;
    unsigned int total;
    unsigned int time_scale;

    for (i = 0; i < num_cmds; i++) {
        int motor_id = cmds[i].motor_id;
        if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
            printk(KERN_ERR "Invalid motor ID %d\n", motor_id);
            continue;
        }

        motor = &motor_states[motor_id];
        default_accel = CONFIG_ACCELERATION_PULSES;
        default_decel = CONFIG_DECELERATION_PULSES;
        total = cmds[i].total_pulses;

        if (total <= 2 * MIN_PHASE_PULSES) {
            motor->accel_pulses = total / 2;
            motor->decel_pulses = total - motor->accel_pulses;
        } else if (total < default_accel + default_decel) {
            motor->accel_pulses = total / 2;
            motor->decel_pulses = total - motor->accel_pulses;
            if (motor->accel_pulses < MIN_PHASE_PULSES) {
                motor->accel_pulses = MIN_PHASE_PULSES;
                motor->decel_pulses = total - MIN_PHASE_PULSES;
            }
            if (motor->decel_pulses < MIN_PHASE_PULSES) {
                motor->decel_pulses = MIN_PHASE_PULSES;
                motor->accel_pulses = total - MIN_PHASE_PULSES;
            }
        } else {
            motor->accel_pulses = default_accel;
            motor->decel_pulses = default_decel;
        }

        current_duration = estimate_motion_duration(total, motor->accel_pulses, motor->decel_pulses);
        if (current_duration > max_duration) max_duration = current_duration;
    }

    for (i = 0; i < num_cmds; i++) {
        int motor_id = cmds[i].motor_id;
        if (motor_id < 0 || motor_id >= MOTOR_COUNT) continue;

        motor = &motor_states[motor_id];

        switch (motor_id) {
            case 0: limit_switch_pin = CONFIG_LIMIT_SWITCH1_PIN; break;
            case 1: limit_switch_pin = CONFIG_LIMIT_SWITCH2_PIN; break;
            case 2: limit_switch_pin = CONFIG_LIMIT_SWITCH3_PIN; break;
            default: continue;
        }

        if (cmds[i].direction == 1 && gpio_get_value(limit_switch_pin) == 0) {
            printk(KERN_WARNING "Motor %d: Motion suppressed - limit switch triggered\n", motor_id);
            continue;
        }

        motor->direction = cmds[i].direction;
        motor->total_pulses = cmds[i].total_pulses;
        motor->pulse_count = 0;
        motor->abort = false;

        current_duration = estimate_motion_duration(motor->total_pulses, motor->accel_pulses, motor->decel_pulses);
        if (current_duration > 0) {
            time_scale = (unsigned int)((max_duration * TIME_SCALE_PRECISION) / current_duration);
        } else {
            time_scale = TIME_SCALE_PRECISION; // 1.0 in fixed-point (1000)
        }
        motor->time_scale = time_scale;

        printk(KERN_INFO "Starting motor %d: %d pulses, accel=%d, decel=%d, direction=%d, time_scale=%u\n",
               motor_id, motor->total_pulses, motor->accel_pulses, motor->decel_pulses, motor->direction, motor->time_scale);
        gpio_set_value(motor->gpio_dir, motor->direction);
        timers[motor_id] = &motor->timer;
    }

    for (i = 0; i < num_cmds; i++) {
        int motor_id = cmds[i].motor_id;
        if (timers[motor_id]) {
            hrtimer_start(timers[motor_id], ktime_set(0, 0), HRTIMER_MODE_REL);
        }
    }

    printk(KERN_INFO "Synchronized motion initiated for %d motors\n", num_cmds);
}
