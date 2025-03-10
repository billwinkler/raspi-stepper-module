#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "../include/delta_robot_config.h"
#include "../include/delta_robot.h"
#include "../include/stepper_control.h"

// Forward declaration of estimate_motion_duration
static long long estimate_motion_duration(unsigned int total_pulses, unsigned int accel_pulses, unsigned int decel_pulses);

#define MIN_PHASE_PULSES 10
#define NS_PER_SEC 1000000000LL
#define TIME_SCALE_PRECISION 10000

static int debug = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable debug logging (1 = on, 0 = off)");

struct stepper_motor motor_states[MOTOR_COUNT] = {
    { .id = 0, .gpio_step = CONFIG_MOTOR0_STEP_PIN, .gpio_dir = CONFIG_MOTOR0_DIR_PIN },
    { .id = 1, .gpio_step = CONFIG_MOTOR1_STEP_PIN, .gpio_dir = CONFIG_MOTOR1_DIR_PIN },
    { .id = 2, .gpio_step = CONFIG_MOTOR2_STEP_PIN, .gpio_dir = CONFIG_MOTOR2_DIR_PIN }
};

static ktime_t calculate_next_period(struct stepper_motor *state)
{
    /* Declare all variables at the top */
    unsigned int k = state->pulse_count;
    unsigned int total = state->total_pulses;
    unsigned int accel = state->accel_pulses;
    unsigned int decel = state->decel_pulses;
    long long min_period = NS_PER_SEC / CONFIG_MAX_FREQUENCY;
    long long max_period = NS_PER_SEC / CONFIG_MIN_FREQUENCY;
    long long period_accel = min_period;
    long long period_decel = min_period;
    long long period_k;
    unsigned int time_scale = state->time_scale;
    long long next_period_ns;  /* Declaration moved here */

    /* Executable code starts here */
    if (total <= 1 || k >= total)
        return ktime_set(0, 0);

    if (k < accel && accel > 1) {
        long long delta = (max_period - min_period) * k / (accel - 1);
        period_accel = max_period - delta;
    }
    if (k >= total - decel && decel > 1) {
        unsigned int m = total - 1 - k;
        long long delta = (max_period - min_period) * (decel - 1 - m) / (decel - 1);
        period_decel = min_period + delta;
    }

    period_k = (period_accel > period_decel) ? period_accel : period_decel;
    period_k = (period_k * time_scale) / TIME_SCALE_PRECISION;

    /* Assign value to next_period_ns here */
    next_period_ns = period_k - (PULSE_WIDTH_US * 1000LL);
    if (next_period_ns < 0)
        next_period_ns = 0;

    return ktime_set(0, next_period_ns);
}

static enum hrtimer_restart motor_timer_callback(struct hrtimer *timer)
{
    struct stepper_motor *state = container_of(timer, struct stepper_motor, timer);
    ktime_t next_period;
    ktime_t stop_time;

    gpio_set_value(state->gpio_step, 1);
    udelay(PULSE_WIDTH_US);  // 100 µs
    stop_time = ktime_get();
    gpio_set_value(state->gpio_step, 0);

    state->pulse_count++;

    if (debug && (state->pulse_count % 10 == 0 || state->pulse_count == 1)) {
      if (state->pulse_count == 1) {
        // First pulse: no frequency available
        printk(KERN_DEBUG "Motor %d: Pulse %u at %lld ns\n",
               state->id, state->pulse_count, ktime_to_ns(stop_time));
      } else {
        // Every 10th pulse: calculate and include frequency
        long long delta = ktime_to_ns(stop_time) - ktime_to_ns(state->last_debug_time);
        if (delta > 0) {
          long long freq_hz = 10000000000LL / delta;  // Frequency in Hz
          printk(KERN_DEBUG "Motor %d: Pulse %u at %lld ns, approx freq of last 10 pulses: %lld Hz\n",
                 state->id, state->pulse_count, ktime_to_ns(stop_time), freq_hz);
        } else {
          // Fallback if delta is invalid (shouldn’t happen after pulse 1)
          printk(KERN_DEBUG "Motor %d: Pulse %u at %lld ns\n",
                 state->id, state->pulse_count, ktime_to_ns(stop_time));
        }
      }
      state->last_debug_time = stop_time;  // Update for the next print
    }

    if (state->pulse_count >= state->total_pulses || state->abort) {
        printk(KERN_INFO "Motor %d: Motion stopped at %lld ns - pulse_count=%u, total=%u, abort=%d\n",
               state->id, ktime_to_ns(stop_time), state->pulse_count, state->total_pulses, state->abort);
        return HRTIMER_NORESTART;
    }

    next_period = calculate_next_period(state);
    if (next_period <= 0) return HRTIMER_NORESTART;

    hrtimer_start(timer, next_period, HRTIMER_MODE_REL); // Revert to original
    return HRTIMER_RESTART;
}

int stepper_init(void)
{
    int i;
    int ret;

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
        motor_states[i].time_scale = TIME_SCALE_PRECISION;
    }

    if (debug) {
        printk(KERN_INFO "stepper_control: GPIOs and hrtimers initialized for all motors\n");
    }
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
    if (debug) {
        printk(KERN_INFO "stepper_control: GPIOs freed and timers canceled\n");
    }
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
        if (debug) {
            printk(KERN_WARNING "Motor %d: Motion suppressed - limit switch already triggered\n", motor_id);
        }
        return;
    }

    if (debug) {
        printk(KERN_INFO "stepper_control: Starting motor %d for %d pulses, accel=%d, decel=%d, direction=%d\n",
               motor_id, motor->total_pulses, motor->accel_pulses, motor->decel_pulses, motor->direction);
    }

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
    long long min_period = NS_PER_SEC / CONFIG_MAX_FREQUENCY; // 500,000 ns
    long long max_period = NS_PER_SEC / CONFIG_MIN_FREQUENCY; // 1,000,000 ns
    long long duration = 0;
    long long pulse_overhead = 113909; // Adjusted to match actual
    unsigned int constant_pulses = total_pulses > (accel_pulses + decel_pulses) ? total_pulses - accel_pulses - decel_pulses : 0;

    if (total_pulses <= 1) return 0;

    duration += ((max_period + min_period) * accel_pulses) / 2 + accel_pulses * pulse_overhead;
    duration += constant_pulses * (min_period + pulse_overhead);
    duration += ((min_period + max_period) * decel_pulses) / 2 + decel_pulses * pulse_overhead;

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
    unsigned int default_accel = CONFIG_ACCELERATION_PULSES;
    unsigned int default_decel = CONFIG_DECELERATION_PULSES;
    unsigned int total;

    printk(KERN_INFO "Starting synchronized motion for %d motors\n", num_cmds);

    // First pass: Calculate natural durations and find max_duration
    for (i = 0; i < num_cmds; i++) {
        int motor_id = cmds[i].motor_id;
        if (motor_id < 0 || motor_id >= MOTOR_COUNT) continue;

        motor = &motor_states[motor_id];
        total = cmds[i].total_pulses;

        // Set accel_pulses and decel_pulses based on total_pulses
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

        motor->direction = cmds[i].direction;
        motor->total_pulses = total;
        motor->pulse_count = 0;
        motor->abort = false;
    }

    // Second pass: Set time_scale for each motor based on max_duration
    for (i = 0; i < num_cmds; i++) {
        int motor_id = cmds[i].motor_id;
        if (motor_id < 0 || motor_id >= MOTOR_COUNT) continue;

        motor = &motor_states[motor_id];
        current_duration = estimate_motion_duration(motor->total_pulses, motor->accel_pulses, motor->decel_pulses);
        if (current_duration > 0) {
            motor->time_scale = (unsigned int)(((max_duration * (long long)TIME_SCALE_PRECISION) + (current_duration / 2)) / current_duration);
        } else {
            motor->time_scale = TIME_SCALE_PRECISION; // Default if no pulses
        }

        if (debug) {
            printk(KERN_INFO "Starting motor %d: %u pulses, accel=%u, decel=%u, dir=%d, time_scale=%u, natural_duration=%lld ns, target_duration=%lld ns\n",
                   motor_id, motor->total_pulses, motor->accel_pulses, motor->decel_pulses, motor->direction, motor->time_scale, current_duration, max_duration);
        }

        if (motor->total_pulses > 0) {
            gpio_set_value(motor->gpio_dir, motor->direction);
            timers[motor_id] = &motor->timer;
        }
    }

    // Start timers for all motors
    for (i = 0; i < num_cmds; i++) {
        int motor_id = cmds[i].motor_id;
        if (timers[motor_id]) {
            switch (motor_id) {
                case 0: limit_switch_pin = CONFIG_LIMIT_SWITCH1_PIN; break;
                case 1: limit_switch_pin = CONFIG_LIMIT_SWITCH2_PIN; break;
                case 2: limit_switch_pin = CONFIG_LIMIT_SWITCH3_PIN; break;
                default: continue;
            }
            if (motor->direction == 1 && gpio_get_value(limit_switch_pin) == 0) {
                if (debug) printk(KERN_WARNING "Motor %d: Motion suppressed\n", motor_id);
                continue;
            }
            hrtimer_start(timers[motor_id], ktime_set(0, 0), HRTIMER_MODE_REL);
        }
    }

    if (debug) {
        printk(KERN_INFO "Synchronized motion initiated for %d motors, target duration=%lld ns\n", num_cmds, max_duration);
    }
}
