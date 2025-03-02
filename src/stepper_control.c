#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "../include/delta_robot_config.h"
#include "../include/delta_robot.h"
#include "../include/stepper_control.h"

// Global motor state array: now using macros from the config file
struct stepper_motor motor_states[MOTOR_COUNT] = {
    { .id = 0, .gpio_step = CONFIG_MOTOR0_STEP_PIN, .gpio_dir = CONFIG_MOTOR0_DIR_PIN },
    { .id = 1, .gpio_step = CONFIG_MOTOR1_STEP_PIN, .gpio_dir = CONFIG_MOTOR1_DIR_PIN },
    { .id = 2, .gpio_step = CONFIG_MOTOR2_STEP_PIN, .gpio_dir = CONFIG_MOTOR2_DIR_PIN }
};

#define NS_PER_SEC 1000000000LL

static ktime_t calculate_next_period(struct stepper_motor *state)
{
    unsigned int k = state->pulse_count;
    unsigned int total = state->total_pulses;
    unsigned int accel = state->accel_pulses;
    unsigned int decel = state->decel_pulses;
    long long min_period = NS_PER_SEC / CONFIG_MAX_FREQUENCY;
    long long max_period = NS_PER_SEC / CONFIG_MIN_FREQUENCY;
    long long period_accel = min_period;
    long long period_decel = min_period;

    if (total <= 1 || k >= total)
        return ktime_set(0, 0);

    if (k < accel && accel > 1) {
        long long delta = (max_period - min_period) * k / (accel - 1);
        period_accel = max_period - delta;
    }
    if (k >= total - decel && decel > 1) {
        unsigned int m = total - 1 - k;
        long long delta = (max_period - min_period) * m / (decel - 1);
        period_decel = min_period + delta;
    }

    long long period_k = (period_accel > period_decel) ? period_accel : period_decel;
    if (period_k > max_period)
        period_k = max_period;

    return ktime_set(0, period_k);
}

static enum hrtimer_restart motor_timer_callback(struct hrtimer *timer)
{
    struct stepper_motor *state = container_of(timer, struct stepper_motor, timer);
    ktime_t next_period;

    if (state->pulse_count >= state->total_pulses || state->abort) {
        gpio_set_value(state->gpio_step, 0);
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
        // Request and configure step GPIO
        ret = gpio_request(motor_states[i].gpio_step, "step");
        if (ret) {
            printk(KERN_ERR "stepper_control: Failed to request GPIO %d for step\n", 
                   motor_states[i].gpio_step);
            goto cleanup;
        }
        gpio_direction_output(motor_states[i].gpio_step, 0);

        // Request and configure direction GPIO
        ret = gpio_request(motor_states[i].gpio_dir, "dir");
        if (ret) {
            printk(KERN_ERR "stepper_control: Failed to request GPIO %d for direction\n", 
                   motor_states[i].gpio_dir);
            gpio_free(motor_states[i].gpio_step);
            goto cleanup;
        }
        gpio_direction_output(motor_states[i].gpio_dir, 0);

        // Initialize hrtimer
        hrtimer_init(&motor_states[i].timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        motor_states[i].timer.function = motor_timer_callback;
        motor_states[i].pulse_count = 0;
        motor_states[i].abort = false;
    }

    printk(KERN_INFO "stepper_control: GPIOs and hrtimers initialized for all motors\n");
    return 0;

cleanup:
    // Clean up any successfully requested GPIOs on failure
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

    // Validate motor ID
    if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
        printk(KERN_ERR "stepper_control: Invalid motor id %d\n", motor_id);
        return;
    }

    motor = &motor_states[motor_id];

    // Set motor parameters
    motor->direction = cmd->direction;
    motor->total_pulses = cmd->total_pulses;
    motor->accel_pulses = CONFIG_ACCELERATION_PULSES;
    motor->decel_pulses = CONFIG_DECELERATION_PULSES;
    motor->pulse_count = 0;
    motor->abort = false;

    // Set direction GPIO
    gpio_set_value(motor->gpio_dir, motor->direction);

    // Determine limit switch pin
    switch (motor_id) {
        case 0: limit_switch_pin = CONFIG_LIMIT_SWITCH1_PIN; break;
        case 1: limit_switch_pin = CONFIG_LIMIT_SWITCH2_PIN; break;
        case 2: limit_switch_pin = CONFIG_LIMIT_SWITCH3_PIN; break;
        default: return; // Shouldn’t happen due to prior validation
    }

    // Pre-check limit switch
    if (motor->direction == 1 && gpio_get_value(limit_switch_pin) == 0) {
        printk(KERN_WARNING "Motor %d: Motion suppressed - limit switch already triggered\n", 
               motor_id);
        return;
    }

    // Log motion start
    printk(KERN_INFO "stepper_control: Starting motor %d for %d pulses, direction %d\n",
           motor_id, motor->total_pulses, motor->direction);

    // Start the hrtimer
    hrtimer_start(&motor->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
}

void start_synchronized_motion(struct delta_robot_cmd cmds[], int num_cmds) {
    int i;

    for (i = 0; i < num_cmds; i++) {
        int motor_id = cmds[i].motor_id;
        if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
            printk(KERN_ERR "Invalid motor ID %d\n", motor_id);
            continue;
        }

        struct stepper_motor *motor = &motor_states[motor_id];
        int limit_switch_pin;

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
        motor->accel_pulses = CONFIG_ACCELERATION_PULSES;
        motor->decel_pulses = CONFIG_DECELERATION_PULSES;
        motor->pulse_count = 0;
        motor->abort = false;

        gpio_set_value(motor->gpio_dir, motor->direction);
        printk(KERN_INFO "Starting motor %d: %d pulses, direction %d\n",
               motor_id, motor->total_pulses, motor->direction);

        hrtimer_start(&motor->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
    }

    printk(KERN_INFO "Synchronized motion initiated for %d motors\n", num_cmds);
}




