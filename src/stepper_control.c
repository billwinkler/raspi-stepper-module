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

static ktime_t calculate_next_period(struct motor_state *state)
{
    unsigned int k = state->pulse_count;
    unsigned int total = state->total_pulses;
    unsigned int accel = state->accel_pulses;
    unsigned int decel = state->decel_pulses;
    double f_min = CONFIG_MIN_FREQUENCY;
    double f_max = CONFIG_MAX_FREQUENCY;
    double f_k;

    if (total <= 1 || k >= total)
        return ktime_set(0, 0);

    // Acceleration frequency
    double f_accel = f_max;
    if (k < accel && accel > 1)
        f_accel = f_min + (f_max - f_min) * ((double)k / (accel - 1));

    // Deceleration frequency
    double f_decel = f_max;
    if (k >= total - decel && decel > 1)
    {
        unsigned int m = total - 1 - k;
        f_decel = f_min + (f_max - f_min) * ((double)m / (decel - 1));
    }

    // Take minimum frequency
    f_k = (f_accel < f_decel) ? f_accel : f_decel;
    if (f_k < f_min)
        f_k = f_min; // Clamp to minimum frequency

    // Convert to period in nanoseconds
    return ktime_set(0, (long long)(1000000000.0 / f_k));
}

static enum hrtimer_restart motor_timer_callback(struct hrtimer *timer)
{
    struct stepper_motor *state = container_of(timer, struct stepper_motor, timer);

    // Check if motion should stop
    if (state->pulse_count >= state->total_pulses || state->abort) {
        gpio_set_value(state->gpio_step, 0);
        return HRTIMER_NORESTART;
    }

    // Generate pulse
    gpio_set_value(state->gpio_step, 1);
    udelay(PULSE_WIDTH_US);
    gpio_set_value(state->gpio_step, 0);

    // Increment pulse count
    state->pulse_count++;

    // Calculate and schedule next pulse
    ktime_t next_period = calculate_next_period(state);
    if (next_period > 0) {
        ktime_t delay = ktime_sub(next_period, ktime_set(0, PULSE_WIDTH_US * 1000));
        if (delay < 0)
            delay = 0;
        hrtimer_start(timer, delay, HRTIMER_MODE_REL);
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
    struct stepper_motor *motors[MOTOR_COUNT];  // Array of motor states
    int i, j, max_pulses = 0;
    int base_delay;
    int accumulators[num_cmds];
    bool active[num_cmds];  // Tracks which motors are still active

    // Step 1: Initialize motors and pre-check limit switches
    for (i = 0; i < num_cmds; i++) {
        int motor_id = cmds[i].motor_id;
        int direction = cmds[i].direction;
        int limit_switch_pin;

        // Map motor ID to its limit switch pin
        switch (motor_id) {
            case 0: limit_switch_pin = CONFIG_LIMIT_SWITCH1_PIN; break;
            case 1: limit_switch_pin = CONFIG_LIMIT_SWITCH2_PIN; break;
            case 2: limit_switch_pin = CONFIG_LIMIT_SWITCH3_PIN; break;
            default: continue;
        }

        // Pre-check: If moving toward the limit (direction == 1) and switch is closed, suppress motion
        if (direction == 1 && gpio_get_value(limit_switch_pin) == 0) {
            printk(KERN_WARNING "Motor %d: Motion suppressed - limit switch already triggered\n", motor_id);
            active[i] = false;  // This motor won’t move
        } else {
            active[i] = true;   // This motor is active
            motor_states[motor_id].abort = false;  // Reset abort flag
            motor_states[motor_id].direction = direction;
            motors[i] = &motor_states[motor_id];
            if (cmds[i].total_pulses > max_pulses) {
                max_pulses = cmds[i].total_pulses;  // Find longest motion
            }
        }
    }

    // Log limit switch states for debugging
    printk(KERN_DEBUG "Starting synchronized motion: LS1=%d, LS2=%d, LS3=%d\n",
           gpio_get_value(CONFIG_LIMIT_SWITCH1_PIN),
           gpio_get_value(CONFIG_LIMIT_SWITCH2_PIN),
           gpio_get_value(CONFIG_LIMIT_SWITCH3_PIN));

    base_delay = 1000000 / (2 * CONFIG_MAX_FREQUENCY);  // Pulse timing in microseconds

    // Step 2: Set direction for active motors
    for (i = 0; i < num_cmds; i++) {
        if (active[i]) {
            gpio_set_value(motors[i]->gpio_dir, cmds[i].direction);
            printk(KERN_DEBUG "Motor %d: Direction=%d, Pulses=%d\n",
                   cmds[i].motor_id, cmds[i].direction, cmds[i].total_pulses);
        }
    }

    // Step 3: Initialize accumulators for pulse scaling
    for (i = 0; i < num_cmds; i++) {
        accumulators[i] = 0;
    }

    // Step 4: Synchronized pulse loop
    for (j = 0; j < max_pulses; j++) {
        // Generate pulses for active motors that haven’t aborted
        for (i = 0; i < num_cmds; i++) {
            if (active[i] && !motor_states[cmds[i].motor_id].abort) {
                accumulators[i] += cmds[i].total_pulses;
                if (accumulators[i] >= max_pulses) {
                    gpio_set_value(motors[i]->gpio_step, 1);  // Step high
                    accumulators[i] -= max_pulses;
                }
            }
        }
        udelay(base_delay);  // High pulse duration

        // Reset step pins for active motors
        for (i = 0; i < num_cmds; i++) {
            if (active[i] && !motor_states[cmds[i].motor_id].abort) {
                gpio_set_value(motors[i]->gpio_step, 0);  // Step low
            }
        }
        udelay(base_delay);  // Low pulse duration
    }

    // Step 5: Ensure all step pins are low at the end
    for (i = 0; i < num_cmds; i++) {
        if (active[i]) {
            gpio_set_value(motors[i]->gpio_step, 0);
        }
    }

    printk(KERN_INFO "Synchronized motion complete\n");
}



