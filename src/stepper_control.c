#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "../include/delta_robot.h"
#include "../include/stepper_control.h"

/* Global motor state array */
struct stepper_motor motor_states[MOTOR_COUNT] = {
    { .id = 0, .gpio_step = 17, .gpio_dir = 27 },
    { .id = 1, .gpio_step = 18, .gpio_dir = 23 },
    { .id = 2, .gpio_step = 19, .gpio_dir = 25 }
};

int stepper_init(void)
{
    int i, ret;
    for (i = 0; i < MOTOR_COUNT; i++) {
        ret = gpio_request(motor_states[i].gpio_step, "step");
        if (ret) {
            printk(KERN_ERR "stepper_control: Failed to request GPIO %d for step\n", motor_states[i].gpio_step);
            return ret;
        }
        ret = gpio_request(motor_states[i].gpio_dir, "dir");
        if (ret) {
            printk(KERN_ERR "stepper_control: Failed to request GPIO %d for direction\n", motor_states[i].gpio_dir);
            return ret;
        }

        gpio_direction_output(motor_states[i].gpio_step, 0);
        gpio_direction_output(motor_states[i].gpio_dir, 0);
    }

    printk(KERN_INFO "stepper_control: GPIOs initialized for all motors\n");
    return 0;
}

void stepper_exit(void)
{
    int i;
    for (i = 0; i < MOTOR_COUNT; i++) {
        gpio_free(motor_states[i].gpio_step);
        gpio_free(motor_states[i].gpio_dir);
    }
    printk(KERN_INFO "stepper_control: GPIOs freed\n");
}

void start_motor_motion(int motor_id, struct delta_robot_cmd *cmd) {
    struct stepper_motor *motor;
    int pulse_delay_us;
    int i;

    if (motor_id < 0 || motor_id >= MOTOR_COUNT) {
        printk(KERN_ERR "stepper_control: Invalid motor id %d\n", motor_id);
        return;
    }

    motor = &motor_states[motor_id];  // Assign the correct motor

    motor->total_pulses = cmd->total_pulses;
    motor->target_freq  = cmd->target_freq;
    motor->accel_pulses = cmd->accel_pulses;
    motor->decel_pulses = cmd->decel_pulses;
    motor->direction    = cmd->direction;

    printk(KERN_INFO "stepper_control: Moving motor %d for %d pulses at %d Hz\n",
           motor_id, motor->total_pulses, motor->target_freq);

    if (motor->target_freq == 0) {
        printk(KERN_ERR "stepper_control: ERROR! target_freq is 0! Motion aborted.\n");
        return;
    }

    gpio_set_value(motor->gpio_dir, motor->direction);
    pulse_delay_us = (motor->target_freq > 0) ? (1000000 / (2 * motor->target_freq)) : 0;

    for (i = 0; i < motor->total_pulses; i++) {
        gpio_set_value(motor->gpio_step, 1);
        udelay(pulse_delay_us);
        gpio_set_value(motor->gpio_step, 0);
        udelay(pulse_delay_us);
    }

    printk(KERN_INFO "stepper_control: Motor %d motion complete\n", motor_id);
}

void start_synchronized_motion(struct delta_robot_cmd cmds[], int num_cmds) {
    int ls1_state, ls2_state, ls3_state;
    struct stepper_motor *motors[MOTOR_COUNT];
    int i, j, max_pulses = 0;
    int base_delay;

    /* Check current limit switch states before starting movement.
       (For now, we ignore the abort flag later so we clear it here.) */
    ls1_state = gpio_get_value(LIMIT_SWITCH1_PIN);
    ls2_state = gpio_get_value(LIMIT_SWITCH2_PIN);
    ls3_state = gpio_get_value(LIMIT_SWITCH3_PIN);
    printk(KERN_DEBUG "Starting movement: Limit Switch States: LS1=%d, LS2=%d, LS3=%d\n",
           ls1_state, ls2_state, ls3_state);

    /* Clear abort flags regardless of the limit switch state */
    motor_states[0].abort = 0;
    motor_states[1].abort = 0;
    motor_states[2].abort = 0;

    /* Set up motor pointers and determine the maximum pulses among the commands */
    for (i = 0; i < num_cmds; i++) {
        motors[i] = &motor_states[cmds[i].motor_id];
        if (cmds[i].total_pulses > max_pulses) {
            max_pulses = cmds[i].total_pulses;
        }
    }

    /* Set each motor's direction pin */
    for (i = 0; i < num_cmds; i++) {
        gpio_set_value(motors[i]->gpio_dir, cmds[i].direction);
        printk(KERN_DEBUG "Motor %d: Direction set to %d\n", cmds[i].motor_id, cmds[i].direction);
    }

    /* Choose the motor with max pulses to set the base delay */
    int max_index = 0;
    for (i = 0; i < num_cmds; i++) {
        if (cmds[i].total_pulses == max_pulses) {
            max_index = i;
            break;
        }
    }
    base_delay = 1000000 / (2 * cmds[max_index].target_freq);
    printk(KERN_DEBUG "Base_delay: %d\n", base_delay);

    /* Initialize accumulators for each motor */
    int accumulators[num_cmds];
    for (i = 0; i < num_cmds; i++) {
        accumulators[i] = 0;
    }

    /* Synchronized pulse generation loop.
       Abort checking and detailed per-pulse debug printks have been removed. */
    for (j = 0; j < max_pulses; j++) {
        /* First half-cycle: update accumulators and set step HIGH when threshold reached */
        for (i = 0; i < num_cmds; i++) {
            accumulators[i] += cmds[i].total_pulses;
            if (accumulators[i] >= max_pulses) {
                gpio_set_value(motors[i]->gpio_step, 1);
                accumulators[i] -= max_pulses;
            }
        }
        udelay(base_delay);

        /* Second half-cycle: set step LOW */
        for (i = 0; i < num_cmds; i++) {
            gpio_set_value(motors[i]->gpio_step, 0);
        }
        udelay(base_delay);
    }

    printk(KERN_INFO "stepper_control: Synchronized motion complete\n");
}
