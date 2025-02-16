#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "../include/delta_robot.h"
#include "../include/stepper_control.h"

/* Global motor state array */
struct stepper_motor motor_states[MOTOR_COUNT] = {
    { .id = 0, .gpio_step = 17, .gpio_dir = 27 },
    { .id = 1, .gpio_step = 23, .gpio_dir = 18 },
    { .id = 2, .gpio_step = 25, .gpio_dir = 19 }
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

    motor = &motor_states[motor_id];

    printk(KERN_INFO "start_motor_motion: Received motor_id=%d, total_pulses=%d, target_freq=%d, accel_pulses=%d, decel_pulses=%d, direction=%d\n",
           motor_id, cmd->total_pulses, cmd->target_freq, cmd->accel_pulses, cmd->decel_pulses, cmd->direction);

    motor->total_pulses = cmd->total_pulses;
    motor->target_freq  = cmd->target_freq;
    motor->accel_pulses = cmd->accel_pulses;
    motor->decel_pulses = cmd->decel_pulses;
    motor->direction    = cmd->direction;

    printk(KERN_INFO "stepper_control: Stored motor state for motor %d: target_freq=%d\n",
           motor_id, motor->target_freq);

    gpio_set_value(motor->gpio_dir, motor->direction);
    pulse_delay_us = (motor->target_freq > 0) ? (1000000 / (2 * motor->target_freq)) : 0;

    printk(KERN_INFO "stepper_control: Moving motor %d for %d pulses at %d Hz\n",
           motor_id, motor->total_pulses, motor->target_freq);

    if (motor->target_freq == 0) {
        printk(KERN_ERR "stepper_control: ERROR! target_freq is 0! Motion aborted.\n");
        return;
    }

    for (i = 0; i < motor->total_pulses; i++) {
        gpio_set_value(motor->gpio_step, 1);
        udelay(pulse_delay_us);
        gpio_set_value(motor->gpio_step, 0);
        udelay(pulse_delay_us);
    }

    printk(KERN_INFO "stepper_control: Motor %d motion complete\n", motor_id);
}

