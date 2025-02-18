#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include "../include/delta_robot.h"

#define MOTOR_COUNT 3

/* Define stepper motor structure */
struct stepper_motor {
    int id;
    int total_pulses;
    int target_freq;
    int accel_pulses;
    int decel_pulses;
    int direction;
    int gpio_step;
    int gpio_dir;
    bool abort;
};

/* Declare motor_states globally so other modules can use it */
extern struct stepper_motor motor_states[MOTOR_COUNT];

int stepper_init(void);
void stepper_exit(void);
void start_motor_motion(int motor_id, struct delta_robot_cmd *cmd);

#endif
