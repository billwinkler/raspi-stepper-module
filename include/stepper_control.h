#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <linux/hrtimer.h>

#define MOTOR_COUNT 3

struct stepper_motor {
    int id;
    int gpio_step;
    int gpio_dir;
    unsigned int total_pulses;
    unsigned int pulse_count;
    unsigned int accel_pulses;
    unsigned int decel_pulses;
    int direction;
    bool abort;
    struct hrtimer timer;
    unsigned int time_scale; // Fixed-point: scale factor * 1000
};

extern struct stepper_motor motor_states[MOTOR_COUNT];


int stepper_init(void);
void stepper_exit(void);
void start_motor_motion(int motor_id, struct delta_robot_cmd *cmd);
void start_synchronized_motion(struct delta_robot_cmd cmds[], int num_cmds);
int get_motor_step_pin(int motor_id);


#endif
