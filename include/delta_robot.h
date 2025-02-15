#ifndef DELTA_ROBOT_H
#define DELTA_ROBOT_H

#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <stdbool.h>

/* Limit switch pin definitions */
#define LIMIT_SWITCH1_PIN 22
#define LIMIT_SWITCH2_PIN 24
#define LIMIT_SWITCH3_PIN 26

/* Define the motor state structure, including a timer member */
struct motor_state {
    unsigned int pulse_count;
    unsigned int total_pulses;
    unsigned int accel_pulses;
    unsigned int decel_pulses;
    ktime_t current_period;
    ktime_t target_period;
    bool abort;
    struct hrtimer timer;  /* Required for container_of in timer callback */
};

/* Declare the global motor_states array */
extern struct motor_state motor_states[3];

/* Define the command structure */
struct delta_robot_cmd {
    unsigned int total_pulses[3];
    unsigned int target_freq[3];
    unsigned int accel_pulses[3];
    unsigned int decel_pulses[3];
    unsigned int direction[3];
};

/* Prototype for starting motor motion */
extern void start_motor_motion(int motor_index, struct delta_robot_cmd *cmd);

#endif /* DELTA_ROBOT_H */
