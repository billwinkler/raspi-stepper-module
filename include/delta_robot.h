#ifndef DELTA_ROBOT_H
#define DELTA_ROBOT_H

#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <stdbool.h>
#include "delta_robot_config.h"  // All pin definitions come from here

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

/* Define the command structure */
struct delta_robot_cmd {
    int motor_id;
    int total_pulses;
    int direction;
} __attribute__((packed));

/* Prototype for starting motor motion */
extern void start_motor_motion(int motor_index, struct delta_robot_cmd *cmd);

#endif /* DELTA_ROBOT_H */
