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

/* Define the command structure */
struct delta_robot_cmd {
    int motor_id;       // Add motor ID to the struct
    int total_pulses;
    int target_freq;
    int accel_pulses;
    int decel_pulses;
    int direction;
} __attribute__((packed));  // Ensure correct memory alignment


/* Prototype for starting motor motion */
extern void start_motor_motion(int motor_index, struct delta_robot_cmd *cmd);

#endif /* DELTA_ROBOT_H */
