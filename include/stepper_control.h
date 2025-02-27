#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include "../include/delta_robot.h"

#define MOTOR_COUNT 3

/* Define stepper motor structure */

struct stepper_motor {
    int id;                // Motor identifier
    int gpio_step;         // GPIO pin for step signal
    int gpio_dir;          // GPIO pin for direction
    int direction;         // Motion direction (0 or 1)
    int total_pulses;      // Total pulses to generate
    int accel_pulses;      // Number of pulses for acceleration
    int decel_pulses;      // Number of pulses for deceleration
    int pulse_count;       // Current pulse count
    bool abort;            // Flag to abort motion
    struct hrtimer timer;  // HRTimer for pulse scheduling
};

/* Declare motor_states globally so other modules can use it */
extern struct stepper_motor motor_states[MOTOR_COUNT];

int stepper_init(void);
void stepper_exit(void);
void start_motor_motion(int motor_id, struct delta_robot_cmd *cmd);

#endif
