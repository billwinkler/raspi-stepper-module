#ifndef DELTA_ROBOT_CONFIG_H
#define DELTA_ROBOT_CONFIG_H

/* Configuration parameters for motor control */
#define PULSE_WIDTH_US 10          // Fixed pulse width in microseconds
#define CONFIG_MIN_FREQUENCY       100.0   // Minimum frequency (Hz)
#define CONFIG_MAX_FREQUENCY       5000.0  // Maximum frequency (Hz)
#define CONFIG_ACCELERATION_PULSES 50   // Number of pulses during acceleration
#define CONFIG_DECELERATION_PULSES 50   // Number of pulses during deceleration

/* Limit switch pin definitions */
#define CONFIG_LIMIT_SWITCH1_PIN   22
#define CONFIG_LIMIT_SWITCH2_PIN   24
#define CONFIG_LIMIT_SWITCH3_PIN   26

/* Stepper motor GPIO pin definitions */
#define CONFIG_MOTOR0_STEP_PIN     17
#define CONFIG_MOTOR0_DIR_PIN      27
#define CONFIG_MOTOR1_STEP_PIN     18
#define CONFIG_MOTOR1_DIR_PIN      23
#define CONFIG_MOTOR2_STEP_PIN     19
#define CONFIG_MOTOR2_DIR_PIN      25

#endif /* DELTA_ROBOT_CONFIG_H */
