# Raspi Stepper Module

This repository contains a Linux kernel module designed for controlling stepper motors on a Raspberry Pi. Originally developed for delta robot applications, the module integrates both stepper motor control and limit switch monitoring to provide coordinated and safe motion control.  It is currently a work-in-progress.

## Features

- **Stepper Motor Control:**  
  Control up to three stepper motors with configurable pulse frequency, acceleration, and deceleration settings.

- **Limit Switch Integration:**  
  Three hardware limit switches are monitored via GPIO interrupts. These switches immediately signal the module to abort motor motion if a limit is reached.

- **Synchronized Motion:**  
  Supports simultaneous movement of multiple motors, ensuring coordinated operation essential for delta robot control.

- **Kernel Module Implementation:**  
  Designed as a loadable Linux kernel module for direct access to Raspberry Pi GPIOs, allowing low-level hardware control.

## Hardware Requirements

- **Raspberry Pi:**  
  Compatible with Raspberry Pi running a Linux-based OS.

- **Stepper Motors:**  
  Up to three stepper motors (configured in the source code).

- **Limit Switches:**  
  Three limit switches connected to designated GPIO pins (configured as follows in [delta_robot.h]&#8203;:contentReference[oaicite:0]{index=0}).

- **GPIO Wiring:**  
  Proper wiring for stepper drivers and limit switches is required. Verify GPIO pin assignments in the source code:
  - Limit switches: Pins 22, 24, and 26
  - Stepper control pins: See motor_states configuration in [stepper_control.c]&#8203;:contentReference[oaicite:1]{index=1}

## Software Requirements

- Linux kernel (module tested on recent Linux kernels)
- Build tools (e.g., gcc, make)
- Raspberry Pi OS (or another compatible Linux distribution)

## Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/billwinkler/raspi-stepper-module.git
   cd raspi-stepper-module
   ```
2. **Build the Module:**
   A Makefile is provided. Compile the module by running:
   ```bash
   make
   ```
3. **Load the Kernel Module:**
Insert the module into the kernel:
   ```bash
   sudo insmod delta_robot.ko
   ```
4. **Verify the Module:**
   Check that the module has loaded correctly:
   ```bash
   lsmod | grep delta_robot
   dmesg | tail
   ```
   The module creates a device file at `/dev/delta_robot` for user-space interaction.
   
## Configuration
## Configuration

The delta robot module's key parameters are centralized in the `delta_robot_config.h` file. This file lets you customize settings for motor control, limit switch pin assignments, and stepper motor GPIO configurations to match your specific hardware setup.

### Motor Control Parameters

- **CONFIG_MIN_FREQUENCY (100 Hz):**  
  Specifies the minimum operating frequency for the stepper motors. Lower frequencies help ensure smooth startup and stable low-speed performance. This is intended to be (not yet implemented) the starting and ending frequencies for ramping up and down pulses during accleration and deceleration.

- **CONFIG_MAX_FREQUENCY (5000 Hz):**  
  Sets the maximum frequency for the motors, which limits the top speed. It is used as the target frequency for the motors after accleration from the minimum specified frequency.

- **CONFIG_ACCELERATION_PULSES (200 pulses):**  
  Determines the number of pulses used during the acceleration phase. This helps in gradually ramping up the motor speed, reducing mechanical stress.

- **CONFIG_DECELERATION_PULSES (200 pulses):**  
  Specifies the number of pulses during deceleration, ensuring a controlled slowdown to prevent overshooting or undue wear on the hardware.

### Limit Switch Pin Definitions

- **CONFIG_LIMIT_SWITCH1_PIN (22):**  
  GPIO pin assigned for the first limit switch.

- **CONFIG_LIMIT_SWITCH2_PIN (24):**  
  GPIO pin assigned for the second limit switch.

- **CONFIG_LIMIT_SWITCH3_PIN (26):**  
  GPIO pin assigned for the third limit switch.

These pins are used to monitor the position of the limit switches, which provide safety feedback by aborting motor movement when triggered (not yet implemenented).

### Stepper Motor GPIO Pin Definitions

Each motor uses a pair of GPIO pins to control its operation:

- **Motor 0:**
  - **CONFIG_MOTOR0_STEP_PIN (17):**  
  - **CONFIG_MOTOR0_DIR_PIN (27):**  

- **Motor 1:**
  - **CONFIG_MOTOR1_STEP_PIN (18):**  
  - **CONFIG_MOTOR1_DIR_PIN (23):**  

- **Motor 2:**
  - **CONFIG_MOTOR2_STEP_PIN (19):**  
  - **CONFIG_MOTOR2_DIR_PIN (25):**  

### Customizing the Configuration

Before building and loading the module, review and modify `delta_robot_config.h` as needed to match your hardware configuration. Any changes in this file will directly affect the behavior of the motor control routines, the handling of limit switches, and the GPIO assignments used by the module.

## Usage

### Sending Commands

Write delta robot command structures to `/dev/delta_robot` to control motor movement. The revised command structure now includes only the following parameters:

- **Motor ID:** Specifies which motor to control.
- **Total Pulses:** The number of pulses to send to the motor, which determines the distance or extent of movement.
- **Direction:** Indicates the direction of movement (for example, `0` for one direction and `1` for the opposite).

Each command is therefore 12 bytes in total (assuming 4 bytes per integer). Here are some important points to consider when creating your binary file:

- **No Padding:**  
  Ensure that the structure is packed without any extra padding bytes. This is typically enforced using the `__attribute__((packed))` directive in the source code.

- **Byte Order:**  
  The integers should be stored in your system's native byte order. On a Raspberry Pi, this is usually little-endian. If you generate the binary file on a different system, be sure to convert the integers to little-endian format if necessary.

- **Multiple Commands:**  
  If you wish to send multiple commands in one operation (for example, three commands), the binary file should be a concatenation of the command structures. For three commands, the file should be 36 bytes long (3 commands × 12 bytes each).

Once your binary file is formatted correctly, you can use the `dd` command to send the commands to the device. For example:

#### Example using `dd`

If you have prepared a binary file (for example, `/tmp/delta_robot.bin`) containing three motor commands (each command being 12 bytes, for a total of 36 bytes), you can send the commands to the device using the following command:

   ```bash
   sudo dd if=/tmp/delta_robot.bin of=/dev/delta_robot bs=36 count=1
   ```

### Testing

Upon initialization, the module currently runs a test routine that toggles a motor's GPIO to confirm proper operation. Check the kernel logs (using `dmesg`) to verify that the command has been executed as expected.


