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
2. **Build the Module:**
   A Makefile is provided. Compile the module by running:
   ```bash
   make
3. **Load the Kernel Module:**
Insert the module into the kernel:
   ```bash
   sudo insmod delta_robot.ko
   
4. **Verify the Module:**
   Check that the module has loaded correctly:
   ```bash
   lsmod | grep delta_robot
   dmesg | tail
   
   The module creates a device file at `/dev/delta_robot` for user-space interaction.
   
## Configuration

This module includes several configurable settings that you may need to adjust to match your hardware setup. The key configuration options are:

- **GPIO Pin Assignments:**
  - **Limit Switches:**  
    The GPIO pins for the limit switches are defined in [delta_robot.h]&#8203;:contentReference[oaicite:0]{index=0}:
    - `LIMIT_SWITCH1_PIN` (default: 22)
    - `LIMIT_SWITCH2_PIN` (default: 24)
    - `LIMIT_SWITCH3_PIN` (default: 26)
  
  - **Stepper Motor Control:**  
    The step and direction GPIO pins for each motor are configured in the `motor_states` array in [stepper_control.c]&#8203;:contentReference[oaicite:1]{index=1}:
    - **Motor 0:**  
      - Step Pin: 17  
      - Direction Pin: 27
    - **Motor 1:**  
      - Step Pin: 18  
      - Direction Pin: 23
    - **Motor 2:**  
      - Step Pin: 19  
      - Direction Pin: 25

- **Motor Command Parameters:**
  The command structure for motor control is defined in [delta_robot.h]&#8203;:contentReference[oaicite:2]{index=2}. The revised command structure now includes:
  - **Motor ID:** Identifies which motor to control.
  - **Total Pulses:** Specifies the number of pulses to be sent.
  - **Direction:** Determines the direction of rotation.

- **Compile-Time and Build Options:**
  - Check and modify the Makefile if needed to match your kernel version or to resolve any GPIO conflicts.
  - Ensure that any changes to the default GPIO assignments are reflected both in the source code and in your hardware wiring.

Review these settings before building and loading the module to ensure that they align with your specific Raspberry Pi configuration and connected hardware.

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

### Testing

Upon initialization, the module currently runs a test routine that toggles a motor's GPIO to confirm proper operation. Check the kernel logs (using `dmesg`) to verify that the command has been executed as expected.


