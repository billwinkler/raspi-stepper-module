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
