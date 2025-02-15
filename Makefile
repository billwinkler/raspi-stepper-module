# Path to the kernel headers directory
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# Define the composite module "delta_robot"
obj-m += delta_robot.o
delta_robot-objs := src/delta_robot_main.o src/limit_switch.o src/stepper_control.o

# Tell the build system to add the include directory for headers
ccflags-y := -I$(PWD)/include

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
