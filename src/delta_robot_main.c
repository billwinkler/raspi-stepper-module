#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include "../include/delta_robot.h"
#include "../include/stepper_control.h"

/* Forward declarations for submodule initializations */
extern int limit_switch_init(void);
extern void limit_switch_exit(void);
extern void start_synchronized_motion(struct delta_robot_cmd cmds[], int num_cmds);
void start_motor_motion(int motor_id, struct delta_robot_cmd *cmd);

/* Open callback */
static int delta_robot_open(struct inode *inode, struct file *file)
{
    printk(KERN_ALERT "delta_robot: Device opened\n");
    return 0;
}

static void test_gpio_toggle(void)
{
    int i;
    int gpio = motor_states[1].gpio_step;  // Use motor 1's step pin for testing
    printk(KERN_INFO "test_gpio_toggle: Starting toggle test on GPIO %d\n", gpio);

    for (i = 0; i < 10; i++) {
        gpio_set_value(gpio, 1);
        printk(KERN_INFO "test_gpio_toggle: Iteration %d - GPIO %d set HIGH\n", i, gpio);
        msleep(500);  // Sleep for 500 milliseconds

        gpio_set_value(gpio, 0);
        printk(KERN_INFO "test_gpio_toggle: Iteration %d - GPIO %d set LOW\n", i, gpio);
        msleep(500);  // Sleep for 500 milliseconds
    }

    printk(KERN_INFO "test_gpio_toggle: Toggle test complete.\n");
}

static ssize_t delta_robot_write(struct file *file, const char __user *buf,
                                 size_t count, loff_t *ppos)
{
    int num_cmds;
    struct delta_robot_cmd *cmds;
    size_t cmd_size = sizeof(struct delta_robot_cmd);

    if (count % cmd_size != 0) {
        printk(KERN_ERR "delta_robot: Command size %zu is not a multiple of %zu bytes\n",
               count, cmd_size);
        return -EINVAL;
    }

    num_cmds = count / cmd_size;
    if (num_cmds < 1 || num_cmds > 3) {
        printk(KERN_ERR "delta_robot: Invalid number of motor commands: %d. Supported range is 1-3.\n", num_cmds);
        return -EINVAL;
    }

    cmds = kmalloc(count, GFP_KERNEL);
    if (!cmds)
        return -ENOMEM;

    if (copy_from_user(cmds, buf, count)) {
        kfree(cmds);
        return -EFAULT;
    }

    printk(KERN_INFO "delta_robot: Received %d motor command(s), executing in sync\n", num_cmds);

    // ✅ Call the function only once with all motor commands
    start_synchronized_motion(cmds, num_cmds);

    kfree(cmds);
    return count;
}

/* Define file operations */
static const struct file_operations delta_robot_fops = {
    .owner = THIS_MODULE,
    .open  = delta_robot_open,
    .write = delta_robot_write,
};

/* Register the misc device; this creates /dev/delta_robot */
static struct miscdevice delta_robot_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "delta_robot",
    .fops  = &delta_robot_fops,
};

static int __init delta_robot_init(void)
{
    int ret;
    printk(KERN_INFO "Delta Robot Module: Initializing\n");

    ret = limit_switch_init();
    if (ret) return ret;

    ret = stepper_init();
    if (ret) return ret;

    ret = misc_register(&delta_robot_misc);
    if (ret) return ret;

   // Call the test function
    test_gpio_toggle();

    printk(KERN_INFO "Delta Robot Module: Initialization complete\n");
    return 0;
}

static void __exit delta_robot_exit(void)
{
    misc_deregister(&delta_robot_misc);
    stepper_exit();
    limit_switch_exit();
    printk(KERN_INFO "Delta Robot Module: Exiting\n");
}

module_init(delta_robot_init);
module_exit(delta_robot_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bill Winkler");
MODULE_DESCRIPTION("Delta Robot Control Module with Stepper and Limit Switch Integration");
MODULE_VERSION("1.0.0");
