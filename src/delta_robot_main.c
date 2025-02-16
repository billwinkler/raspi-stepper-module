#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "../include/delta_robot.h"
#include "../include/stepper_control.h"

/* Forward declarations for submodule initializations */
extern int limit_switch_init(void);
extern void limit_switch_exit(void);
extern void start_motor_motion(int motor_index, struct delta_robot_cmd *cmd);

/* Open callback */
static int delta_robot_open(struct inode *inode, struct file *file)
{
    printk(KERN_ALERT "delta_robot: Device opened\n");
    return 0;
}

static ssize_t delta_robot_write(struct file *file, const char __user *buf,
                                 size_t count, loff_t *ppos)
{
    int num_cmds;
    int *kbuf;
    int i, offset;
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

    kbuf = kmalloc(count, GFP_KERNEL);
    if (!kbuf)
        return -ENOMEM;

    if (copy_from_user(kbuf, buf, count)) {
        kfree(kbuf);
        return -EFAULT;
    }

    printk(KERN_INFO "delta_robot: Received %d motor command(s)\n", num_cmds);

    offset = 0;
    for (i = 0; i < num_cmds; i++) {
        struct delta_robot_cmd cmd;

        // Correctly copy the entire struct
        memcpy(&cmd, &kbuf[offset], sizeof(struct delta_robot_cmd));
        offset += cmd_size / sizeof(int);  // Move offset correctly

        printk(KERN_INFO "delta_robot: Received command for motor_id=%d, total_pulses=%d, target_freq=%d\n",
               cmd.motor_id, cmd.total_pulses, cmd.target_freq);

        if (cmd.motor_id < 0 || cmd.motor_id > 2) {
            printk(KERN_ERR "delta_robot: Invalid motor id %d\n", cmd.motor_id);
            kfree(kbuf);
            return -EINVAL;
        }

        start_motor_motion(cmd.motor_id, &cmd);
    }

    kfree(kbuf);
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
