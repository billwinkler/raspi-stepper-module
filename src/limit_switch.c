#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include "../include/delta_robot.h"
#include "../include/delta_robot_config.h" 
#include "../include/stepper_control.h"

/* Interrupt numbers for the limit switches; these would be determined via GPIO mappings */
static int irq_limit1, irq_limit2, irq_limit3;

/* ISR for limit switch 1 */
static irqreturn_t limit_switch1_isr(int irq, void *dev_id)
{
    printk(KERN_INFO "limit_switch1_isr: Limit switch 1 triggered (IRQ: %d)\n", irq);
    /* Signal the stepper control: set the abort flag for motor 0 */
    motor_states[0].abort = true;
    return IRQ_HANDLED;
}

static irqreturn_t limit_switch2_isr(int irq, void *dev_id)
{
    printk(KERN_INFO "limit_switch2_isr: Limit switch 2 triggered (IRQ: %d)\n", irq);
    /* Signal the stepper control: set the abort flag for motor 1 */
    motor_states[1].abort = true;
    return IRQ_HANDLED;
}

static irqreturn_t limit_switch3_isr(int irq, void *dev_id)
{
    printk(KERN_INFO "limit_switch3_isr: Limit switch 3 triggered (IRQ: %d)\n", irq);
    /* Signal the stepper control: set the abort flag for motor 2 */
    motor_states[2].abort = true;
    return IRQ_HANDLED;
}

/* Initialization function: configure GPIOs and request IRQs */
int limit_switch_init(void)
{
    int ret;

    /* Setup limit switch 1 */
    ret = gpio_request(CONFIG_LIMIT_SWITCH1_PIN, "limit_switch1");
    if (ret) {
        printk(KERN_ERR "Failed to request GPIO for limit switch 1: %d\n", ret);
        return ret;
    }
    gpio_direction_input(CONFIG_LIMIT_SWITCH1_PIN);
    irq_limit1 = gpio_to_irq(CONFIG_LIMIT_SWITCH1_PIN);
    if (irq_limit1 < 0) {
        printk(KERN_ERR "Failed to get IRQ for limit switch 1: %d\n", irq_limit1);
        gpio_free(CONFIG_LIMIT_SWITCH1_PIN);
        return irq_limit1;
    }
    ret = request_irq(irq_limit1, limit_switch1_isr, IRQF_TRIGGER_FALLING,
                      "limit_switch1", NULL);
    if (ret) {
        printk(KERN_ERR "Failed to request IRQ for limit switch 1: %d\n", ret);
        gpio_free(CONFIG_LIMIT_SWITCH1_PIN);
        return ret;
    }

    /* Setup limit switch 2 */
    ret = gpio_request(CONFIG_LIMIT_SWITCH2_PIN, "limit_switch2");
    if (ret) {
        printk(KERN_ERR "Failed to request GPIO for limit switch 2: %d\n", ret);
        return ret;
    }
    gpio_direction_input(CONFIG_LIMIT_SWITCH2_PIN);
    irq_limit2 = gpio_to_irq(CONFIG_LIMIT_SWITCH2_PIN);
    if (irq_limit2 < 0) {
        printk(KERN_ERR "Failed to get IRQ for limit switch 2: %d\n", irq_limit2);
        gpio_free(CONFIG_LIMIT_SWITCH2_PIN);
        return irq_limit2;
    }
    ret = request_irq(irq_limit2, limit_switch2_isr, IRQF_TRIGGER_FALLING,
                      "limit_switch2", NULL);
    if (ret) {
        printk(KERN_ERR "Failed to request IRQ for limit switch 2: %d\n", ret);
        gpio_free(CONFIG_LIMIT_SWITCH2_PIN);
        return ret;
    }

    /* Setup limit switch 3 */
    ret = gpio_request(CONFIG_LIMIT_SWITCH3_PIN, "limit_switch3");
    if (ret) {
        printk(KERN_ERR "Failed to request GPIO for limit switch 3: %d\n", ret);
        return ret;
    }
    gpio_direction_input(CONFIG_LIMIT_SWITCH3_PIN);
    irq_limit3 = gpio_to_irq(CONFIG_LIMIT_SWITCH3_PIN);
    if (irq_limit3 < 0) {
        printk(KERN_ERR "Failed to get IRQ for limit switch 3: %d\n", irq_limit3);
        gpio_free(CONFIG_LIMIT_SWITCH3_PIN);
        return irq_limit3;
    }
    ret = request_irq(irq_limit3, limit_switch3_isr, IRQF_TRIGGER_FALLING,
                      "limit_switch3", NULL);
    if (ret) {
        printk(KERN_ERR "Failed to request IRQ for limit switch 3: %d\n", ret);
        gpio_free(CONFIG_LIMIT_SWITCH3_PIN);
        return ret;
    }

    printk(KERN_INFO "Limit switches initialized successfully.\n");
    return 0;
}

void limit_switch_exit(void)
{
    free_irq(irq_limit1, NULL);
    gpio_free(CONFIG_LIMIT_SWITCH1_PIN);

    free_irq(irq_limit2, NULL);
    gpio_free(CONFIG_LIMIT_SWITCH2_PIN);

    free_irq(irq_limit3, NULL);
    gpio_free(CONFIG_LIMIT_SWITCH3_PIN);

    printk(KERN_INFO "Limit switches cleaned up.\n");
}

EXPORT_SYMBOL(limit_switch_init);
EXPORT_SYMBOL(limit_switch_exit);
