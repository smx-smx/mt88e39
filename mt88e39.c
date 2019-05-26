/*
	Copyright 2015 Smx
	Linux Kernel driver for MT88E39 Caller ID Decoder
*/

#include <linux/init.h>			// Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>		// Core header for loading LKMs into the kernel
#include <linux/moduleparam.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/device.h>		// Header to support the kernel Driver Model
#include <linux/kernel.h>		// Contains types, macros, functions for the kernel
#include <linux/gpio.h>			// Header for the GPIO Interface
#include <linux/interrupt.h>	// Header for the Interrupts interface
#include <linux/fs.h>			// Header for the Linux file system support
#include <asm/uaccess.h>		// Required for the copy to user function
#include <linux/slab.h>			// kmalloc, kfree
#include <linux/wait.h>			// waitqueue
#include <linux/sched.h>		// TASK_INTERRUPTIBLE

#include "mt88e39.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stefano Moioli");
MODULE_DESCRIPTION("MT88E39 Caller ID Driver");

#define DEVICE_NAME "mt88e39"
#define CLASS_NAME "mt88_gpio"

#define MT_PRINTK(level, fmt, ...) printk(level DEVICE_NAME ": " fmt, ##__VA_ARGS__)

static void mt88e39_state_next(unsigned char);
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static int mt88e39_get_power_state(void);
static void mt88e39_set_power_state(int value);

static ssize_t mt88e_sysfs_power_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t mt88e_sysfs_power_store(struct kobject *, struct kobj_attribute *,  const char *, size_t);
static ssize_t mt88e_sysfs_reset_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t mt88e_sysfs_reset_store(struct kobject *, struct kobj_attribute *,  const char *, size_t);

static int mt88e39_power_state = 1;

static struct file_operations fops =
{
	.open = dev_open,
	.read = dev_read,
	.write = dev_write,
	.release = dev_release
};


static struct gpio pins[] = {
	{2,  GPIOF_IN, "Data Clock"},			//0
	{3,  GPIOF_IN, "Data"},					//1
	{4,  GPIOF_IN, "Data Ready"},			//2
	{17, GPIOF_IN, "Carrier Detect"},		//3
	{27, GPIOF_OUT_INIT_HIGH, "Power Down"}	//4
};

static int pins_irqs[] = {
	-1, //dclk
	-1, //drdy
	-1 //carrier detect
};

static struct kobj_attribute mt88e_power_attr =
	__ATTR(power_control, 0644, mt88e_sysfs_power_show, mt88e_sysfs_power_store);
static struct kobj_attribute mt88e_reset_attr =
	__ATTR(reset, 0644, mt88e_sysfs_reset_show, mt88e_sysfs_reset_store);

static struct attribute *mt88e_pmattrs[] = {
	&mt88e_power_attr.attr,
	&mt88e_reset_attr.attr,
	NULL
};

static struct attribute_group mt88e_pmattr_group = {
	.attrs = mt88e_pmattrs
};

static const struct attribute_group *mt88e_groups[] = {
	&mt88e_pmattr_group,
	NULL
};

static unsigned int 	start = 0;
static unsigned char 	byte = 0;
static unsigned char 	_mType = 0;
static unsigned char 	*buffer = NULL;
static size_t 			bufSize = 0;
static unsigned int 	bufIdx = 0;

static int 				majorNumber;
static struct class*	mt88charClass  = NULL; ///< The device-driver class struct pointer
static struct device*	mt88charDevice = NULL; ///< The device-driver device struct pointer

mt88e_state				state = STS_STBY; /* State machine */

DECLARE_WAIT_QUEUE_HEAD (wq);
static DEFINE_MUTEX(data_mutex);

/***** SYSFS OPS *****/
static ssize_t mt88e_sysfs_power_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
	return sprintf(buf, "%d\n", mt88e39_get_power_state());
}

static ssize_t mt88e_sysfs_power_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t len){
	int value;
	if( sscanf(buf, "%d", &value) != 1 ||
		(value != 0 && value != 1))
	{
		pr_err("%s: Invalid value\n", __func__);
		return -EINVAL;
	}
	mt88e39_set_power_state(value);
	return 0;
}


static ssize_t mt88e_sysfs_reset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
	return 0;
}

static ssize_t mt88e_sysfs_reset_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t len){
	int value;
	if( sscanf(buf, "%d", &value) != 1 || value != 1)
	{
		pr_err("%s: Invalid value\n", __func__);
		return -EINVAL;
	}
	state = STS_STBY;
	byte = 0;
	start = 0;
	if(buffer){
		kfree(buffer);
		buffer = NULL;
	}
	bufIdx = 0;
	bufSize = 0;
	return 0;
}


static int mt88e39_get_power_state(void){
	return mt88e39_power_state;
}

static void mt88e39_set_power_state(int value){
	if(value < 0 || value > 1){
		pr_err("%s: Invalid value\n", __func__);
		return;
	}
	gpio_set_value(pins[4].gpio, value);
	mt88e39_power_state = value;
	return;
}

static void mt88e39_state_next(unsigned char byte){
	switch(state){
		case STS_STBY:
			if(byte == TYPE_MDMF || byte == TYPE_SDMF){
				_mType = byte;
				state = STS_MLEN;
			}
			break;
		case STS_MLEN:
			if(byte < 2 || byte + 3 > MAX_LENGTH){
				MT_PRINTK(KERN_ERR, "Invalid message length (%u) received!\n", (unsigned int)byte);
				state = STS_STBY;
				break;
			} else {
				if(buffer != NULL){
					MT_PRINTK(KERN_WARNING, "Freeing the buffer (dropping calldata)\n");
					kfree(buffer);
					buffer = NULL;
				}
				buffer = kzalloc((size_t)(byte + 3), GFP_KERNEL | GFP_ATOMIC);
				if(!buffer){
					MT_PRINTK(KERN_ERR, "Kmalloc failed!\n");
					state = STS_STBY;
					break;
				} else {
					bufIdx = 0;
					buffer[bufIdx++] = _mType; //0-1
					buffer[bufIdx++] = byte; //1-2
					bufSize = byte + 3; //_mType, checksum added
					MT_PRINTK(KERN_INFO, "Kmalloc OK\n");
				}
			}
			state = STS_DONE;
			break;
		case STS_DONE:
		default:
			MT_PRINTK(KERN_ERR, "Unexpected State %d!\n", state);
			break;
	}
}

static unsigned char mt88_calc_checksum(void){
	unsigned char checksum = 0x00;
	int i;
	for(i=0; i<bufSize - 1; i++){
		checksum += buffer[i];
	}
	checksum = (~checksum + 1) % (MAX_LENGTH + 1);
	return checksum;
}

/***** INTERRUPT SERVICE ROUTINES *****/
static irqreturn_t dclk_isr(int irq, void *data){
	int bit = 0;
	if(!start){
		return IRQ_HANDLED;
	}
	bit = gpio_get_value(pins[1].gpio);
	//buffer[nbyte] = (buffer[nbyte] >> 1) | ((bit & 1) << 7);
	byte = (byte >> 1) | ((bit & 1) << 7);
	return IRQ_HANDLED;
}

static irqreturn_t cd_isr(int irq, void *data){
	MT_PRINTK(KERN_INFO, "INBOUND CALL DETECTED\n");
	state = STS_STBY;
	//Erase the byte
	byte = 0;
	//Set start flag
	start = 1;
	return IRQ_HANDLED;
}

static irqreturn_t drdy_isr(int irq, void *data){
	if(state != STS_DONE){
		mt88e39_state_next(byte);
	} else {
		uint8_t checksum;
		if((checksum = mt88_calc_checksum()) != buffer[bufSize - 1]){
			MT_PRINTK(KERN_ERR, "Checksum mismatch (calculated 0x%x, expected 0x%x!, dropping packet!\n",
				checksum, buffer[bufSize - 1]);
		}
		buffer[bufIdx++] = byte;
		if (bufIdx + 1 >= bufSize){
			MT_PRINTK(KERN_INFO, "Done, written %u bytes!\n", bufIdx);
			wake_up_interruptible(&wq);
		}
		/*} else {
			MT_PRINTK(KERN_INFO, "Done, written %u bytes!\n", bufIdx + 1);
			wake_up_interruptible(&wq);
		}*/
	}
	byte = 0;
	return IRQ_HANDLED;
}

/***** FILE OPERATIONS *****/
static ssize_t dev_open(struct inode *inodep, struct file *filep){
	return 0;
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
	return 0;
}

static int dev_release(struct inode *inodep, struct file *filep){
	return 0;
}


static ssize_t dev_read(struct file *filep, char *user_buffer, size_t count, loff_t *offset){
	ssize_t result 	= 0;
	int error_count = 0;
	//size_t len		= 0;
	char *startPtr	= NULL;

	if(wait_event_interruptible(wq, (buffer != NULL && bufIdx >= bufSize && state == STS_DONE))){
		return -ERESTARTSYS;
	}

	mutex_lock(&data_mutex);

	if(*offset > bufSize){
		goto out;
	}

	startPtr = buffer + *offset;
	result = bufSize - *offset; //Data to read
	if(result > count){
		result = count;
	}

	MT_PRINTK(KERN_INFO, "Start copying %u bytes\n", result);

	// copy_to_user has the format ( * to, *from, size) and returns 0 on success
	/*len = result;
	while(len > 0){
		MT_PRINTK(KERN_INFO, "%x\n", *(startPtr));
		put_user(*(startPtr++), user_buffer++);
		len--;
	}
	MT_PRINTK(KERN_INFO, "\n");
	MT_PRINTK(KERN_INFO, "Done copying %u bytes\n", len);
	if(result == bufSize){
		MT_PRINTK(KERN_INFO, "Setting stop flag...\n");
		state = STS_STBY;
	}*/
	error_count = copy_to_user(user_buffer, startPtr, result);
	if (!error_count){ // if true then have success
		MT_PRINTK(KERN_INFO, "Sent %d characters to the user\n", result);
		if(result == bufSize){
			MT_PRINTK(KERN_INFO, "Setting stop flag...\n");
			state = STS_STBY;
		}
	}
	else {
		MT_PRINTK(KERN_INFO, "Failed to send %d characters to the user\n", error_count);
		result = -EFAULT; // Failed -- return a bad adgdress message (i.e. -14)
	}
	out:
		mutex_unlock(&data_mutex);
		return result;
}

static int __init mt88e39_init(void){
	int result = EXIT_FAILURE, ret = 1;
	MT_PRINTK(KERN_INFO, "%s\n", __func__);

	/* Step 1: Acquire GPIO pins */
	ret = gpio_request_array(pins, ARRAY_SIZE(pins));
	if(ret){
		MT_PRINTK(KERN_ERR, "Unable to acquire GPIOs: %d\n", ret);
		goto exit_e0;
	}

	/* Step 2: Allocate GPIO IRQs */
	ret = gpio_to_irq(pins[0].gpio);
	if(ret < 0){
		MT_PRINTK(KERN_ERR, "Unable to acquire \"DATA CLOCK\" IRQ: %d\n", ret);
		goto exit_e1;
	}
	pins_irqs[0] = ret;

	ret = gpio_to_irq(pins[2].gpio);
	if(ret < 0){
		MT_PRINTK(KERN_ERR, "Unable to acquire \"DATA READY\" IRQ: %d\n", ret);
		goto exit_e1;
	}
	pins_irqs[1] = ret;

	ret = gpio_to_irq(pins[3].gpio);
	if(ret < 0){
		MT_PRINTK(KERN_ERR, "Unable to acquire \"CARRIER DETECT\" IRQ: %d\n", ret);
		goto exit_e1;
	}
	pins_irqs[2] = ret;


	/* Step 3: Request GPIO IRQs */
	ret = request_irq(pins_irqs[0], dclk_isr, IRQF_TRIGGER_RISING, "mt88e39#dclk", NULL);
	if(ret){
		MT_PRINTK(KERN_ERR, "Unable to request \"DATA CLOCK\" IRQ: %d\n", ret);
		goto exit_e1;
	}

	ret = request_irq(pins_irqs[1], drdy_isr, IRQF_TRIGGER_FALLING, "mt88e39#drdy", NULL);
	if(ret){
		MT_PRINTK(KERN_ERR, "Unable to request \"DATA READY\" IRQ: %d\n", ret);
		goto exit_e2;
	}

	ret = request_irq(pins_irqs[2], cd_isr, IRQF_TRIGGER_FALLING, "mt88e39#cd", NULL);
	if(ret){
		MT_PRINTK(KERN_ERR, "Unable to request \"CARRIER DETECT\" IRQ: %d\n", ret);
		goto exit_e3;
	}

	// Try to dynamically allocate a major number for the device -- more difficult but worth it
	majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
	if (majorNumber<0){
		MT_PRINTK(KERN_ERR, "Failed to register a major number\n");
		goto exit_e4;
	}
	MT_PRINTK(KERN_INFO, "Registered correctly with major number %d\n", majorNumber);

	// Register the device class
	mt88charClass = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(mt88charClass)){	//Check for error and clean up if there is
		MT_PRINTK(KERN_ERR, "Failed to register device class\n");
		result = PTR_ERR(mt88charClass);
		goto exit_e5;
	}
	MT_PRINTK(KERN_INFO, "Device class registered correctly\n");

	// Register the device driver
	mt88charDevice = device_create_with_groups(mt88charClass, NULL, MKDEV(majorNumber, 0), NULL, mt88e_groups, DEVICE_NAME);
	if (IS_ERR(mt88charDevice)){ // Clean up if there is an error
		MT_PRINTK(KERN_ERR, "Failed to create the device\n");
		goto exit_e6;
	}

	MT_PRINTK(KERN_INFO, "Device class created correctly\n"); // Made it! device was initialized
	init_waitqueue_head (&wq);

	result = EXIT_SUCCESS;
	goto exit_e0;

	exit_e6:
		class_destroy(mt88charClass);
	exit_e5:
		unregister_chrdev(majorNumber, DEVICE_NAME);
	exit_e4:
		free_irq(pins_irqs[2], NULL);
	exit_e3:
		free_irq(pins_irqs[1], NULL);
	exit_e2:
		free_irq(pins_irqs[0], NULL);
	exit_e1:
		gpio_free_array(pins, ARRAY_SIZE(pins));
	exit_e0:
		return result;
}

static void __exit mt88e39_exit(void){
	MT_PRINTK(KERN_INFO, "%s\n", __func__);
	if(buffer != NULL){
		MT_PRINTK(KERN_INFO, "Freeing Buffer...\n");
		kfree(buffer);
	}

	MT_PRINTK(KERN_INFO, "Releasing chardev...\n");
	device_destroy(mt88charClass, MKDEV(majorNumber, 0));
	class_unregister(mt88charClass);
	class_destroy(mt88charClass);
	unregister_chrdev(majorNumber, DEVICE_NAME);

	MT_PRINTK(KERN_INFO, "Releasing Interrupts...\n");
	free_irq(pins_irqs[0], NULL);
	free_irq(pins_irqs[1], NULL);
	free_irq(pins_irqs[2], NULL);

	MT_PRINTK(KERN_INFO, "Releasing GPIOs...\n");
	gpio_free_array(pins, ARRAY_SIZE(pins));
}

module_init(mt88e39_init);
module_exit(mt88e39_exit);
