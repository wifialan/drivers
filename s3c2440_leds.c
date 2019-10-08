
/*
 *  Driver for S3C2440 base board.
 *
 *  Copyright (C) 2019  Alan NWPU <alantian.at@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *  MULTIBEANS, NPU Youyi West Ave, Beilin District, Xi'an, China.
 */

#include <linux/module.h>   /* Every Linux kernel module must include this head */
#include <linux/init.h>     /* Every Linux kernel module must include this head */
#include <linux/kernel.h>   /* printk() */
#include <linux/fs.h>       /* struct fops */
#include <linux/errno.h>    /* error codes */
#include <linux/cdev.h>     /* cdev_alloc()  */
#include <linux/ioport.h>   /* request_mem_region() */
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <asm/irq.h>
#include <mach/gpio-nrs.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>

#define DRV_NAME "s3c2440_leds"
#define DRV_AUTHOR "Alan Tian <alantian.at@gmail.com>"
#define DRV_DESC "S3C2440 LED Pin Driver"

#define S3C2440_LED_SIZE 0x1000

#define S3C2440_LED_MAJOR 230
#define S3C2440_LED_MINOR 0

static int major = S3C2440_LED_MAJOR;
static int minor = S3C2440_LED_MINOR;

/* 应用程序执行ioctl(fd, cmd, arg)时的第2个参数 */
#define IOCTL_LED_ON    0
#define IOCTL_LED_OFF   1

static int s3c2440_leds_open( struct inode *inodes, struct file *filp );
static long s3c2440_leds_ioctl( struct file  *file, unsigned int cmd, unsigned long arg );
static ssize_t s3c2440_leds_write( struct file *filp, const char __user *buffer, \
                                     size_t size, loff_t *f_pos );
static ssize_t s3c2440_leds_read( struct file *filp, const char __user *buffer, \
                                     size_t size, loff_t *f_pos );

struct s3c2440_leds_dev_t
{
    struct cdev cdev;
    unsigned char mem[S3C2440_LED_SIZE];
} *s3c2440_leds_dev;

//Step 2: Add file operations
static const struct file_operations s3c2440_leds_fileops = {
    .owner  = THIS_MODULE,
    .write  = s3c2440_leds_write,
    .read   = s3c2440_leds_read,
    .open   = s3c2440_leds_open,
    .unlocked_ioctl  = s3c2440_leds_ioctl,
};

static int s3c2440_leds_open( struct inode *inodes, struct file *filp )
{
    //int ret;

    filp->private_data = s3c2440_leds_dev;
    printk(DRV_NAME"\tS3C2440 open function...\n");

    return 0;
}

static long s3c2440_leds_ioctl( struct file  *file, unsigned int cmd, unsigned long arg )
{
    printk(DRV_NAME "\tRecv cmd: %u\n", cmd);
    printk(DRV_NAME "\tRecv arg: %lu\n", arg);
    //IO operations function.
    if(arg > 4) {
        return -EINVAL;
    }

    switch (cmd) {
        case IOCTL_LED_ON:
            s3c2410_gpio_setpin(S3C2410_GPF(arg+3), 0);//Set pin
            printk("Open LED %lu ",arg);
        return 0;

        case IOCTL_LED_OFF:
            s3c2410_gpio_setpin(S3C2410_GPF(arg+3), 1);
            printk("Close LED %lu ",arg);
        return 0;

        default:
            return -EINVAL;
    }
}


static ssize_t s3c2440_leds_write( struct file *filp, const char __user *buffer, \
                                     size_t size, loff_t *f_pos )
{
    unsigned long p = *f_pos;
    unsigned int count = size;
    int ret = 0;
    struct s3c2440_leds_dev_t *dev = filp->private_data;

    if(p >= S3C2440_LED_SIZE)
        return 0;
    if(count > S3C2440_LED_SIZE - p)
        count = S3C2440_LED_SIZE - p;

    memset(dev->mem, 0, S3C2440_LED_SIZE);

    if(copy_from_user(dev->mem + p, buffer, count)) {
        ret = -EFAULT;
    } else {
        *f_pos += count;
        ret = count;
        printk(KERN_INFO "writter %u bytes(s) from %lu\n", count, p);
    }

    return ret;
    
}

static ssize_t s3c2440_leds_read( struct file *filp, const char __user *buffer, \
                                     size_t size, loff_t *f_pos )
{
    unsigned long p = *f_pos;
    unsigned int count = size;
    int ret = 0;
    struct s3c2440_leds_dev_t *dev = filp->private_data;
    
    if(p >= S3C2440_LED_SIZE)
        return 0;
    if(count > S3C2440_LED_SIZE - p)
        count = S3C2440_LED_SIZE - p;
    if(copy_to_user(buffer, dev->mem + p, count)) {
        ret = -EFAULT;
    } else {
        *f_pos += count;
        ret = count;
        printk(KERN_INFO "read %u bytes(s) from %lu\n", count, p);
    }

    return ret;
}

static int __init s3c2440_leds_init(void)
{
    int ret,err;
    dev_t devid;
    
    if(major) {
        devid = MKDEV(major, 0);
        ret = register_chrdev_region(devid, 1, DRV_NAME);
        printk("Origin Creat node %d\n",major);
    } else {
        ret = alloc_chrdev_region(&devid, 0, 1, DRV_NAME);
        major = MAJOR(devid);
        printk("Arrage1 Creat node %d\n",major);
    }
    if(ret < 0) {
        printk(DRV_NAME "\ts3c2440 new device failed\n");
        //goto fail_malloc;
        return ret;
    }
    s3c2440_leds_dev = kzalloc(sizeof(struct s3c2440_leds_dev_t), GFP_KERNEL);
    if(!s3c2440_leds_dev) {
        ret = -ENOMEM;
        goto fail_malloc;
    }
    printk("success init leds\n");
    
    cdev_init(&s3c2440_leds_dev->cdev, &s3c2440_leds_fileops);
    err = cdev_add(&s3c2440_leds_dev->cdev, devid, 1);
    if(err)
        printk(KERN_NOTICE "Error %d adding s2c2440_leds %d",err, 1);
    return 0;
    
fail_malloc:
    printk("Fail malloc\n");
    unregister_chrdev_region(devid, 1);
    return ret;
}

static void __exit s3c2440_leds_exit(void)
{
    printk("Starting delet node %d\n",major);
    cdev_del(&s3c2440_leds_dev->cdev);
    kfree(s3c2440_leds_dev);
    unregister_chrdev_region(MKDEV(major, minor), 1);
    printk("Delete node %d\n",major);
}

module_init(s3c2440_leds_init);
module_exit(s3c2440_leds_exit);

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");


