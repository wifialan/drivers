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
 *  MULTIBEANS, NPU Youyi West Ave, Beilin District, Xi'an, China.*/

#include <linux/module.h>   /* Every Linux kernel module must include this head */
#include <linux/init.h>     /* Every Linux kernel module must include this head */
#include <linux/kernel.h>   /* printk() */
#include <linux/fs.h>       /* struct fops */
#include <linux/errno.h>    /* error codes */
#include <linux/cdev.h>     /* cdev_alloc()  */
#include <linux/ioport.h>   /* request_mem_region() */
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/spi/spi.h>

#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/device.h>
//#include <mach/spi.h>
#include <asm/irq.h>
#include <mach/gpio-nrs.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <linux/spi/spi.h>


#define DRV_NAME "s3c2440_w25q16"
#define DRV_AUTHOR "Alan Tian <alantian.at@gmail.com>"
#define DRV_DESC "S3C2440 W25Q16 Driver"

#define S3C2440_W25Q16_SIZE 0x1000

#define S3C2440_W25Q16_MAJOR 0
#define S3C2440_W25Q16_MINOR 0

static int major = S3C2440_W25Q16_MAJOR;
static int minor = S3C2440_W25Q16_MINOR;

static __devinit w25q16_bus_spi_probe(struct spi_device *spi);
static int w25q16_bus_spi_remove(struct spi_device *spi);

static int w25q16_open( struct inode *inodes, struct file *filp );
static long w25q16_ioctl( struct file  *file, unsigned int cmd, unsigned long arg );
static ssize_t w25q16_write( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos );
static ssize_t w25q16_read( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos );

struct spi_master *spi_master_p;
struct spi_device *spi_w25q16_pdev;
struct spi_message *spi_mesg_p;
static struct class *class;

struct w25q16_dev_t {
    struct cdev cdev;
    unsigned char mem[S3C2440_W25Q16_SIZE];
} *w25q16_pdev;

static struct file_operations w25q16_ops = 
{
    .owner      = THIS_MODULE,
    .open       = w25q16_open,
    .read       = w25q16_read,
    .write      = w25q16_write,
    .unlocked_ioctl = w25q16_ioctl,
};
    
static int w25q16_open( struct inode *inodes, struct file *filp )
{
    return 0;
}
static long w25q16_ioctl( struct file  *file, unsigned int cmd, unsigned long arg )
{
    return 0;
}
static ssize_t w25q16_write( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos )
{
    return 0;
}
static ssize_t w25q16_read( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos )
{
    return 0;
}

struct spi_board_info w25q16_board_info[] = 
{
    {
    .modalias   = "w25q16",
    .platform_data  = NULL,
    .max_speed_hz   = 10*1000*1000,
    .bus_num        = 0,
    .chip_select    = S3C2410_GPG(2),
    .mode           = SPI_MODE_3,
//    .controller_data= &w25q16_csi,
    }
};


static const struct spi_device_id w25q16_spi_id[] =
{
    {DRV_NAME,1},
    {},
};

static struct spi_driver w25q16_spi_driver =
{
    .driver     =
    {
        .name   = "w25q16",
        .owner  = THIS_MODULE,
    },
    .probe      = w25q16_bus_spi_probe,
    .remove     = __devexit_p(w25q16_bus_spi_remove),
};

static struct spi_device w25q16_spi_device =
{
    .modalias       = "w25q16",
    .mode           = SPI_MODE_3,
    .bits_per_word  = 16,
    .chip_select    = SPI_CS_HIGH,
    .max_speed_hz   = 100000,
    .dev            = {
       // .controller_data = &w25q16_board_info;
    }
};


static int __devinit w25q16_bus_spi_probe(struct spi_device *spi)
{ 
    int ret,err;
    dev_t devid;
    spi_w25q16_pdev = spi;
    s3c2410_gpio_cfgpin(spi->chip_select, S3C2410_GPIO_OUTPUT);

    if(major) {
       devid = MKDEV(major, 0);
       ret = register_chrdev_region(devid, 1, DRV_NAME);
       printk(DRV_NAME "\tOrigin Creat node %d\n",major);
    } else {
        ret = alloc_chrdev_region(&devid, 0, 1, DRV_NAME);
        major = MAJOR(devid);
        printk(DRV_NAME "\tArrage Creat node %d\n",major);
    }
    if(ret < 0) {
        printk(DRV_NAME "\tnew device failed\n");
        //goto fail_malloc;
        return ret;
    }
    
    w25q16_pdev = kzalloc(sizeof(struct w25q16_dev_t), GFP_KERNEL);
    if(!w25q16_pdev) {
       ret = -ENOMEM;
       goto fail_malloc;
    }
    cdev_init(&w25q16_pdev->cdev, &w25q16_ops);
    err = cdev_add(&w25q16_pdev->cdev, devid, 1);
    if(err)
        printk(DRV_NAME "\tError %d adding w25q16 %d\n",err, 1);

    class = class_create(THIS_MODULE, "w25q16");
    device_create(class, NULL, MKDEV(major, minor), NULL, "w25q16");
    printk(DRV_NAME "\tcreat device node /dev/w25q16 \n");

    return 0;

fail_malloc:
    printk("Failed to allocate memory!\n");
    return ret;

}

static int w25q16_bus_spi_remove(struct spi_device *spi)
{
    device_destroy(class, MKDEV(major, 0));
    class_destroy(class);
    cdev_del(&w25q16_pdev->cdev);
    kfree(w25q16_pdev);
    unregister_chrdev_region(MKDEV(major, minor), 1);
    return 0;
}

MODULE_DEVICE_TABLE(spi, w25q16_spi_id);

#if 0
static struct spi_master w25q16_spi_master = 
{
    .num_chipselect = 4;
}
#endif


#if 1

#if 0
static struct miscdevice w25q16_miscdev =
{
    .name       = DRV_NAME,
    .fops       = &w25q16_ops,
    .nodename   = DRV_NAME,
};
#endif
#endif

static int __init w25q16_driver_init(void)
{
    int ret;
    printk("\n************ driver init begin ************\n\n");
    ret = spi_register_driver(&w25q16_spi_driver);
    if(ret)
    {
        spi_unregister_driver(&w25q16_spi_driver);
        printk(DRV_NAME "\tFailed register spi driver. Error: %d\n",ret);
    }
    printk("\n************* driver init end *************\n\n");
    return ret;
}

static void __exit w25q16_driver_exit(void)
{
    printk("Unregister w25q16 devices!\n");
//    misc_deregister(&w25q16_miscdev);
    spi_unregister_driver(&w25q16_spi_driver);
    spi_unregister_device(spi_w25q16_pdev);
    cdev_del(&w25q16_pdev->cdev);
    kfree(w25q16_pdev);
    unregister_chrdev_region(MKDEV(major, minor), 1);
    printk("Unregister w25q16 devices!\n");
}


module_init(w25q16_driver_init);
module_exit(w25q16_driver_exit);

MODULE_AUTHOR( DRV_AUTHOR );
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");



