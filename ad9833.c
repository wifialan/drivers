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
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>

#include <asm/irq.h>
#include <mach/gpio-nrs.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>

#define DRV_NAME "ad9833-ADI"
#define DRV_AUTHOR "Alan Tian <alantian.at@gmail.com>"
#define DRV_DESC "AD9833-ADI Driver"

#define AD9833_SIZE 0x1000
#define AD9833_MAJOR 231
#define AD9833_MINOR 0

#define AD9833_REG_RESET 0x0100
#define AD9833_FREQ0_REG 0
#define AD9833_FREQ1_REG 1

#define AD9833_MAGIC 'k'
#define CMD_PARA_FREQ 0x10
#define CMD_PARA_PHASE 0x11
#define CMD_PARA_TYPE 0x12

//#define CMD_TYPE_SIN _IO( AD9833_MAGIC, 0)
//#define CMD_TYPE_TRI _IO( AD9833_MAGIC, 1)
//#define CMD_TYPE_SQE _IO( AD9833_MAGIC, 2)
#define CMD_TYPE_SIN 0
#define CMD_TYPE_TRI 1
#define CMD_TYPE_SQE 3

#define CMD_FREQ_SET(X) _IO( CMD_PARA_FREQ, X)
#define CMD_PHASE_SET(X) _IO( CMD_PARA_PHASE, X )
#define CMD_TYPE_SET(X) _IO( CMD_PARA_TYPE,X )


#define AD9833_FSY_IO S3C2410_GPG(2)
#define AD9833_CLK_IO S3C2410_GPG(7)
#define AD9833_DAT_IO S3C2410_GPG(6)
#define AD9833_SIN_INT_IO S3C2410_GPF(0)
#define AD9833_TRI_INT_IO S3C2410_GPF(2)
#define AD9833_SQU_INT_IO S3C2410_GPG(3)


#define io_fsy(x) s3c2410_gpio_setpin(AD9833_FSY_IO,(x))
#define io_clk(x) s3c2410_gpio_setpin(AD9833_CLK_IO,(x))
#define io_dat(x) s3c2410_gpio_setpin(AD9833_DAT_IO,(x))

#define IO_HIGH 1
#define IO_LOW 0

static int major = AD9833_MAJOR;
static int minor = AD9833_MINOR;

enum ad9833_wave_type_t {
    SIN,TRI,SQU
};

typedef struct ad9833_dev_t {

    struct ad9833_dev_t *self;
    enum ad9833_wave_type_t wave_type;

    struct cdev cdev;
    unsigned char mem[AD9833_SIZE];

    void (*write_reg)   ( struct ad9833_dev_t *self, unsigned int reg_value);
    void (*init_device) ( struct ad9833_dev_t *self );
    void (*set_wave_freq)( struct ad9833_dev_t *self , unsigned long freqs_data);
    void (*set_wave_type)( struct ad9833_dev_t *self, enum ad9833_wave_type_t wave_type );
    void (*set_wave_phase)( struct ad9833_dev_t *self, unsigned int phase );
    void (*set_wave_para)( struct ad9833_dev_t *self, unsigned long freqs_data, unsigned int phase, enum ad9833_wave_type_t wave_type );
    
}AD9833;

AD9833 *ad9833;

static int ad9833_open( struct inode *inodes, struct file *filp );
static long ad9833_ioctl( struct file  *file, unsigned int cmd, unsigned long arg );
static void ad9833_set_wave_type( AD9833 *dev, enum ad9833_wave_type_t wave_type );
static void ad9833_set_phase( AD9833 *dev, unsigned int phase_value );
static void ad9833_set_freq( AD9833 *dev, unsigned long freq );
static void ad9833_set_para( AD9833 *dev, unsigned long freqs_value, unsigned int phase_value, enum ad9833_wave_type_t wave_type );
static void ad9833_init_device( AD9833 *dev );
static void ad9833_write_reg( AD9833 *dev, unsigned int reg_value );

static struct gpio_irq_desc {

    int irq_sin;
    int irq_tri;
    int irq_squ;
    unsigned long flags;
    char *name_1;
    char *name_2;
    char *name_3;
    int ret_sin;
    int ret_tri;
    int ret_squ;
} ad9833_irq_desc = {
    IRQ_S3C2440_AC97,IRQ_S3C2440_AC97,IRQ_S3C2440_AC97,//
    IRQ_TYPE_EDGE_FALLING,
    "push_button_sin",
    "push_button_tri",
    "push_button_squ",
    0,0,0
};

AD9833 *ad9833_dev_new(void)
{
    AD9833 *dev = (AD9833*)kzalloc(sizeof(AD9833),GFP_ATOMIC);

    //dev->hw.fsy           =   AD9833_FSY_IO;
    //dev->hw.sdi           =   AD9833_DAT_IO;
    //dev->hw.clk           =   AD9833_CLK_IO;

    dev->set_wave_para    =   &ad9833_set_para;
    dev->init_device      =   &ad9833_init_device;
    dev->write_reg        =   &ad9833_write_reg;
    dev->set_wave_freq    =   &ad9833_set_freq;
    dev->set_wave_phase   =   &ad9833_set_phase;
    dev->set_wave_type    =   &ad9833_set_wave_type;
    dev->init_device( dev );

    return dev;
}

static void ad9833_write_reg(AD9833 *dev,
                             unsigned int reg_value )
{
    unsigned short i;

    io_clk(IO_HIGH);
    io_fsy(IO_HIGH);
    udelay(10);
    io_fsy(IO_LOW);

    for ( i = 0; i < 16; i++ ) {

        if ( reg_value & 0x8000 )
            io_dat(IO_HIGH);
        else
            io_dat(IO_LOW);
        ndelay(10);
        io_clk(IO_LOW);
        ndelay(10);
        io_clk(IO_HIGH);
        reg_value = reg_value << 1;
    }
    io_fsy(IO_HIGH);
}

static void ad9833_init_device( AD9833 *dev )
{
    dev->write_reg( dev, AD9833_REG_RESET );
    dev->set_wave_para( dev,1500, 0 ,SIN );
    s3c2410_gpio_cfgpin(AD9833_FSY_IO, S3C2410_GPIO_OUTPUT);
    s3c2410_gpio_cfgpin(AD9833_CLK_IO, S3C2410_GPIO_OUTPUT);
    s3c2410_gpio_cfgpin(AD9833_DAT_IO, S3C2410_GPIO_OUTPUT);
    s3c2410_gpio_cfgpin(AD9833_SIN_INT_IO, S3C2410_GPIO_INPUT);
    s3c2410_gpio_cfgpin(AD9833_TRI_INT_IO, S3C2410_GPIO_INPUT);
    s3c2410_gpio_cfgpin(AD9833_SQU_INT_IO, S3C2410_GPIO_INPUT);
}


static void ad9833_set_para( AD9833 *dev, \
                            unsigned long freqs_value, unsigned int phase_value, \
                            enum ad9833_wave_type_t wave_type )
{
    unsigned long dds_frequence_data;
    unsigned int dds_frequence_low;
    unsigned int dds_frequence_high;
    unsigned int phase_data;
    phase_data      =   phase_value | 0xC000;

    dds_frequence_data      =   freqs_value * 10;
    dds_frequence_low       =   dds_frequence_data & 0x3FFF;
    dds_frequence_low       |=  0x4000;
    dds_frequence_data      =   dds_frequence_data >> 14;
    dds_frequence_high      =   dds_frequence_data & 0x3FFF;
    dds_frequence_high      |=  0x4000;
    //  reset device
    dev->write_reg( dev, 0x0110 );
    dev->write_reg( dev, 0x2100 );

    dev->write_reg( dev,dds_frequence_low );
    dev->write_reg( dev,dds_frequence_high );
    dev->write_reg( dev, phase_data );

    if( wave_type == TRI ) {
        dev->write_reg( dev, 0x2002 );
    }else if( wave_type == SQU ) {
        dev->write_reg(  dev, 0x2028);
    }else {
        dev->write_reg( dev, 0x2000 );
    }
}

static void
ad9833_set_freq( AD9833 *dev, unsigned long freq )
{
    unsigned long dds_frequence_data;
    unsigned long dds_frequence_low;
    unsigned long dds_frequence_high;

    dds_frequence_data      =   freq;
    dds_frequence_low       =   dds_frequence_data & 0x3FFF;
    dds_frequence_low       |=  0x4000;
    dds_frequence_data      =   dds_frequence_data >> 14;
    dds_frequence_high      =   dds_frequence_data & 0x3FFF;
    dds_frequence_high      |=  0x4000;

    dev->write_reg( dev, dds_frequence_low );
    dev->write_reg( dev, dds_frequence_high );

}

static void
ad9833_set_phase( AD9833 *dev, unsigned int phase_value )
{
    unsigned int phase_temp;
    phase_temp = phase_value | 0xC000;
    dev->write_reg( dev, phase_temp );
}

static void
ad9833_set_wave_type( AD9833 *dev, enum ad9833_wave_type_t wave_type )
{
    if( wave_type == TRI ) {
        dev->write_reg( dev, 0x2002 );
    }else if( wave_type == SQU ) {
        dev->write_reg(  dev, 0x2028);
    }else {
        dev->write_reg( dev, 0x2000 );
    }
}

static const struct file_operations ad9833_fileops = {
    .owner          = THIS_MODULE,
    .open           = ad9833_open,
    .unlocked_ioctl = ad9833_ioctl,
};

static int ad9833_open( struct inode *inodes, struct file *filp )
{
    return 0;
}
static long ad9833_ioctl( struct file  *file, unsigned int cmd, unsigned long arg )
{
    printk(DRV_NAME "\tRecv cmd: %u\n", cmd);
    printk(DRV_NAME "\tRecv arg: %lu\n", arg);
    switch( cmd ) {
    case CMD_TYPE_SIN:
        ad9833->set_wave_freq(ad9833, 1500);
        ad9833->set_wave_type(ad9833, SIN);
        printk( DRV_NAME " set wave is sine wave! arg = %lu\n" , arg );
        break;

    case CMD_TYPE_TRI:
        ad9833->set_wave_freq(ad9833, 1500);
        ad9833->set_wave_type(ad9833, TRI);
        printk( DRV_NAME " set wave is tri wave! arg = %lu\n" , arg );
        break;

    case CMD_TYPE_SQE:
        ad9833->set_wave_freq(ad9833, 1500);
        ad9833->set_wave_type(ad9833, SQU);
        printk( DRV_NAME " set wave is sw wave! arg = %lu\n" , arg );
        break;

    }
    return  0;
}

static irqreturn_t ad9833_press_intHandle(int irq, void *dev_id)
{
    if( irq == ad9833_irq_desc.irq_sin) {
        ad9833->set_wave_type(ad9833, SIN);
        printk(DRV_NAME "\tSetting wave type SIN\n");
    } else if (irq == ad9833_irq_desc.irq_tri) {
        ad9833->set_wave_type(ad9833, TRI);
        printk(DRV_NAME "\tSetting wave type TRI\n");
    } else if(irq == ad9833_irq_desc.irq_squ) {
        ad9833->set_wave_type(ad9833, SQU);
        printk(DRV_NAME "\tSetting wave type SQU\n");
    }
    
    return IRQ_RETVAL(IRQ_HANDLED);
}

static int __init ad9833_init(void)
{
    int ret,err;
    dev_t devid;
    
    if(major) {
        devid = MKDEV(major, minor);
        ret = register_chrdev_region(devid, 1, DRV_NAME);
    } else {
        ret = alloc_chrdev_region(&devid, 0, 1, DRV_NAME);
        major = MAJOR(devid);
    }
    if(ret < 0) {
        printk(DRV_NAME "\t new device failed\n");
        return ret;
    }

    printk(DRV_NAME "\tApply memory for AD9833\n");
    
    ad9833 = ad9833_dev_new();
    if(ad9833 == NULL) {
        ret = -ENOMEM;
        goto fail_malloc;
    }

    cdev_init(&ad9833->cdev, &ad9833_fileops);
    err = cdev_add(&ad9833->cdev, devid, 1);
    if(err) {
        printk(KERN_NOTICE "Error %d adding ad9833 %d",err,1);
    }
    printk(DRV_NAME "\tApply memory success\n");

    /*
    Apply for button irq
    */
    ad9833_irq_desc.irq_sin = gpio_to_irq(AD9833_SIN_INT_IO);
    ad9833_irq_desc.irq_tri = gpio_to_irq(AD9833_TRI_INT_IO);
    ad9833_irq_desc.irq_squ = gpio_to_irq(AD9833_SQU_INT_IO);
    ad9833_irq_desc.ret_sin = request_irq(ad9833_irq_desc.irq_sin, &ad9833_press_intHandle, ad9833_irq_desc.flags, ad9833_irq_desc.name_1,(void*)0);
    ad9833_irq_desc.ret_tri = request_irq(ad9833_irq_desc.irq_tri, &ad9833_press_intHandle, ad9833_irq_desc.flags, ad9833_irq_desc.name_2,(void*)0);
    ad9833_irq_desc.ret_squ = request_irq(ad9833_irq_desc.irq_squ, &ad9833_press_intHandle, ad9833_irq_desc.flags, ad9833_irq_desc.name_3,(void*)0);
    ret = ad9833_irq_desc.ret_sin & ad9833_irq_desc.ret_tri & ad9833_irq_desc.ret_squ;
    if(ret) {
        if(ad9833_irq_desc.ret_sin) {
        printk( DRV_NAME "\terror %d: IRQ = %d number failed!\n",ret,gpio_to_irq(AD9833_SIN_INT_IO));
        ret = -EBUSY;
        gpio_free(AD9833_SIN_INT_IO);
        }
        if(ad9833_irq_desc.ret_tri) {
        printk( DRV_NAME "\terror %d: IRQ = %d number failed!\n",ret,gpio_to_irq(AD9833_TRI_INT_IO));
        ret = -EBUSY;
        gpio_free(AD9833_TRI_INT_IO);
        }
        if(ad9833_irq_desc.ret_squ) {
        printk( DRV_NAME "\terror %d: IRQ = %d number failed!\n",ret,gpio_to_irq(AD9833_SQU_INT_IO));
        ret = -EBUSY;
        gpio_free(AD9833_SQU_INT_IO);
        }
        unregister_chrdev_region(devid, 1);
        kfree(ad9833);
        return ret;
    }

    printk(DRV_NAME "\tIrq apply success\n");
    
    return 0;
    
fail_malloc:
    unregister_chrdev_region(devid, 1);
    printk(DRV_NAME "\tFail to allocate memory\n");
    printk(DRV_NAME "\tUnregister AD9833 device\n");
    return ret;
}

static void __exit ad9833_exit(void)
{
    cdev_del(&ad9833->cdev);
    kfree(ad9833);
    unregister_chrdev_region(MKDEV(major, minor), 1);
    free_irq(ad9833_irq_desc.irq_sin,(void*)0);
    free_irq(ad9833_irq_desc.irq_tri,(void*)0);
    free_irq(ad9833_irq_desc.irq_squ,(void*)0);
    gpio_free(AD9833_SIN_INT_IO);
    gpio_free(AD9833_TRI_INT_IO);
    gpio_free(AD9833_SQU_INT_IO);
    printk(DRV_NAME "\tUnregister AD9833 device\n");
}

module_init(ad9833_init);
module_exit(ad9833_exit);

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");


