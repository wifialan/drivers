#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>
#include <linux/log2.h>
#include <linux/bitops.h>
#include <linux/property.h>
#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/nvmem-provider.h>
#include <linux/regmap.h>
#include <linux/platform_data/at24.h>
#include <linux/pm_runtime.h>
#include <linux/gpio/consumer.h>
#include <linux/cdev.h>     /* cdev_alloc()  */
#include <linux/fs.h>       /* struct fops */
#include <linux/errno.h>    /* error codes */
#include <linux/cdev.h>     /* cdev_alloc()  */
#include <linux/uaccess.h>

#define DRV_NAME "AT24C256"
#define DRV_AUTHOR "Alan Tian <alantian.at@gmail.com>"
#define DRV_DESC "AT24C256 eeprom Driver"

#define AT24_SIZE 0x1000

#define AT24_MAJOR 230
#define AT24_MINOR 0

static int major = AT24_MAJOR;
static int minor = AT24_MINOR;

static int at24_open( struct inode *inodes, struct file *filp );
static long at24_ioctl( struct file  *file, unsigned int cmd, unsigned long arg );
static ssize_t at24_write( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos );
static ssize_t at24_read( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos );
static int at24_probe(struct i2c_client * client, const struct i2c_device_id *id);
static int at24_remove(struct i2c_client *client);


struct at24_chip_data {
	/*
	 * these fields mirror their equivalents in
	 * struct at24_platform_data
	 */
	u32 byte_len;
	u8 flags;
};

#define AT24_CHIP_DATA(_name, _len, _flags)				\
	static const struct at24_chip_data _name = {			\
		.byte_len = _len, .flags = _flags,			\
	}

/* needs 8 addresses as A0-A2 are ignored */
AT24_CHIP_DATA(at24_data_24c00, 128 / 8, AT24_FLAG_TAKE8ADDR);
/* old variants can't be handled with this generic entry! */
AT24_CHIP_DATA(at24_data_24c01, 1024 / 8, 0);
AT24_CHIP_DATA(at24_data_24cs01, 16,
	AT24_FLAG_SERIAL | AT24_FLAG_READONLY);
AT24_CHIP_DATA(at24_data_24c02, 2048 / 8, 0);
AT24_CHIP_DATA(at24_data_24cs02, 16,
	AT24_FLAG_SERIAL | AT24_FLAG_READONLY);
AT24_CHIP_DATA(at24_data_24mac402, 48 / 8,
	AT24_FLAG_MAC | AT24_FLAG_READONLY);
AT24_CHIP_DATA(at24_data_24mac602, 64 / 8,
	AT24_FLAG_MAC | AT24_FLAG_READONLY);
/* spd is a 24c02 in memory DIMMs */
AT24_CHIP_DATA(at24_data_spd, 2048 / 8,
	AT24_FLAG_READONLY | AT24_FLAG_IRUGO);
AT24_CHIP_DATA(at24_data_24c04, 4096 / 8, 0);
AT24_CHIP_DATA(at24_data_24cs04, 16,
	AT24_FLAG_SERIAL | AT24_FLAG_READONLY);
/* 24rf08 quirk is handled at i2c-core */
AT24_CHIP_DATA(at24_data_24c08, 8192 / 8, 0);
AT24_CHIP_DATA(at24_data_24cs08, 16,
	AT24_FLAG_SERIAL | AT24_FLAG_READONLY);
AT24_CHIP_DATA(at24_data_24c16, 16384 / 8, 0);
AT24_CHIP_DATA(at24_data_24cs16, 16,
	AT24_FLAG_SERIAL | AT24_FLAG_READONLY);
AT24_CHIP_DATA(at24_data_24c32, 32768 / 8, AT24_FLAG_ADDR16);
AT24_CHIP_DATA(at24_data_24cs32, 16,
	AT24_FLAG_ADDR16 | AT24_FLAG_SERIAL | AT24_FLAG_READONLY);
AT24_CHIP_DATA(at24_data_24c64, 65536 / 8, AT24_FLAG_ADDR16);
AT24_CHIP_DATA(at24_data_24cs64, 16,
	AT24_FLAG_ADDR16 | AT24_FLAG_SERIAL | AT24_FLAG_READONLY);
AT24_CHIP_DATA(at24_data_24c128, 131072 / 8, AT24_FLAG_ADDR16);
AT24_CHIP_DATA(at24_data_24c256, 262144 / 8, AT24_FLAG_ADDR16);
AT24_CHIP_DATA(at24_data_24c512, 524288 / 8, AT24_FLAG_ADDR16);
AT24_CHIP_DATA(at24_data_24c1024, 1048576 / 8, AT24_FLAG_ADDR16);



static const struct of_device_id at24c256_of_match[] = {
	{ .compatible = "atmel,24c00",		.data = &at24_data_24c00 },
	{ .compatible = "atmel,24c01",		.data = &at24_data_24c01 },
	{ .compatible = "atmel,24cs01",		.data = &at24_data_24cs01 },
	{ .compatible = "atmel,24c02",		.data = &at24_data_24c02 },
	{ .compatible = "atmel,24cs02",		.data = &at24_data_24cs02 },
	{ .compatible = "atmel,24mac402",	.data = &at24_data_24mac402 },
	{ .compatible = "atmel,24mac602",	.data = &at24_data_24mac602 },
	{ .compatible = "atmel,spd",		.data = &at24_data_spd },
	{ .compatible = "atmel,24c04",		.data = &at24_data_24c04 },
	{ .compatible = "atmel,24cs04",		.data = &at24_data_24cs04 },
	{ .compatible = "atmel,24c08",		.data = &at24_data_24c08 },
	{ .compatible = "atmel,24cs08",		.data = &at24_data_24cs08 },
	{ .compatible = "atmel,24c16",		.data = &at24_data_24c16 },
	{ .compatible = "atmel,24cs16",		.data = &at24_data_24cs16 },
	{ .compatible = "atmel,24c32",		.data = &at24_data_24c32 },
	{ .compatible = "atmel,24cs32",		.data = &at24_data_24cs32 },
	{ .compatible = "atmel,24c64",		.data = &at24_data_24c64 },
	{ .compatible = "atmel,24cs64",		.data = &at24_data_24cs64 },
	{ .compatible = "atmel,24c128",		.data = &at24_data_24c128 },
	{ .compatible = "atmel,24c256",		.data = &at24_data_24c256 },
	{ .compatible = "atmel,24c512",		.data = &at24_data_24c512 },
	{ .compatible = "atmel,24c1024",	.data = &at24_data_24c1024 },
	{ /* END OF LIST */ },
};
MODULE_DEVICE_TABLE(of, at24c256_of_match);
struct i2c_client *client_2;

static struct at24_dev_t {
    struct cdev cdev;
    struct class *class;
    struct i2c_client *client;
    unsigned char mem[AT24_SIZE];
}*at24_dev;

static int at24_open( struct inode *inodes, struct file *filp )
{
    return 0;
}
static long at24_ioctl( struct file  *file, unsigned int cmd, unsigned long arg )
{
    printk("at24 ioctl function\n");
    return 0;
}
static ssize_t at24_write( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos )
{
    int ret,tmp;
    char i;
    struct i2c_msg msg;
    char buffer_data[100];

    copy_from_user(buffer_data, buffer, 10);

    memset(buffer_data,0,sizeof(buffer_data));
    buffer_data[0] = (char)0x00;
    buffer_data[1] = (char)0x00;

    for (i = 0; i < 64; ++i)
    {
        buffer_data[i+2] = i;
    }
    
    msg.addr = at24_dev->client->addr | 0x01;
    msg.flags = at24_dev->client->flags & 0;
    msg.buf = &buffer_data[0];
    msg.len = 66;

    ret = i2c_transfer(at24_dev->client->adapter,&msg,1);
    tmp = (ret == 1) ? msg.len : ret;

    printk("i2c code: %d  return code: %d addr: 0x%02x%02x ",\
        ret,tmp,buffer_data[0],buffer_data[1]);

    for (i = 0; i < 64; ++i)
    {
        printk("Write Data: 0x%02x",buffer_data[i+2]);
    }
    
    return 0;
}
static ssize_t at24_read( struct file *filp, const char __user *buffer, size_t size, loff_t *f_pos )
{
    int ret,tmp;
    unsigned long i;
    struct i2c_msg msg[2];
    char buffer_data[100];

    memset(buffer_data,0,sizeof(buffer_data));
    buffer_data[0] = (char)0x00;
    buffer_data[1] = (char)0x00;
    buffer_data[2] = (char)0xAA;
    buffer_data[3] = (char)0xAB;
    buffer_data[4] = (char)0x0C;
    
    msg[0].addr = at24_dev->client->addr | 0x01;
    msg[0].flags = I2C_M_RD;
    msg[0].buf = &buffer_data[2];
    msg[0].len = 64;

    ret = i2c_transfer(at24_dev->client->adapter, &msg[0], 1);
    tmp = (ret == 1) ? msg[0].len : ret;

    printk("i2c code: %d  return code: %d addr: 0x%02x%02x ",\
        ret,tmp,buffer_data[0],buffer_data[1]);

    for (i = 0; i < 64; ++i)
    {
        printk("Read Data: 0x%02x",buffer_data[i+2]);
    }

    return 0;
}


static const struct file_operations at24_fileops = {
    .owner      = THIS_MODULE,
    .write      = at24_write,
    .read       = at24_read,
    .open       = at24_open,
    .unlocked_ioctl = at24_ioctl,
};

static struct i2c_driver at24c256_driver = {
    .driver     = {
        .name   = "at24c256",
        .owner  = THIS_MODULE,
        .of_match_table = at24c256_of_match,
    },
    .probe      = at24_probe,
    .remove     = at24_remove,
    //.id_table   = at24_ids,

};
        
static int at24_remove(struct i2c_client *client)
{
    printk(DRV_NAME "\tPrepera to remove at24c256 device...\n");
    device_destroy(at24_dev->class, MKDEV(major, minor));
    class_destroy(at24_dev->class);
    cdev_del(&at24_dev->cdev);
    kfree(at24_dev);
    unregister_chrdev_region(MKDEV(major, minor), 1);
    
    printk(DRV_NAME "\tRemove at24c256 device success\n");
    
    return 0;
}

static int at24_probe(struct i2c_client * client, const struct i2c_device_id *id)
{
    int ret,err;
    dev_t devid;

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
        printk(DRV_NAME "\tCreat device node failed\n");
        //goto fail_malloc;
        return ret;
    }
    at24_dev = kzalloc(sizeof(struct at24_dev_t), GFP_KERNEL);
    if(!at24_dev) {
        ret = -ENOMEM;
        goto fail_malloc;
    }
    printk(DRV_NAME "\tSuccess init at24c256\n");

    at24_dev->client = client;
        
    cdev_init(&at24_dev->cdev, &at24_fileops);
    err = cdev_add(&at24_dev->cdev, devid, 1);
    if(err)
    {
        printk(KERN_NOTICE "Error %d adding at24c256 eeprom %d",err, 1);
        return err;
    }

    at24_dev->class = class_create(THIS_MODULE, "at24c256");
    device_create(at24_dev->class,NULL,MKDEV(major, minor),NULL,"at24c256");
    printk(DRV_NAME "\tCreat device node /dev/at24c256\n");

    
    return 0;
    
fail_malloc:
    unregister_chrdev_region(devid, 1);
    return ret;    
}



static int __init at24_init(void)
{
    return i2c_add_driver(&at24c256_driver);
}

static void __exit at24_exit(void)
{
    i2c_del_driver(&at24c256_driver);
    printk(DRV_NAME "\tRemove i2c driver success\n");
}

module_init(at24_init);
module_exit(at24_exit);



MODULE_DESCRIPTION("Driver for most I2C EEPROMs");
MODULE_AUTHOR("David Brownell and Wolfram Sang");
MODULE_LICENSE("GPL");






