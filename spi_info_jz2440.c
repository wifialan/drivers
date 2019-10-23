#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>

static struct spi_board_info spi_info_jz2440[] = {
	{
    	 .modalias = "oled",  
    	 .max_speed_hz = 10000000,	
    	 .bus_num = 1,     
    	 .mode    = SPI_MODE_0,
    	 .chip_select   = S3C2410_GPF(1), 
    	 //.platform_data = (const void *)S3C2410_GPG(4) ,
	 },
	 {
    	 .modalias = "w25q16",  
    	 .max_speed_hz = 80000000,	/* max spi clock (SCK) speed in HZ */
    	 .bus_num = 0,
    	 .mode    = SPI_MODE_0,
    	 .chip_select   = S3C2410_GPG(2), 
	 }
};

static int spi_info_jz2440_init(void)
{
    printk("spi_info_jz2440_init function..\n");
    return spi_register_board_info(spi_info_jz2440, ARRAY_SIZE(spi_info_jz2440));
}

module_init(spi_info_jz2440_init);
MODULE_DESCRIPTION("W25Q16 SPI Driver");
MODULE_AUTHOR("ALAN");
MODULE_LICENSE("GPL");


