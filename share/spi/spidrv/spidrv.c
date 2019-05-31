#include <linux/init.h>
#include <linux/module.h>           /* get MOD_DEC_USE_COUNT, not the version string */
#include <linux/moduleparam.h>
#include <linux/version.h>          /* need it for conditionals in geoad.h */
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/errno.h>            /* error codes */
#include <linux/types.h>            /* size_t */
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/cdev.h>
#include <linux/delay.h>
 
#include <asm/system.h>             /* rmb(), wmb() */
#include <asm/io.h>                 /* inb(), outb() */
#include <asm/uaccess.h>
 
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91_pmc.h>
#include <mach/at91_spi.h>

//命令宏定义
#define		SPI_TEST_CMD			0x25
#define		SPI_SHOW_SEG_CMD		0x26


//设备信息定义 
#define 		SPI_NR_DEVS    	1       	/* device number */
static int spi_major = 106;               /* major device number */
static int spi_nr_devs = SPI_NR_DEVS;    /* number of spi control devices */
static char spi_name[16]; 


static void __iomem *spi1_base;
#define at91_spi1_read(reg)		    __raw_readl(spi1_base + (reg))
#define at91_spi1_write(reg, val)	__raw_writel((val), spi1_base + (reg))

static const char code_table[] = {0xBE, 0x06, 0x7C, 0x5E, 0xC6, 0xDA, 0xFA, 0x0E, 0xFE, 0xDE};
//ioctl映射函数定义 
int spi_ioctl( struct inode *inode, struct file *filp, unsigned int cmd,
                unsigned long arg )
{  
	unsigned long rec = 0x10;
    switch( cmd )
    {
		case SPI_TEST_CMD:
			at91_spi1_write(AT91_SPI_TDR , (0x07 << 16) | (code_table[1]) );//NPCS[3：0]=0111
			printk(KERN_INFO"test spi\n");
			break;
		case SPI_SHOW_SEG_CMD:
           copy_from_user(&rec , &arg, sizeof(long));
			if(rec < 16)
			{
				at91_spi1_write(AT91_SPI_TDR , (0x07 << 16) | (code_table[arg]) );//0xff - 
				printk(KERN_INFO"SEG show %lu\n",arg);
			}
			else	
				printk(KERN_INFO"Num is not valid\n");
			break;
	    default:
				return -ENOTTY;
    } 
    return 0;
}
/*
 * Open and close.
 */
int spi_open( struct inode *inode, struct file *filp )
{    
    return 0;
}
 
int spi_release( struct inode *inode, struct file *filp )
{
    return 0;
}

/* for device spi control */
static struct file_operations spi_fops = 
{
    .owner   =  THIS_MODULE,
    .open    =  spi_open,
    .release =  spi_release,
    .ioctl   =  spi_ioctl,
};

//定义SPI字符设备结构体
struct SPI_DEVS
{
    struct cdev cdev;                   /* Char device structure */
};

static struct SPI_DEVS spi_device; 

static struct spi_dev_info
{
    char *name;
    struct SPI_DEVS *dev;
    struct file_operations *fops;
} spi_devs[] = {
    { "spi_ctl", &spi_device, &spi_fops }
};

void spi_setup_cdev( struct spi_dev_info *devinfo, int index )
{
    int err, devno = MKDEV( spi_major, index );
    struct SPI_DEVS *dev = devinfo->dev;
     
    /* Do the cdev stuff. */
    cdev_init( &dev->cdev, devinfo->fops );
    kobject_set_name( &dev->cdev.kobj, devinfo->name );
    dev->cdev.owner = THIS_MODULE;
    dev->cdev.ops = devinfo->fops;
    err = cdev_add( &dev->cdev, devno, 1 );
     
    /* Fail gracefully if need be */
    if( err )
    {
        printk( KERN_NOTICE "Error %d adding %s\n", err, devinfo->name );
        kobject_put( &dev->cdev.kobj );
    }
    else
        printk( KERN_NOTICE "%s registered at %x\n", devinfo->name, devno );
}

static int __init spi1_cfg_gpio(void)
{
	at91_set_A_periph(AT91_PIN_PD19, 1);  //RD19，SPI1NPCS3，使能内部上拉
	at91_set_A_periph(AT91_PIN_PB16, 1);  //SPI1_SPCK
	at91_set_A_periph(AT91_PIN_PB15, 1);  //SPI1_MOSI
	return 0;
}

static int __init spi_init( void )
{
    int err = -1 , i = 0;
    dev_t devno = MKDEV( spi_major, 0 );
     
    memset( spi_name, '\0', sizeof(spi_name) );
    sprintf( spi_name, "%s", "spi_ctl" );
     
    /*
     * Get a range of minor numbers to work with, asking for a dynamic
     * major unless directed otherwise at load time.
     */
    if( spi_major )
        err = register_chrdev_region( devno, spi_nr_devs, spi_name );
    else
    {
        err = alloc_chrdev_region( &devno, 0, spi_nr_devs, spi_name );
        spi_major = MAJOR( devno );
    }
     
    if( err < 0 )
    {
        printk( KERN_WARNING "spi: can't get major %d\n", spi_major );
        goto err001;
    }
     
    // Setup the geo devices.
    for( i = 0; i < spi_nr_devs; i++ )
        spi_setup_cdev( &spi_devs[i], i );
    
	spi1_cfg_gpio();
	spi1_base = ioremap(AT91SAM9G45_BASE_SPI1, SZ_16K);
	if (!spi1_base)
		return -ENOMEM;
	at91_sys_write(AT91_PMC_PCER , 1 << AT91SAM9G45_ID_SPI1);  //外设时钟开启寄存器
	at91_spi1_write(AT91_SPI_CR , AT91_SPI_SWRST); //SPI软件复位
	
	at91_spi1_write(AT91_SPI_CSR(3) , AT91_SPI_NCPHA | AT91_SPI_BITS_8 | (133 << 8) );//MCLK分频133
	//数据会在 SPCK 的首个触发沿捕获，并在下一个触发沿改变|每次传输8位|SPCK 波特率为MCLK/133
	at91_spi1_write(AT91_SPI_MR , AT91_SPI_MSTR | AT91_SPI_PS | AT91_SPI_MODFDIS);
	//主控模式|可变外设|禁止模式错误检测
	at91_spi1_write(AT91_SPI_CR , AT91_SPI_SPIEN);  //使能SPI
    printk( KERN_INFO "***************** spi control driver started **************\n" );
     
    return 0;
 
err001:
    return err;
}
 
static void __exit spi_exit( void )
{
    int i = 0;
    dev_t devno = MKDEV( spi_major, 0 );
         

 
    /* Clean up the static devs */
    for( i = 0; i < spi_nr_devs; i++ )
    {
        struct SPI_DEVS *dev = spi_devs[i].dev;
        cdev_del( &dev->cdev );
    }
	
	iounmap(spi1_base);
    unregister_chrdev_region( devno, spi_nr_devs );
	
	printk( KERN_INFO "***************** spi control driver bye **************\n" );
}
module_init( spi_init );
module_exit( spi_exit );

MODULE_AUTHOR("ZTX");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SPI driver for ESSK_V2");
