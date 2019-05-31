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
#include <mach/at91_twi.h>

//����궨��
#define 	CMD_TWI_WRITE_TEST       10
#define 	CMD_TWI_READ_TEST        11

#define 	TWI_READ_MODE             1   //��ģʽ
#define 	TWI_WRITE_MODE            2   //дģʽ
#define 	TWI_INTERNAL_REG_NFLAG    0   //���ڲ��Ĵ�����ַģʽ
#define 	TWI_INTERNAL_REG_YFLAG    1  //���ڲ��Ĵ�����ַģʽ

//�豸��Ϣ���� 
#define 		TWI_NR_DEVS    	1       	/* device number */
static int twi_major = 107;               /* major device number */
static int twi_nr_devs = TWI_NR_DEVS;     /* number of twi control devices */
static char twi_name[16]; 

static void __iomem *twi0_base;
#define at91_twi0_read(reg)		    __raw_readl(twi0_base + (reg))
#define at91_twi0_write(reg, val)	__raw_writel((val), twi0_base + (reg))

#define AT91_TWI_CLOCK		100000  //�˴���ֵ�ǳ���Ҫ������̫��ʱ��̫�첻����������

/*
 ʹ��˵����
 1��Ӧ�ò��趨��ַ��
 2��Ӧ�ò�ȷ���Ƿ��������ڲ��Ĵ�����ַ,�趨�ڲ���ַ��
 */
static short at91_poll_status(unsigned long bit)
 {
	int loop_cntr = 10000;
	do 
	{
		udelay(10);
	} while (!(at91_twi0_read(AT91_TWI_SR) & bit) && (--loop_cntr > 0));
	return (loop_cntr > 0);
}
//twixfmode 1:read;
//          0:write
//regflag�����Ƿ�Ҫ�����ڲ���ַ
//reg_addr:�����ڲ��Ĵ�����ַ

//int at91_twi_xfer(int address, char *data, int size,int flag)
//int at91_twi_xfer(int address, char *data, int size,u8 twixfmode)
//devAddr���豸��ַ
//reg_addr:�ڲ��Ĵ�����ַ
static int at91_twi_xfer(int devAddr, u8 *data, int size,u8 twixfmode,u8 regflag, u8 reg_addr)
{
	//�رմ��豸ģʽ �������豸ģʽ
	at91_twi0_write(AT91_TWI_CR,AT91_TWI_SVDIS | AT91_TWI_MSEN);  //Master enable	slave disable	2012.09.12
			
	if (size && data) // sanity check 
		{				   
			switch(twixfmode)
			{
			 case TWI_READ_MODE:
				if(regflag)	     //ʹ���ڲ���ַ
					{
						at91_twi0_write(AT91_TWI_MMR, (devAddr << 16) | AT91_TWI_IADRSZ_1 | AT91_TWI_MREAD);
						at91_twi0_write(AT91_TWI_IADR, reg_addr);   //�������ڲ��Ĵ�����ַ Ŀǰֻ֧��1�ֽڵ��ڲ���ַ
					}
				else
					at91_twi0_write(AT91_TWI_MMR, (devAddr << 16) | AT91_TWI_IADRSZ_NO | AT91_TWI_MREAD);
					
				at91_twi0_write(AT91_TWI_CR , AT91_TWI_START);
				
				while (size--) 
				{
					if (!size)  //ֻ����һ��
						at91_twi0_write(AT91_TWI_CR , AT91_TWI_STOP);
						
					if (!at91_poll_status(AT91_TWI_RXRDY))
						{
							printk(KERN_ERR "at91_i2c: read timeout 1 - rx not ready\n");
							return -1;
					  }
					  
					*data++ = at91_twi0_read(AT91_TWI_RHR);
				}
						
				if (!at91_poll_status(AT91_TWI_TXCOMP))
				{
						printk(KERN_ERR "at91_i2c: read timeout 2 - tx not complete\n");
						return -2;
				}
				break;
		case TWI_WRITE_MODE:
					//at91_twi0_write(AT91_TWI_MMR , ( (address << 16)| AT91_TWI_IADRSZ_1 ) & ~AT91_TWI_MREAD);	
					if(regflag)
					{
						at91_twi0_write(AT91_TWI_MMR , ( (devAddr << 16)| AT91_TWI_IADRSZ_1 ) & ~AT91_TWI_MREAD);
						at91_twi0_write(AT91_TWI_IADR, reg_addr);   //�������ڲ��Ĵ�����ַ Ŀǰֻ֧��1�ֽڵ��ڲ���ַ	
					}
					else
						at91_twi0_write(AT91_TWI_MMR , ( (devAddr << 16)| AT91_TWI_IADRSZ_NO ) & ~AT91_TWI_MREAD);	
						
					//at91_twi0_write(AT91_TWI_CR , AT91_TWI_START);
					
					while (size--) 
					{
						at91_twi0_write(AT91_TWI_THR , *data++);
						
						if (!size)
							at91_twi0_write(AT91_TWI_CR , AT91_TWI_STOP);
							
						if (!at91_poll_status(AT91_TWI_TXRDY)) 
							{
								printk(KERN_ERR "at91_i2c: write timeout 3 - tx not ready\n");
								return -3;
						}
					}
					
					if (!at91_poll_status(AT91_TWI_TXCOMP))
					 {
							 printk(KERN_ERR "at91_i2c: write timeout 4 - tx not complete\n");
							 return -4;
					  }
			  break;
			default:
				break;
			}
  }
	return 0;
}

static void at91_i2c_cfg_gpio(void)
{
	at91_set_A_periph(AT91_PIN_PA20, 0);	 /* TWD */
	at91_set_multi_drive(AT91_PIN_PA20, 1);
	at91_set_A_periph(AT91_PIN_PA21, 0);	 /* TWCK */
	at91_set_multi_drive(AT91_PIN_PA21, 1);
}

void at91_twi_setclock(void)
{
	int sclock;
	/* Here, CKDIV = 1 and CHDIV=CLDIV  ==> CLDIV = CHDIV = 1/4*((Fmclk/FTWI) -6)*/
	  sclock = (10 * AT91SAM9_MASTER_CLOCK /AT91_TWI_CLOCK);
	if (sclock % 10 >= 5)
		sclock = (sclock /10) - 5;
	else
		sclock = (sclock /10)- 6;
  	sclock = (sclock + (4 - sclock %4)) >> 2;	// div 4
    at91_twi0_write(AT91_TWI_CWGR,0x00010000 | sclock | (sclock << 8));
} 

char writeBufTmp[3] = {4,5,6} ;
char readBufTmp[10];
int slaAddr = 0x51;
//ioctlӳ�亯������ 
int twi_ioctl( struct inode *inode, struct file *filp, unsigned int cmd,
                unsigned long arg )
{    
    switch( cmd )
    {
		case CMD_TWI_WRITE_TEST:
			at91_twi_xfer(slaAddr, writeBufTmp, 3,TWI_WRITE_MODE,TWI_INTERNAL_REG_YFLAG,0);
			printk("TWI0 Send Test OK\n");
			break;
		case CMD_TWI_READ_TEST:
			at91_twi_xfer(slaAddr, readBufTmp, 3,TWI_READ_MODE,TWI_INTERNAL_REG_YFLAG,0);
			printk(KERN_INFO"Read Data is:%x,%x,%x\n",readBufTmp[0],readBufTmp[1],readBufTmp[2]);
			break;
	    default:
				return -ENOTTY;
    } 
    return 0;
}
/*
 * Open and close.
 */
int twi_open( struct inode *inode, struct file *filp )
{    
    return 0;
}
 
int twi_release( struct inode *inode, struct file *filp )
{
    return 0;
}
/* for device twi control */
static struct file_operations twi_fops = 
{
    .owner   =  THIS_MODULE,
    .open    =  twi_open,
    .release =  twi_release,
    .ioctl   =  twi_ioctl,
};

//����TWI�ַ��豸�ṹ��
struct TWI_DEVS
{
    struct cdev cdev;                   /* Char device structure */
};

static struct TWI_DEVS twi_device; 

static struct twi_dev_info
{
    char *name;
    struct TWI_DEVS *dev;
    struct file_operations *fops;
} twi_devs[] = {
    { "twi_ctl", &twi_device, &twi_fops }
};

void twi_setup_cdev( struct twi_dev_info *devinfo, int index )
{
    int err, devno = MKDEV( twi_major, index );
    struct TWI_DEVS *dev = devinfo->dev;
     
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

static int __init twi_init( void )
{
    int err = -1 , i = 0;
		
    dev_t devno = MKDEV( twi_major, 0 );   
    memset( twi_name, '\0', sizeof(twi_name) );
    sprintf( twi_name, "%s", "twi_ctl" );

	twi0_base = ioremap(AT91SAM9G45_BASE_TWI0, SZ_16K);
	if (!twi0_base)
		return -ENOMEM;

	at91_sys_write(AT91_PMC_PCER,1 << AT91SAM9G45_ID_TWI0);	
	at91_i2c_cfg_gpio();
	at91_twi_setclock();    	
    /*
     * Get a range of minor numbers to work with, asking for a dynamic
     * major unless directed otherwise at load time.
     */
    if( twi_major )
        err = register_chrdev_region( devno, twi_nr_devs, twi_name );
    else
    {
        err = alloc_chrdev_region( &devno, 0, twi_nr_devs, twi_name );
        twi_major = MAJOR( devno );
    }
     
    if( err < 0 )
    {
        printk( KERN_WARNING "twi: can't get major %d\n", twi_major );
        goto err001;
    }
     
    // Setup the geo devices.
    for( i = 0; i < twi_nr_devs; i++ )
        twi_setup_cdev( &twi_devs[i], i );      
    printk( KERN_INFO "***************** twi control driver started **************\n" );     
    return 0;
 
err001:
    return err;
}
 
static void __exit twi_exit( void )
{
    int i = 0;
    dev_t devno = MKDEV( twi_major, 0 ); 
    /* Clean up the static devs */
    for( i = 0; i < twi_nr_devs; i++ )
    {
        struct TWI_DEVS *dev = twi_devs[i].dev;
        cdev_del( &dev->cdev );
    }	
	iounmap(twi0_base);
    unregister_chrdev_region( devno, twi_nr_devs );	
	printk( KERN_INFO "***************** twi control driver bye **************\n" );
}
module_init( twi_init );
module_exit( twi_exit );

MODULE_AUTHOR("ZTX");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TWI driver for ESSK_V2");
