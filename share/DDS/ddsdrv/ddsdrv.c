#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <asm/mach/time.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/at91_spi.h>
#include <mach/at91sam9g45.h>
#include <mach/at91_pmc.h>
#include <mach/at91_twi.h>
#include <mach/gpio.h>


#define DEBUG

#define CMD_TWI_WRITE_TEST       10
#define CMD_TWI_READ_TEST        11
#define CMD_TWI_SET_ADDR         12
#define CMD_TWI_SET_REGADDR      13
#define CMD_TWI_REG_NONE         14
#define CMD_TWI_REG_YES          15
#define CMD_TWI_READ_REG_TEST    16


#define TWI_INTERNAL_REG_NFLAG    0   
#define TWI_INTERNAL_REG_YFLAG    1  
#define AT91_EEPROM_I2C_ADDRESS   0x50

#define TWI_READ_MODE             1   
#define TWI_WRITE_MODE            2
/*------------------------------------------------------------*/
#define CMD_SET_DDS_SINWAVE   35
#define CMD_SET_DDS_SQUARE    36
#define CMD_SET_DDS_FREQ      37

#define DDS_CS       			  6 

#define DDS_CS_PIN            	  AT91_PIN_PD31
#define SCK_PIN                   AT91_PIN_PB16
#define SPI_DOUT_PIN              AT91_PIN_PB15

#define DDS_CS_HIGH()      	      at91_set_gpio_value(DDS_CS_PIN, 1)
#define DDS_CS_LOW()      	      at91_set_gpio_value(DDS_CS_PIN, 0)
#define SCK_HIGH()     		      at91_set_gpio_value(SCK_PIN, 1)
#define SCK_LOW()      		      at91_set_gpio_value(SCK_PIN, 0)
#define OUT_HIGH()     		      at91_set_gpio_value(SPI_DOUT_PIN, 1)
#define OUT_LOW()      		      at91_set_gpio_value(SPI_DOUT_PIN, 0)

#define DDS_SPI_MODE     		  2
#define SC      				  4    //SC = 0 continuous conversion mode;1: signal conversion mode
#define SINE_WAVE    			  0
#define SQUARE_WAVE  			  1

static u16 DDSInitCMD[5];  //DDS ³õÊ¼»¯Êý¾Ý
static u8 DDSWaveFlg = 1;  //ÉèÖÃDDS²¨ÐÎµÄFlag  0£ºsinwave 1:Square wave
u32 DDSFreq = 10000;//8192000;   //Ä¬ÈÏDDS 8.192MHz

/*--------------------------------------------------------------------*/
u8 twiRegMode = 0;  
u8 twiRegAddr = 0;

#define AT91_TWI_CLOCK		      100000  
static void __iomem *twi0_base;
#define at91_twi0_read(reg)		    __raw_readl(twi0_base + (reg))
#define at91_twi0_write(reg, val)	__raw_writel((val), twi0_base + (reg))

#define TWI0DEV_MAJOR  102
char twi0dev_name[]="dds";
int slaAddr = 0x51;

u8 spiMode = 0;      
u8 spibus_id = 0;      
u8 spibuf[100];

//cycle check status of byte(NAK.OVER.TXRDY.RXRDY.TX_COMP) in AT91_TWI_SR 
static short at91_poll_status(unsigned long bit)
 {
	int loop_cntr = 10000;
	do 
	{
		udelay(10);
	} while (!(at91_twi0_read(AT91_TWI_SR) & bit) && (--loop_cntr > 0));
	return (loop_cntr > 0);
}

static int at91_twi_xfer(int devAddr, u8 *data, int size,u8 twixfmode,u8 regflag, u8 reg_addr)
{
	//¹Ø±Õ´ÓÉè±¸Ä£Ê½ ¿ªÆôÖ÷Éè±¸Ä£Ê½
	at91_twi0_write(AT91_TWI_CR,AT91_TWI_SVDIS | AT91_TWI_MSEN);  //Master enable	slave disable	2012.09.12
			
	if (size && data) // sanity check 
		{				   
			switch(twixfmode)
			{
			 case TWI_READ_MODE:
				if(regflag)	     //Ê¹ÓÃÄÚ²¿µØÖ·
					{
						at91_twi0_write(AT91_TWI_MMR, (devAddr << 16) | AT91_TWI_IADRSZ_1 | AT91_TWI_MREAD);
						at91_twi0_write(AT91_TWI_IADR, reg_addr);   //Æ÷¼þµÄÄÚ²¿¼Ä´æÆ÷µØÖ· Ä¿Ç°Ö»Ö§³Ö1×Ö½ÚµÄÄÚ²¿µØÖ·
					}
				else
					at91_twi0_write(AT91_TWI_MMR, (devAddr << 16) | AT91_TWI_IADRSZ_NO | AT91_TWI_MREAD);
					
				at91_twi0_write(AT91_TWI_CR , AT91_TWI_START);
				
				while (size--) 
				{
					if (!size)  //Ö»·¢ËÍÒ»´Î
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
						at91_twi0_write(AT91_TWI_IADR, reg_addr);   //Æ÷¼þµÄÄÚ²¿¼Ä´æÆ÷µØÖ· Ä¿Ç°Ö»Ö§³Ö1×Ö½ÚµÄÄÚ²¿µØÖ·	
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

static int twi_open(struct inode *inode,struct file *filp)
{
	return 0;
}

static int twi_close(struct inode *inode,struct file *filp)
{      
	return 0;
}

u8 twiReadBuf[20];
static int twi_read(struct file *fd, unsigned char *command,int count,loff_t *l)
{
	//twiXFMode = TWI_READ_MODE;
	at91_twi_xfer(slaAddr,twiReadBuf,count,TWI_READ_MODE,twiRegMode,twiRegAddr);
  if (copy_to_user(command,twiReadBuf,count))
		return -EFAULT;
		
	#ifdef DEBUG
			printk("TWI Read OK\n");
	#endif	
		
  return 0;
}

u8 twiWriteBuf[20];

static int twi_write(struct file *fd, char *command,int count,loff_t *l)
{
	//twiXFMode = TWI_WRITE_MODE;
	if (copy_from_user(twiWriteBuf,command,count))
			return -EFAULT;
	at91_twi_xfer(slaAddr,twiWriteBuf,count,TWI_WRITE_MODE,twiRegMode,twiRegAddr);
	
	#ifdef DEBUG
			printk("TWI Write OK\n");
	#endif	
	
	return 0;
}

/*-----------------DDS----------------------------------------------------------*/
void spi_access_bus(short device)
{
	if(device > 6)
	{
		printk("Set Device Error\n");
		return;
	}
	switch(device)
	{
		case DDS_CS:
			DDS_CS_LOW();
			break;
		default:
			break;	
	}
}

void spi_release_bus(short device)
{
	if(device > 6)
	{
		printk("Set Device Error\n");
		return;
	}
	switch(device)
	{
		case DDS_CS:
			DDS_CS_HIGH();
			break;
		default:
			break;	
	}
}

int write_spi_byte(char data)
{
	int result = 0;
	char mask = 0x80;

	switch(spiMode)
	{
	case DDS_SPI_MODE:
		SCK_HIGH();
		while(mask)
		{
			if(mask & data) 
				OUT_HIGH();
			else 
				OUT_LOW();
			
			SCK_HIGH();  
			udelay(5);
			SCK_LOW();
						
			//if(IS_DIN_HIGH())   
				//result |= mask;
			
			udelay(5);
			mask >>= 1;
		}
		SCK_HIGH();	
		break;
	default:
		printk("Wrong SPI Mode Set\n");
		break;
	}	
	return result;
}

int write_spi(char *p, int num)
{
	int i;

	for(i=0; i<num; i++)
	{
		write_spi_byte(*(p+i));
	}
	return 0;
}

static int __init at91_spi_init(void)
{

	at91_set_gpio_output(DDS_CS_PIN, 1); 
	DDS_CS_HIGH();
	
	at91_set_gpio_output(SCK_PIN, 0);
	at91_set_gpio_output(SPI_DOUT_PIN, 0);	
	//at91_set_gpio_input(SPI_DIN_PIN, 0);
	return 0;
}

static void Set_DDS(void)
{
	u32 tempdds = 0;
	u8 i = 0;
	
	if(DDSWaveFlg == SINE_WAVE)  //sine
	{
		DDSInitCMD[0] = 0x2100;
		DDSInitCMD[4] = 0x2000;
		printk("Set Sine\n");
	}
	else if(DDSWaveFlg == SQUARE_WAVE)
	{
		DDSInitCMD[0] = 0x2128;
		DDSInitCMD[4] = 0x2028;
		printk("Set Square\n");
	}
	tempdds = DDSFreq;
	
	printk("tmp:%X\n",tempdds);
	DDSInitCMD[1] = tempdds & 0x3fff;       //config_data[1]´æ·ÅÆµÂÊ¼Ä´æÆ÷0µÍ14Î»
	printk("%04X\n",DDSInitCMD[1]);
	DDSInitCMD[2] = (tempdds >> 14) & 0x3fff; //config_data[2]´æ·ÅÆµÂÊ¼Ä´æÆ÷0¸ß14Î»
	printk("%04X\n",DDSInitCMD[2]);
	DDSInitCMD[1] = DDSInitCMD[1] | 0x4000;    //»ò0x4000:¼´DB14=1,Ñ¡ÔñÆµÂÊ¼Ä´æÆ÷0
	DDSInitCMD[2] = DDSInitCMD[2] | 0x4000;
	DDSInitCMD[3] = 0xC000;                      //DB15DB14=11:ÉèÖÃÏàÎ»¼Ä´æÆ÷0

	printk("%04X%04X\n",DDSInitCMD[2],DDSInitCMD[1]);

	spiMode = DDS_SPI_MODE;
	for(i = 0 ; i < 5 ; i ++)
	{
		spi_access_bus(DDS_CS);
		udelay(10);
		write_spi_byte((DDSInitCMD[i] >> 8) & 0xFF);   //¸ß°ËÎ»
		write_spi_byte(DDSInitCMD[i] & 0xFF);  //µÍ°ËÎ»
		spi_release_bus(DDS_CS);
		udelay(10);
	}
	//spiMode = AD_SPI_MODE;
	
	printk("Set DDS OK\r\n");
	
}

/*--------------------------------------------------------*/

char writeBufTmp[3] = {4,5,6} ;
char readBufTmp[10];
u32 regTmp = 0; 

static int twi_ioctl(struct inode * s_node,struct file * s_file,unsigned int cmd,unsigned long arg)
{  
	switch(cmd)
	{
	case CMD_TWI_WRITE_TEST:
    at91_twi_xfer(slaAddr, writeBufTmp, 3,TWI_WRITE_MODE,TWI_INTERNAL_REG_YFLAG,0);
    printk("TWI0 Send Test OK\n");
		break;
	case CMD_TWI_READ_TEST:
    at91_twi_xfer(slaAddr, readBufTmp, 3,TWI_READ_MODE,TWI_INTERNAL_REG_YFLAG,0);
    printk("Read Data is:%x,%x,%x\n",readBufTmp[0],readBufTmp[1],readBufTmp[2]);
		break;
	case CMD_TWI_SET_REGADDR:
		if(copy_from_user(&twiRegAddr,(u8 *)arg,1))
			return -EFAULT;	
		#ifdef DEBUG
			printk("Set TWI Reg Addr:0x%x\n",twiRegAddr);
		#endif	
		break;
    case CMD_TWI_SET_ADDR:
  		if(copy_from_user(&slaAddr,(u8 *)arg,1))
			return -EFAULT;	
		#ifdef DEBUG
			printk("Set TWI Device Addr:0x%x\n",slaAddr);
		#endif		
		break;
	case CMD_TWI_REG_NONE:
		#ifdef DEBUG
			printk("TWI No Internal Reg\n");
		#endif	
		twiRegMode = TWI_INTERNAL_REG_NFLAG;
		break;
	case CMD_TWI_REG_YES:
		#ifdef DEBUG
			printk("TWI Has Internal Reg\n");
		#endif	
		twiRegMode = TWI_INTERNAL_REG_YFLAG;
		break;
	case CMD_TWI_READ_REG_TEST:
		regTmp = at91_twi0_read(AT91_TWI_SR);
		printk("TWI_SR:%04X\n",regTmp);
		regTmp = at91_twi0_read(AT91_TWI_THR);
		printk("TWI_THR:%04X\n",regTmp);
		break; 
/*-----------------------------------------------------------*/
	/*case CMD_ACCESS_SPI:
		if(copy_from_user(&spibus_id,(u8*)arg,  1))
			return -EFAULT;
		spi_access_bus(spibus_id);
		#ifdef DEBUG
		  printk("SPI Access CS ID:%d\n",spibus_id);
		#endif
		break;
	case CMD_RELEASE_SPI:
		if(copy_from_user(&spibus_id,(u8*)arg,  1))
			return -EFAULT;
		spi_release_bus(spibus_id);
		#ifdef DEBUG
		  printk("SPI Release CS ID:%d\n",spibus_id);
		#endif
		break; */
	case CMD_SET_DDS_SQUARE:
		DDSWaveFlg = SQUARE_WAVE;
		#ifdef DEBUG
			printk("Set DDS output squqre wave\n");
		#endif
		Set_DDS();
		break;
	case CMD_SET_DDS_SINWAVE:
		DDSWaveFlg = SINE_WAVE;
		#ifdef DEBUG
			printk("Set DDS output sine wave\n");
		#endif
		Set_DDS();
		break;
	case CMD_SET_DDS_FREQ:
		if(copy_from_user(&DDSFreq,(u32*)arg,4))
			return -EFAULT;
		printk("Set DDS Freq:%d\n",DDSFreq);
		 break;		
  default:
   	break;
		}
  return 0;
  }

static struct file_operations twi_fops=
{
	open	:(void(*))twi_open,
	write	:(void(*))twi_write,
	release	:(void(*))twi_close,  
	read	:(void(*))twi_read,  
	ioctl	:(void(*))twi_ioctl,  
};

static int __init at91_twi0_init(void)
{
    
    int retv;
    twi0_base = ioremap(AT91SAM9G45_BASE_TWI0, SZ_16K); //twi reg map 
	if (!twi0_base)
		return -ENOMEM;
    
    at91_spi_init();
	at91_sys_write(AT91_PMC_PCER,1 << AT91SAM9G45_ID_TWI0);	
	at91_twi_setclock();
	at91_set_A_periph(AT91_PIN_PA20, 0);            // TWD 
	at91_set_multi_drive(AT91_PIN_PA20, 1);
	at91_set_A_periph(AT91_PIN_PA21, 0);            // TWCK PA21
	at91_set_multi_drive(AT91_PIN_PA21, 1);
				
	retv=register_chrdev(TWI0DEV_MAJOR,twi0dev_name,&twi_fops);
	if(retv<0)
	{
		printk("Register Fail!\n");
		return retv;
	}
	printk("TWI0 Driver For AT91SAM9G45 OK!\n");
	return retv;
}


static void __exit at91_twi0_exit(void)
{
  unregister_chrdev(TWI0DEV_MAJOR, twi0dev_name);
  iounmap(twi0_base);
  printk("TWI0 driver bye!\n");
}

module_init(at91_twi0_init);
module_exit(at91_twi0_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("QY");
MODULE_DESCRIPTION("TWI0 Driver For AT91SAM9G45");
