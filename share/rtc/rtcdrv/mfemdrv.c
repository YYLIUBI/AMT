#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>

#include <asm/mach/time.h>
#include <asm/uaccess.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/gpio.h>
#include <mach/hardware.h>
#include <mach/at91sam9g45.h>
#include <mach/at91_pmc.h>
#include <mach/at91_spi.h>
#include <mach/at91_ssc.h>
#include <mach/at91_tc.h>
#include <mach/gpio.h>

#define CMD_ACCESS_SPI       12
#define CMD_RELEASE_SPI      13 
#define CMD_RTC_READ         14
#define CMD_RTC_WRITE        15
#define CMD_ALARM_INIT       17

int RFMR_INT = 3;
#define DS1390_INTCN    	0x1<<2	//DS1390 reg
#define DS1390_AIE      	0x1<<0
#define RTC_ACQ_CS        	3  

#define LED1_PIN        	AT91_PIN_PB24 // LED1
#define LED0_PIN			AT91_PIN_PC0  //LED0
#define PPS_IN_PIN          AT91_PIN_PB21
#define RTC_INT_PIN      	AT91_PIN_PD19

#define RTC_INT_PIN        	AT91_PIN_PD19  //A

//spi Pin
#define RTC_ACQ_CS_PIN     	AT91_PIN_PD30 //A
#define SCK_PIN            	AT91_PIN_PB16 //A
#define SPI_MOSI_PIN      	AT91_PIN_PB15 //A
#define SPI_MISO_PIN        AT91_PIN_PB14 //A

#define RTC_ACQ_CS_HIGH()  	at91_set_gpio_value(RTC_ACQ_CS_PIN, 1)          //Êý¾Ý²É¼¯Ê¹ÓÃµÄRTC CS¹Ü½Å¸ßµçÆ½
#define RTC_ACQ_CS_LOW()   	at91_set_gpio_value(RTC_ACQ_CS_PIN, 0)

#define SCK_HIGH()     		(at91_set_gpio_value(SCK_PIN, 1))
#define SCK_LOW()      		(at91_set_gpio_value(SCK_PIN, 0))
#define MOSI_HIGH()     	(at91_set_gpio_value(SPI_MOSI_PIN, 1)) //??
#define MOSI_LOW()      	(at91_set_gpio_value(SPI_MOSI_PIN, 0)) //??
#define IS_MISO_HIGH()  	(at91_get_gpio_value(SPI_MISO_PIN))  //??

#define LED_HIGH()        	(at91_set_gpio_value(LED1_PIN, 1))
#define LED_LOW()        	(at91_set_gpio_value(LED1_PIN, 0))

#define RTC_SPI_MODE  		1 
#define AD_SPI_MODE   		0 
//#define DEBUG

u8 spiMode = 0;           
u8 spibus_id = 0;       
u8 spibuf[100];

#define MFEMDEV_MAJOR  109	
char mfemdev_name[]="mfem";

    
static int mfem_open(struct inode *inode,struct file *filp)
{
	return 0;
}

static int mfem_close(struct inode *inode,struct file *filp)
{      
	return 0;
}


void spi_access_bus(short device)
{
	if(device > 6)
		{	
			printk("Set Device Error\n");	
			return;	
		}
	switch(device)
	{
		case RTC_ACQ_CS:  	
			RTC_ACQ_CS_LOW();			
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
		case RTC_ACQ_CS: 	
			RTC_ACQ_CS_HIGH();			
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
		
		case RTC_SPI_MODE:   //¿ÕÏÐÊ±¿ÌSCKµÍµçÆ½£¬SCKÉÏÉýÑØ±ä»¯£¬ÏÂ½µÑØ²ÉÑù
			SCK_LOW();
			while(mask)
			{
				if(mask & data)	  
				    MOSI_HIGH();
				else 
				    MOSI_LOW();
				
				//SCK_LOW();
				//udelay(2);
				SCK_HIGH();	
				udelay(5);
				SCK_LOW();		
				
				if(IS_MISO_HIGH())  //ÏÂ½µÑØ²ÉÑù
					result |= mask;		
				
				udelay(5);
				mask >>= 1;
			}
			SCK_LOW();
			break;
		
		default:
			printk("Wrong SPI Mode Set\n");
		break;
	}	
	//printk("read_spi:%x\n",result);
	//printk("write_spi:%x\n",data);
	return result;
}

int write_spi(char *p, int num)
{
	int i;

	for(i=0; i<num; i++)
	{
		write_spi_byte(*(p+i));
		//printk("write:%x\n", *(p+i));
	}
	return 0;
}
int read_spi(char *p, int num)
{
	int i;
	for(i=0; i<num; i++)
	{
		*(p+i)= write_spi_byte(0xFF);
		//printk("read:%x\n", *(p+i));
	}
	return 0;
}

static int __init at91_spi_init(void)
{
	
	//at91_set_gpio_input(RTC_INT_PIN,1);
	at91_set_gpio_output(RTC_ACQ_CS_PIN, 1);     
	RTC_ACQ_CS_HIGH();
	at91_set_gpio_output(SCK_PIN, 0);	     //Êä³ö£¬²»ÓÃÉÏÀ­
	at91_set_gpio_output(SPI_MOSI_PIN, 0);	 //Êä³ö£¬ÉèÖÃÉÏÀ­
	//at91_set_gpio_output(SCK_PIN, 1);	     //Êä³ö£¬²»ÓÃÉÏÀ­
	//at91_set_gpio_output(SPI_DOUT_PIN, 1);	 //Êä³ö£¬ÉèÖÃÉÏÀ­
	at91_set_gpio_input(SPI_MISO_PIN, 0);	 //²»ÄÜÉèÖÃÉÏÀ­  ºÜÖØÒª
	return 0;
}
//RTC read one reg	
static void rtc_rfer(u8 *reg_addr,u8 *data_addr,short device)
{
	spi_access_bus(device);
	udelay(2);
	write_spi(reg_addr,1);
  	read_spi(data_addr,1);
	spi_release_bus(device);
	udelay(2);
	//printk("read:%x %x\n",*reg_addr,*data_addr);
}

static void rtc_xfer(u8 *reg_addr,u8 *data_addr,short device)
{
	spi_access_bus(device);
	udelay(2);
	write_spi(reg_addr,1);
  	write_spi(data_addr,1);
	spi_release_bus(device);
	udelay(2);
	//printk("write:%x %x\n",*reg_addr,*data_addr);
}

u8 rtc_write_addr[7]={0x81,0x82,0x83,0x84,0x85,0x86,0x87};  //RTC write addr 0x81=Seconds
u8 rtc_init_data[10];

static int rtc_write(u8 *rtc_initdata,short rtccs)  //=>write 
{
  u8 HunSecWrAddr=0x80,HunSecData=0x00;  //Hundred Sec Reg
  spiMode = RTC_SPI_MODE;
  rtc_xfer(&HunSecWrAddr,&HunSecData,rtccs);  //Hundred Sec=>0
  rtc_xfer(rtc_write_addr,rtc_initdata,rtccs);
  rtc_xfer(rtc_write_addr+1,rtc_initdata+1,rtccs);
  rtc_xfer(rtc_write_addr+2,rtc_initdata+2,rtccs);
  rtc_xfer(rtc_write_addr+3,rtc_initdata+3,rtccs);
  rtc_xfer(rtc_write_addr+4,rtc_initdata+4,rtccs);
  rtc_xfer(rtc_write_addr+5,rtc_initdata+5,rtccs);
  rtc_xfer(rtc_write_addr+6,rtc_initdata+6,rtccs);
  spiMode = AD_SPI_MODE;
  return 1;
}
 
u8 rtc_data[7];
//static int rtc_read(u8 *rtc_readdata)
u8 con_write_addr[2]={0x8D,0x8E};
u8 con_reg_data[2];

static int rtc_read(u8 *rtc_readdata,short rtccs)
{
	u8 CMD_READ_RTC[7]={0x01,0x02,0x03,0x04,0x05,0x06,0x07};
	spiMode = RTC_SPI_MODE;
	con_reg_data[0] = DS1390_INTCN | DS1390_AIE | 0X80;

	rtc_rfer(CMD_READ_RTC,rtc_readdata,rtccs);     //second
	rtc_rfer(CMD_READ_RTC + 1,rtc_readdata + 1,rtccs); //minute
	rtc_rfer(CMD_READ_RTC + 2,rtc_readdata + 2,rtccs); //hour
	rtc_rfer(CMD_READ_RTC + 3,rtc_readdata + 3,rtccs); //day
	rtc_rfer(CMD_READ_RTC + 4,rtc_readdata + 4,rtccs); //date
	rtc_rfer(CMD_READ_RTC + 5,rtc_readdata + 5,rtccs); //month
	rtc_rfer(CMD_READ_RTC + 6,rtc_readdata + 6,rtccs); //year
	rtc_rfer(CMD_READ_RTC,rtc_readdata,rtccs);  

	udelay(10);
	rtc_xfer(con_write_addr + 1,con_reg_data + 1,rtccs);  
	rtc_xfer(con_write_addr,con_reg_data,rtccs);		  

	spiMode = AD_SPI_MODE;
  	return 1;
 }

u8 alarm_reg[5]={0x88,0x89,0x8A,0x8B,0x8C};//u8 alarm_reg[4]={0x89,0x8A,0x8B,0x8C};
u8 alarm_data[4];

u8 con_read_addr[1]={0x0D};
//void set_alarm(void)
/*
	参数1:u8 * pbuffalaram，设置相关Alarm寄存器的数组
	参数2:short rtccs，RTC片选信号
*/
void set_alarm(u8 * pbuffalaram,short rtccs)
{
	u8 HundredthsofSeconds=0;
	spiMode = RTC_SPI_MODE;
	con_reg_data[0] = DS1390_INTCN | DS1390_AIE;
	rtc_xfer(alarm_reg,&HundredthsofSeconds,rtccs);
	rtc_xfer(alarm_reg+1,pbuffalaram,rtccs);
	rtc_xfer(alarm_reg+2,pbuffalaram + 1,rtccs);
	rtc_xfer(alarm_reg+3,pbuffalaram + 2,rtccs);
	rtc_xfer(alarm_reg+4,pbuffalaram + 3,rtccs);
	udelay(10);
	rtc_xfer(con_write_addr + 1,con_reg_data + 1,rtccs);  //status reg 清除状态很重要
	rtc_xfer(con_write_addr,con_reg_data,rtccs);          //control reg
	spiMode = AD_SPI_MODE;                                //default SPI Mode
}

//static void clear_alarm(void)
static void clear_alarm(short rtccs)
{
	char Status_Reg_Data = 0x00;
	spiMode = RTC_SPI_MODE;
	//rtc_xfer(con_write_addr + 1,&Status_Reg_Data,RTC_ACQ_CS);       //status reg 清除状态很重要
	//rtc_xfer(con_write_addr + 1,&Status_Reg_Data,RTC_ECORRODE_CS);  //status reg 清除状态很重要
	rtc_xfer(con_write_addr + 1,&Status_Reg_Data,rtccs);  //status reg 清除状态很重要
	spiMode = AD_SPI_MODE;
}

static void clear_wsqw(void)
{
	con_reg_data[0] = 0x05;
	spiMode = RTC_SPI_MODE;
	rtc_xfer(con_write_addr,con_reg_data,RTC_ACQ_CS); 
	spiMode = AD_SPI_MODE;
}

static irqreturn_t at91_rtc_interrupt(int irq, void *dev_id)
{
	int status = 0 ;
	status = at91_get_gpio_value(RTC_INT_PIN);

	if(!status)  //low level intertupt 
	{	
		//flag_rtcalarm = 1;
  		//wake_up_interruptible(&rtc_alarm);
  		//LED_HIGH();
  		at91_set_gpio_value(LED0_PIN, 1);
  		LED_HIGH();
  		printk("RTC interrupt");
	}
	
	return IRQ_RETVAL(IRQ_HANDLED);
}

static irqreturn_t at91_pps_interrupt(int irq, void *dev_id)
{
	//if(PPSWakeUpFlag)
	//{//printk("PPS!\n");
		if(at91_get_gpio_value(PPS_IN_PIN)==1)  
		{	/*flag_pps = 1;
			at91_ssc0_write(AT91_SSC_CR , AT91_SSC_RXEN);
			at91_set_gpio_value(SSC_START_PIN, 1);  
			wake_up_interruptible(&at91_pps_int);
	   		PPSWakeUpFlag=0;}	*/	
	   	    //LED_HIGH();
	   	    int status;
	   		status = at91_get_gpio_value(LED0_PIN);
	   		at91_set_gpio_value(LED0_PIN,!status);
	   		//printk("PPS interrupt");
	    } 
	     //CPLD
	return IRQ_RETVAL(IRQ_HANDLED);
}

//数据采集使用的定时RTC中断初始化
static int __init at91_rtc_interrupt_init(void)
{
  	at91_set_gpio_input(RTC_INT_PIN,1);
    gpio_irq_unmask(RTC_INT_PIN);
	gpio_irq_type(RTC_INT_PIN,1);

	if(request_irq(gpio_to_irq(RTC_INT_PIN),at91_rtc_interrupt, IRQF_DISABLED ,"mfem-rtc",NULL)<0)
		printk("RTC interrupt request fail!\n");
	return 0;
}

static int __init at91_pps_interrupt_init(void)
{
	at91_set_gpio_input(PPS_IN_PIN,1);   //设置引脚输入
	gpio_irq_unmask(PPS_IN_PIN); 		 //使能引脚中断
	gpio_irq_type(PPS_IN_PIN,1);  		 //设置中断类型
	

	if(request_irq(gpio_to_irq(PPS_IN_PIN),at91_pps_interrupt, IRQF_DISABLED ,"mfem-pps",NULL)<0)
		printk("PPS interrupt request fail!\n");
	return 0;
}


static int mfem_write(struct file *fd, char *command,int count,loff_t *l)
{
	if (copy_from_user(spibuf,command,count))
		return -EFAULT;
	write_spi(spibuf, count);	
  return 0;
}

static int mfem_read(struct file *fd, u32 *command,int count,loff_t *l)
{

  return 0;
}


static int mfem_ioctl(struct inode * s_node,struct file * s_file,int cmd,unsigned long arg)
{ 	
  switch(cmd)
	{
	case CMD_ACCESS_SPI:
						if(copy_from_user(&spibus_id,(u8*)arg,  1))	return -EFAULT;
						spi_access_bus(spibus_id);
						#ifdef DEBUG
						printk("SPI Access CS ID:%d\n",spibus_id);
						#endif
						break;
	case CMD_RELEASE_SPI:
						if(copy_from_user(&spibus_id,(u8*)arg,  1))	return -EFAULT;
						spi_release_bus(spibus_id);
						#ifdef DEBUG
						printk("SPI Release CS ID:%d\n",spibus_id);
						#endif
						break;
	case CMD_RTC_READ:
						#ifdef DEBUG
						printk("RTC read\n");
						#endif
						rtc_read(rtc_data,RTC_ACQ_CS);
						printk("rtc_read_data:%x,%x,%x,%x,%x,%x,%x\n",rtc_data[0],
							rtc_data[1],rtc_data[2],rtc_data[3],rtc_data[4],
							rtc_data[5],rtc_data[6]);
						if(copy_to_user((u8*)arg,rtc_data,7))			return -EFAULT;
						break;
	case CMD_RTC_WRITE: 
						#ifdef DEBUG
						printk("RTC write\n");
						#endif
						if(copy_from_user(rtc_init_data,(u8*)arg,  7))	return -EFAULT;
						rtc_write(rtc_init_data,RTC_ACQ_CS);
						//printk("rtc_write_data:%x,%x,%x,%x,%x,%x,%x\n",rtc_init_data[0],
						//	rtc_init_data[1],rtc_init_data[2],rtc_init_data[3],rtc_init_data[4],
						//	rtc_init_data[5],rtc_init_data[6]);
						break;
	case CMD_ALARM_INIT:
						if(copy_from_user(alarm_data,(u8*)arg,4))
							return -EFAULT;					
						set_alarm(alarm_data,spibus_id);   //HJJ 01.22
						break;					
 
  default:
    break;
     }  
  return 0;
  }

  static struct file_operations mfem_fops=
{
	open	:(void(*))mfem_open,
	write	:(void(*))mfem_write,
	release	:(void(*))mfem_close,  
	read	:(void(*))mfem_read,  
	ioctl	:(void(*))mfem_ioctl,  
};

static  int __init at91_mfem_init(void)
{
	int retv;
	at91_spi_init();
	at91_set_gpio_output(LED0_PIN, 0);
	at91_set_gpio_output(LED1_PIN, 0);
	//at91_set_GPIO_periph(AT91_PIN_PD30, 1);  //RD19£¬SPI1NPCS3£¬Ê¹ÄÜÄÚ²¿ÉÏÀ­
	//at91_set_GPIO_periph(AT91_PIN_PB16, 1);  //SPI1_SPCK
	//at91_set_GPIO_periph(AT91_PIN_PB15, 1);
	at91_spi_init();
	clear_alarm(RTC_ACQ_CS);	
	clear_wsqw();
	at91_rtc_interrupt_init();    //数据采集RTC中断初始化
 	at91_pps_interrupt_init();
	retv=register_chrdev(MFEMDEV_MAJOR,mfemdev_name,&mfem_fops);
	if(retv<0)
	{
		printk("Register Fail!\n");
		return retv;
	}
	printk("Register MFEM driver for SAM9G45 OK! 2017-10-25\n");
	#ifdef DEBUG
		printk("MTEM Debug Version\n");
	#endif
	return retv;
	
}


static void __exit at91_mfem_exit(void)  
{  
	free_irq(gpio_to_irq(RTC_INT_PIN), 0);         
	free_irq(gpio_to_irq(PPS_IN_PIN), 0);

 	unregister_chrdev(MFEMDEV_MAJOR, mfemdev_name);   //ºÍÔ­À´µÄ²»Ò»ÑùÁË£¬Ã»ÓÐ·µ»ØÖµ  --HJJ
  	printk(KERN_INFO"Unregister MFEM driver ; 2017-10-25\n"); 
}  
  
module_init(at91_mfem_init);  
module_exit(at91_mfem_exit);

MODULE_DESCRIPTION("QY RTC TEST");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("QY");

