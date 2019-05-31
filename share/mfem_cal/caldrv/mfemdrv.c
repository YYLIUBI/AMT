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

#include <asm/io.h>
#include <asm/irq.h>
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
#define CMD_ALARM_READ       18

#define CMD_MS_COUNT_START   33
#define CMD_MS_COUNT_CLEAR   34

#define CMD_SEL_MODE         50 
#define CMD_SETMCLK			 56
#define	CMD_SETGAIN			 57
#define CMD_SETACDC			 59
#define	CMD_SETGAINH		 63

#define CMD_BUFFER_SET       60
#define CMD_SETCAL		     61
#define CMD_SETSSCRFMR	     62
#define CMD_SETCAL_COIL      64
#define CMD_SSC_START_MT     65
#define CMD_READ_CPLD_MS     30

#define CMD_SETHLSC          51
#define CMD_OPEN_COILCAL     52

int RFMR_INT = 3;
#define DS1390_INTCN    	0x1<<2	//DS1390 reg
#define DS1390_AIE      	0x1<<0
#define RTC_ACQ_CS        	3  
#define CPLD_CS   			5
#define RTC_INT_PIN        	AT91_PIN_PD19  //A

//spi Pin
#define RTC_ACQ_CS_PIN     	AT91_PIN_PD30 //A
#define SCK_PIN            	AT91_PIN_PB16 //A
#define SPI_MOSI_PIN      	AT91_PIN_PB15 //A
#define SPI_MISO_PIN        AT91_PIN_PB14 //A
//SW1 SW2 SW3
#define SWITCHH_PIN         AT91_PIN_PC3//E18//A   
#define SWITCHL_PIN         AT91_PIN_PC6//E17//A   
#define SWITCH_COIL_CAI_PIN AT91_PIN_PC1
/*-----------------------------------------SSC----------------------------------*/
#define CMD_SSC_START        22
#define CMD_SSC_STOP         23

#define CMD_WAIT_PPS         27
#define CMD_WAIT_RTCALARM    28

//*********I/O********
#define LED1_PIN        	 AT91_PIN_PB24 // LED1
#define PPS_IN_PIN           AT91_PIN_PB21
#define SSC_START_PIN        AT91_PIN_PC0
#define SYNC_PIN         	 AT91_PIN_PA31

//未分配
#define MCLK_SEL0         	 AT91_PIN_PB20 //M18 A   
#define MCLK_SEL1    	 	 AT91_PIN_PB10 //U17   A  
#define SWITCH_CAL_PIN    	 AT91_PIN_PB27//P7      A  
#define SWITCH_CAL_COIL_PIN  AT91_PIN_PB26//t18      A  

#define EGAIN0_PIN           AT91_PIN_PC28 //N9//A
#define EGAIN1_PIN           AT91_PIN_PC22 //N8 //A
#define HGAIN0_PIN           AT91_PIN_PC12 //K17//A
#define HGAIN1_PIN           AT91_PIN_PC7 //K18 //A

//#define Config_PIN           AT91_PIN_PD25//P10 A   ????
#define MS_CLEAR_PIN         AT91_PIN_PD8 //U18 A  

#define MODE_SEL             AT91_PIN_PD17 //L17 A  
//#define MODE_SEL1         	 AT91_PIN_PE16 //M17 A  ????
//#define MODE_SEL2          	 AT91_PIN_PE19 //L18 A  ????

#define RAM_ORDER 5 

static void __iomem *ssc0_base;
#define at91_ssc0_read(reg)		    __raw_readl(ssc0_base + (reg))
#define at91_ssc0_write(reg, val)	__raw_writel((val), ssc0_base + (reg))
	
static int ssc_begin(void);

static DECLARE_WAIT_QUEUE_HEAD(transfer_wq0);

u8 SSCRevFlag = 0;   
u8 flag_A = 0;
u8 flag_B = 0;
u8 FirstReadCPLDMS=0;
//u8 flag_1 = 0;
int int_cnt0 = 0;  //SSC0
int int_cnt1 = 0;  //SSC0
char PPSWakeUpFlag = 0;
static DECLARE_WAIT_QUEUE_HEAD(at91_pps_int);
static int flag_pps = 0;
static DECLARE_WAIT_QUEUE_HEAD(rtc_alarm);  
static int flag_rtcalarm = 0;

/*-----------------------------------------SSCEND----------------------------------*/

#define RTC_ACQ_CS_HIGH()  	at91_set_gpio_value(RTC_ACQ_CS_PIN, 1)          
#define RTC_ACQ_CS_LOW()   	at91_set_gpio_value(RTC_ACQ_CS_PIN, 0)

#define SCK_HIGH()     		(at91_set_gpio_value(SCK_PIN, 1))
#define SCK_LOW()      		(at91_set_gpio_value(SCK_PIN, 0))
#define MOSI_HIGH()     	(at91_set_gpio_value(SPI_MOSI_PIN, 1)) //??
#define MOSI_LOW()      	(at91_set_gpio_value(SPI_MOSI_PIN, 0)) //??
#define IS_MISO_HIGH()  	(at91_get_gpio_value(SPI_MISO_PIN))  //??

#define RTC_SPI_MODE  		1 
#define AD_SPI_MODE   		0 
//#define DEBUG

u8 spiMode = 0;           
u8 spibus_id = 0;       
u8 spibuf[100];

//CPLD根据这两个IO 的电平组合设置LED的状态
#define MS_CLEAR_PIN_HIGH()  (at91_set_gpio_value(MS_CLEAR_PIN, 1)) 
#define MS_CLEAR_PIN_LOW()   (at91_set_gpio_value(MS_CLEAR_PIN, 0))

//#define Config1282  at91_set_gpio_value(Config_PIN, 0)   
//#define Read1282   at91_set_gpio_value(Config_PIN, 1)

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
		case AD_SPI_MODE:       //AD 和CPLD都是用的这个时序
			SCK_LOW();
			while(mask)
			{
				if(mask & data) MOSI_HIGH();else MOSI_LOW();
					
				SCK_LOW();udelay(2);SCK_HIGH();udelay(1);
				if(IS_MISO_HIGH())
					result |= mask;
				mask >>= 1;
			}
			SCK_LOW();
		break;
		
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

/*
	参数说明:参数1:u8 *rtc_initdata:初始化寄存器的值
				    参数2:short rtccs:RTC片选信号
*/
//static int rtc_write(u8 *rtc_initdata)  //=>write 
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

static void IO_init(void)
{

	at91_set_gpio_output(MS_CLEAR_PIN,0); 
	//at91_set_gpio_output(Config_PIN,0);//20160505 ch
	at91_set_gpio_output(SSC_START_PIN,0);
	at91_set_gpio_output(MODE_SEL,0);
	//SW1 SW2 SW3
	at91_set_gpio_output(SWITCHL_PIN,0);
	at91_set_gpio_output(SWITCHH_PIN,0);
	at91_set_gpio_output(AT91_PIN_PC1,0);
	
	at91_set_gpio_output(LED1_PIN,0);

	at91_set_gpio_value(SSC_START_PIN,0);
	at91_set_gpio_value(LED1_PIN, 0);
	at91_set_gpio_value(MS_CLEAR_PIN,1);
	//at91_set_gpio_value(Config_PIN,1);
	at91_set_gpio_value(MODE_SEL,0);

	at91_set_gpio_value(SWITCHL_PIN,1);
	at91_set_gpio_value(SWITCHH_PIN,1); 
	at91_set_gpio_value(SWITCH_COIL_CAI_PIN,0);

	at91_set_gpio_output(SWITCH_CAL_PIN,0);
	at91_set_gpio_output(EGAIN0_PIN,0);
	at91_set_gpio_output(EGAIN1_PIN,0);
	at91_set_gpio_output(HGAIN0_PIN,0);
	at91_set_gpio_output(HGAIN1_PIN,0);
	//at91_set_gpio_output(SWITCHHP_PIN,0);
	//at91_set_gpio_output(SWITCHLP_PIN,0);
	at91_set_gpio_output(MCLK_SEL0,0);
	at91_set_gpio_output(MCLK_SEL1,0);
	at91_set_gpio_output(SWITCH_CAL_COIL_PIN,0);
	at91_set_gpio_output(SYNC_PIN,1);



	at91_set_gpio_value(SWITCH_CAL_PIN,1);//SIGNAL MODE
	at91_set_gpio_value(SWITCH_CAL_COIL_PIN,1);//SIGNAL MODE
	at91_set_gpio_value(EGAIN0_PIN,0);
	at91_set_gpio_value(EGAIN1_PIN,0);
	at91_set_gpio_value(HGAIN0_PIN,0);
	at91_set_gpio_value(HGAIN1_PIN,0);
	//at91_set_gpio_value(SWITCHHP_PIN,0);
	//at91_set_gpio_value(SWITCHLP_PIN,0);
	at91_set_gpio_value(MCLK_SEL0,0);
	at91_set_gpio_value(MCLK_SEL1,0);  
	at91_set_gpio_value(SYNC_PIN,1); 
}

static irqreturn_t at91_rtc_interrupt(int irq, void *dev_id)
{
	int status = 0 ;
	status = at91_get_gpio_value(RTC_INT_PIN);

	if(!status)  //low level intertupt 
	{	
		flag_rtcalarm = 1;
  		wake_up_interruptible(&rtc_alarm);
  		//printk("RTC interrupt");
	}
	
	return IRQ_RETVAL(IRQ_HANDLED);
}

static irqreturn_t at91_pps_interrupt(int irq, void *dev_id)
{
	int status;
	status = at91_get_gpio_value(PPS_IN_PIN);
	at91_set_gpio_value(LED1_PIN,status);
	if(PPSWakeUpFlag)
	{//printk("PPS!\n");
		if(at91_get_gpio_value(PPS_IN_PIN)==1)  
		{	
			flag_pps = 1;	
			at91_ssc0_write(AT91_SSC_CR , AT91_SSC_RXEN);
			at91_set_gpio_value(SSC_START_PIN, 1);			
			wake_up_interruptible(&at91_pps_int);
	   		PPSWakeUpFlag=0;
	   		//printk("PPS interrupt");
	   	}		
	} //CPLD
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

static int at91_pps_interrupt_init(void)
{
	at91_set_gpio_input(PPS_IN_PIN,1);   //设置引脚输入
	gpio_irq_unmask(PPS_IN_PIN); 		 //使能引脚中断
	gpio_irq_type(PPS_IN_PIN,1);  		 //设置中断类型
	

	if(request_irq(gpio_to_irq(PPS_IN_PIN),at91_pps_interrupt, IRQF_DISABLED ,"mfem-pps",NULL)<0)
		printk("PPS interrupt request fail!\n");
	return 0;
}

//读取CPLD 1000Hz 计数值 也就是时间的ms
//发送0x5  得到第四字节-最高字节；0x6 ：第三字节；0x7:第二字节；0x8：最低位字节
//SPI mode : AD SPI mode
u8 cpldMSBuf[4];
static void ReadCPLDMS(u8 *p)
{
	u8 i = 0 ;
	
	spi_access_bus(CPLD_CS);  //CPLD 
	for(; i < 4 ; i ++)
	{

	  //读取CPLD 计时数据
	  
	  read_spi(p + 3 - i, 1);
	  
	  //udelay(5);

	}
	spi_release_bus(CPLD_CS);
}
/*--------------------------------------------SSC-----------------------------------*/
static void at91_ssc_io_init(void)
{
	at91_set_A_periph(AT91_PIN_PD3, 0);  //RD0
	at91_set_A_periph(AT91_PIN_PD5, 0);  //RF0
	at91_set_A_periph(AT91_PIN_PD4, 0);  //RK0
}

static int at91_ssc_reg_init(void)
{
	//RCMR接收时钟模式寄存器
	//AT91_SSC_CKS_PIN:选择时钟引脚
	//AT91_SSC_CKO_NONE:RK引脚只是输入
	//AT91_SSC_CKG_RFLOW：RF为低时，使能接收时钟
	//AT91_SSC_START_FALLING_RF:RF下降沿触发开始接收
	//(1 << 16):接收延迟
	//at91_ssc0_write(AT91_SSC_RCMR , (1 << 16) | AT91_SSC_CKS_PIN | AT91_SSC_CKO_NONE | AT91_SSC_CKG_NONE | AT91_SSC_START_FALLING_RF);
	//RFMR接收帧模式寄存器
	//AT91_SSC_FSOS_NONE:RF引脚只是输入
	//AT91_SSC_MSBF:高位先发送
	//AT91_SSC_DATALEN:数据的长度（DATLEN)传输32位字
	//每一个开始事件要传输的数据个数（DATNB）
	//3 //8:每帧数据个数是4个

	at91_sys_write(AT91_PMC_PCER,1 << AT91SAM9G45_ID_SSC0);
	at91_ssc0_write(AT91_SSC_CR , AT91_SSC_SWRST);
	at91_ssc0_write(AT91_SSC_RCMR ,  AT91_SSC_CKS_PIN | AT91_SSC_CKO_NONE | AT91_SSC_CKG_RFLOW | AT91_SSC_START_FALLING_RF);
	at91_ssc0_write(AT91_SSC_RFMR , AT91_SSC_DATALEN | AT91_SSC_MSBF | AT91_SSC_FSOS_NONE | (RFMR_INT << 8));//chnums 4ch ---- 3<<8
	
	return 0;
}

static void at91_ssc_exit(void)
{
	at91_sys_write(AT91_PMC_PCDR , 1 << AT91SAM9G45_ID_SSC0);

}

unsigned int MemSize=76832;//MT130848//AMT129744;
#define MemNum 2

u32 *pssc_buf[MemNum];
dma_addr_t dmaaddr[MemNum];

static int ssc_begin(void)
{
	int i=0;
	flag_A = 0;
	flag_B = 0;
	int_cnt0=0;
	int_cnt1=0;
	init_waitqueue_head(&transfer_wq0);
	
	at91_ssc_reg_init();

	for(i=0;i<MemNum;i++)
	{
		dmaaddr[i]=pci_map_single(NULL, pssc_buf[i], MemSize, PCI_DMA_FROMDEVICE);	
	}
	

	
	at91_ssc0_write(AT91_SSC_RPR , dmaaddr[0]);
	at91_ssc0_write(AT91_SSC_RCR , MemSize/4);
	at91_ssc0_write(AT91_SSC_RNCR , 0);
	at91_ssc0_write(AT91_SSC_PTCR , AT91_PDC_RXTEN);
	at91_ssc0_write(AT91_SSC_IER , AT91_SSC_ENDRX);  //允许接收结束中断
		
	return 0;
}

static int ssc_stop(void)
{
	int i=0;
	flag_A = 0;
	flag_B = 0;

	at91_ssc0_write(AT91_SSC_CR , AT91_SSC_RXDIS);		// Disable SSC0
	at91_ssc0_write(AT91_SSC_PTCR , AT91_PDC_RXTDIS);
	at91_ssc0_write(AT91_SSC_IDR , AT91_SSC_ENDRX);
	at91_ssc0_write(AT91_SSC_RCR , 0);
	at91_ssc0_write(AT91_SSC_RNCR , 0);
	
	for(i=0;i<MemNum;i++)
	{
		pci_unmap_single(NULL, dmaaddr[i], MemSize, PCI_DMA_FROMDEVICE);
		//memset(dmaaddr[i],0X0,sizeof(dmaaddr[i]));
	}

	at91_set_gpio_value(SSC_START_PIN, 0);  
	return 0;
}


/*
	SSC0:
*/
static irqreturn_t ssc0_interrupt(int irq, void *dev_id)
{


	at91_ssc0_write(AT91_SSC_RCR , MemSize/4);//很重要，要放在前面
	at91_ssc0_write(AT91_SSC_RPR , dmaaddr[(int_cnt0+1)%MemNum]);
		
	flag_A = 1;
	wake_up_interruptible(&transfer_wq0);
	int_cnt0 ++;
		
	return IRQ_RETVAL(IRQ_HANDLED);
}
/*--------------------------------------------SSCEND-----------------------------------*/

static int mfem_write(struct file *fd, char *command,int count,loff_t *l)
{
	if (copy_from_user(spibuf,command,count))
		return -EFAULT;
	write_spi(spibuf, count);	
  return 0;
}

static int mfem_read(struct file *fd, u32 *command,int count,loff_t *l)
{
	wait_event_interruptible(transfer_wq0, flag_A == 1 );
	if (flag_A)
	{
		flag_A = 0;//printk("SSC0 :%d\n",MemSize);
		dma_sync_single_for_cpu(NULL, dmaaddr[int_cnt1%MemNum], MemSize, PCI_DMA_FROMDEVICE);
		
		if(copy_to_user(command, pssc_buf[int_cnt1%MemNum], MemSize))
			return -EFAULT;

		dma_sync_single_for_device(NULL, dmaaddr[int_cnt1%MemNum], MemSize, PCI_DMA_FROMDEVICE);
		
	}
	int_cnt1++;

  return 0;
}

u32	status = 0xffffffff;
//int ModeSizeSel[10]={0,76832,0,0,76832,129744,130848,0,0,0};

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
	case CMD_READ_CPLD_MS:
					    ReadCPLDMS(cpldMSBuf);
					    if(copy_to_user((u8*)arg, cpldMSBuf, 4))
							return -EFAULT;
						break;					
						
	case CMD_SSC_START: 
						init_waitqueue_head(&at91_pps_int);flag_pps = 0;
						init_waitqueue_head(&rtc_alarm);flag_rtcalarm = 0;
						ssc_stop();//防止CTRL C操作
						if (arg==1)
						{	
							wait_event_interruptible(rtc_alarm,flag_rtcalarm != 0);
							flag_rtcalarm = 0;
						}

						at91_set_gpio_value(SYNC_PIN,0);
						while(at91_get_gpio_value(PPS_IN_PIN)==1);//20150904
						//msleep(1000);						
						//wait_event_interruptible(at91_pps_int,flag_pps != 0);
						ssc_begin();						
						PPSWakeUpFlag=1;
						//wait_event_interruptible(at91_pps_int,flag_pps != 0);  //在FPGA中利用PPS触发SYNC，屏蔽掉ARM中的PPS中断
						//flag_pps = 0;
						at91_set_gpio_value(SYNC_PIN,1);
						break;	
	case CMD_SSC_STOP:
						ssc_stop();
						break;

	case CMD_ALARM_INIT:
						if(copy_from_user(alarm_data,(u8*)arg,4))
							return -EFAULT;
						set_alarm(alarm_data,spibus_id);   //HJJ 01.22
						break;					
    case CMD_MS_COUNT_START:	
    						MS_CLEAR_PIN_HIGH(); 	
    						//printk("start\n");
    						break;
	
	case CMD_MS_COUNT_CLEAR:	
							MS_CLEAR_PIN_LOW();  
							//printk("clear\n");	
							break;
		
	case CMD_SEL_MODE:
						//arg=1 sip arg=4 csamt arg=5 AMT arg=6 MT
						at91_set_gpio_value(MODE_SEL,arg); 
						//at91_set_gpio_value(MODE_SEL1,(arg/2)%2); 
						//at91_set_gpio_value(MODE_SEL2,(arg/4)%2); 
						//MemSize=ModeSizeSel[arg];
						//printk("MemSize=%d %d\n",MemSize,arg);
						break;
	case CMD_BUFFER_SET:
						MemSize=arg;//printk("MemSize=%d %d\n",MemSize,arg);
						break;

	case CMD_SETMCLK:
						at91_set_gpio_value(MCLK_SEL0,arg%2);
						at91_set_gpio_value(MCLK_SEL1,arg/2);
						break;
	case CMD_SETGAIN:
						at91_set_gpio_value(EGAIN0_PIN,arg%2); 
						at91_set_gpio_value(EGAIN1_PIN,arg/2); 
						break;
	case CMD_SETGAINH:
						at91_set_gpio_value(HGAIN0_PIN,arg%2); 
						at91_set_gpio_value(HGAIN1_PIN,arg/2); 
						printk("set_gain\n");
						break;
	case CMD_SETHLSC:  //switch high or low speed channel
						at91_set_gpio_value(SWITCHH_PIN,arg%2);
						at91_set_gpio_value(SWITCHL_PIN,arg/2);
						break;
	case CMD_OPEN_COILCAL:  //SW3
						at91_set_gpio_value(SWITCH_COIL_CAI_PIN,arg);
	case CMD_SETCAL:
						at91_set_gpio_value(SWITCH_CAL_PIN,arg);	
						break;
	case CMD_SETCAL_COIL:
						at91_set_gpio_value(SWITCH_CAL_COIL_PIN,arg);
						break;
	case CMD_SETSSCRFMR:
						RFMR_INT = arg;
						printk("RFMR_INT=%d \t",RFMR_INT);
						break;
	case CMD_SSC_START_MT: 
						init_waitqueue_head(&at91_pps_int);
						flag_pps = 0;
						init_waitqueue_head(&rtc_alarm);
						flag_rtcalarm = 0;
						ssc_stop();//防止CTRL C操作
						at91_set_gpio_value(SYNC_PIN,0);
						if (arg==1)
						{	
							wait_event_interruptible(rtc_alarm,flag_rtcalarm != 0);
							flag_rtcalarm = 0;
						}
						
						while(at91_get_gpio_value(PPS_IN_PIN)==1);//20150904
						ssc_begin();
						at91_set_gpio_value(SYNC_PIN,1);
						PPSWakeUpFlag=1;
						//wait_event_interruptible(at91_pps_int,flag_pps != 0);  //在FPGA中利用PPS触发SYNC，屏蔽掉ARM中的PPS中断
						flag_pps = 0;
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
	int retv, i=0;
	init_waitqueue_head(&transfer_wq0);
	ssc0_base = ioremap(AT91SAM9G45_BASE_SSC0, SZ_16K);
	if (!ssc0_base)
		return -ENOMEM;	
	IO_init();
	at91_spi_init();
	clear_alarm(RTC_ACQ_CS);	
	clear_wsqw();
	at91_ssc_io_init();
	at91_ssc_reg_init();
	at91_rtc_interrupt_init();    //数据采集RTC中断初始化
 	at91_pps_interrupt_init();

 	if (request_irq(AT91SAM9G45_ID_SSC0, ssc0_interrupt, IRQF_DISABLED , "ssc0", NULL))
				return -EBUSY;

	for(i=0;i<MemNum;i++)
	{
		pssc_buf[i] = (int *) __get_free_pages(GFP_KERNEL, RAM_ORDER);
		if (!pssc_buf[i])
					return -ENOMEM;
		memset(pssc_buf[i], 0, PAGE_SIZE << RAM_ORDER);

	}

	retv=register_chrdev(MFEMDEV_MAJOR,mfemdev_name,&mfem_fops);
	if(retv<0)
	{
		printk("Register Fail!\n");
		return retv;
	}
	printk("Register MFEM driver for SAM9G45 OK! 2019-03-01\n");
	#ifdef DEBUG
		printk("MTEM Debug Version\n");
	#endif
	return retv;
	
}


static void __exit at91_mfem_exit(void)  
{  
	int i=0;
	for(i=0;i<MemNum;i++)
	{
		free_pages((u32)pssc_buf[i], RAM_ORDER);
	}
	
	at91_ssc_exit();
	
	free_irq(AT91SAM9G45_ID_SSC0, 0);
	free_irq(gpio_to_irq(RTC_INT_PIN), 0);         
	free_irq(gpio_to_irq(PPS_IN_PIN), 0);
	
	iounmap(ssc0_base);
	at91_set_gpio_value(SSC_START_PIN, 0); 
	        
 	unregister_chrdev(MFEMDEV_MAJOR, mfemdev_name);   //ºÍÔ­À´µÄ²»Ò»ÑùÁË£¬Ã»ÓÐ·µ»ØÖµ  --HJJ
  	printk(KERN_INFO"Unregister MFEM driver ; 2019-03-01\n"); 
}  
  
module_init(at91_mfem_init);  
module_exit(at91_mfem_exit);

MODULE_DESCRIPTION("QY MTAMT TEST");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("QY");

