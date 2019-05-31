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

#define CMD_SSC_START        22
#define CMD_SSC_STOP         23

#define CMD_WAIT_PPS         27
#define CMD_WAIT_RTCALARM    28

#define PPS_IN_PIN            		AT91_PIN_PB21
#define SSC_START_PIN         		AT91_PIN_PC0
#define SYNC_PIN         			AT91_PIN_PA31

#define RAM_ORDER 5 

#define MFEMDEV_MAJOR  101	
char mfemdev_name[]="ssc";

static void __iomem *ssc0_base;
#define at91_ssc0_read(reg)		    __raw_readl(ssc0_base + (reg))
#define at91_ssc0_write(reg, val)	__raw_writel((val), ssc0_base + (reg))
	
static int ssc_begin(void);

static DECLARE_WAIT_QUEUE_HEAD(transfer_wq0);

u8 SSCRevFlag = 0;   
u8 flag_A = 0;
u8 flag_B = 0;

u8 flag_1 = 0;


int int_cnt0 = 0;  //SSC0
int int_cnt1 = 0;  //SSC0

char PPSWakeUpFlag = 0; 
static DECLARE_WAIT_QUEUE_HEAD(at91_pps_int);
static int flag_pps = 0;

static int mfem_open(struct inode *inode,struct file *filp)
{
	return 0;
}

static int mfem_close(struct inode *inode,struct file *filp)
{      
	return 0;
}

static void IO_init(void)
{

	at91_set_gpio_output(SSC_START_PIN,0);
}

static irqreturn_t at91_pps_interrupt(int irq, void *dev_id)
{
	if(PPSWakeUpFlag)
	{//printk("PPS!\n");
		if(at91_get_gpio_value(PPS_IN_PIN)==1)  
		{	flag_pps = 1;
			at91_ssc0_write(AT91_SSC_CR , AT91_SSC_RXEN);
			at91_set_gpio_value(SSC_START_PIN, 1);  
			wake_up_interruptible(&at91_pps_int);
	   		PPSWakeUpFlag=0;
	   	}		
	} //CPLD
	return IRQ_RETVAL(IRQ_HANDLED);
}

static int at91_pps_interrupt_init(void)
{
	at91_set_gpio_input(PPS_IN_PIN,1);   //设置引脚输入
	gpio_irq_unmask(PPS_IN_PIN); 		 //使能引脚中断
	gpio_irq_type(PPS_IN_PIN,1);  		 //设置中断类型IRQF_IRIGGER_RISING
	

	if(request_irq(gpio_to_irq(PPS_IN_PIN),at91_pps_interrupt, IRQF_DISABLED ,"mfem-pps",NULL)<0)
		printk("PPS interrupt request fail!\n");
	return 0;
}

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


static int mfem_write(struct file *fd, char *command,int count,loff_t *l)
{
	if (copy_from_user(spibuf,command,count))
		return -EFAULT;
	write_spi(spibuf, count);	
  return 0;
}


//如果想多次使用DMA流映射，同时在DMA传输过程中访问数据，此时需要合适地同步数据缓冲区，
//这样可以让处理器及外设可以看到最新的更新和正确的DMA缓冲区数据
//dma_sync_single_for_cpu()在一次DMA传输完成后，访问DMA缓冲区并将数据传输到CPU内部缓冲区
//dma_sync_single_for_device()将缓冲区传给硬件前，让设备再次获得DMA缓冲区，完成CPU对数据的访问
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

static int mfem_ioctl(struct inode * s_node,struct file * s_file,int cmd,unsigned long arg)
{ 	
  switch(cmd)
	{
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
						ssc_begin();
						PPSWakeUpFlag=1;
						//wait_event_interruptible(at91_pps_int,flag_pps != 0);  //在FPGA中利用PPS触发SYNC，屏蔽掉ARM中的PPS中断
						flag_pps = 0;
						at91_set_gpio_value(SYNC_PIN,1);
						break;	
	case CMD_SSC_STOP:
						ssc_stop();
						break;		

	case CMD_SEL_MODE:
						//arg=1 sip arg=4 csamt arg=5 AMT arg=6 MT
						at91_set_gpio_value(MODE_SEL0,arg%2); 
						at91_set_gpio_value(MODE_SEL1,(arg/2)%2); 
						at91_set_gpio_value(MODE_SEL2,(arg/4)%2); 
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
						at91_set_gpio_value(BGAIN0_PIN,arg%2); 
						at91_set_gpio_value(BGAIN1_PIN,arg/2); 
						break;
	case CMD_SETSSCRFMR:
						RFMR_INT = arg;
						printk("RFMR_INT=%d \n",RFMR_INT);
						break;
	case CMD_SETGAINH:
						at91_set_gpio_value(HGAIN0_PIN,arg%2); 
						at91_set_gpio_value(HGAIN1_PIN,arg/2); 
						break;
	case CMD_SETCAL:
						at91_set_gpio_value(SWITCH_CAL_PIN,arg);	
						break;					
	case CMD_SETCAL_COIL:
						at91_set_gpio_value(SWITCH_CAL_COIL_PIN,arg);
						break;
	case CMD_SSC_START_MT: 
						init_waitqueue_head(&at91_pps_int);
						flag_pps = 0;
						//init_waitqueue_head(&rtc_alarm);flag_rtcalarm = 0;
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
	int retv,i=0;
	init_waitqueue_head(&transfer_wq0);
	ssc0_base = ioremap(AT91SAM9G45_BASE_SSC0, SZ_16K);
	if (!ssc0_base)
		return -ENOMEM;	

	IO_init();
	at91_ssc_io_init();
	at91_ssc_reg_init();
   	//at91_rtc_interrupt_init();    //数据采集RTC中断初始化
 	at91_pps_interrupt_init();
	//Read1282;
	
	if (request_irq(AT91SAM9G45_ID_SSC0, ssc0_interrupt, IRQF_DISABLED , "ssc0", NULL))
				return -EBUSY;

	for(i=0;i<MemNum;i++)
	{
		pssc_buf[i] = (int *) __get_free_pages(GFP_KERNEL, RAM_ORDER);//分配2^5（页） * 4kb = 128kb
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
	printk("Register SSC driver for SAM9G45 OK! 2017-10-25\n");
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
	//free_irq(gpio_to_irq(RTC_INT_PIN), 0);         
	free_irq(gpio_to_irq(PPS_IN_PIN), 0);
	
	iounmap(ssc0_base);
	at91_set_gpio_value(SSC_START_PIN, 0);  
 	unregister_chrdev(MFEMDEV_MAJOR, mfemdev_name);   //和原来的不一样了，没有返回值  --HJJ
  	printk(KERN_INFO"Unregister SSC driver ; 2018-11-15\n"); 
}  
  
module_init(at91_mfem_init);  
module_exit(at91_mfem_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("QY");
MODULE_DESCRIPTION("MFEM driver for SAM9G45");
 
