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

#define CMD_READ    10
#define CMD_WRITE   11


#define IIC_DATA_PIN    AT91_PIN_PA20
#define IIC_CLK_PIN     AT91_PIN_PA21

u8 spibuf[100];


#define IIC_DATA_HIGH()    (at91_set_gpio_value(IIC_DATA_PIN, 1))
#define IIC_DATA_LOW()    (at91_set_gpio_value(IIC_DATA_PIN, 0))
#define IIC_CLK_HIGH()    (at91_set_gpio_value(IIC_CLK_PIN, 1))
#define IIC_CLK_LOW()    (at91_set_gpio_value(IIC_CLK_PIN, 0))
#define IS_RESPONSE()	 (at91_get_gpio_value(IIC_DATA_PIN))

#define EEPROMDEV_MAJOR 106
char eepromdev_name[] = "eeprom";

static int eeprom_open(struct inode *inode,struct file *filp)
{
	return 0;
}

static int eeprom_close(struct inode *inode,struct file *filp)
{      
	return 0;
}



void iic_strat(void)  //start
{
	IIC_DATA_HIGH();
	udelay(2);
	IIC_CLK_HIGH();
	udelay(5);
	IIC_DATA_LOW();
	udelay(2);
	IIC_CLK_LOW();
}

void iic_stop(void)
{
	IIC_DATA_LOW();
	udelay(2);
	IIC_CLK_HIGH();
	udelay(5);
	IIC_DATA_HIGH();
	udelay(2);
 	//IIC_CLK_LOW();
}

int iic_send_byte(char data)  //first send high byte
{
	char i;
	for(i=0; i<8; i++)
	{
		
		IIC_CLK_LOW();
		udelay(2);	
		if(0x80 & data)
			{
				IIC_DATA_HIGH();
				printk("data:1\n");
			}
		else
			{
				IIC_DATA_LOW();
				printk("data:0\n");
			}	
		IIC_CLK_HIGH();
		udelay(2);	
		data <<= 1;	
	}	
	IIC_CLK_LOW();
	udelay(2);
	//IIC_DATA_HIGH();
	//udelay(10);
	//IIC_CLK_HIGH();
	//IIC_DATA_HIGH();
	return 1;
}

int iic_rcv_byte(void)
{
	u8 i, j;
	u8 result = 0;
	IIC_CLK_LOW();
	udelay(2);
	IIC_DATA_HIGH();
	for(i=0; i<8; i++)
	{
		//IIC_CLK_LOW();
		//udelay(10);
		IIC_CLK_HIGH();
		udelay(2);
		if(IS_RESPONSE()==1)
		{
			j = 0x01;
			printk("rcv:1\n");
		}
		else
		{
			j = 0;
			printk("rcv:0\n");
		}
		//udelay(15);
		result |= (j<<(7 - i));
		//udelay(5);
		IIC_CLK_LOW();
		udelay(2);
	}
	//udelay(2);
	return (result);
}


void clock(void)
{
	u8 i = 0;
	IIC_CLK_HIGH();
	udelay(2);
	while((IS_RESPONSE()==1)&&(i<255)) i++;
	printk("i:%d\n",i);
	IIC_CLK_LOW();
	udelay(2);
}
void iic_ack(void)         //响应信号
{
	IIC_DATA_LOW();			   
	IIC_CLK_HIGH();			   
	udelay(5); 	
	IIC_CLK_LOW();              
}
void iic_noack(void)       //非响应信号
{
	IIC_DATA_HIGH();			   
	IIC_CLK_HIGH();			   
	udelay(5); 	
	IIC_CLK_LOW();              
}

static int send_char(u8 devaddr,u8 dataaddr, u8 *data, u8 num)
{
	u8 i;
	iic_strat();
	iic_send_byte(devaddr);
	//udelay(50); 	
	clock();
	iic_send_byte(dataaddr);
    //udelay(50); 
	clock();
	//udelay(50);	
	for(i=0; i<num; i++)
	{		
		iic_send_byte(*data);
		//udelay(2);
		clock();
		data++;
	}
	iic_stop();
	//udelay(50);
	//IIC_CLK_HIGH();
	return 1;
}

static int rcv_char(u8 devaddr, u8 dataaddr, u8 *data, u8 num)
{
	u8 i;
	iic_strat();
	iic_send_byte(devaddr);
	//udelay(50);
	clock();
	iic_send_byte(dataaddr);
	//udelay(50);
	clock();		
	iic_strat();
	iic_send_byte(devaddr + 1);
	//udelay(50);
	clock();
	//udelay(50);
	for(i=0; i<num - 1; i++)
	{
		*data = iic_rcv_byte();
		//udelay(1000);
		iic_ack();
		//clock();
		printk("temp:%x\n",*data);
		data++;
	}
	*data = iic_rcv_byte();
	//udelay(2);
	printk("temp:%x\n",*data);
	iic_noack();
	//udelay(20);
	iic_stop();
	//IIC_CLK_HIGH();
	//udelay(50);
	return 1;
}


static int eeprom_write(struct file *fd, char *command,int count,loff_t *l)
{
	if (copy_from_user(spibuf,command,count))
		return -EFAULT;
	//write_spi(spibuf, count);	
  return 0;
}

static int eeprom_read(struct file *fd, u32 *command,int count,loff_t *l)
{

  return 0;
}


u8 temp[11];
u8 test[10] ={4,2,3,4,5,6,7,8,9};
static int eeprom_ioctl(struct inode * s_node,struct file * s_file,int cmd,unsigned long arg)
{ 	
  switch(cmd)
	{
		case CMD_WRITE:
				send_char(0xa2,0x02, test, 4);
				break;
		case CMD_READ: 
				rcv_char(0xa2, 0x02, temp, 4);	
				printk("read data:%x,%x,%x,%x\n",temp[0],temp[1],temp[2],temp[3]);
				break;
  		default:
    			break;
     }  
  return 0;
  }

static struct file_operations eeprom_fops=
{
	open	:(void(*))eeprom_open,
	write	:(void(*))eeprom_write,
	release	:(void(*))eeprom_close,  
	read	:(void(*))eeprom_read,  
	ioctl	:(void(*))eeprom_ioctl,  
};

static  int __init at91_eeprom_init(void)
{
	int retv;

	at91_set_gpio_output(IIC_CLK_PIN, 1);
	at91_set_gpio_output(IIC_DATA_PIN, 1);
	
	retv=register_chrdev(EEPROMDEV_MAJOR,eepromdev_name,&eeprom_fops);
	if(retv<0)
	{
		printk("Register Fail!\n");
		return retv;
	}
	printk("Register EEPROM driver for SAM9G45 OK! 2018-10-09\n");
	#ifdef DEBUG
		printk("MTEM Debug Version\n");
	#endif
	return retv;
}

static void __exit at91_eeprom_exit(void)  
{  
	        
 	unregister_chrdev(EEPROMDEV_MAJOR,eepromdev_name);   //ºÍÔ­À´µÄ²»Ò»ÑùÁË£¬Ã»ÓÐ·µ»ØÖµ  --HJJ
  	printk(KERN_INFO"Unregister EEPROM driver ; 2018-10-09\n"); 
}  
  
module_init(at91_eeprom_init);  
module_exit(at91_eeprom_exit);

MODULE_DESCRIPTION("QY EEPROM TEST");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("QY");


