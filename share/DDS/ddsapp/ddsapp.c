#include "type.h"
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <getopt.h>
#include <time.h>
#include <sys/time.h>
#include <linux/rtc.h>
#include <math.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <sys/ipc.h>
#include <sys/shm.h> 
#include <dirent.h>

#define CMD_TWI_WRITE_TEST       10
#define CMD_TWI_READ_TEST        11
#define CMD_TWI_SET_ADDR     	 12
#define CMD_TWI_SET_REGADDR      13
#define CMD_TWI_REG_NONE         14
#define CMD_TWI_REG_YES          15


//OBEM
#define CMD_SET_DDS_FREQ         37
#define CMD_SET_DDS_SINWAVE      35
#define CMD_SET_DDS_SQUARE       36

#define	CMD_STATUS_LED_ON	     77
#define	CMD_STATUS_LED_OFF	     78
#define u8 unsigned char
#define u32 unsigned int

u8 eepromAddr = 0x51;

int opendds(void)
{ 
	int fd;

	fd = open("/dev/dds", O_RDWR);
	if(fd < 0)
	{
		printf("obem open fail!\n");
		return -1;
	}
	return fd;
}

void set_ddsfreq(u32 ddsfreq)
{	
	int fd;	
	u8 regAddr = 4;	
	u8 ddsBuf[4];
	int i = 0;  //Ñ­»·±äÁ¿
	
	ddsBuf[0] = (u8)(ddsfreq >> 24 );  //MSB
	ddsBuf[1] = (u8)(ddsfreq >> 16 );   //MLSB
	ddsBuf[2] = (u8)(ddsfreq >> 8 ); //MLSB
	ddsBuf[3] = (u8)ddsfreq;   //LSB
	 
	fd = opendds();	
	ioctl(fd, CMD_TWI_SET_ADDR,&eepromAddr);
	ioctl(fd, CMD_TWI_REG_YES);  //ÓÐÄÚ²¿µØÖ·	
	ioctl(fd, CMD_TWI_SET_REGADDR,&regAddr);  //ÄÚ²¿µØÖ·ÊÇ4	

	for( ; i < 4 ; i ++)
	{
		ioctl(fd, CMD_TWI_SET_REGADDR,&regAddr);
		write(fd,ddsBuf + i,1);	
		usleep(1000);
		regAddr ++;
	}
	
	printf("Set DDS freq:%u , %02X %02X %02X %02X\n",ddsfreq,ddsBuf[0],ddsBuf[1],ddsBuf[2],ddsBuf[3]);	
	close(fd);	
}

int get_ddsfreq(void)
{	
	int fd;	
	u32 ddsfreq = 0;
	u8 regAddr = 4;	
	u8 ddsBuf[4];
	int i = 0;  //Ñ­»·±äÁ¿
	memset(ddsBuf,0x00,4); //clear array
	
	fd = opendds();	
	ioctl(fd, CMD_TWI_SET_ADDR,&eepromAddr);
	ioctl(fd, CMD_TWI_REG_YES);  //ÓÐÄÚ²¿µØÖ·
	

	for( ; i < 4 ; i ++)
	{
		ioctl(fd, CMD_TWI_SET_REGADDR,&regAddr);
		read(fd,ddsBuf + i,1);
		usleep(1000);
		regAddr ++;
	}
	close(fd);	
	ddsfreq =  ((int)ddsBuf[0] << 24) + ((int)ddsBuf[1] << 16) + ((int)ddsBuf[2] << 8) + (int)ddsBuf[3];
	printf("DDS freq get:%u , %02X %02X %02X %02X\n",ddsfreq,ddsBuf[0],ddsBuf[1],ddsBuf[2],ddsBuf[3]);
	return ddsfreq;
}

void Set_DDS(void)
{
	int fd; 
	u32 ddsFreqGet1 = 0;
	u32 ddsFreqGet2 = 0;
	int readCnt = 0;

    while(readCnt < 10)
    {
		ddsFreqGet1 = get_ddsfreq();
		usleep(2000);
		ddsFreqGet2 = get_ddsfreq();

		if(ddsFreqGet2 == ddsFreqGet1)
			break;

		readCnt ++ ;
    }

	fd = opendds();
	ioctl(fd,CMD_SET_DDS_FREQ,&ddsFreqGet1);
	ioctl(fd, CMD_SET_DDS_SQUARE);
	close(fd);
}


int Usage(int status) 
{
	FILE   *fp;
	static char version[] = "\nlsetddsapp V1.0 \n";
	static char copyright[] = "copyright (C) 2010---2012 HJJ\n";
	fp = (status == EXIT_SUCCESS) ? stdout : stderr; 
	fprintf(fp, version);
	fprintf(fp, copyright);
	fprintf(fp, "usage: ledapp -c change -f off -n on\n\n");
	exit(status);
}

int main(int argc,char *argv[])
{
	int fd_dds;
	//int fd_dds;
	u32 ddsfreq = 0;

	int ch;
	while((ch = getopt(argc,argv, "s:gdwr"))!= -1)
	{
		switch(ch)
		{
			case 's':
				set_ddsfreq(atoi(argv[2]));  //string to int
				break;
			case 'g':
				ddsfreq = get_ddsfreq();
				//printf("%x\n", );
				break;
			case 'd':
				Set_DDS();	
				break;
			case 'w':
				fd_dds = opendds();
				ioctl(fd_dds , CMD_TWI_WRITE_TEST);
				close(fd_dds);
				break;
			case 'r':
				fd_dds = opendds();
				ioctl(fd_dds , CMD_TWI_READ_TEST);
				close(fd_dds);				
				break;			
			default:
				Usage(EXIT_SUCCESS);
				break;
		} 
	}
	return 0;
}


