#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>

//命令宏定义
#define		SPI_TEST_CMD			0x25
#define		SPI_SHOW_SEG_CMD		0x26

int opendev(void)
{ 
	int fd;

	fd = open("/dev/spi_ctl", O_RDWR);
	if(fd < 0)
	{
		printf("dev open fail!\n");
		return -1;
	}
	return fd;
}

int Usage(int status) 
{
	FILE   *fp;
	static char version[] = "\n gpioapp V1.0 \n";
	static char copyright[] = "copyright (C) 2010---2014 ZTX\n";
	fp = (status == EXIT_SUCCESS) ? stdout : stderr; 
	fprintf(fp, version);
	fprintf(fp, copyright);
	fprintf(fp, "usage: gpioapp -o on -f off -n on -c change \n");
	exit(status);
}

int main(int argc,char *argv[])
{
	int fd_dev;	
	int ch;
	while((ch = getopt(argc,argv, "tos:"))!= -1)
	{
		switch(ch)
		{
			case 'o':
				fd_dev = opendev();
				ioctl(fd_dev , SPI_TEST_CMD );
				close(fd_dev);
				break;
			case 's':
				fd_dev = opendev();
				ioctl(fd_dev , SPI_SHOW_SEG_CMD , atoi(argv[2]));
				close(fd_dev);
				break;
			case 't':
				fd_dev = opendev();
				int i = 0;
				for( i = 0;i < 10; i++){
					ioctl(fd_dev , SPI_SHOW_SEG_CMD , i);
					sleep(1);
				}
				close(fd_dev);
				break;
			default:
				Usage(EXIT_SUCCESS);
				break;
		} 
	}
	return 0;
}

