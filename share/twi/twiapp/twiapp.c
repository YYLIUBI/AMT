#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>

#define 	CMD_TWI_WRITE_TEST       10
#define 	CMD_TWI_READ_TEST        11

int opendev(void)
{ 
	int fd;
	fd = open("/dev/twi_ctl", O_RDWR);
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
	int ch,fd_twi;
	while((ch = getopt(argc,argv, "wri"))!= -1)
	{
		switch(ch)
		{
			case 'w':
				fd_twi = opendev();
				ioctl(fd_twi , CMD_TWI_WRITE_TEST);
				close(fd_twi);
				break;
			case 'r':
				fd_twi = opendev();
				ioctl(fd_twi , CMD_TWI_READ_TEST);
				close(fd_twi);				
				break;			
			default:
				Usage(EXIT_SUCCESS);
				break;
		} 
	}
	return 0;
}
