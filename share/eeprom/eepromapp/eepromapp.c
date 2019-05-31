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

#define CMD_READ    10
#define CMD_WRITE   11

#define u8 unsigned char

int file_exists(char *filename)
{
	return (access(filename, 0));
}

static int open_eeprom(void)
{ 
	int fd;
	fd = open("/dev/eeprom", O_RDWR);
	if(fd < 0)
	{
		printf("EEPROM open fail!\n");
		return -1;
	}
	return fd;
}
int Usage(int status) 
{
	FILE   *fp;
	static char version[] = "EEPROM_TEST 2018-10-09\n";
	static char copyright[] = "copyright (C) 2015\n";
	fp = (status == EXIT_SUCCESS) ? stdout : stderr; 
	fprintf(fp, version);
	fprintf(fp, copyright);
	fprintf(fp, "usage: eepromapp WR");
	fprintf(fp, "\n\n");
	exit(status);
}

int main(int argc,char *argv[])
{
	int fd,cmd;
	struct tm acqRtcTime;
    u8 rcvdata[10];
	
	while((cmd = getopt(argc,argv, "rw"))!= -1)
	{
		switch(cmd)
		{
			case 'r':   fd = open_eeprom();    
			       		ioctl(fd,CMD_READ,rcvdata);
			       		printf("rcvdata:%x\n", *rcvdata);
			       		close(fd);
			            break;

			case 'w':   fd = open_eeprom(); 
                        ioctl(fd,CMD_WRITE);
                        close(fd);
                        break;
        default:
			Usage(EXIT_SUCCESS);
			break;
		} 
	}	
	close(fd);
	return 0;
}



