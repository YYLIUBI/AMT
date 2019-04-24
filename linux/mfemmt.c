/***************************************************************
* China University of Geoscience BeiJing
* www.cugb.edu.cn
* File Name: mfemmt.c
* Description: mt/amt control
* Author: CK ck@cugb.edu.cn
***************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
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
#include <errno.h>

#include "macro_def.h"
#include "gps_rtc.h"
//#include "build_prm.h"
#include "ad.h"
#include "tbl.h"
#include <sys/ipc.h>   
#include <sys/shm.h> 
#include"fcntl.h"
#include"dirent.h"


struct timeval tpstart,tpend; 
float timeuse; 

//struct surprm sur_prm;


#define CMD_SYS_CHECK        0x14   //系统自检命令
#define CMD_ECORRODE         0x15   //电腐蚀命令
#define		AD_REF			 2.5
#define 	VOL_DIV			 11
#define CMD_MS_COUNT_START   33
#define CMD_MS_COUNT_CLEAR   34


extern u8 eepromAddr ;
extern u8 adeepromAddr;  
extern u8 hAmpBoardAddr;  
extern u8 tmpAddr ;
extern u8 ad0Addr ;
extern u8 ad5Addr;  
extern u8 msp430g1Addr;//13 0509


//判断文件是否存在 如果存在就返回0 如果不存在就返回 -1
int file_exists(char *filename)
{
	return (access(filename, 0));
}

static int open_dev(void)
{ 
	int fd;
	fd = open("/dev/mfem", O_RDWR);
	if(fd < 0)
	{
		printf("mfem open fail!\n");
		return -1;
	}
	return fd;
}

int Usage(int status) 
{
	FILE   *fp;
	static char version[] = "MFEMMT 2016-09-13\n";
	static char copyright[] = "copyright (C) 2010----2020\n";
	fp = (status == EXIT_SUCCESS) ? stdout : stderr; 
	fprintf(fp, version);
	fprintf(fp, copyright);
	fprintf(fp, "usage: mfemapp H:gsS:E:I:B:t:C:T:mnflp:abc");
	fprintf(fp, "\n\n");
	exit(status);
}

extern int sock;//sendto中使用的对方地址
extern struct sockaddr_in ClientAddr;
u8 nTotalAcqPeriod;
u8 nCurAcqPeriod;
int acq_total_ok=1;
u32 nTotalAcqLen = 0;//记录本次采集的总长度
u32 nCurAcqNum = 0;       //记录当前的采集长度


#define BUF_SIZE 1   
#define MYKEY 1920
char *ACQFLAG=0;

u8 *TimeBuf = NULL;
//struct gps_info gpsinit;
struct gps_info *pgpsinit ;//= &gpsinit;

int shmid,shmid1;  
u8 *FileNameBuf="199011AA.TS*";

u32 *ACQNUM;


int acq_sur()
{	
	char display_on = 0;
	struct tm rtc_time;
	char i=0;
	u32 actualDiskSpace = 0;
	int fd = 0;
	acq_total_ok=0;
	fd = open_dev();

	ioctl(fd,CMD_SETHLSC,1);
	//sleep(10);
	close(fd);
	set_filename();
	//printf("set_filename finish!\n");
	ad_init_reg(); 	
	//printf("ad_init_reg finish!\n");
	acq_onerate();		
	//printf("acq_onerate finish!\n");
	acq_total_ok=1;

	return 0;	
}

/*void lock_gps(void)
{
	int fd;
	char str_temp[30];
	char str_datetime[30];
	struct gps_info gpsinfo;
	struct gps_info *pgpsinfo = &gpsinfo;
	FILE *fp;
	
	fd = open_dev();
  	if(file_exists("/mnt/sd/timelog.txt") == 0){
	  	fp = fopen("/mnt/sd/timelog.txt", "ab+"); 
		if(NULL == fp)	return;
	}
  	else
  	{
	  	fp = fopen("/mnt/sd/timelog.txt", "wb");
	  	if(NULL == fp)	return;
  	}
  	
	gettimeofday(&tpstart,NULL);   //开始计时
	
	if (wait_pps()!=0)	return;		

	if (get_gps(pgpsinfo)!=0)
		printf("GPS NOT EXIST!");
	else
	{ 
		//等待PPS 再写入RTC
		wait_pps();
		set_rtc(&pgpsinfo->gpstime,RTC_ACQ_CS);       //HJJ 11.03 修改成GMT时间
		set_rtc(&pgpsinfo->gpstime,RTC_ECORRODE_CS);  //写入电腐蚀RTC的时间
		fprintf(fp,"lock gps time:%04d-%02d-%02d %02d:%02d:%02d\n",
																	pgpsinfo->gpstime.tm_year+1900,
																	pgpsinfo->gpstime.tm_mon+1,
																	pgpsinfo->gpstime.tm_mday,
																	pgpsinfo->gpstime.tm_hour,
																	pgpsinfo->gpstime.tm_min,
																	pgpsinfo->gpstime.tm_sec);
		fclose(fp);  //及时关闭文件
	
		ioctl(fd,CMD_MS_COUNT_START);    //MS 开始计数
		
    	gettimeofday(&tpend,NULL);       //计时停止
	  	timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ tpend.tv_usec-tpstart.tv_usec; //该时间单位为微妙
    	printf("Lock GPS Used Time:%d ms\n",(int)(timeuse / 1000)); 
  
  		//写入系统时间 GMT
		sprintf(str_temp, "%02d%02d%02d%02d%04d.%02d",
		pgpsinfo->gpstime.tm_mon + 1, pgpsinfo->gpstime.tm_mday, 
		pgpsinfo->gpstime.tm_hour, pgpsinfo->gpstime.tm_min, 
		pgpsinfo->gpstime.tm_year + 1900,pgpsinfo->gpstime.tm_sec);
		sprintf(str_datetime, "date -u \"%s\"", str_temp);
		system(str_datetime);  

		//read_rtc(&pgpsinfo->gpstime);
		read_rtc(&pgpsinfo->gpstime,RTC_ACQ_CS);
		
		printf("lock gps ok!\n");
	}
	close(fd);
} */

/*
	函数功能:将浮点型数据转换成整形，CTD串口监听线程中使用
*/
/*int Float2Int(float floatData)
{
	int int_val = 0;
	float float_val = 0;
	float_val = floatData;
	int *p;
	p=(int *)&float_val;
	int_val = (*p);
	return int_val;
}*/
unsigned long get_file_size(const char *path)  
{  
    unsigned long filesize = -1;  
    FILE *fp= fopen(path, "r");  
    fseek(fp, 0L, SEEK_END);  
    filesize = ftell(fp);  
    fclose(fp);  
    return filesize;  
}

void ShareMemSet()
{ 
	if((shmid = shmget(1990,1,IPC_CREAT|0666) ==-1) ) 
    {  
    printf("shmget error \n");  
    //exit(1);  
    }  
  
    if((ACQFLAG =shmat(shmid,0,0))==(void *)-1)  
    {  
    printf("shmat error!\n");  
    //exit(1);  
    }  

	if((shmid = shmget(1992,sizeof(struct gps_info),IPC_CREAT|0666)) ==-1)  
	{  
		printf("shmget1 error \n");	
	//	exit(1);  
	}  
	  
	if((TimeBuf =shmat(shmid,0,0))==(void *)-1)  
	{  
		printf("shmat1 error!\n");  
	//	exit(1);  
	} 
    pgpsinit = (struct gps_info*)TimeBuf;

	if((shmid = shmget(1994,12,IPC_CREAT|0666)) ==-1)  
	{  
		printf("shmget2 error \n");	
	//	exit(1);  
	}  
	  
	if((FileNameBuf=shmat(shmid,0,0))==(void *)-1)  
	{  
		printf("shmat2 error!\n");  
	//	exit(1);  
	} 
	if((shmid = shmget(1996,4,IPC_CREAT|0666)) ==-1)  
		{  
			printf("shmget2 error \n"); 
		//	exit(1);  
		}  

	if((ACQNUM=shmat(shmid,0,0))==(void *)-1)  
	{  
		printf("shmat2 error!\n");  
	//	exit(1);  
	} 
}

void LockPPS()
{
	char time[11];
    //struct *pgpsinit = &pgpsinit;
	int fd_driver=open("/dev/mfem", O_RDWR);	
	ioctl(fd_driver, CMD_MS_COUNT_CLEAR);
	get_gps(pgpsinit);
	ioctl(fd_driver, CMD_MS_COUNT_START);
    sprintf(time , "%02d%02d%02d%02d%02d%02d",
			pgpsinit->gpstime.tm_hour,
			pgpsinit->gpstime.tm_min,
			pgpsinit->gpstime.tm_sec,
            pgpsinit->gpstime.tm_mday,
            pgpsinit->gpstime.tm_mon+1,
            pgpsinit->gpstime.tm_year);
	set_rtc_cmd(time,RTC_ACQ_CS);	
	close(fd_driver);
}

int main(int argc,char *argv[])
{
	acq_total_ok=1;char *p="010511392014";u8 a=0;

	int fd,cmd;int i = 0;int res = 0; FILE *fptest;char testname[15],FILENAME[15];
	char display_on = 0;
	
	struct tm acqRtcTime;
	u8 tmp_str[10];
	ShareMemSet();

	fd = open("/dev/mfem", O_RDWR);	
	while((cmd = getopt(argc,argv, "hgsrl:EiB:t:cTmnflp:aV"))!= -1)
	{
		switch(cmd)
		{
			
			case 'g':	
						read_rtc(&acqRtcTime,RTC_ACQ_CS);//获取数据采集的RTC时间信息	

						//print_gps();
						//get_powerboard();
						//printf("FileName:%s\n",FileNameBuf);
						//printf("SYNC TIME:%s\n",TimeBuf);
						//printf("CurrentACQ:%d@\n",*ACQNUM);
				break;
			case 's':
				LockPPS();
				break;
		/*	case 'b':
				ioctl(fd, CMD_CONFIG1282);
				printf("config\n");break;
			case 'd':
				ioctl(fd, CMD_READ1282);
				printf("read\n");break;
			*/	
			case 'r':
				//ioctl(fd, CMD_SYNC_NO_PPS);20160505
				break;
			case 'h':
				//ioctl(fd, CMD_SYNC);	20160505
				set_rtc_cmd(p,RTC_ACQ_CS);
				break;
			case 'E':
				*ACQFLAG=1;
				LockPPS();				
				acq_sur();
				break;
			case 'i':
				read_rtc(&acqRtcTime,RTC_ACQ_CS);
				//display_on = 0;
				//build_prm_list(argv[2], display_on);
				//ad_init(&acq_prm_list[0]);
				break;
			case 'B':
				display_on = 1;
				//build_prm_list(argv[2], display_on);
				break;
			case 'c':
				*ACQFLAG = 0;
				break;
			case 'T':
				//buf[11]=0;buf[10]=0;buf[9]=0x28;buf[8]=0;
				//mkTimeSlot(TimeSlot,buf,TimeBuf);
				//tblInit();
				//nametag();
				//ads1282_init();
				//ioctl(fd, CMD_SEL_AMTMT,AMTMTFLAG); 
				//sprintf(testname,"%d%X%c%c%c",8000,10,'9','A','A');//测试文件名，无法大写
				//sprintf(FILENAME,"%s.TS6",testname);
				//fptest=fopen(FILENAME,"wr");
				//fclose(fptest);
				break;
			case 'm':
				printf("SYNC TIME:%s\n",TimeBuf);//LockPPS();LockPPS();//GetCPLDCount();
	      	     break;
			case 'a':   //测试CPLD  SPI
			    	fd = open_dev();
		    		for(i = 0 ; i < 100; i ++)
		    		{
						ioctl(fd,CMD_CPLD_SPI_TEST); //CPLD SPI测试
						//usleep(10000);
					}
					close(fd);
					printf("CPLD SPI Read Test OK\n");
					/*
					//测试RTC
					for(i = 0 ; i < 100; i ++)
						{
						read_rtc(&rtc_time);	
						sleep(1);
					}
					*/
			     break;
			case 'V':
				printf("Mfemmt Version:2018-12-07\n");
	      	break;		
		default:
			Usage(EXIT_SUCCESS);
			break;
		} 
	}	
	close(fd);
	return 0;
}
