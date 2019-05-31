/***************************************************************
* China University of Geoscience BeiJing
* www.cugb.edu.cn
* File Name: mefmcal.c
* Description: mfem cal control
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
#include "build_prm.h"
#include "ad.h"
//#include "crc.h"
#include <sys/ipc.h>   
#include <sys/shm.h> 

#define CMD_SETSSCRFMR	     62
#define CMD_SETCAL_COIL      64
#define	CMD_SETGAIN			 57
#define CMD_SETCAL		     61
struct timeval tpstart,tpend; 
float timeuse; 
struct gps_info *pgpsinit ;
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
		printf("MTEM open fail!\n");
		return -1;
	}
	return fd;
}

int Usage(int status) 
{
	FILE   *fp;
	static char version[] = "mfemcal 2016-06-13 Success! \n";
	static char copyright[] = "copyright (C) 2010----2020\n";
	fp = (status == EXIT_SUCCESS) ? stdout : stderr; 
	fprintf(fp, version);
	fprintf(fp, copyright);
	fprintf(fp, "usage: mtemapp H:gsEc");
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
u8 *TimeBuf="000000,000000";
int shmid,shmid1;  



int acq_sur(char *p_prm_filename)
{
	char display_on = 0;//CH 20150804	
	struct tm rtc_time;
	char i=0;
	u32 actualDiskSpace = 0;
	int fd = 0;
	fd = open("/dev/mfem", O_RDWR);
	ioctl(fd, CMD_SEL_MODE,0);	
	ioctl(fd, CMD_SETSSCRFMR,7);
	ioctl(fd, CMD_BUFFER_SET,128*1024);
	
	
	
	acq_total_ok=0;
	
	build_prm(p_prm_filename, display_on);
    if(file_exists("/mnt/sd/cal/") != 0)
		system("mkdir /mnt/sd/cal");
	
    switch(calprm.cal_coil)
    {
    	case 0:
    		ioctl(fd, CMD_SETHLSC,2);
    		ioctl(fd, CMD_OPEN_COILCAL,0);
			ioctl(fd, CMD_SETCAL,0);
			ioctl(fd, CMD_SETCAL_COIL,1);
			//ioctl(fd, CMD_SEL_MODE,3);
			ioctl(fd, CMD_SETGAIN,0);
    		printf("BOX CAL!!!\n");
		break;

		case 1:
		    ioctl(fd, CMD_SETHLSC,1);
		    ioctl(fd, CMD_OPEN_COILCAL,1);
			ioctl(fd, CMD_SETCAL,1);
			ioctl(fd, CMD_SETCAL_COIL,0);
			//ioctl(fd, CMD_SEL_MODE,7);
			ioctl(fd, CMD_SETGAIN,0);//16
			//ioctl(fd, CMD_CONFIG1282);//20160506
			//usleep(6000);
			//ioctl(fd, CMD_READ1282);	
			//usleep(600000);
			//ioctl(fd, CMD_CONFIG1282);//20160506
			//usleep(6000);
			//ioctl(fd, CMD_READ1282);
			printf("COIL CAL!!!\n");
		break;
		//case 2:
		//	ioctl(fd, CMD_SEL_MODE,0);break;
		default:
			ioctl(fd, CMD_SEL_MODE,0);
			break;
    }
	    

	for (i = 0; i < ACQ_NUM; i++)
	{
		printf("\nCAL_ID_%C begin!\n",i+'A');	
		sleep(1);//2s		
		adc_init();
		
		//read_rtc(&rtc_time,RTC_ACQ_CS);
		nTotalAcqLen = acq_prm_list[i].callen; 
		nCurAcqPeriod=i+1;
		set_filename(&acq_prm_list[i]);
		//if (acq_onerate(&acq_prm_list[i],sur_prm.lockgps,0) == -1)
		//	continue;
//		acq_onerate(&acq_prm_list[i],sur_prm.lockgps);	
		acq_onerate(&acq_prm_list[i]);		
		//tranfile(acq_prm_list[i].file_name);		
	}
	printf("acq total finish!\n");
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
}
*/
/*
	函数功能:将浮点型数据转换成整形，CTD串口监听线程中使用
*/
int Float2Int(float floatData)
{
	int int_val = 0;
	float float_val = 0;
	float_val = floatData;
	int *p;
	p=(int *)&float_val;
	int_val = (*p);
	return int_val;
}
unsigned long get_file_size(const char *path)  
{  
    unsigned long filesize = -1;  
    FILE *fp= fopen(path, "r");  
    fseek(fp, 0L, SEEK_END);  
    filesize = ftell(fp);  
    fclose(fp);  
    return filesize;  
}
struct gps_info *pgpsinit ;
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
	acq_total_ok=1;
	char *p="010511392014";
	int fd,cmd;int i = 0;int res = 0; 
	char display_on = 0;
	struct tm acqRtcTime;
	u8 tmp_str[10];
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

	if((shmid1 = shmget(1992,14,IPC_CREAT|0666)) ==-1)  
	{  
		printf("shmget1 error \n");	
	//	exit(1);  
	}  
	  
	if((TimeBuf =shmat(shmid1,0,0))==(void *)-1)  
	{  
		printf("shmat1 error!\n");  
	//	exit(1);  
	} 
	pgpsinit = (struct gps_info*)TimeBuf;
	//memset(TimeBuf,0,13);TimeBuf[6]=',';TimeBuf[13]='\0';

	fd = open("/dev/mfem", O_RDWR);	
	while((cmd = getopt(argc,argv, "h:gsr:l:E:iB:t:cT:mnflp:a"))!= -1)
	{
		switch(cmd)
		{
			case 'g':	
				read_rtc(&acqRtcTime,RTC_ACQ_CS);//获取数据采集的RTC时间信息		
				//print_gps();
				//get_powerboard();	
				break;
			case 's':
				LockPPS();
				break;
			case 'h':
				ioctl(fd, CMD_SETHLSC,argc);
				break;
//			case 'r':
//				*ACQFLAG=0;
//			//	ReadHSAD(argv[2]);
//				break;
			case 'E':
				*ACQFLAG=1;
				LockPPS();
				acq_sur(argv[2]);
				break;
			case 'i':
				read_rtc(&acqRtcTime,RTC_ACQ_CS);
				//display_on = 0;
				//build_prm_list(argv[2], display_on);
				//ad_init(&acq_prm_list[0]);
				break;
			case 'B':
				display_on = 1;
				build_prm(argv[2], display_on);
				break;
			case 'c':
				*ACQFLAG=0;
				break;
//			case 'T':
//				tranfile(argv[2]);
//				break;
//			case 'm':
//				printf("SYNC TIME:%s\n",TimeBuf);//LockPPS();LockPPS();//GetCPLDCount();
//	      	break;
//			case 'a':   //测试CPLD  SPI
//			    	fd = open_dev();
//			break;
		default:
			Usage(EXIT_SUCCESS);
			break;
		} 
	}	
	close(fd);
	return 0;
}
