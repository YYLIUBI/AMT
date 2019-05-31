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

//#include "gps_rtc.h"
#include <sys/ipc.h>
#include <sys/shm.h> 
#include <dirent.h>



#define BCD2BIN(val)	(((val) & 0x0f) + ((val)>>4)*10)
#define BIN2BCD(val)	((((val)/10)<<4) + (val)%10)
#define BUFSIZE 526
#define u8 unsigned char
u8 buff[BUFSIZ];
int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
                  B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,  
                  19200,  9600, 4800, 2400, 1200,  300, };

#define CMD_SPI_TEST         10
#define CMD_SPI_WRITE_TEST   11
#define CMD_ACCESS_SPI       12
#define CMD_RELEASE_SPI      13 
#define CMD_RTC_READ         14
#define CMD_RTC_WRITE        15
#define RTC_ACQ_CS           3
#define FALSE 				 0
#define TRUE                 1

int file_exists(char *filename)
{
	return (access(filename, 0));
}

static int open_mfem(void)
{ 
	int fd;
	fd = open("/dev/mfem", O_RDWR);
	if(fd < 0)
	{
		printf("OBEM open fail!\n");
		return -1;
	}
	return fd;
}
int Usage(int status) 
{
	FILE   *fp;
	static char version[] = "MFEM-SAM9G45 2015-04-14\n";
	static char copyright[] = "copyright (C) 2015\n";
	fp = (status == EXIT_SUCCESS) ? stdout : stderr; 
	fprintf(fp, version);
	fprintf(fp, copyright);
	fprintf(fp, "usage: mfemapp sr");
	fprintf(fp, "\n\n");
	exit(status);
}

// mon day hour min sec  year weekday
//void set_rtc_cmd(char *p)
int weekcal(char *p) // Zeller formula
{
	int year=(*(p+10)-'0')*10 + (*(p+11)-'0'),centery=20,month=(*(p+8) - '0') * 10 + (*(p+9) -'0'),day=(*(p+6) - '0') * 10 + *(p+7)-'0';
	int week;
	if (month==1||month==2)
	 {
	 	year=year-1;
	 	if(year<0) 
	 	{
	 		year=year+100;
	 		centery=centery-1;
	 	}
	 	month=month+12;
	 }
	week=year+(year/4)+(centery/4)-2*centery+(26*(month+1)/10)+day-1;
	week=week%7;
	if(week<0) 
		week=week+7;
	return week;
}
//"000000,000000"

int set_rtc(struct tm *tval,u8 rtccs)
{
	int fd;
	u8 rtcdata[9];
	u8 cs = rtccs;
	rtcdata[0] = BIN2BCD(tval->tm_sec);
	rtcdata[1] = BIN2BCD(tval->tm_min);
	rtcdata[2] = BIN2BCD(tval->tm_hour);
	rtcdata[3] = BIN2BCD(tval->tm_wday);
	rtcdata[4] = BIN2BCD(tval->tm_mday);
	rtcdata[5] = BIN2BCD(tval->tm_mon) + 1 + (0 << 7);
	rtcdata[6] = BIN2BCD(tval->tm_year % 100);
	fd = open_mfem();
	ioctl(fd,CMD_ACCESS_SPI,&cs);
	ioctl(fd,CMD_RTC_WRITE,rtcdata);
	ioctl(fd,CMD_RELEASE_SPI,&cs);
	close(fd);
	//printf("setdata:%x\n",*rtcdata);
	return 0;
}

void set_rtc_cmd(char *p,u8 rtccs)
{
	struct tm tmp_time;
	u8 cs = rtccs;
	
	tmp_time.tm_sec = (*(p+4) - '0') * 10 + (*(p+5) -'0');
	tmp_time.tm_mon = (*(p+8) - '0') * 10 + (*(p+9) -'0')-1;
	tmp_time.tm_mday = (*(p+6) - '0') * 10 + *(p+7)-'0';
	tmp_time.tm_hour = (*(p+0) - '0') * 10 + *(p+1)-'0';
	tmp_time.tm_min = (*(p+2) - '0') * 10 + *(p+3)-'0';
	tmp_time.tm_year = 2000 + (*(p+10)-'0')*10 + (*(p+11)-'0') - 1900;
	tmp_time.tm_wday = weekcal(p);//(*(p+12) - '0');
	
	//sleep(1);
	//set_rtc(&tmp_time);
	set_rtc(&tmp_time,cs);
}
int read_rtc(struct tm *tval,u8 rtccs)
{
	int fd;
	u8 rtcdata[9];
	int res=0;
	u8 cs = rtccs;

	fd = open_mfem();
	ioctl(fd,CMD_ACCESS_SPI,&cs);
	ioctl(fd,CMD_RTC_READ,rtcdata);
	ioctl(fd,CMD_RELEASE_SPI,&cs);
	close(fd);
	
	tval->tm_sec = BCD2BIN(rtcdata[0]&0x7F);
	tval->tm_min = BCD2BIN(rtcdata[1]&0x7F);
	tval->tm_hour = BCD2BIN(rtcdata[2]&0x3F);
	tval->tm_wday = BCD2BIN(rtcdata[3]&0x7);
	tval->tm_mday = BCD2BIN(rtcdata[4]&0x3F);
	tval->tm_mon = BCD2BIN(rtcdata[5] & 0x1F)-1;
	tval->tm_year = BCD2BIN(rtcdata[6]);
	tval->tm_year +=  100;
	//time2str(TimeStr,tval);
	//if(RTC_ACQ_CS == rtccs)
	printf("Acq RTC Time:%s\n",asctime(tval));

	return res;
}

//-----------------------------------GPS-------------------------------------
//相关结构体定义
struct gps_info{
	struct tm gpstime;
	struct tm localtime;
	char cav;    
	char ns[13];   //纬度表示
	char cns;    //确定是 N 还是 S
	char ew[14];   //经度表示
	char cew;    //确定是 E 还是 W
	char altitude[7]; 
};

int OpenDev(const char * strdev,int flg)
{
    //int fd = open(Dev, O_RDONLY | O_NOCTTY);  //旧的方式
    //int fd = open(Dev, O_RDWR | O_NOCTTY);       //旧的方式
    //int fd = open(Dev, O_RDWR | O_NOCTTY | O_NDELAY);  //01.03
    int fd = open(strdev, flg);       //HJJ 01.22
    if (-1 == fd)
    {
       perror("Can't Open Serial Port");
       return -1;
    }
    else
       return fd;
}

//设置波特率
void set_speed(int fd, int speed)
{
	int   i; 
	int   status; 
	struct termios   Opt;
	tcgetattr(fd, &Opt); 
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) 
	{ 
	   if  (speed == name_arr[i]) 
	   	{ 
				/**
				 * tcflush函数刷清(抛弃)输入缓存(终端驱动程序已接收到，但用户程序尚未读)或输出缓存(用户程序已经写，但尚未发送)。queue参数应是下列三个常数之一：
				* TCIFLUSH刷清输入队列。
				 * TCOFLUSH刷清输出队列。 
				* TCIOFLUSH刷清输入、输出队列。
				 */
		   tcflush(fd, TCIOFLUSH);//设置前flush     
		   cfsetispeed(&Opt, speed_arr[i]);  
		   cfsetospeed(&Opt, speed_arr[i]);   
		   //通过tcsetattr函数把新的属性设置到串口上。
		   //tcsetattr(串口描述符，立即使用或者其他标示，指向termios的指针)
		   status = tcsetattr(fd, TCSANOW, &Opt);  
		   if  (status != 0) 
				{        
			      perror("tcsetattr fd1");  
			      return;     
				 }    
		    tcflush(fd,TCIOFLUSH);  //设置后flush 
		}  
	}
}

//设置数据位他、停止位、校验特性
int set_Parity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    if (tcgetattr(fd, &options) != 0)
    {
       perror("SetupSerial 1");
       return (FALSE);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits)
    /*设置数据位数*/
    {
    case 7:
       options.c_cflag |= CS7;
       break;
    case 8:
       options.c_cflag |= CS8;
       break;
    default:
       fprintf(stderr,"Unsupported data size\n");
       return (FALSE);
    }
    
    switch (parity)
    {
    case 'n':
    case 'N':
       options.c_cflag &= ~PARENB; /* Clear parity enable */
       options.c_iflag &= ~INPCK; /* Enable parity checking */
       break;
    case 'o':
    case 'O':
       options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
       options.c_iflag |= INPCK; /* Disnable parity checking */
       break;
    case 'e':
    case 'E':
       options.c_cflag |= PARENB; /* Enable parity */
       options.c_cflag &= ~PARODD; /* 转换为偶效验*/
       options.c_iflag |= INPCK; /* Disnable parity checking */
       break;
    case 'S':
    case 's': /*as no parity*/
       options.c_cflag &= ~PARENB;
       options.c_cflag &= ~CSTOPB;
       break;
    default:
       fprintf(stderr,"Unsupported parity\n");
       return (FALSE);
    }
    
    /* 设置停止位*/
    switch (stopbits)
    {
    case 1:
       options.c_cflag &= ~CSTOPB;
       break;
    case 2:
       options.c_cflag |= CSTOPB;
       break;
    default:
       fprintf(stderr,"Unsupported stop bits\n");
       return (FALSE);
    }
    /* Set input parity option */
    if (parity != 'n')
       options.c_iflag |= INPCK;
       
    tcflush(fd, TCIFLUSH);
    options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    
    //如果不是开发终端之类的，只是串口传输数据，而不需要串口来处理，那么使用原始模式(Raw Mode)方式来通讯，设置方式如下：
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
		options.c_oflag  &= ~OPOST;   /*Output*/

    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
       perror("Setup Serial");
       return (FALSE);
    }
    
    return (TRUE);
}

struct tm modify_timezone(struct tm timeold,u8 offset)
{
	struct tm timenew;
	
	timenew.tm_min=timeold.tm_min;
	timenew.tm_sec=timeold.tm_sec;	
	
	if( timeold.tm_hour<16)
	{
		timenew.tm_hour=timeold.tm_hour+offset;
		timenew.tm_mday=timeold.tm_mday;
	}
	else
	{
		timenew.tm_hour=timeold.tm_hour+offset-24;
		timenew.tm_mday=timeold.tm_mday+1;
	}
	//To solve the leap year problem
	if (timeold.tm_mon==2)
	{	if ( ( ((timeold.tm_year+2000)%4==0) && ((timeold.tm_year+2000)%100!=0) ) || ((timeold.tm_year+2000)%400==0) )
		{
			if (timenew.tm_mday>29)
			{
				timenew.tm_mday=1;
				timenew.tm_mon=timeold.tm_mon+1;
			}
			else
				timenew.tm_mon=timeold.tm_mon;
		}
		else
		{
			if (timenew.tm_mday>28)
			{
				timenew.tm_mday=1;
				timenew.tm_mon=timeold.tm_mon+1;
			}
			else
				timenew.tm_mon=timeold.tm_mon;
		}
	}
	//To solve the 30 days of one month
	else if ((timeold.tm_mon==4)||(timeold.tm_mon==6)||(timeold.tm_mon==9)||(timeold.tm_mon==11))
	{
		if (timenew.tm_mday>30)
			{
				timenew.tm_mday=1;
				timenew.tm_mon=timeold.tm_mon+1;
			}
		else
			timenew.tm_mon=timeold.tm_mon;
	}
	//To solve the 31 days of one month
	else
	{
		if (timenew.tm_mday>31)
			{
				timenew.tm_mday=1;
				timenew.tm_mon=timeold.tm_mon+1;
			}	
		else
			timenew.tm_mon=timeold.tm_mon;
	}
	
	//To solve the months
	if (timenew.tm_mon>12)
	{
		timenew.tm_mon=1;
		timenew.tm_year=timeold.tm_year+1;
	}
	else
		timenew.tm_year=timeold.tm_year;

	timenew.tm_wday=timeold.tm_wday;
	return timenew;
}

//HJJ 10.31检验起始字符
int CheckStr(const char *inChar,const char *strBegin,const char *strEnd)
{
	char *posBegin = strstr(inChar,strBegin);
	char *posEnd = strstr(inChar,strEnd);
	
	if(posBegin !=NULL && posEnd !=NULL)
		return 1;
	else
		return 0;
}

u8 GPStime[21]="08-08-08 08:18:18.18\n";
int deal_with_GPS_data(char *GPS_RecvBuf, struct gps_info *pgps )
{
	char *p;
	p = strstr(GPS_RecvBuf,"$GPRMC");
	if(p == NULL)
	{
		//printf("$GPSCLK not be found");
		return -1;
	}
//LEA-5T :
/*$GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,*hh
	<1>UTC时间:hhmmss:sss格式
	<2>状态:A=定位；V=导航;
	<3>纬度:ddmm.mmm格式;
	<4>纬度方向：N或者S;
	<5>经度:ddmm.mmmm格式;
	<6>经度方向：E或者W；
	<7>对地航速:单位 里/小时；
	<8>对地航向；
	<9>当前UTC时间；
	<10>磁偏角。
*/
//$GPRMC,022500.00,A,3959.48976,N,11620.66886,E,0.048,,170812,,,D*71
//$GPRMC,025905.00,A,3959.57933,N,11620.71209,E,2.533,14.00,170812,,,A*5E
//$GPRMC,061552.00,A,3959.50315,N,11620.66480,E,0.162,,050712,,,A*7D
//格式是：GPRMC + 时间（061552：14时（06 + 8） 15 分 52 秒 + A + 纬度 + N + 经度 + E + “0.162未知” + 时间（date + mon + year））
//Fastrax :
//$GPRMC,024811.19,A,3959.3159,N,11620.7382,E,0.00,349.8,030906,5.8,W,A*14
//GPStime[21]="06-09-03 02:48:11.19\n";
	{
		GPStime[9]=*(p+7);   //022500.00 =>格林尼治时间
		GPStime[10]=*(p+8);
		GPStime[12]=*(p+9);
		GPStime[13]=*(p+10);
		GPStime[15]=*(p+11);
		GPStime[16]=*(p+12);	
		GPStime[18]=*(p+14);
		GPStime[19]=*(p+15);

		sscanf(p+19,"%lf,%c,%lf,%c", &pgps->ns, &pgps->cns, &pgps->ew, &pgps->cew);
	
		//p = strstr((char *)GPS_RecvBuf,"W");//It is important here.
		//p = strstr((char *)GPS_RecvBuf,"A*");//It is important here.
		p = strstr(GPS_RecvBuf,",,,"); //170812,,,
		if(p != NULL)
		{
			//08.17
			GPStime[0] = *(p - 2);  //year
			GPStime[1] = *(p - 1);	
			
			GPStime[3] = *(p - 4);  //mon
			GPStime[4] = *(p - 3);
			
			GPStime[6] = *(p - 6); //day	
			GPStime[7] = *(p - 5);
			//printf("%c%c\n",GPStime[6],GPStime[7]);
		}
	}
	pgps->gpstime.tm_sec=(GPStime[15]-0x30)*10+(GPStime[16]-0x30);                                 
	pgps->gpstime.tm_min=(GPStime[12]-0x30)*10+(GPStime[13]-0x30);
	pgps->gpstime.tm_hour=(GPStime[9]-0x30)*10+(GPStime[10]-0x30);
  	pgps->gpstime.tm_mday=(GPStime[6]-0x30)*10+(GPStime[7]-0x30);
  	pgps->gpstime.tm_mon=(GPStime[3]-0x30)*10+(GPStime[4]-0x30)-1;
  	pgps->gpstime.tm_year=(GPStime[0]-0x30)*10+(GPStime[1]-0x30)+100;
  	pgps->gpstime.tm_wday = 1;
  	//pgps->localtime = modify_timezone(pgps->gpstime,8);  
  	pgps->localtime = pgps->gpstime;   //直接默认本地时间是GMT时间
  
  	return 0;
}

int get_gps(struct gps_info *pgps)
{
	int readcnt=0;
	int rev = -1;
	int fd;
    int nread=0;
	int ns_cnt=0;
	int ew_cnt=0;
	int altitude_cnt=0;
	int buff_num=0;
	int check=0;
	char *p;
	u8 nGPSReadOKflg = 0;

  struct timeval tv;
  //struct termios oldtio,newtio;
  fd_set rfds;
  //置零 
  memset(buff,0x0,BUFSIZE);
  /************************************/
  //设置超时退出
  tv.tv_sec= 1;
  tv.tv_usec= 0;
  /************************************/
  char *dev = "/dev/ttyS1"; //TXD0
  //fd = OpenDev(dev);
  //fd = OpenDev(dev,O_RDONLY | O_NONBLOCK);
  fd = OpenDev(dev,O_RDONLY);   //串口阻塞读取数据
  
 set_speed(fd, 9600);
  if (set_Parity(fd, 8, 1, 'N') == FALSE)
  {
     printf("Set Parity Error\n");
     //exit (0);
	 return -1;
  }
  
  FD_ZERO(&rfds);    //每次循环都要清空集合，否则不能检测描述符变化
  FD_SET(fd, &rfds); //添加描述符

  //select(int nfds,fd_set *readfds,fd_set *writefds,fd_set *errorfds,struct timeval *timeout);
  if (select(1 + fd, &rfds, (fd_set *)NULL, (fd_set *)NULL, &tv) > 0)  //判断是否可读
  { 	
	if(FD_ISSET(fd,&rfds))  //测试串口是否可读
   	{
   		//printf("GPS Com exist,start reading\n");
   		while(++readcnt <= 5)   //HJJ 01.22注释掉
		{  
	   		nread = read(fd,buff,BUFSIZE);
			buff[nread+1]='\0';
			if(CheckStr(buff,"$GPRMC",",")== 1)
			{
				//printf("Get GPS Info OK\n");
				printf("%s\n",buff);   
				nGPSReadOKflg = 1;
				break;
			}
			//usleep(1000);
		 }     	
	}
		
	if(nGPSReadOKflg)  //说明GPS获取成功
	{
		ns_cnt=0;ew_cnt=0;altitude_cnt=0;
		rev = deal_with_GPS_data(buff, pgps);   //开始解析字符串
		for(buff_num=0;buff_num<strlen(buff);buff_num++)
		{	
			if(check==3)
			{
				if(buff[buff_num]==',')
					pgps->ns[ns_cnt] = '\0';
				else
					{
						pgps->ns[ns_cnt++]=buff[buff_num];

				    }
			}
			if(check==4)
				{
					pgps->cns = buff[buff_num-1];

				}
			if(check==5)
			{
				if(buff[buff_num]==',')
					pgps->ew[ew_cnt] = '\0';
				else
					{
						pgps->ew[ew_cnt++]=buff[buff_num];

					}
			}
			if(check==6)
				{
					pgps->cew = buff[buff_num-1];

				}
			/*if(check==9)
			{
				if(buff[buff_num]==',')
					pgps->altitude[altitude_cnt] = '\0';
				else
					pgps->altitude[altitude_cnt++]=buff[buff_num];
			}*/
			if(buff[buff_num]==',')
				check++;
		}		
	}
	else
	{
		//如果没有成功获取GPS信息就设置错误GPS信息
		strcpy(pgps->ns,"0000.00000");
		pgps->cns = 'E';
		strcpy(pgps->ew,"00000.00000");
		pgps->cew = 'N';
		strcpy(pgps->altitude,"99999.9");
		//printf("GPS Read Error!\n"); 
		printf("Read data is:%s\n",buff);
	} 
  }
  else
  {
	  //如果没有成功获取GPS信息就设置错误GPS信息
	  strcpy(pgps->ns,"0000.00000");
	  pgps->cns = 'E';
	  strcpy(pgps->ew,"00000.00000");
	  pgps->cew = 'N';
	  strcpy(pgps->altitude,"99999.9");
	  printf("GPS block not exist!\n");
	  rev = -1;
  }

	close(fd);
	return rev;
}


//----------------------------------GPS_END-----------------------------------



int main(int argc,char *argv[])
{
	char *p="080808171018"; //h:min:s: d mon y
	char time[11];
	u8 a=0;
	int fd,cmd;int i = 0;int res = 0; FILE *fptest;char testname[15],FILENAME[15];
	//char display_on = 0;
	struct gps_info gpsinfo;
    struct gps_info *pgpsinfo = &gpsinfo;
	//char *pstr,*pserial;
	//FILE *serialNum;
	//int file_size;
	//int serial;
	
	struct tm acqRtcTime;
	

	//fd = open_mfem();
	while((cmd = getopt(argc,argv, "rsg"))!= -1)
	{
		switch(cmd)
		{
			case 'r':       
			             read_rtc(&acqRtcTime,RTC_ACQ_CS);
			             //printf("read_rtc\n");
			             break;

			case 's':     
                         set_rtc_cmd(p,RTC_ACQ_CS);
                         //printf("Init TIME: %s\n",asctime(p));
                         //printf("set_rtc\n");
                         break;
                      
            case 'g':   
            			get_gps(pgpsinfo);
            			sprintf(time , "%02d%02d%02d%02d%02d%02d",
																	pgpsinfo->gpstime.tm_hour,
																	pgpsinfo->gpstime.tm_min,
																	pgpsinfo->gpstime.tm_sec,
            													    pgpsinfo->gpstime.tm_mday,
            													    pgpsinfo->gpstime.tm_mon+1,
            													    pgpsinfo->gpstime.tm_year-100);
            			
            			printf("%c: ",pgpsinfo->cns);
            			printf("%s\n", pgpsinfo->ns);
            			printf("%c: ",pgpsinfo->cew);
            			printf("%s\n", pgpsinfo->ew);
            			puts(time);
                        //time[0] = pgpsinfo->gpstime.tm_hour;
            			set_rtc_cmd(time,RTC_ACQ_CS);

            			break;
            			

        default:
			Usage(EXIT_SUCCESS);
			break;
		} 
	}	
	//close(fd);
	return 0;
}
