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
#include <pthread.h>
#include <semaphore.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include "fcntl.h"
#include "dirent.h"

//#include "build_prm.h"
#include "ad.h"
#include "gps_rtc.h"
#include "macro_def.h"
#include "tbl.h"
//05 30

/***************数据处理流程*****************************
1、生成数据文件名*.TS*8;
2、将原始数据文件重组生成：*.TS*T; 有*.TSHT *.TSMT *.TSLT
*********************************************************/
int fd;
FILE *fp_TSHfile;
FILE *fp_TSMfile;
FILE *fp_TSLfile;
FILE *fp_TBLfile;

#define BufNum	2
u8 *psscdata[BufNum];
//#define MemSize 129744
unsigned int MemSize=130848;//MT130848//AMT129744;

#define CalDataNum	2

//struct gps_info gps_str;

void *writefile(void *arg );
sem_t set_write;
sem_t set_udp;
sem_t time_up;

u8 acq_one_ok = 0;
int cnt1=0,cnt2=0,cnt3=0,cnt4=0,cnt5=0,cnt6=0;
extern char *ACQFLAG;

//char strCurPRFileName[50];    //当前的PR文件名

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

#define CMD_SETACDC			  		59
#define CMD_SETCAL					61
#define CMD_SETMCLK			  	 	56
#define	CMD_SETGAIN			  	  	57
#define CMD_SETSSCRFMR		        62
#define	CMD_SETGAINH			  	63
#define CMD_SSC_START               22
#define CMD_SSC_START_MT            65
#define CMD_SETCAL_COIL             64
#define CMD_SEL_MODE                50 
#define CMD_BUFFER_SET              60

extern unsigned char arg_gain_e;
extern unsigned char arg_gain_h;
extern unsigned char acdc;//AC 1 DC 0
//extern struct gps_info gpsinit;
extern struct gps_info *pgpsinit;

int ads_init(void)
{

	int fd;
	fd = open_mfem();
	ioctl(fd, CMD_SETCAL,1);//SIGNAL
	ioctl(fd, CMD_SETCAL_COIL,1);
	ioctl(fd, CMD_SETSSCRFMR,3);

	//ioctl(fd, CMD_SETGAIN,FastLog2(arg_gain_e)/2);
	//printf("arg_gain_e:%d\n",arg_gain_e);
	//ioctl(fd, CMD_SETGAINH,FastLog2(arg_gain_h)/2);
	//printf("arg_gain_h:%d\n",arg_gain_h);
	if(AMTMTFLAG==1)
	{
		MemSize=130848;					
        ioctl(fd, CMD_SETMCLK,1); //Fs=2.4KHz					
		ioctl(fd, CMD_SEL_MODE,1); 		
		//ioctl(fd, CMD_SETACDC,0*2+1);	
	}
	else
	{
        MemSize=129744;
		ioctl(fd, CMD_SETMCLK,0); //Fs=24KHz	
		ioctl(fd, CMD_SEL_MODE,0); 		 		
		//ioctl(fd, CMD_SETACDC,1*2+0); 
	}
	ioctl(fd, CMD_BUFFER_SET,MemSize);  //?????
    usleep(2000);
	close(fd);
	return 0;	
}

void SYNCTIME2TM(u8 *buf)  
{
	//struct tm tmp_time;
	
	buf[0]=pgpsinit->gpstime.tm_sec;
	buf[1]=pgpsinit->gpstime.tm_min; 
	buf[2]=pgpsinit->gpstime.tm_hour;
	buf[3]=pgpsinit->gpstime.tm_mday;
	buf[4]=pgpsinit->gpstime.tm_mon+1;
	buf[5]=pgpsinit->gpstime.tm_year;
	buf[6]=pgpsinit->gpstime.tm_wday;
	buf[7]=(2000 + pgpsinit->gpstime.tm_year)/100;
}

char STIM[8]={30,0x1E,0x08,0x02,0x0C,0x0D,0x01};//START TIME
char TBLNameInfo[8];
u8 TimeSlot[32];
//extern u8 *TimeBuf;
extern u8 *FileNameBuf;
extern u32 *ACQNUM;
//extern struct gps_info *pgpsinit;
u8 AMTMTFLAG=0;

#define name_to_str(name_31415926)  (#name_31415926) 
#define SetVal(val) WritePT(name_to_str(val),&val)
char TS2FileName[15];
char TS3FileName[15];
char TS4FileName[15];
char TS5FileName[15];

char TBLFileName[15];

void writeTBL()
{
	int n=0,j=0,t=0,i=0;
	FILE *fp_datatbl;
	struct tm rtc_time;
	fp_datatbl = fopen(TBLFileName, "wb");
	char FTIM[8]={26,0x22,0x08,0x0A,0x0C,0x0D,0x02}; //START TIME
	char LTIM[8]={26,0x22,0x08,0x0A,0x0C,0x0D,0x02}; //???
	char HTIM[8]={26,0x22,0x08,0x0A,0x0C,0x0D,0x02}; //高频采集起始时间
	char ETIM[8]={51,0x01,0x08,0x0A,0x0C,0x0D,0x02}; //END TIME//Important
	char ETMH[8]={51,0x01,0x08,0x0A,0x0C,0x0D,0x02,0x14};//高频采集结束时间
	char LFIX[8]={52,0x01,0x08,0x0A,0x0C,0x0D,0x02}; //最近一次锁定卫星时间
	char NUTC[8]={53,0x01,0x08,0x0A,0x0C,0x0D,0x02}; //?
	char TSYN[8]={0x0A,0x0A,0x08,0x02,0x0C,0x0D,0x01};//sec min hour day mon year week 最近一次对钟卫星时间
	int  ELEV=0; //海拔
	char LATG[13]={0x30,0x30,0x30,0x30,0x2E,0x30,0x30,0x30,0x30,0x30,0x2C,0x4E,0x00}; //北纬
	char LNGG[13]={0x30,0x30,0x30,0x30,0x30,0x2E,0x30,0x30,0x30,0x30,0x30,0x2C,0x45}; //东经
	SYNCTIME2TM(TSYN);
	read_rtc(&rtc_time,RTC_ACQ_CS);        //获取结束时间
	ETIM[0]=rtc_time.tm_sec,ETIM[1]=rtc_time.tm_min,ETIM[2]=rtc_time.tm_hour,ETIM[3]=rtc_time.tm_mday,
	ETIM[4]=rtc_time.tm_mon+1,ETIM[5]=(rtc_time.tm_year+1900)%100,ETIM[6]=rtc_time.tm_wday;ETIM[7]=(rtc_time.tm_year+1900)/100;
	//int L2NS=1,L3NS=2,L4NS=3;	
	int HSMP=1;//Set the high range sample interval in minutes.//Important 上位机设置完毕
	unsigned char buf[25];
	memcpy(FTIM,STIM,8);
	memcpy(HTIM,STIM,8);
	memcpy(LFIX,ETIM,8);
	memcpy(ETMH,ETIM,8);
	memcpy(NUTC,ETIM,8);
	memcpy(LTIM,ETIM,8);
	ELEV=atoi(pgpsinit->altitude);
	memcpy(LATG,pgpsinit->ns,10);
	memcpy(LNGG,pgpsinit->ew,11);
	LATG[11]=pgpsinit->cns;
	LNGG[12]=pgpsinit->cew;
	//SetVal(L2NS);SetVal(L3NS);SetVal(L4NS);//
	//SetVal(HSMP);//Important 上位机设置完毕

	SetVal(TSYN);
	SetVal(LTIM);
	SetVal(LFIX);
	SetVal(NUTC);
	SetVal(STIM);
	SetVal(FTIM);
	SetVal(ETIM);
	SetVal(HTIM);
	SetVal(ETMH);
	SetVal(ELEV);
	SetVal(LATG);
	SetVal(LNGG);
	int TOTL,HGN,EGN;
/*	
	if(AMTMTFLAG==1)
	{
		 TOTL=(*ACQNUM)*60;
		 EGN=1;
		 HGN=1;
	}
	else
	{	
		TOTL=(*ACQNUM)*6;
		EGN=10;
		HGN=1;
	}
*/
    TOTL=(*ACQNUM)*6;  //????
	EGN=1;
	HGN=1;
	SetVal(TOTL);
	SetVal(EGN);
	SetVal(HGN);
	WritePT("FILE",TBLNameInfo);
	WritePT("SITE",TBLNameInfo);
	for (i=0;i<140;i++)
	{
			fwrite(&DataTbl[i], 1, 25, fp_datatbl);
	}
	fclose(fp_datatbl);
}
void dealtagadd(char *tag)	
{
	if(tag[1]<'z') tag[1]=tag[1]+1;
	else 
	{
		tag[1]='a';
		if(tag[0]<'z') tag[0]=tag[0]+1;
		else tag[0]='a';
	}
	
}

char tag[2]={'a','a'};
int SNUM;
void nametag(int mon,char day)
{
	int i=0,total=0;;
	char tagmax[2]={'a','a'};
	char serialnum[6],findflag=0;
	struct dirent **namelist;
	
	sprintf(serialnum,"%d%x%c",SNUM,mon,day);
	total = scandir("/mnt/sd/", &namelist, 0, alphasort);  //返回当前目录下所有文件数目并对文件排序存放在namelist中
	if(total < 0)
		tag[0]=tag[1]='a';
	else
	{
		for(i = 0; i < total; i++)
		{
			if(strstr(namelist[i]->d_name,serialnum)>0 && strcasestr(namelist[i]->d_name,"tbl")>0)
				{
					memcpy(tag,namelist[i]->d_name+6,2);
					dealtagadd(tag);
					
				}
		}
		
	}
}


void GetBoxID(void)
{
	FILE *serialNum;
	int file_size;
	char *pstr,*pserial;
	serialNum=fopen("startup.prm","r");  //暂且当做是2进制文件
	if (serialNum == NULL)
	  {
		 printf("prm file not found!\n");
		 return;
	  }						
	fseek(serialNum, 0, SEEK_END);   //指定文件结尾
	file_size = ftell(serialNum);    //返回文件开头到结尾的字节数
	pstr = (char *)malloc(file_size);     
	if (pstr == NULL)
	  {
		printf("allocate error!\n");
		return;
	 }							
	fseek(serialNum, 0 , SEEK_SET); //指定文件开头
	fread(pstr, file_size, 1, serialNum);
	fclose(serialNum);	
	serialNum=NULL;							
	pserial = strstr(pstr,"BOX_ID = ");

	if(pserial != NULL)
	 {
		sscanf(pserial+9, "%d", &SNUM);
	 }
	 printf("SNUM:%d\n", SNUM);
	 
}

void set_filename()
{
	struct tm rtc_time;
	
	tblInit();
	//get_gps(&gps_str);
	//ReadPT("SNUM",&SNUM);
	GetBoxID();//20170422 从startup.prm文件中获取prm文件
	/////////////////////////////////
	read_rtc(&rtc_time,RTC_ACQ_CS);        //获取采集RTC时间
	///////////////////////////////
	//time2str(TimeStr,&rtc_time); //????
	char DAY[31]={'1','2','3','4','5','6','7','8','9','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v'};

	nametag(rtc_time.tm_mon+1,DAY[rtc_time.tm_mday-1]);  //命名tbl文件
	sprintf(TS2FileName,"%d%x%c%c%c.ts2",SNUM,rtc_time.tm_mon+1,DAY[rtc_time.tm_mday-1],tag[0],tag[1]);//必须是8位
	sprintf(TS3FileName,"%d%x%c%c%c.ts3",SNUM,rtc_time.tm_mon+1,DAY[rtc_time.tm_mday-1],tag[0],tag[1]);
	sprintf(TS4FileName,"%d%x%c%c%c.ts4",SNUM,rtc_time.tm_mon+1,DAY[rtc_time.tm_mday-1],tag[0],tag[1]);
	sprintf(TS5FileName,"%d%x%c%c%c.ts5",SNUM,rtc_time.tm_mon+1,DAY[rtc_time.tm_mday-1],tag[0],tag[1]);

	sprintf(TBLFileName,"%d%x%c%c%c.tbl",SNUM,rtc_time.tm_mon+1,DAY[rtc_time.tm_mday-1],tag[0],tag[1]);
	sprintf(TBLNameInfo,"%d%x%c%c%c",SNUM,rtc_time.tm_mon+1,DAY[rtc_time.tm_mday-1],tag[0],tag[1]);

	STIM[0]=rtc_time.tm_sec,STIM[1]=rtc_time.tm_min,STIM[2]=rtc_time.tm_hour,STIM[3]=rtc_time.tm_mday,
	STIM[4]=rtc_time.tm_mon+1,STIM[5]=(rtc_time.tm_year+1900)%100,STIM[6]=rtc_time.tm_wday;//月份必须加1因为rtc 月份为0-11
	STIM[7]=(rtc_time.tm_year+1900)/100;
	

}

/*
 每1MB数据加入4个4B 
第一个4B 0XFFFF 2B 1MB的块ID 2B
第二个4B 距离起始对钟时间的整秒数2*109s 约为24 855days
第三个4B 为毫秒数
第四个4B 数据校验和*/
pthread_t a_thread,b_thread;
void *thread_result,*thread_result1;

void mkTimeSlot(u8 *TimeSlot,u8* buf,u8 TSflag)
{
	u8 ts2[32]={32,0x1e,0x08,2,12,13,1,0x14,0x1d,0x0b,0x60,0x09,0x05,0x20,0x00,0x00,0x00,0x03,0xc0,0x5d,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 ts3[32]={31,0x1e,8,2,12,13,1,0x14,0x1d,0x0b,0x60,0x09,0x05,0x20,0x00,0x00,0x00,0x03,0x60,0x09,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 ts4[32]={30,0x1E,8,0x2,0x0c,13,1,0x14,0x1d,0x0b,0x96,0x00,0x05,0x20,0x00,0x00,0x00,0x03,0x96,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 ts5[32]={30,0x1E,8,0x2,0x0c,13,1,0x14,0x1d,0x0b,0xF,0x00,0x05,0x20,0x00,0x00,0x00,0x03,0xF,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	struct tm tmSYNC,*curTime;
	time_t diffseconds; 	
	tmSYNC.tm_sec  = pgpsinit->gpstime.tm_sec;
	tmSYNC.tm_mon  = pgpsinit->gpstime.tm_mon;
	tmSYNC.tm_mday = pgpsinit->gpstime.tm_mday;
	tmSYNC.tm_hour = pgpsinit->gpstime.tm_hour;
	tmSYNC.tm_min  = pgpsinit->gpstime.tm_min;
	tmSYNC.tm_year = pgpsinit->gpstime.tm_year;
	tmSYNC.tm_wday = pgpsinit->gpstime.tm_wday;//(*(p+12) - '0');

	diffseconds=mktime(&tmSYNC);
//	diffseconds+= (long)((buf[11]*256*256*256+buf[10]*256*256+buf[9]*256+buf[8])/1024.0+0.5);
	diffseconds+= (long)((buf[11]*256*256*256+buf[10]*256*256+buf[9]*256+buf[8])/2400.0+0.5);
	curTime = localtime(&diffseconds);//ctime(&diffseconds)

	switch(TSflag)
	{
		case 2://Linux 月份是0-11所以要加上1
			ts2[0]=curTime->tm_sec,ts2[1]=curTime->tm_min,ts2[2]=curTime->tm_hour,ts2[3]=curTime->tm_mday,
			ts2[4]=curTime->tm_mon+1,ts2[5]=(curTime->tm_year+2000)%100,ts2[6]=curTime->tm_wday,ts2[7]=(curTime->tm_year+2000)/100;
			memcpy(TimeSlot,ts2,32);
		break;
		case 3:
			ts3[0]=curTime->tm_sec,ts3[1]=curTime->tm_min,ts3[2]=curTime->tm_hour,ts3[3]=curTime->tm_mday,
			ts3[4]=curTime->tm_mon+1,ts3[5]=(curTime->tm_year+2000)%100,ts3[6]=curTime->tm_wday,ts3[7]=(curTime->tm_year+2000)/100;
			memcpy(TimeSlot,ts3,32);
		break;
		case 4:
			ts4[0]=curTime->tm_sec,ts4[1]=curTime->tm_min,ts4[2]=curTime->tm_hour,ts4[3]=curTime->tm_mday,
			ts4[4]=curTime->tm_mon+1,ts4[5]=(curTime->tm_year+2000)%100,ts4[6]=curTime->tm_wday,ts4[7]=(curTime->tm_year+2000)/100;
			memcpy(TimeSlot,ts4,32);
		break;
		case 5:
			ts5[0]=curTime->tm_sec,ts5[1]=curTime->tm_min,ts5[2]=curTime->tm_hour,ts5[3]=curTime->tm_mday,
			ts5[4]=curTime->tm_mon+1,ts5[5]=(curTime->tm_year+2000)%100,ts5[6]=curTime->tm_wday,ts5[7]=(curTime->tm_year+2000)/100;
			memcpy(TimeSlot,ts5,32);

		break;
	}ts2[0]=ts3[0]=ts4[0]=ts5[0]=0;
	//printf("%s \n%ld\n", asctime(curTime),(long)((buf[11]*256*256*256+buf[10]*256*256+buf[9]*256+buf[8])/1024.0+0.5));
}
FILE *fp;u8 writefinishflag=0;
void *writefile(void *arg)//write file thread 
{
	int fd,i=0,n=0,m=0;
	fd=open_mfem();
	u8 buf[16];
	//fp=fopen("test.3TS","wb");
	while(!acq_one_ok)
	{
		sem_wait(&set_write);
		if(!acq_one_ok)
		{
		//写数据 和写16字节的顺序不能弄错
		i=0;
		writefinishflag=0;
		for (i=0;i<MemSize/16;i+=1)
		{
			memcpy(buf,psscdata[cnt2%BufNum]+i*16,16);
			//fwrite(buf,1,16,fp);	
			if(buf[15]==0xAA&&buf[0]==0xEE&&buf[1]==0xEE&&buf[2]==0x0&&buf[3]==0x0)
				{
					mkTimeSlot(TimeSlot,buf,AMTMTFLAG+2);
					fwrite(TimeSlot,1,32,fp_TSHfile);
					continue;
				}
			
			if(buf[15]==0xBB&&buf[0]==0xEE&&buf[1]==0xEE&&buf[2]==0x0&&buf[3]==0x0)
				{
					mkTimeSlot(TimeSlot,buf,AMTMTFLAG+3);
					fwrite(TimeSlot,1,32,fp_TSMfile);
					m=m+1;
					continue;
				}

			if(buf[15]==0xCC&&buf[0]==0xEE&&buf[1]==0xEE&&buf[2]==0x0&&buf[3]==0x0)
				{
					mkTimeSlot(TimeSlot,buf,AMTMTFLAG+4);
					fwrite(TimeSlot,1,32,fp_TSLfile); 
					n=n+1;
					continue;
				}

			if(buf[15]==0xAA) fwrite(buf,1,15,fp_TSHfile);
			fflush(fp_TSHfile);
			if(buf[15]==0xBB) fwrite(buf,1,15,fp_TSMfile);
			fflush(fp_TSMfile);
			if(buf[15]==0xCC) fwrite(buf,1,15,fp_TSLfile);
			fflush(fp_TSLfile);
		};
		writeTBL();
		writefinishflag=1;
		cnt2++;	
		}
	}
	close(fd);
	//fclose(fp);
	pthread_exit(NULL);
}

int ad_init_reg()
{
	int res;
	int j=0;
	ads_init();
	cnt1=cnt2=cnt3=0;
	acq_one_ok = 0;
	*ACQNUM=0;
	res = sem_init(&set_write, 0, 0);	//创建一个信号量并赋予初值
	if (res != 0)
	{
		perror("semaphore init error!\n ");
		exit(EXIT_FAILURE);
	}
	/*res = sem_init(&set_udp, 0, 0);  
	if (res != 0)
	{
		perror("semaphore init error!\n ");
		exit(EXIT_FAILURE);
	}*/
	
	res = pthread_create(&a_thread, NULL, writefile, NULL); //创建一个写文件线程
	if (res != 0)
	{
		perror("thread function create error!\n ");
		exit(EXIT_FAILURE);
	}
	/*res = pthread_create(&b_thread, NULL, UDP_SEND, NULL);  //创建一个UDP发送线程
	if (res != 0)
	{
		perror("thread function create error!\n ");
		exit(EXIT_FAILURE);
	}*/
	//UDP_INIT();
	printf("ads_init finish\n");
	sprintf(FileNameBuf,"%s.ts*",TBLNameInfo);
	printf("FileName:%s.ts*\n",TBLNameInfo);  // puts()
	if(AMTMTFLAG==1) //1：MT 0:AMT
		{
			fp_TSHfile = fopen(TS3FileName, "wb");	
			fp_TSMfile = fopen(TS4FileName, "wb");	
			fp_TSLfile = fopen(TS5FileName, "wb");	
		}
	else
		{
			fp_TSHfile = fopen(TS2FileName, "wb");	
			fp_TSMfile = fopen(TS3FileName, "wb");	
			fp_TSLfile = fopen(TS4FileName, "wb");			
		}
	
	for(j=0;j<BufNum;j++)
		{
			psscdata[j] = malloc(MemSize);
		}
	if (NULL == psscdata[BufNum-1])  
		{
			printf("malloc psscdata error!");
			return -1;
		}
	usleep(2000);
	return 1;
}
int acq_onerate() //acq one rate
{
	int i=0,j = 0,N=0;
	int res;	
	int fd=open_mfem();	
	if(fd == -1)
		return -1;	
	struct tm CURTime;
	read_rtc(&CURTime,RTC_ACQ_CS);
	//N=59-CURTime.tm_sec;
	//TimeAddS(&CURTime,N);
	if(CURTime.tm_sec==59)
		CURTime.tm_min++;
	else
		CURTime.tm_sec=59;
	set_alarm(CURTime.tm_mday,CURTime.tm_hour,CURTime.tm_min,CURTime.tm_sec);
	ioctl(fd, CMD_SSC_START,1); //SSC_begin  //1 wait rtc 0 no wait rtc    
	do
	{	
		*ACQNUM = *ACQNUM+1;  //写次数计数
		read(fd, psscdata[cnt1%BufNum], MemSize);  //SSC1
		if(*ACQFLAG==0)
		 {
		 	acq_one_ok=1;
		 }
		sem_post(&set_write);
		//sem_post(&set_udp);	
		cnt1++;
	}while(!acq_one_ok);
	
	ioctl(fd, CMD_SSC_STOP); 
	//TranIdlePackage();
	writeTBL();
	
	*ACQNUM=0;	
	
	res = pthread_join(a_thread, &thread_result);
	if(res != 0)	
		printf("thread join error!\n");
	//res = pthread_join(b_thread, &thread_result1);
	//if(res != 0)	
		//printf("thread join error!\n");
	sem_destroy(&set_write);
	//sem_destroy(&set_udp);		
	for(j=0;j<BufNum;j++)free(psscdata[j]);
	fclose(fp_TSHfile);
	fclose(fp_TSMfile);
	fclose(fp_TSLfile);	
	close(fd);
	return 0;
}
