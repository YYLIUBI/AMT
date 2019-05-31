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

#include "build_prm.h"
#include "ad.h"
#include "gps_rtc.h"
#include "macro_def.h"


#define CMD_SETCAL					61
#define CMD_SETMCLK			  	 	56
#define	CMD_SETGAIN			  	  	57
#define CMD_SETCAL_COIL             64

//05 30
#define		AD_REF			2.5
#define		TEMP_0C			403//参考电压2.5V时0摄氏度转换值
#define 	VOL_DIV			11
#define 	SPI_CONFIGURE_DELAY		2

int fd;
FILE *fp_datafile;
//u32 *psscdata;
#define BufNum	2
u32 *psscdata[BufNum];
#define MemSize 1024*128

#define CalDataNum	2

int FreCalDataOffset=0;
int WriteDataOffset=0;

#define FreCalDataLength (2*1024*1024)
#define WriteDataLength (1*1024*1024)

u8 FreCalData[CalDataNum][FreCalDataLength];
u8 CalFlag=0;
u8 WriteData[BufNum][WriteDataLength];

void *writefile(void *arg );
sem_t set_write;
sem_t set_udp;
sem_t time_up;

u8 acq_one_ok = 0;
int cnt1=0,cnt2=0,cnt3=0,cnt4=0,cnt5=0,cnt6=0;
extern char *ACQFLAG;
extern struct gps_info *pgpsinit;

char strCurPRFileName[50];    //当前的PR文件名

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
char PrmFileName[21];
struct tm start_time;
u8 TimeStr[14];
char AcqFileName[40];
char FreFileName[40];
FILE *fpFreCal;

void time2str(u8 *timestr,struct tm *tval)
{
	//printf("%d %d\n",tval->tm_year+1900,'0'-0);

	TimeStr[0]=(tval->tm_year+1900)/1000+48;TimeStr[1]=((tval->tm_year+1900)%1000)/100+48;TimeStr[2]=((tval->tm_year+1900)%100)/10+48;TimeStr[3]=(tval->tm_year+1900)%10+48;
	TimeStr[4]=(tval->tm_mon+1)/10+48;TimeStr[5]=(tval->tm_mon+1)%10+48;
	TimeStr[6]=(tval->tm_mday)/10+48;TimeStr[7]=(tval->tm_mday)%10+48;
	TimeStr[8]=(tval->tm_hour)/10+48;TimeStr[9]=(tval->tm_hour)%10+48;
	TimeStr[10]=(tval->tm_min)/10+48;TimeStr[11]=(tval->tm_min)%10+48;
	TimeStr[12]=(tval->tm_sec)/10+48;TimeStr[13]=(tval->tm_sec)%10+48;
	//TimeStr[14]=(tval->tm_mday)/10+48;TimeStr[15]=(tval->tm_mday)%10+48;

	//printf("%s\n",TimeStr);
}

void set_filename(struct acq_prm *pacq)
{
	int fd;
	FILE *fp_prmfile;
	char tmp_str[60];
	read_rtc(&start_time,RTC_ACQ_CS);        //获取采集RTC时间
  	time2str(TimeStr,&start_time);
	AcqFileName[40]=PrmFileName[40]='\0';
//	printf("\ntestAcqNum=%d\n",pcal->testAcqNum);
//	if(pcal->testAcqNum!=0)
//	{
		switch(pacq->samplerate)
		{
			case 24000:
				sprintf(AcqFileName,"%s%s_%s.TSS",calprm.cal_path,calprm.box_id,TimeStr+2);
				sprintf(PrmFileName,"%s%s_%s.PRS",calprm.cal_path,calprm.box_id,TimeStr+2);break;		
			case 2400:
				sprintf(AcqFileName,"%s%s_%s.TSH",calprm.cal_path,calprm.box_id,TimeStr+2);
				sprintf(PrmFileName,"%s%s_%s.PRH",calprm.cal_path,calprm.box_id,TimeStr+2);break;						
			case 150:
				sprintf(AcqFileName,"%s%s_%s.TSM",calprm.cal_path,calprm.box_id,TimeStr+2);
				sprintf(PrmFileName,"%s%s_%s.PRM",calprm.cal_path,calprm.box_id,TimeStr+2);break;
			case 15:
				sprintf(AcqFileName,"%s%s_%s.TSL",calprm.cal_path,calprm.box_id,TimeStr+2);
				sprintf(PrmFileName,"%s%s_%s.PRL",calprm.cal_path,calprm.box_id,TimeStr+2);break;	
			default:
				sprintf(AcqFileName,"%s%s_%s.TSS",calprm.cal_path,calprm.box_id,TimeStr+2);
				sprintf(PrmFileName,"%s%s_%s.PRS",calprm.cal_path,calprm.box_id,TimeStr+2);break;					
		}

	fd = open("/dev/mfem", O_RDWR);
	switch(pacq->samplerate)
	{
		case 24000:ioctl(fd,CMD_SETMCLK,0);break;
		case 2400:ioctl(fd,CMD_SETMCLK,1);break;
		case 150:ioctl(fd,CMD_SETMCLK,2);break;
		case 15:ioctl(fd,CMD_SETMCLK,3);break;
		default:ioctl(fd,CMD_SETMCLK,0);break;
	}
	
	
	fp_prmfile = fopen(PrmFileName, "wr");
	fprintf(fp_prmfile,"//OBEM CAL PRM FILE\n");
	fprintf(fp_prmfile,"VERSION = %s\n",calprm.version);
//	fprintf(fp_prmfile,"SURID = %s\n",pcal->sname);
//	fprintf(fp_prmfile,"LINE  = %s\n",psur->line);
//	fprintf(fp_prmfile,"SITE  = %s\n",psur->site);
//	fprintf(fp_prmfile,"PATH  = %s\n",psur->file_path);
//	fprintf(fp_prmfile,"LOCKGPS  = %d\n",psur->lockgps);
	fprintf(fp_prmfile,"BOX_ID = %s\n",calprm.box_id);
//	fprintf(fp_prmfile,"AD_Board_ID = %s\n",psur->boxid);   //11.03
//	fprintf(fp_prmfile,"HAMP_Board_ID = %s\n",psur->boxid);   //11.03
//	fprintf(fp_prmfile,"EAMP_Board_ID = %s\n",psur->boxid);   //11.03	
//	fprintf(fp_prmfile,"RELEASE_ON_CODE = %s\n",psur->releaseOnCode);
//	fprintf(fp_prmfile,"RELEASE_OFF_CODE = %s\n",psur->releaseOffCode);
//	fprintf(fp_prmfile,"RELEASE_TIME = %04d-%02d-%02d %02d:%02d:%02d\n",
//	                                   psur->releaseTime.tm_year+1900,
//	                                   psur->releaseTime.tm_mon+1,
//	                                   psur->releaseTime.tm_mday,
//	                                   psur->releaseTime.tm_hour,
//	                                   psur->releaseTime.tm_min,
//	                                   psur->releaseTime.tm_sec);

	fprintf(fp_prmfile,"CH_NUM = %d\n",calprm.chnum);
	fprintf(fp_prmfile,"HxSN = %s\n",calprm.hxsn);
	fprintf(fp_prmfile,"HySN = %s\n",calprm.hysn);
	fprintf(fp_prmfile,"HxSN = %s\n",calprm.hzsn);
	
	fprintf(fp_prmfile,"ExLEN = %d\n",calprm.exlen);
	fprintf(fp_prmfile,"EyLEN = %d\n",calprm.eylen);
//	fprintf(fp_prmfile,"NOTCH = %d\n",psur->notch);	
	fprintf(fp_prmfile,"E_ANALOG_GAIN_HS= %d\n",calprm.egianA_HS);
	fprintf(fp_prmfile,"H_ANALOG_GAIN_HS= %d\n",calprm.hgianA_HS);
	fprintf(fp_prmfile,"E_ANALOG_GAIN_LS= %d\n",calprm.egianA_LS);
	fprintf(fp_prmfile,"H_ANALOG_GAIN_LS= %d\n",calprm.hgianA_LS);
	
	//SYNC_TIME
	fprintf(fp_prmfile,"SYNC_TIME = %04d-%02d-%02d %02d:%02d:%02d\n",
		pgpsinit->gpstime.tm_year+2000,
		pgpsinit->gpstime.tm_mon+1,
		pgpsinit->gpstime.tm_mday,
		pgpsinit->gpstime.tm_hour,
		pgpsinit->gpstime.tm_min,
		pgpsinit->gpstime.tm_sec);//psur->syncTime
																	
	//START_TIME
	fprintf(fp_prmfile,"START_TIME = %04d-%02d-%02d %02d:%02d:%02d\n",
																	start_time.tm_year+1900,
																	start_time.tm_mon+1,
																	start_time.tm_mday,
																	start_time.tm_hour,
																	start_time.tm_min,
																	start_time.tm_sec);
	//fprintf(fp_prmfile,"SAMPLE_RATE = %d\n",p_acq_prm->srate);
/*	switch(p_acq_prm->sratechar)   //6(1000)=> H 1(100) => M 10 => L  08.14
	{
		case 'S':   //1000Hz
			fprintf(fp_prmfile,"SAMPLE_RATE = S\n");
			break;
		case 1:  //100Hz
			fprintf(fp_prmfile,"SAMPLE_RATE = H\n");
			break;
		case 11:  //10Hz
			fprintf(fp_prmfile,"SAMPLE_RATE = M\n");
			break;
		default:
			fprintf(fp_prmfile,"SAMPLE_RATE = %d\n",p_acq_prm->srate);
			break;
	}
	*/
	fprintf(fp_prmfile,"SAMPLE_RATE = %d\n",pacq->samplerate);
	fprintf(fp_prmfile,"E_DIGITAL_GAIN = %d\n",pacq->egain);
	fprintf(fp_prmfile,"H_DIGITAL_GAIN = %d\n",pacq->hgain);


//	fprintf(fp_prmfile,"BAT_START = %.2f V\n",(tmp_str[0]*10+tmp_str[1])/1023.0*AD_REF*VOL_DIV);
//	fprintf(fp_prmfile,"TEMP_START = %.1f 'C\n",((tmp_str[2]*10+tmp_str[3])/1023.0*AD_REF-0.986)/0.00355);
	//写入GPS
	//LATG = 3959.490,N
	//LNGG = 11620.669,E
	//"%lf,%c,%lf,%c", &pgps->ns, &pgps->cns, &pgps->ew, &pgps->cew
	fprintf(fp_prmfile,"LATG = %.3f,%c\n",calprm.str_gps.ns,calprm.str_gps.cns);
	fprintf(fp_prmfile,"LNGG = %.3f,%c\n",calprm.str_gps.ew,calprm.str_gps.cew);
	/*
	L3NS = 2
	SLOT_TIME = 120
	DATA_OFFSET = 0
	*/
//	fprintf(fp_prmfile,"L3NS = %d\n",psur->nL3NS);
//	fprintf(fp_prmfile,"SLOT_TIME = %d\n",psur->nSlotTime);
//	fprintf(fp_prmfile,"DATA_OFFSET = %d\n",psur->nDataOffset);
	fclose(fp_prmfile);
}

/***************UDP相关*****************************/
int UdpPackageSize=ONE_K * 48;
int sock;//sendto中使用的对方地址
struct sockaddr_in ServerAddr;//在recvfrom中使用的对方主机地址
struct sockaddr_in ClientAddr;
int sendLen;
unsigned int addrLen;

//u32 sendBuffer[ONE_K * 32];
void UDP_INIT(void)
{
	/*int j=0;
	for(j=0;j<ONE_K * 48;j++) memset(sendBuffer,0,ONE_K * 48);
	for(j=0;j<ONE_K * 48;j++) memset(sendBuffer+ONE_K * 48/32,1,ONE_K * 48);
	for(j=0;j<ONE_K * 32;j++) memset(sendBuffer+ONE_K * 96,2,ONE_K * 32);*/
	
	sock = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
	if(sock < 0)
	{
 		printf("创建套接字失败了.\r\n");
		exit(0);
	}
	memset(&ServerAddr,0,sizeof(ServerAddr));
	ServerAddr.sin_family=AF_INET;
	ServerAddr.sin_addr.s_addr=inet_addr("192.168.1.100");//htonl(INADDR_ANY);//
	//ServerAddr.sin_port = htons(34568);
	
	memset(&ServerAddr,0,sizeof(ServerAddr));
	ClientAddr.sin_family=AF_INET;
	ClientAddr.sin_addr.s_addr=inet_addr("192.168.1.105");//htonl(INADDR_ANY);//
	ClientAddr.sin_port = htons(34566);
	
	if(bind(sock,(struct sockaddr*)&ServerAddr,sizeof(ServerAddr))<0)
	{
 		printf("bind() 函数使用失败了.\r\n");
 		close(sock);
 		exit(1);
	}
	addrLen = sizeof(ServerAddr);
}



extern u32 nTotalAcqLen;  //记录本次采集的总长度 
extern u32 nCurAcqNum;    //记录当前的采集长度

extern u8 nTotalAcqPeriod;
extern u8 nCurAcqPeriod;
u8 GpsStatus=0;
u8 UDP_STATUS[7];


/********************************************/
int wrcnt = 0,sendcnt=0;
static u32 blkID =0 ;
static u32 blkFlg = 0xFFFFFFFF;
static u32 secCnt = 0;
static u32 msCnt = 0;
static u32 crcData = 0;  //校验和

u32 TimeTag[4];
void TranTimeTag(void)
{
	TimeTag[0]=blkFlg;TimeTag[1]=blkID;TimeTag[2]=secCnt;TimeTag[3]=msCnt;
	sendto(sock,TimeTag,16,0,(struct sockaddr*)&ClientAddr,sizeof(ClientAddr));
}

void *UDP_SEND(void *arg)//write file thread 
{
	while(!acq_one_ok)
	{
		sem_wait(&set_udp);
	
		sendLen=sendto(sock,psscdata[cnt3%BufNum],UdpPackageSize,0,(struct sockaddr*)&ClientAddr,sizeof(ClientAddr));
		sendLen=sendto(sock,psscdata[cnt3%BufNum]+UdpPackageSize/4,UdpPackageSize,0,(struct sockaddr*)&ClientAddr,sizeof(ClientAddr));
		sendLen=sendto(sock,psscdata[cnt3%BufNum]+UdpPackageSize*2/4,UdpPackageSize/3*2,0,(struct sockaddr*)&ClientAddr,sizeof(ClientAddr));

		UDP_STATUS[0]=nCurAcqNum%256,UDP_STATUS[1]=(nCurAcqNum%65536)/256,UDP_STATUS[2]=(nCurAcqNum%16777216)/65536,UDP_STATUS[3]=nCurAcqNum/16777216;
		//UDP_STATUS[0]=1,UDP_STATUS[1]=2,UDP_STATUS[2]=3,UDP_STATUS[3]=4;
		sendLen=sendto(sock,UDP_STATUS,4,0,(struct sockaddr*)&ClientAddr,sizeof(ClientAddr));
		cnt3++;	
		
	}
	pthread_exit(NULL);
}
/*
const double PI = 3.1415926;
#define Data int
void bitrp (double xreal [], double ximag [], int n)
{
	// 位反转置换 Bit-reversal Permutation
	int i, j, a, b, p;double t;

	for (i = 1, p = 0; i < n; i *= 2)
	{
		p ++;
	}
	for (i = 0; i < n; i ++)
	{
		a = i;
		b = 0;
		for (j = 0; j < p; j ++)
		{
			b = (b << 1) + (a & 1);     // b = b * 2 + a % 2;
			a >>= 1;         // a = a / 2;
		}
		if ( b > i)
		{
			t=xreal[i],xreal[i]=	xreal[b],xreal[b]=t;
			t=ximag[i],ximag[i]=	ximag[b],ximag[b]=t;
		}
	}
}

#define	nfft (FreCalDataLength/8)
float fs=32000;
#define Data int
double y[4];
double wreal[nfft/2];
double wimag[nfft/2];
int x[2][nfft];
double pxreal[nfft];
double pximag[nfft];

void findmainfreq (void)
{
	//int i,j,b,k,sk,nfft;//f_num频点号，err_num误差数
	int i=0,j=0,m,k,l,t,b,channel;
	int index1, index2;
	float treal, timag, ureal, uimag, arg;
	double A_eff[2]={0},max=0;
	for(i=0;i<FreCalDataLength;i+=8)
	{
		x[0][j]=(int)((FreCalData[cnt5%CalDataNum][i]+FreCalData[cnt5%CalDataNum][i+1]*256+FreCalData[cnt5%CalDataNum][i+2]*256*256)*256)/256;
		x[1][j]=(int)((FreCalData[cnt5%CalDataNum][i+4]+FreCalData[cnt5%CalDataNum][i+5]*256+FreCalData[cnt5%CalDataNum][i+6]*256*256)*256);	
		//fprintf(fp1,"%d\n",x[j][0]);
		j++;
	}
	cnt5++;
	for(channel=0;channel<2;channel++)
	{
		for(i=0;i<nfft;i++)
		{
			pximag[i]=0;
			pxreal[i]=x[channel][i]*0.000298023223876953125;
		}
		bitrp (pxreal, pximag, nfft);
		// 计算 1 的前 n / 2 个 n 次方根的共轭复数 W'j = wreal [j] + i * wimag [j] , j = 0, 1,   , n / 2 - 1
		arg = - 2 * PI / nfft;
		treal = cos (arg);
		timag = sin (arg);
		wreal [0] = 1.0;
		wimag [0] = 0.0;
		for (j = 1; j < nfft / 2; j ++)
		{
			wreal [j] = wreal [j - 1] * treal - wimag [j - 1] * timag;
			wimag [j] = wreal [j - 1] * timag + wimag [j - 1] * treal;
		}

		for (m = 2; m <= nfft; m *= 2)
		{
			for (k = 0; k < nfft; k += m)
			{
				for (j = 0; j < m / 2; j ++)
				{
					index1 = k + j;
					index2 = index1 + m / 2;
					t = nfft / m* j ;     // 旋转因子 w 的实部在 wreal [] 中的下标为 t
					treal = wreal [t] * pxreal [index2] - wimag [t] * pximag [index2];
					timag = wreal [t] * pximag [index2] + wimag [t] * pxreal [index2];
					ureal = pxreal [index1];
					uimag = pximag [index1];
					pxreal [index1] = ureal + treal;
					pximag [index1] = uimag + timag;
					pxreal [index2] = ureal - treal;
					pximag [index2] = uimag - timag;
				}
			}
		}

		//确定第一个频点
		//b=findmax(pxreal,pximag,2,nfft/2);
		for(i=1; i < nfft/2; i++) 
		{   
			//if(max <fabs(x[i+1]-x[i])&&((x[i+1]-x[i])*(x[i+fs+1]-x[i+fs]))>0) 

			max=sqrt(pxreal[i]*pxreal[i]+pximag[i]*pximag[i]);
			if(A_eff[channel] <max)
			{ 
				A_eff[channel] = max; 
				b = i; 
			} 
		} 

		y[channel*2]=fs/nfft*b;
		y[channel*2+1]=A_eff[channel]/sqrt((double)2)*2/nfft;


	}
}
struct tm FreCalTime;

#define BIN2BCD(val)	((((val)/10)<<4) + (val)%10)
*/
/*
void *Frecal(void *arg)//write file thread 
{
	while(!acq_one_ok)
	{
		sem_wait(&time_up);
		if(!acq_one_ok){
		findmainfreq();CalFlag=1;
		read_rtc(&FreCalTime,RTC_ACQ_CS);
		time2str(TimeStr,&FreCalTime);
*/
		/*tval->tm_sec = BCD2BIN(rtcdata[0]&0x7F);
		tval->tm_min = BCD2BIN(rtcdata[1]&0x7F);
		tval->tm_hour = BCD2BIN(rtcdata[2]&0x3F);
		tval->tm_wday = BCD2BIN(rtcdata[3]&0x7);
		tval->tm_mday = BCD2BIN(rtcdata[4]&0x3F);
		tval->tm_mon = BCD2BIN(rtcdata[5] & 0x1F)-1;
		tval->tm_year = BCD2BIN(rtcdata[6]);*/
/*
		//asctime(tval)
		//printf("%d%x%x%x%x%x\n",BIN2BCD(FreCalTime.tm_year),BIN2BCD(FreCalTime.tm_mon+1),BIN2BCD(FreCalTime.tm_mday),BIN2BCD(FreCalTime.tm_hour),BIN2BCD(FreCalTime.tm_min),BIN2BCD(FreCalTime.tm_sec));
		fprintf(fpFreCal,"%s %f %f %f %f\n",TimeStr,y[0],y[1],y[2],y[3]);
		//printf("%f\n",y[0]);
		}
	}
	pthread_exit(NULL);
}*/
/*
void sigalrm_fn(int sig)
{	
	CalFlag=1;
	if(!acq_one_ok) alarm(10);
	sem_post(&set_write);
    return;
}*/
			
/*
 每1MB数据加入4个4B 
第一个4B 0XFFFF 2B 1MB的块ID 2B
第二个4B 距离起始对钟时间的整秒数2*109s 约为24 855days
第三个4B 为毫秒数
第四个4B 数据校验和*/
void *writefile(void *arg)//write file thread 
{
	int fd;fd=open_mfem(); 

	while(!acq_one_ok)
	{
		sem_wait(&set_write);
		
		//写数据 和写16字节的顺序不能弄错
		fwrite(psscdata[cnt2%BufNum], 1, MemSize, fp_datafile);  //SSC0 SSC1
		
		/*memcpy(WriteData[cnt6%BufNum]+WriteDataOffset,psscdata[cnt2%BufNum],MemSize);
		WriteDataOffset+=MemSize;

		if(WriteDataOffset==WriteDataLength)
		{
			fwrite(WriteData[cnt6%BufNum], WriteDataLength, 1, fp_datafile);  //SSC0 SSC1
			WriteDataOffset=0;CalFlag=0;
			cnt6++;//sem_post(&time_up);
		}*/
		wrcnt++;
		//1M 写一次CPLD MS
		if(wrcnt %8==0 )	{
			msCnt = GetCPLDMS(fd); blkID ++;
			fwrite(&blkFlg,4,1,fp_datafile); //FF FF FF FF
			fwrite(&blkID,4,1,fp_datafile);   //ID
			fwrite(&secCnt,4,1,fp_datafile);	//秒		
			fwrite(&msCnt,4,1,fp_datafile);  //mS	
		} //先读数据
		fflush(fp_datafile);
		cnt2++; 
		//led_change();
	}
	close(fd);
	pthread_exit(NULL);
}


pthread_t a_thread,b_thread,c_thread,d_thread;
void *thread_result,*thread_result1,*thread_result2;


int adc_init(void)//创建线程
{
	UDP_INIT();
	//fpFreCal=fopen
	cnt1=cnt2=cnt3=cnt4=cnt5=cnt6=0;FreCalDataOffset=0;WriteDataOffset=0;
	int res;
	res = sem_init(&set_write, 0, 0);
	acq_one_ok = 0;
	if (res != 0)
	{
		perror("semaphore init error!\n ");
		exit(EXIT_FAILURE);
	}
	//res = sem_init(&set_udp, 0, 0);
	//if (res != 0)
	//{
	//	perror("semaphore init error!\n ");
	//	exit(EXIT_FAILURE);
	//}
	res = sem_init(&time_up, 0, 0);
		if (res != 0)
		{
			perror("semaphore init error!\n ");
			exit(EXIT_FAILURE);
		}
	res = pthread_create(&a_thread, NULL, writefile, NULL);
	if (res != 0)
	{
		perror("thread function create error!\n ");
		exit(EXIT_FAILURE);
	}
	//res = pthread_create(&b_thread, NULL, UDP_SEND, NULL);
	//if (res != 0)
	//{
	//	perror("thread function create error!\n ");
	//	exit(EXIT_FAILURE);
	//}

/*	if(hsflag==0)					//CH20150804
	{	res = pthread_create(&c_thread, NULL, Frecal, NULL);
		if (res != 0)
		{
			perror("thread function create error!\n ");
			exit(EXIT_FAILURE);
		}
	}
*/	
	usleep(100);
	usleep(5000);
	return 1;
}

void TranIdlePackage(void)
{
	int FinishPackage[32];
	sendto(sock,FinishPackage,32,0,(struct sockaddr*)&ClientAddr,sizeof(ClientAddr));
}

//int acq_onerate(struct acqprm *p_acq_prm,int gpsenable) //acq one rate
int acq_onerate(struct acq_prm *pacq) //acq one rate
{
	u32 i = 0;int j=0;int kill_rc=1;int test=0;
	int fd;int res;
	cnt1=cnt2=cnt3=cnt4=cnt5=cnt6=0;FreCalDataOffset=0;WriteDataOffset=0;
	//sin(1);
	fd=open_mfem();	
	fp_datafile = fopen(AcqFileName, "wb");	
	for(j=0;j<BufNum;j++){psscdata[j] = malloc(MemSize);}
	
	//signal(SIGALRM, sigalrm_fn);alarm(10);

	if(fd == -1)return -1;
	if(fp_datafile == NULL){perror("Data File Failed!");return -1;}
	else printf("DataFileName:%s\n",AcqFileName);
	if (NULL == psscdata[BufNum-1])
		{
			printf("malloc psscdata error!");
			return -1;
	
		}
	printf ("Start...\n");
	blkID = 0;wrcnt = 0;blkID ++;sendcnt=0;
	u8 cpldMSBuf[4] = "";
	CalFlag=1;
	ioctl(fd, CMD_SSC_START,0); //ioctl(fd, CMD_SSC_START);//
	acq_one_ok = 0;	        //SSC_begin
	//ioctl(fd, CMD_READ_CPLD_MS,cpldMSBuf);
	//msCnt = ((u32)cpldMSBuf[0] << 24) + ((u32)cpldMSBuf[1] << 16) + ((u32)cpldMSBuf[2] << 8) + cpldMSBuf[3];
	//msCnt = GetCPLDMS(fd); 
	blkID ++;
	fwrite(&blkFlg,4,1,fp_datafile); //FF FF FF FF
	fwrite(&blkID,4,1,fp_datafile);   //ID
	fwrite(&secCnt,4,1,fp_datafile);	//秒		
	fwrite(&msCnt,4,1,fp_datafile);  //mS
	do
	{	nCurAcqNum = cnt1 + 1; 
		read(fd, psscdata[cnt1%BufNum], MemSize);  //SSC1
		if(*ACQFLAG==0 || nCurAcqNum==nTotalAcqLen) {acq_one_ok=1;}
		sem_post(&set_write);
		//sem_post(&set_udp);	
		cnt1++;
	}while(!acq_one_ok);
	
	ioctl(fd, CMD_SSC_STOP); 
	//ioctl(fd, CMD_SETCAL,1);
	//ioctl(fd, CMD_SETCAL_COIL,1);
	TranIdlePackage();
	//注销线程

	res = pthread_join(a_thread, &thread_result);
	if(res != 0)	printf("thread join error!\n");
	//res = pthread_join(b_thread, &thread_result1);
	//if(res != 0)	printf("thread join error!\n");
	

	sem_destroy(&set_write);
	//sem_destroy(&set_udp);	
	sem_destroy(&time_up);	
	for(j=0;j<BufNum;j++)free(psscdata[j]);

	fclose(fp_datafile);
	close(fd);
	return 0;
}

