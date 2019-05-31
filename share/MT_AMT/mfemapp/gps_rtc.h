#ifndef GPS_RTC_H
#define GPS_RTC_H

#include "macro_def.h"

//ADS1100 配置寄存器位
#define    ST_BSY  7 
#define    SC      4    //SC = 0 continuous conversion mode;1: signal conversion mode
#define    DR1     3 
#define    DR0     2 
#define    PGA1    1 
#define    PGA0    0 


//TMP275 POINTER REGISTER
//P1:P0  00 :Temperature Register (READ Only);01:Configuration Register (READ/WRITE);
#define  TMP275_POINT_REG_P1   1
#define  TMP275_POINT_REG_P0   0

//TMP275 CONFIGURATION REGISTER
#define  TMP275_CONFIG_REG_SD   0  //shut down mode  1:shut down after one conversion ;0 :maintain continous conversion
#define  TMP275_CONFIG_REG_TM   1
#define  TMP275_CONFIG_REG_POL  2
#define  TMP275_CONFIG_REG_F0   3
#define  TMP275_CONFIG_REG_F1   4
#define  TMP275_CONFIG_REG_R0   5
#define  TMP275_CONFIG_REG_R1   6   //R1 R0 CONVERTER RESOLUTION R1:R0 = 11,12-bit;10,11-bit;01,10-bit;00,9-bit;
#define  TMP275_CONFIG_REG_OS   7


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
//环境模块参数信息
struct env_info{
	float X_angle;
	float Y_angle;
	float Z_angle;
};


int set_Parity(int fd, int databits, int stopbits, int parity);
int set_termios(int fd,long speed,char databit,char stopbit,char oebit);
void set_speed(int fd, int speed);
int OpenDev(const char * strdev,int flg);   //打开串口设备
int get_gps(struct gps_info *pgps);
//int PPS_On();
//int PPS_Off();
int wait_pps(void);

int get_env(struct env_info *penv);
struct env_info print_env(void);        //HJJ 返回值：姿态参数  01.07
//int EnvPowerCtrl(int envpowermode);
//int EnvSigSel(int envsel);
int CheckStr(const char *inChar,const char *strBegin,const char *strEnd);

int read_rtc(struct tm *tval,u8 rtccs);
void set_rtc_cmd(char *p,u8 rtccs);
int set_rtc(struct tm *tval,u8 rtccs);
int set_alarm(int mday,int hour,int min,int sec) ;

void wait_alarm(void);   //等待采集RTC的定时中断


//int read_temp(char *p);
//void set_temp(void) ;
int print_gps(void);
//int set_bat(void);
//float read_bat(char* p);   //返回电池电压值 01.07
//int ReadAD590Temp(char* p);
//int SetAD590TempAD(void);  //设置AD0-ADS1100用于获取温度
//int get_boxid(char* p);
//int set_boxid(char *p);
//void GetCPLDCount(void);
//u32 GetCPLDMS(int fd);

//void LockPPS(u8 *TimeBuf);

//void Set_DDS_Sin(void);
//void Set_DDS_Square(void);

//int set_id(u8 board,char *p);
//int get_id(u8 board,char* p);

//void ECorrodeSet(u8 enable);   //电腐蚀使能设置

//int get_msp430_data(char *p);//13 05 29
//int get_powerboard(void);
#endif
