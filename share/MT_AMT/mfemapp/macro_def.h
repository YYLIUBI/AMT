#ifndef MACRO_DEF_H
#define MACRO_DEF_H

#define FALSE  -1
#define TRUE   0

//SPI CSƬѡ�ź�
#define AD1_CS            0
#define AD2_CS            1
#define ADA_CS            2           //AD All CS,����AD��CSͬʱ����
#define RTC_ACQ_CS        3    //���ݲɼ�CS
#define RTC_ECORRODE_CS   4    //�縯ʴ�ɼ�CS
#define CPLD_CS  				  5
#define DDS_CS            6

//�����ж���ĸ�������
#define CMD_SPI_TEST         10
#define CMD_SPI_WRITE_TEST   11
#define CMD_ACCESS_SPI       12
#define CMD_RELEASE_SPI      13 
#define CMD_RTC_READ         14
#define CMD_RTC_WRITE        15
#define CMD_ALARM_INIT       17
#define CMD_ALARM_READ       18
#define CMD_READ_ADSTATUS    19
#define CMD_AD_RESET         20
#define CMD_CONFIG1282       21//20160505
#define CMD_SSC_START        22
#define CMD_SSC_STOP         23
#define CMD_LED_ON           24
#define CMD_LED_OFF          25
#define CMD_LED_CHANGE       26
#define CMD_WAIT_PPS         27
#define CMD_WAIT_RTCALARM    28
#define CMD_READ_CPLD_DRIFT  29
#define CMD_READ_CPLD_MS     30
#define CMD_PPS_START        31
#define CMD_PPS_STOP         32
#define CMD_MS_COUNT_START   33
#define CMD_MS_COUNT_CLEAR   34
//����DDS��ص�
#define CMD_SET_DDS_SINWAVE  35
#define CMD_SET_DDS_SQUARE   36
#define CMD_SET_DDS_FREQ     37
#define CMD_GET_PPS_FLG      40
#define CMD_SET_SRATE_PIN    41   //���ò�����ָʾ�ܽŵ�״̬
#define CMD_ENV_POWER_ON     42  //ENV�ϵ�
#define CMD_ENV_POWER_OFF    43  //ENV����
#define CMD_ENV1_SEL         44
#define CMD_ENV2_SEL         45
#define CMD_READ1282         46
#define CMD_CPLD_SPI_TEST    47   //����CPLD SPI�Ƿ����
#define CMD_AD_ANA_POWER_ENA  48  //����AD ANA�����ϵ�
#define CMD_AD_ANA_POWER_DIS  49

#define CMD_SEL_AMTMT        50 // 0 amt 1 mt
#define CMD_BUFFER_SET            60
#define CMD_SEL_MODE        	  50 // 0 amt 1 mt
#define CMD_SSC_START_MT     65

#define CMD_SETHLSC  		 51	

/*
#define SRATE_1000Hz    1
#define SRATE_100Hz     2
#define SRATE_10Hz      3
*/ 
#define SRATE_HIGH      1
#define SRATE_MID       2
#define SRATE_LOW       3
#define SRATE_ACQ_STOP  0


//ENV �������
#define ENV_POWER_ON_MODE   1
#define ENV_POWER_OFF_MODE  2
#define ENV_SIG_SEL1        1
#define ENV_SIG_SEL2        2
//�縯ʴ�������
#define ENABLE   1   
#define DISABLE  2

//IIC�������
#define ARM_BOARD  0
#define AD_BOARD   1
#define ANA_BOARD  2
#define HAMP_BOARD 3
#define EAMP_BOARD 4
#define MSP430G1   5

//ARM����ADģ�ⲿ�ֵ�Դ�������
#define AD_ANA_ENABLE     1     //AD ANA�ϵ�
#define AD_ANA_DSIABLE    2     //AD ANA���ϵ�

//�������Ͷ������
#define s8  char
#define u8 unsigned char
#define s16  short
#define u16 unsigned short
#define s32  int
#define u32 unsigned int
#define s64  long long
#define u64 unsigned long long

#define ONE_K 1024
#define ONE_M (1024 * 1024)



#endif

