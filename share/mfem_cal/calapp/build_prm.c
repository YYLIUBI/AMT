/*
//startup.prm
/*
VERSION = MFEM_V0.0
SUR_ID = MFEM
ACQ_PATH = /mnt/sd/data/
CAL_PATH = /mnt/sd/cal/
CAL_COIL = 1
BOX_ID = 1606
CH_Ex = 0
CH_Ey = 1
CH_Ez = 2
CH_Hx = 3
CH_Hy = 4
CH_Hz = 5
CAL_LOOP = 1
GAINA_E_LS = 1
GAINA_H_LS = 1
GAINA_E_HS = 10
GAINA_H_HS = 3
GIAND_E = 1
GIAND_H = 1
ExLEN = 100
EyLEN = 100
EAZM = 0
HxSN = MT5H0000
HySN = MT5H0000
HzSN = MT5H0000
HAZM = 0
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <dirent.h>

#include "build_prm.h"
#include "gps_rtc.h" 
#include "macro_def.h"

extern int file_exists(char *filename);  //判断文件是否存在 如果存在就返回0 如果不存在就返回 -1
void build_prm(char *prm_file_name, char display_on)
{
	FILE *fp;
	FILE *fptime;           //2012.09.14  timelog.txt文件
	char *ptimestr = NULL,*ptime = NULL,*ptimevalid = NULL;  //2012.09.14
	int size_timefile = 0;
	char syncDate[11],syncTime[9];
	int acq_num=0;
	char *pstr, *p;
	int file_size;
	int i;
	int total_file_size = 0;
	struct tm tmptime;

	fp = fopen(prm_file_name, "r");
	if (fp == NULL)
	{
		printf("prm file not found!\n");
		return ;
	}
	fseek(fp, 0, SEEK_END);
	file_size = ftell(fp);
	if (file_size == -1L)
	{
		printf("ftell error!\n");
		fclose(fp);
		return ;
	}
	if (display_on)
		printf("Destinate File Name: %s,file_size: %d\n", prm_file_name, file_size);
		
	pstr = (char *)malloc(file_size);
	if (pstr == NULL)
	{
		printf("allocate error!\n");
		return ;
	}
	fseek(fp, 0 , SEEK_SET);
	fread(pstr, file_size, 1, fp);
	fclose(fp);
	fp = NULL;

	p = strstr(pstr,"VERSION = ");
	if(p != NULL)
		sscanf(p + 10, "%s", calprm.version);
	p = strstr(pstr,"ACQ_PATH = ");
	if(p != NULL)
	 	sscanf(p + 11, "%s", calprm.acq_path);
	p = strstr(pstr,"CAL_PATH = ");
	if(p != NULL)
	 	sscanf(p + 11, "%s", calprm.cal_path);
	p=strstr(pstr,"CAL_LOOP = ");//CAL_LOOP = 1
	if(p != NULL)
		sscanf(p+11, "%d", &calprm.cal_loop);
	p=strstr(pstr,"BOX_ID = ");	//CH20150804
	if(p != NULL)
		sscanf(p+9, "%s", &calprm.box_id);	

    p = strstr(pstr,"CAL_COIL = ");
	if(p != NULL)
		sscanf(p + 11, "%d", &calprm.cal_coil);
			
	p = strstr(pstr,"HxSN = ");
	if(p != NULL)
		sscanf(p + 7, "%s", calprm.hxsn);
	p = strstr(pstr,"HySN = ");
	if(p != NULL)
		sscanf(p + 7, "%s", calprm.hysn);
	p = strstr(pstr,"HzSN = ");
	if(p != NULL)
		sscanf(p + 7, "%s", calprm.hzsn);
	p = strstr(pstr,"ExLEN = ");
	if(p != NULL)
		sscanf(p + 7, "%d", &calprm.exlen);
	p = strstr(pstr,"EyLEN = ");
	if(p != NULL)
		sscanf(p + 7, "%d", &calprm.eylen);
	p = strstr(pstr,"EAZM = ");
	if(p != NULL)
		sscanf(p + 7, "%d", &calprm.eazm);	
	p = strstr(pstr,"HAZM = ");
	if(p != NULL)
		sscanf(p + 7, "%d", &calprm.hazm);	
		
	p = strstr(pstr,"GAINA_E_LS = ");
	if(p != NULL)
		sscanf(p + 13, "%d", &calprm.egianA_LS);
	p = strstr(pstr,"GAINA_H_LS = ");
	if(p != NULL)
		sscanf(p + 13, "%d", &calprm.hgianA_LS);
	p = strstr(pstr,"GAINA_E_HS = ");
	if(p != NULL)
		sscanf(p + 13, "%d", &calprm.egianA_HS);
	p = strstr(pstr,"GAINA_H_HS = ");
	if(p != NULL)
		sscanf(p + 13, "%d", &calprm.hgianA_HS);
	p = strstr(pstr,"GIAND_E = ");	
	if(p != NULL)
		sscanf(p + 10, "%d", &calprm.egain);
	p = strstr(pstr,"GIAND_H = ");
	if(p != NULL)
		sscanf(p + 10, "%d", &calprm.hgain);	

	p = strstr(pstr,"CH_NUM = ");
	if(p != NULL)
		sscanf(p+9, "%d", &(calprm.chnum));
	p = strstr(pstr,"Ex = ");
	if(p != NULL)
		sscanf(p+5, "%d", &(calprm.chid[0]));
	p = strstr(pstr,"Ey = ");
	if(p != NULL)
		sscanf(p+5, "%d", &(calprm.chid[1]));
	p = strstr(pstr,"Ez = ");
	if(p != NULL)
		sscanf(p+5, "%d", &(calprm.chid[2]));
	p = strstr(pstr,"Hx = ");
	if(p != NULL)
		sscanf(p+5, "%d", &(calprm.chid[3]));
	p = strstr(pstr,"Hy = ");
	if(p != NULL)
		sscanf(p+5, "%d", &(calprm.chid[4]));
	p = strstr(pstr,"Hz = ");
	if(p != NULL)
		sscanf(p+5, "%d", &(calprm.chid[5]));	

	if (display_on)
	{
		printf("VERSION: %s\n", calprm.version);
		printf("ACQPATH: %s\n", calprm.acq_path);
		printf("CALPATH: %s\n", calprm.cal_path);

		printf("CAL_LOOP: %d\n", calprm.cal_loop);
		printf("BOXID: %s\n", calprm.box_id);
		
		printf("HxSN : %s\n", calprm.hxsn);
		printf("HySN : %s\n", calprm.hysn);
		printf("HzSN : %s\n", calprm.hzsn);		
		printf("ExLEN: %d\n", calprm.exlen);
		printf("EyLEN: %d\n", calprm.eylen);
		printf("EAZM: %d\n", calprm.eazm);
		printf("HAZM: %d\n", calprm.hazm);	
		printf("E_GAIN_A_HS: %d\n", calprm.egianA_HS);
		printf("E_GAIN_A_LS: %d\n", calprm.egianA_LS);
		printf("H_GAIN_A_HS: %d\n", calprm.hgianA_HS);
		printf("H_GAIN_A_LS: %d\n", calprm.hgianA_LS);
		printf("H_GAIN_D: %d\n", calprm.egain);
		printf("H_GAIN_D: %d\n", calprm.hgain);
			
		printf("chnum: %d\n", calprm.chnum);
		printf("CHID : %d %d %d %d %d %d\n", 
							calprm.chid[0],
							calprm.chid[1],
							calprm.chid[2],
							calprm.chid[3],
							calprm.chid[4],
							calprm.chid[5]);
		
	}
		acq_prm_list[0].samplerate=24000;
		acq_prm_list[0].calmode=0;
		acq_prm_list[0].callen=10*calprm.cal_loop;
		acq_prm_list[0].egain=calprm.egain;
		acq_prm_list[0].hgain=calprm.hgain;
		acq_prm_list[1].samplerate=2400;
		acq_prm_list[1].calmode=0;
		acq_prm_list[1].callen=5*calprm.cal_loop;
		acq_prm_list[1].egain=calprm.egain;
		acq_prm_list[1].hgain=calprm.hgain;
		acq_prm_list[2].samplerate=150;
		acq_prm_list[2].calmode=0;
		acq_prm_list[2].callen=1*calprm.cal_loop;
		acq_prm_list[2].egain=calprm.egain;
		acq_prm_list[2].hgain=calprm.hgain;
		acq_prm_list[3].samplerate=15;
		acq_prm_list[3].calmode=0;
		acq_prm_list[3].callen=1*calprm.cal_loop;
		acq_prm_list[3].egain=calprm.egain;
		acq_prm_list[3].hgain=calprm.hgain;
		acq_prm_list[4].samplerate=2400;
		acq_prm_list[4].calmode=1;
		acq_prm_list[4].callen=5*calprm.cal_loop;
		acq_prm_list[4].egain=calprm.egain;
		acq_prm_list[4].hgain=calprm.hgain;
		acq_prm_list[5].samplerate=150;
		acq_prm_list[5].calmode=1;
		acq_prm_list[5].callen=1*calprm.cal_loop;
		acq_prm_list[5].egain=calprm.egain;
		acq_prm_list[5].hgain=calprm.hgain;
		acq_prm_list[6].samplerate=15;
		acq_prm_list[6].calmode=1;
		acq_prm_list[6].callen=1*calprm.cal_loop;
		acq_prm_list[6].egain=calprm.egain;
		acq_prm_list[6].hgain=calprm.hgain;
		printf("Read acq_prm_list ok!\n");
}

