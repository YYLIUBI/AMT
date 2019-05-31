//startup.prm
/*
VERSION = MFEM_V0.0
SUR_ID = MFEM
ACQ_PATH = /mnt/sd/data/
CAL_PATH = /mnt/sd/cal/
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
#ifndef BUILD_PRM_H
#define BUILD_PRM_H

#include "macro_def.h"
#include "gps_rtc.h"

#define ACQ_NUM 4 

struct acq_prm{
	int samplerate;
	int egain;
	int hgain;
	int calmode;
	int callen;
};
struct acq_prm acq_prm_list[ACQ_NUM]; //prm in list

struct cal_prm {
	struct tm stime;
	struct tm etime;
	char version[5];
	char file_name[40];
	char prm_name[40];
	char cal_path[40];
	char acq_path[40];
	char box_id[4];
	int cal_loop;
	u32 file_size;  //单位是KB
	int egain;
	int hgain;
	int cal_coil;
	int egianA_LS;
	int egianA_HS;
	int hgianA_LS;
	int hgianA_HS;
	char sratechar;   //指定采样率的字符 2000-‘H’ 100-'M' 10-'L'
	struct gps_info str_gps;
	char syncTime[25]; //Mon Sep 14 08:36:13 2012
	int exlen;
	int eylen;
	int eazm;
	int hazm;
	char hxsn[9];
	char hysn[9];
	char hzsn[9];
	int chnum;
	int chid[6];
};
struct cal_prm calprm;

void build_prm(char *prm_file_name, char display_on);
#endif
