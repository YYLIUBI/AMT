#ifndef AD_H
#define AD_H

#include "macro_def.h"

int ad_init_reg();
int acq_onerate(); 
int acq_stop(void); 
int acq_reset(void); 
int led_off(void);
int led_on(void);
int led_change(void);
void set_filename();
void AddEnvData(void);         //采集结束后添加ENV参数

void UDP_INIT(void);
 int ads_init(void);

 void nametag(int mon,char day);
 extern u8 AMTMTFLAG;


#endif
