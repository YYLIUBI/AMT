#ifndef AD_H
#define AD_H

#include "macro_def.h"

int adc_init(void);
//int acq_onerate(struct acqprm *p_acq_prm,int gpsenable); 
int acq_onerate(struct acq_prm *p_acq_prm); 
//int acq_stop(void); 
//int acq_reset(void); 
//int led_on(void);
//int led_off(void);
//int led_change(void);
//int tranfile(char *file_name_source);
//void set_filename(struct acq_prm *p_acq_prm, struct cal_prm *p_cal_prm);
void set_filename(struct acq_prm *p_acq_prm);
//void AddEnvData(void);         //采集结束后添加ENV参数
void UDP_INIT(void);
//int ads_init(struct acq_prm *p_acq_prm);
//void SDRAM_RESET(void);
//void HSAD_DataEn(void);
//int HSAD_START(void);


#endif
