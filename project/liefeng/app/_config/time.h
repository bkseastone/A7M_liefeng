#include "common.h"
#ifndef _TIME_H_
#define _TIME_H_




typedef struct
{
	FTM_Type *FTM_Ftmx;
	FtmChnEnum_Type chn;
	PortPinsEnum_Type pin;
} mypwm;
status ftm_pwm_init(mypwm *pwm, uint32 FTM_PwmFreq, uint32 duty);
uint32 angle_to_period(uint32 angle);
void delay();
uint8 pit_init(PITx PIT_Pitx, uint32 PIT_PeriodUs);
uint8 pdb_init(uint8 PDB_TriggerInputSourceSel, ADC_Type *ADC_Adcx);


#endif
