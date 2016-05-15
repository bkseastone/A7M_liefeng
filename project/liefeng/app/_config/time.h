#include "common.h"
#ifndef _TIME_H_
#define _TIME_H_




typedef struct
{
	FTM_Type *FTM_Ftmx;
	FtmChnEnum_Type chn;
	PortPinsEnum_Type pin;
	uint32 Freq;
	uint32 Duty;
	uint32 Velosity;
} mypwm;
void ftm_pwm_init(mypwm *pwm);
uint32 angle_to_period(int32 angle); //[-180,+180]
void pit_init(PITx PIT_Pitx, uint32 PIT_PeriodUs);
void pdb_init(uint8 PDB_TriggerInputSourceSel, ADC_Type *ADC_Adcx);


#endif
