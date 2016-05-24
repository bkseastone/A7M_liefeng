#include "common.h"
#ifndef _SERVOMOTOR_H_
#define _SERVOMOTOR_H_

typedef struct
{
	void (*init)();
	FTM_Type *FTM_Ftmx;
	FtmChnEnum_Type chn;
	PortPinsEnum_Type pin;
	uint32 Freq;
	uint32 Duty;
	int32 Deflection;
} ServomotorTypeDef;
void ftm_pwmS_init(void);
uint32 angle_to_period(int32 angle);
#endif
