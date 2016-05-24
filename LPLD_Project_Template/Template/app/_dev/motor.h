#include "common.h"
#ifndef _MOTOR_H_
#define _MOTOR_H_

typedef struct
{
	void (*init)();
	FTM_Type *FTM_Ftmx;
	FtmChnEnum_Type chn;
	PortPinsEnum_Type pin;
	uint32 Freq;
	uint32 Duty;
	uint32 Velosity;
	uint32 Acceleration;
	uint32 Target_Velosity;
} MotorTypeDef;
void ftm_motorB_init(void);
void ftm_motorF_init(void);
#endif
