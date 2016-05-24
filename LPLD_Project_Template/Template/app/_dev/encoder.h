#include "common.h"

#ifndef _ENCODER_H_
#define _ENCODER_H_



typedef struct
{
	void (*init)();
	FTM_Type *FTM_Ftmx;
	FtmChnEnum_Type chn;
	PortPinsEnum_Type pin;
	void (*isr)();
} myic;
void icFeedback_init(void);
void icTest_init(void);
void ic2_isr(void);
void ic3_isr(void);
#endif
