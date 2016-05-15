#include "common.h"

#ifndef _CAPTURE_H_
#define _CAPTURE_H_

typedef struct
{
	FTM_Type *FTM_Ftmx;
	FtmChnEnum_Type chn;
	PortPinsEnum_Type pin;
	void (*isr)();
} myic;
void ic_init(myic* ic);
void ic2_isr(void);
void ic3_isr(void);

#endif
