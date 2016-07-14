#include "common.h"

#ifndef _SUP_CHECK_H_
#define _SUP_CHECK_H_

typedef struct
{
	char start_end; //start1 end0
	char lineL;
	char lineR;
	char alert;
	uint32 perioud;
} PhotocellTypeDef;
//光电管
void init_photocellB2B3(void);
void photocell_isr(void);
void init_led(void);
//拨码开关
void init_PlanSelection(void);
#endif
