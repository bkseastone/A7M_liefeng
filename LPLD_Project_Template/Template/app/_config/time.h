#include "common.h"
#ifndef _TIME_H_
#define _TIME_H_

typedef struct
{
	void	(*init)();
	uint32	PeriodUs;
	char	PULSE;
	float	RunTime; //(s)
} SystemTypeDef;
void pit2_init(void);
void pit2_isr(void);
#endif
