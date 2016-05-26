#include "common.h"
#ifndef _TIME_H_
#define _TIME_H_

typedef struct
{
	void	(*init)();
	uint32	PeriodUs;
	char	PULSE;
	float	RunTime; //(s)
	struct {
	PITx 	PIT_Pitx;
	void	(*start)();
	void	(*end)();
	uint32	runtime;
	} timer;
} SystemTypeDef;
void pit2_init(void);
void pit2_isr(void);
void pit3_init(void);
void pit3_isr(void);
void pit3_disinit(void);
#endif
