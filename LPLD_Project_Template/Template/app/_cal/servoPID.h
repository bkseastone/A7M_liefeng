#include "common.h"
#ifndef _SERVOPID_H_
#define _SERVOPID_H_
	typedef struct
	{
		int32 (*cal)();
		float Kp;
		float Ti;
		float Td;
		int32 nowe;		//e(k)
		int32 laste;	//e(k-1)
		int32 paste;	//e(k-2)
		int32 dout;
		int32 out;
	} ServoPIDTypeDef;
	int32 Servo_PID_cal(void);
#endif
