#include "common.h"
#ifndef _SERVOPID_H_
#define _SERVOPID_H_
	typedef struct
	{
		int32 (*cal)();
		float Kp;
		float Ti;
		float Td;
		float nowe;		//e(k)
		float laste;	//e(k-1)
		float paste;	//e(k-2)
		int32 dout;
		int32 out;
	} ServoPIDTypeDef;
	int32 Servo_PID_cal(void);
#endif
