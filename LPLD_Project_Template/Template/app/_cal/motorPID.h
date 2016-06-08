#ifndef _motorPID_H_
#define _motorPID_H_
	#include "common.h"
	#define PID_KP				0.2
	#define PID_KI				0
	#define PID_KD				0
	#define PID_NOWE			0
	#define PID_LASTE			0
	#define PID_PASTE			0
	#define PID_OUT				0
	typedef struct
	{
		float kp;
		float ki;
		float kd;
		int32 nowe;
		int32 laste;
		int32 paste;
		int32 out;
	}PIDTypeDef;
	int32 PID_cal(PIDTypeDef* PID);
#endif
