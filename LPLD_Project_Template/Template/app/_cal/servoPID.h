#include "common.h"
#ifndef _SERVOPID_H_
#define _SERVOPID_H_
//舵机PID
	typedef struct
	{
		int32 (*cal)();
		float Kp;
		float Ti;
		float Td;
		float nowe;	//e(k)
		float laste;	//e(k-1)
		float paste;	//e(k-2)
		int32 dout;
		int32 out;
	} ServoPIDTypeDef;
	int32 Servo_PID_cal(void);
//舵机位置PD
	typedef struct
	{
		int32 (*cal)();
		float Kp;
		float Kd;
		float nowe;	//e(k)
		float laste;	//e(k-1)
		int32 out;
	} WeizhiPIDTypeDef;
	int32 Weizhi_PID_cal(void);
//电机PID        
        typedef struct
	{
		int32 (*cal)();
		float Kp;
		float Ti;
		float Td;
		float nowe;	//e(k)
		float laste;	//e(k-1)
		float paste;	//e(k-2)
		int32 dout;
		int32 out;
	} motorPIDTypeDef;
	int32 motor_PID_cal(void);
#endif
