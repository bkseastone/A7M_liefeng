#include "common.h"
#ifndef _SERVOPID_H_
#define _SERVOPID_H_

//	typedef struct
//	{
//		int32 (*cal)();
//		float Kp;
//		float Ti;
//		float Td;
//		float nowe;	//e(k)
//		float laste;	//e(k-1)
//		float paste;	//e(k-2)
//		int32 dout;
//		int32 out;
//	} ServoPIDTypeDef;
//	int32 Servo_PID_cal(void);

	typedef struct
	{
		int32 (*cal)();
		float Kp;
		float Kd;
		float nowe;	//e(k)
		float laste;	//e(k-1)
		int32 out;
		float e_dis;
	} WeizhiPIDTypeDef;
	int32 Weizhi_PID_cal(void);

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
