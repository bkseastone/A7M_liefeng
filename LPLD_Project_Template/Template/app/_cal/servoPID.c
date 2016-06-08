#include "servoPID.h"
#include "time.h"
#include "motor.h"
#include "OV7725_eagle.h"
extern OvTypeDef			*Ov7725;
extern SystemTypeDef	*Sys;
extern MotorTypeDef			*MotorB;
//舵机PID
ServoPIDTypeDef ServoMotor_PID_struct = { .cal=&Servo_PID_cal,
											.Kp=0.0001,	.Ti=10000,	.Td=0.00,
											.nowe=0,	.laste=0,	.paste=0,
											.dout=0,	.out=0}; // 舵机PID
ServoPIDTypeDef			*Servo_PID = &ServoMotor_PID_struct;
int32 Servo_PID_cal(void)
{
    Servo_PID->dout =  (int32)(Servo_PID->Kp*   (1.0f+    Servo_PID->Td/(Sys->PeriodUs/1000000.0f)    )*Servo_PID->nowe- \
			       Servo_PID->Kp*   (1.0f+  2*Servo_PID->Td/(Sys->PeriodUs/1000000.0f)    )*Servo_PID->laste+ \
			       Servo_PID->Kp*   (         Servo_PID->Td/(Sys->PeriodUs/1000000.0f)    )*Servo_PID->paste);
	Servo_PID->out = Servo_PID->out + Servo_PID->dout;
	Servo_PID->paste = Servo_PID->laste;
	Servo_PID->laste = Servo_PID->nowe;
	return Servo_PID->out;
}
//舵机位置PD
WeizhiPIDTypeDef WeizhiMotor_PID_struct = { .cal=&Weizhi_PID_cal,
											.Kp=0.7,		.Kd=0.00,
											.nowe=0,	.laste=0,
											.out=0}; // 舵机位置PD
WeizhiPIDTypeDef			*Weizhi_PID = &WeizhiMotor_PID_struct;
int32 Weizhi_PID_cal(void)
{
	float e_dis = 0;
	e_dis = Weizhi_PID->nowe-Weizhi_PID->laste;
//	if(e_dis>(8/Weizhi_PID->Kp)){
//		Ov7725->LOCK = 1;
//		LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, 0);
//	}
    Weizhi_PID->out =  (int32)(Weizhi_PID->Kp*       Weizhi_PID->nowe+ \
			       Weizhi_PID->Kd*       (e_dis) );
    Weizhi_PID->laste = Weizhi_PID->nowe;
    return Weizhi_PID->out;
}
//电机PID
motorPIDTypeDef motorMotor_PID_struct = { .cal=&motor_PID_cal,
                                          .Kp=0.1,	.Ti=1.3,	.Td=0, // 空转I0.0081  K0.5    道路.Kp=0.01  .Ti=0.005,
                                          .nowe=0,	.laste=0,	.paste=0,
                                          .dout=0,	.out=0}; // 电机PID
motorPIDTypeDef			*motor_PID = &motorMotor_PID_struct;
#pragma optimize=speed
int32 motor_PID_cal(void)
{
//    motor_PID->dout =  (int32)(motor_PID->Kp*   (1.0f+    motor_PID->Td/(Sys->PeriodUs/1000000.0f)+    (Sys->PeriodUs/1000000.0f)/motor_PID->Ti  )*motor_PID->nowe- \
//			                   motor_PID->Kp*   (1.0f+  2*motor_PID->Td/(Sys->PeriodUs/1000000.0f)                                               )*motor_PID->laste+ \
//			                   motor_PID->Kp*   (         motor_PID->Td/(Sys->PeriodUs/1000000.0f)                                               )*motor_PID->paste);
	motor_PID->dout =  (int32)(motor_PID->Kp			*	(motor_PID->nowe-motor_PID->laste)+ \
			(motor_PID->Ti)	*	motor_PID->nowe + \
			(motor_PID->Td)	*	(motor_PID->nowe-2*motor_PID->laste+motor_PID->paste));
	motor_PID->out = (uint32)(motor_PID->out + motor_PID->dout);
	motor_PID->paste = motor_PID->laste;
	motor_PID->laste = motor_PID->nowe;

	if(motor_PID->out>=INLET_MAX)	motor_PID->out=INLET_MAX;
	if(motor_PID->out<=0)	motor_PID->out=0;
	return motor_PID->out;
}