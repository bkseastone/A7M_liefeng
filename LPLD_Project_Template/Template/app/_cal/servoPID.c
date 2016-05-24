#include "servoPID.h"
#include "time.h"

extern SystemTypeDef	*Sys;
ServoPIDTypeDef ServoMotor_PID_struct = { .cal=&Servo_PID_cal,
											.Kp=0.20,	.Ti=0.00,	.Td=0.00,
											.nowe=0,	.laste=0,	.paste=0,
											.dout=0,	.out=0}; // 舵机PID
ServoPIDTypeDef			*Servo_PID = &ServoMotor_PID_struct;
int32 Servo_PID_cal(void)
{
    Servo_PID->dout =  (int32)(Servo_PID->Kp*(1.0f+((Sys->PeriodUs/1000000.0f)/Servo_PID->Ti)+(Servo_PID->Td/(Sys->PeriodUs/1000000.0f)))*Servo_PID->nowe- \
					Servo_PID->Kp*(1.0f+2*(Servo_PID->Td)/(Sys->PeriodUs/1000000.0f))*Servo_PID->laste+ \
					Servo_PID->Kp*(Servo_PID->Td/(Sys->PeriodUs/1000000.0f))*Servo_PID->paste);
	Servo_PID->out = Servo_PID->out + Servo_PID->dout;
	Servo_PID->paste = Servo_PID->laste;
	Servo_PID->laste = Servo_PID->nowe;
	return Servo_PID->out;
}
