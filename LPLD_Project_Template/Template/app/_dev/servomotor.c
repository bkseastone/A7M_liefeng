#include "servomotor.h"

ServomotorTypeDef Pwm0_C1={&ftm_pwmS_init, FTM0, FTM_Ch0, PTC1,
			200,  (10000 - (uint32)(10000*(0.5+(2.0f*(90.0-0)/180))/(1000/200))), //频率， 占空比
			0			//角度(负为俯视顺时针)
			};
ServomotorTypeDef*	ServoMotor = &Pwm0_C1;

static FTM_InitTypeDef pwm_init_struct3;
#pragma optimize=size
void ftm_pwmS_init(void)
{
	pwm_init_struct3.FTM_Ftmx = ServoMotor->FTM_Ftmx;
	pwm_init_struct3.FTM_Mode = FTM_MODE_PWM;
	pwm_init_struct3.FTM_PwmFreq = ServoMotor->Freq;
//	pwm_init_struct3.FTM_PwmDeadtimeCfg = DEADTIME_CH01;
//	pwm_init_struct3.FTM_PwmDeadtimeDiv = DEADTIME_DIV16;
//	pwm_init_struct3.FTM_PwmDeadtimeVal = 63;
	LPLD_FTM_Init(pwm_init_struct3);
	LPLD_FTM_PWM_Enable(ServoMotor->FTM_Ftmx, ServoMotor->chn, ServoMotor->Duty, ServoMotor->pin, ALIGN_RIGHT);
}

/*
	映射	舵机	pwm高电平时间(ms)
	90		0		0.5
	0  		90		1.5
    -90 	180		2.5
*/
#define S_T			28.0f // security-threhold (+/- 度)
#define	servo_bias	3.4f
uint32 angle_to_period(int32 angle)
{
	//angle = angle+3; // bias
	float  tmp_angle;
	tmp_angle = (float)angle + servo_bias;
	uint32 period = (10000 - (uint32)(10000*(0.5+(2.0f*(90.0-tmp_angle)/180))/(1000/ServoMotor->Freq)));
	if(((10000.0-period)*(1000.0/ServoMotor->Freq)/10000.0 > (1.5+S_T/90.0))|| \
		((10000.0-period)*(1000.0/ServoMotor->Freq)/10000.0 < (1.5-S_T/90.0)))
	{
//		printf("!==error==!\t ServoMotor tmp_angle will be %d.\n", tmp_angle);
		tmp_angle=((tmp_angle+servo_bias)>0)?(S_T+servo_bias):(-S_T);
	}
	return (10000 - (uint32)(10000*(0.5+(2.0f*(90.0-tmp_angle)/180))/(1000/ServoMotor->Freq)));
}
