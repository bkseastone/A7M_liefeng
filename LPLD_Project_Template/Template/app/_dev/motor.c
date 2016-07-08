#include "motor.h"
#include "OV7725_eagle.h"
extern OvTypeDef			*Ov7725;
extern OV_pictureTypeDef OV_pictures @OV_binary_BONDADDR(0, 16);
MotorTypeDef Pwm1_A9=	{&ftm_motorF_init, FTM1, FTM_Ch1, PTA9, //后
				5000, 0,	//频率， 占空比
				2, 			//速度(cm/s)
				2,			//加速度(cm/s2)
				0			//目标速度(cm/s)
				};
MotorTypeDef Pwm1_A8=	{&ftm_motorB_init, FTM1, FTM_Ch0, PTA8, //前
				5000, INLET_MAX,
				3,
				3,
				0
				};
MotorTypeDef*	MotorB = &Pwm1_A8;
MotorTypeDef*	MotorF = &Pwm1_A9;
/*PWM的周期=(MOD–CNTIN + 1) x 计数器周期
 *PWM的占空比=(CnV − CNTIN) / PWM的周期
 */
static FTM_InitTypeDef pwm_init_structF;
#pragma optimize=size
void ftm_motorF_init(void)
{
	pwm_init_structF.FTM_Ftmx = MotorF->FTM_Ftmx;
	pwm_init_structF.FTM_Mode = FTM_MODE_PWM; //使能PWM模式
	pwm_init_structF.FTM_PwmFreq = MotorF->Freq; //PWM频率
//	pwm_init_structF.FTM_PwmDeadtimeCfg = DEADTIME_CH01;
//	pwm_init_structF.FTM_PwmDeadtimeDiv = DEADTIME_DIV16;
//	pwm_init_structF.FTM_PwmDeadtimeVal = 63;
	LPLD_FTM_Init(pwm_init_structF);
	LPLD_FTM_PWM_Enable(MotorF->FTM_Ftmx, MotorF->chn, MotorF->Duty, MotorF->pin, ALIGN_RIGHT);
}
static FTM_InitTypeDef pwm_init_structB;
#pragma optimize=size
void ftm_motorB_init(void)
{
	pwm_init_structB.FTM_Ftmx = MotorB->FTM_Ftmx;
	pwm_init_structB.FTM_Mode = FTM_MODE_PWM; //使能PWM模式
	pwm_init_structB.FTM_PwmFreq = MotorB->Freq; //PWM频率
//	pwm_init_structB.FTM_PwmDeadtimeCfg = DEADTIME_CH01;
//	pwm_init_structB.FTM_PwmDeadtimeDiv = DEADTIME_DIV16;
//	pwm_init_structB.FTM_PwmDeadtimeVal = 63;
	LPLD_FTM_Init(pwm_init_structB);
	LPLD_FTM_PWM_Enable(MotorB->FTM_Ftmx, MotorB->chn, MotorB->Duty, MotorB->pin, ALIGN_RIGHT);
}

void throttle_control(void)
{
  if(Ov7725->distance>160){
    MotorB->Target_Velosity=VELOCITY_MAX;
  }
  else{
    MotorB->Target_Velosity = 650;// 650
  }
}