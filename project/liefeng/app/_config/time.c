#include "time.h"
#include "interrupt.h"


//-- @ configrations for pwm out
/*PWM的周期=(MOD–CNTIN + 1) x 计数器周期
 *PWM的占空比=(CnV − CNTIN) / PWM的周期
 * 输出:
 *    0--配置错误
 *    1--配置成功
 */
FTM_InitTypeDef pwm_init_struct;
#pragma optimize=size
status ftm_pwm_init(mypwm *pwm, uint32 FTM_PwmFreq, uint32 duty)
{
	pwm_init_struct.FTM_Ftmx = pwm->FTM_Ftmx; //使能FTM1通道
	pwm_init_struct.FTM_Mode = FTM_MODE_PWM; //使能PWM模式
	pwm_init_struct.FTM_PwmFreq = FTM_PwmFreq; //PWM频率100Hz
	pwm_init_struct.FTM_PwmDeadtimeCfg = DEADTIME_CH01;
	pwm_init_struct.FTM_PwmDeadtimeDiv = DEADTIME_DIV16;
	pwm_init_struct.FTM_PwmDeadtimeVal = 63;
	if( LPLD_FTM_Init(pwm_init_struct) && \
		LPLD_FTM_PWM_Enable(pwm->FTM_Ftmx, //使用FTM1
						  pwm->chn, //使能Ch1通道
						  duty, //初始化角度30度
						  pwm->pin, //使用Ch1通道的PTD1引脚
						  ALIGN_LEFT        //脉宽左对齐
						  ) )
	{
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}

uint32 angle_to_period(uint32 angle)
{
	return angle*100;
}


//-- @ configrations for ADC
/*
 *  另一个例子
void pdb_init(void)
{
	pdb_init_struct.PDB_CounterPeriodMs = 500;   //计数器溢出周期500微秒
	pdb_init_struct.PDB_TriggerInputSourceSel = TRIGGER_SOFTWARE; //触发源为软件触发
	pdb_init_struct.PDB_ContinuousModeEnable = TRUE;      //连续工作模式
	pdb_init_struct.PDB_DelayMs = 1000;    //中断延时时间1000毫秒
	pdb_init_struct.PDB_IntEnable = TRUE; //使能延时中断
	pdb_init_struct.PDB_Isr = pdb_isr;    //中断函数设置
	LPLD_PDB_Init(pdb_init_struct);
	LPLD_PDB_EnableIrq();
	LPLD_PDB_SoftwareTrigger();
}
 ******************************************************************
 * 输出:
 *    0--配置错误
 *    1--配置成功
 */
PDB_InitTypeDef pdb_init_struct;
#pragma optimize=size
void pdb_init(uint8 PDB_TriggerInputSourceSel, ADC_Type *ADC_Adcx)
{
	pdb_init_struct.PDB_CounterPeriodUs = 10;   //PDB计数器周期设置
	pdb_init_struct.PDB_LoadModeSel = LOADMODE_0; //加载模式设置
	pdb_init_struct.PDB_DelayS = 2;    //中断延时时间2秒
	pdb_init_struct.PDB_ContinuousModeEnable = FALSE;     //禁用连续工作模式
	pdb_init_struct.PDB_TriggerInputSourceSel = PDB_TriggerInputSourceSel;     //配置触发源为PIT0
	LPLD_PDB_Init(pdb_init_struct);
	LPLD_PDB_AdcTriggerCfg(ADC_Adcx, PRETRIG_EN_A, 0);
}

/*
 *  另一个例子
void pit0_init(void)
{
	pit0_init_struct.PIT_Pitx = PIT0;
	pit0_init_struct.PIT_PeriodS = 10;     //定时周期2秒
	pit0_init_struct.PIT_Isr = pit0_isr;  //设置中断函数
	LPLD_PIT_Init(pit0_init_struct);  

	LPLD_PIT_EnableIrq(pit0_init_struct);
}
 ******************************************************************
 *  输出:
 *  0--配置错误
 *  1--配置成功
 */
PIT_InitTypeDef pit_init_struct;
#pragma optimize=size
void pit_init(PITx PIT_Pitx, uint32 PIT_PeriodUs)
{
	pit_init_struct.PIT_Pitx = PIT_Pitx;     //选择PIT0
	pit_init_struct.PIT_PeriodUs = PIT_PeriodUs;  //PIT0计数周期500ms
	LPLD_PIT_Init(pit_init_struct);  
}