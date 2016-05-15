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
void ftm_pwm_init(mypwm *pwm)
{
	pwm_init_struct.FTM_Ftmx = pwm->FTM_Ftmx;
	pwm_init_struct.FTM_Mode = FTM_MODE_PWM; //使能PWM模式
	pwm_init_struct.FTM_PwmFreq = pwm->Freq; //PWM频率
//	pwm_init_struct.FTM_PwmDeadtimeCfg = DEADTIME_CH01;
//	pwm_init_struct.FTM_PwmDeadtimeDiv = DEADTIME_DIV16;
//	pwm_init_struct.FTM_PwmDeadtimeVal = 63;
	LPLD_FTM_Init(pwm_init_struct);
	LPLD_FTM_PWM_Enable(pwm->FTM_Ftmx, //使用FTM1
					  pwm->chn, 	//使能Ch1通道
					  pwm->Duty, 	//初始化占空比
					  pwm->pin, 	//使用通道的引脚
					  ALIGN_RIGHT   //脉宽左对齐
					  );
}

/*
	映射	舵机	时间(ms)
	90		0		0.5
	0  		90		1.5
    -90 	180		2.5
*/
#define S_A			20.0f
extern mypwm* Servomotor_Init;
uint32 angle_to_period(int32 angle)
{
	angle = angle+5;
	uint32 period = (10000 - (uint32)(10000*(0.5+(2.0f*(90.0-angle)/180))/(1000/Servomotor_Init->Freq)));
	if(((10000.0-period)*(1000.0/Servomotor_Init->Freq)/10000.0 > (1.5+S_A/90.0))||((10000.0-period)*(1000.0/Servomotor_Init->Freq)/10000.0 < (1.5-S_A/90.0)))
	{
		printf("====!%d\n", angle);
		angle=(angle>0)?(S_A):(-S_A);
	}
	return (10000 - (uint32)(10000*(0.5+(2.0f*(90.0-angle)/180))/(1000/Servomotor_Init->Freq)));
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