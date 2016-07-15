#include "time.h"
#include "servoPID.h"
#include "motor.h"
#include "OV7725_eagle.h"
#include "SUP_check.h"
#include "main.h"

extern OvTypeDef					*Ov7725;
extern motorPIDTypeDef		        *motor_PID;
extern MotorTypeDef					*MotorB;
extern MotorTypeDef					*MotorF;
extern ServomotorTypeDef	        *ServoMotor;
extern WeizhiPIDTypeDef         	*Weizhi_PID;
extern PhotocellTypeDef				*StartEndLine;

SystemTypeDef Sys_struct={&pit2_init, 1000, //周期
						1, //脉冲标志位
						0,  //运行时间
						{PIT3, &pit3_init, &pit3_disinit, 0}
						};
SystemTypeDef *Sys = &Sys_struct;

/* 采集周期(供pid用) */
PIT_InitTypeDef pit_init_struct2;
void pit2_init(void)
{
	Sys->RunTime = 0;
	pit_init_struct2.PIT_Pitx = PIT2;
	pit_init_struct2.PIT_PeriodUs = Sys->PeriodUs;     //定时周期1ms
	pit_init_struct2.PIT_Isr = pit2_isr;  //设置中断函数
	LPLD_PIT_Init(pit_init_struct2);

	LPLD_PIT_EnableIrq(pit_init_struct2);
}

#define ALLOW_PERIOUD		 20000 //(单位us)
#define STOP_PERIOUD		600000 //(单位us)
void pit2_isr(void)
{
	Sys->RunTime++;
	if(StartEndLine->start_end==0){ // 终止线模式
		if(((Sys->RunTime-StartEndLine->perioud)*Sys->PeriodUs)>STOP_PERIOUD){
			LPLD_FTM_PWM_ChangeDuty(MotorF->FTM_Ftmx, MotorF->chn, 0);
		}
	}
	if((Ov7725->LOCK == 1)||(StartEndLine->start_end==0)){ //锁定电机PID: 计算摄像头数据/终止线模式
		return;
	}
	//电机
	motor_PID->nowe=((float)MotorB->Target_Velosity-(float)(MotorB->Velosity));
	LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, motor_PID->cal());

	if(((Sys->RunTime-StartEndLine->perioud)*Sys->PeriodUs)>ALLOW_PERIOUD){
		StartEndLine->lineR = 0;
		StartEndLine->lineL = 0;
		StartEndLine->perioud = 0;
	}
}
/* 程序运行时间测试 */
PIT_InitTypeDef pit_init_struct3;
void pit3_init(void)
{
	pit_init_struct3.PIT_Pitx = Sys->timer.PIT_Pitx;
	pit_init_struct3.PIT_PeriodUs = 5000;     //定时周期1毫秒
	pit_init_struct3.PIT_Isr = pit3_isr;  //设置中断函数
	LPLD_PIT_Init(pit_init_struct3);
	LPLD_PIT_EnableIrq(pit_init_struct3);
}
void pit3_isr(void)
{
	printf("计时器(%d us)溢出...\n", pit_init_struct3.PIT_PeriodUs);
}
void pit3_disinit(void)
{
	uint32 now_cnt, whole_cnt;
	now_cnt = PIT->CHANNEL[Sys->timer.PIT_Pitx].CVAL;
	whole_cnt = PIT->CHANNEL[Sys->timer.PIT_Pitx].LDVAL;
	Sys->timer.runtime = (int)(((float)((whole_cnt)-(now_cnt)+1)/g_bus_clock)*1000000.0f);// 单位(us)
	LPLD_PIT_Deinit(pit_init_struct3);
}
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
 */