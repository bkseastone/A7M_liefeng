#include "time.h"

SystemTypeDef Sys_struct={&pit2_init, 1000000, //周期
						1, //脉冲标志位
						0  //运行时间
						};
SystemTypeDef *Sys = &Sys_struct;

PIT_InitTypeDef pit_init_struct2;
void pit2_init(void)
{
	pit_init_struct2.PIT_Pitx = PIT2;
	pit_init_struct2.PIT_PeriodUs = Sys->PeriodUs;     //定时周期2秒
	pit_init_struct2.PIT_Isr = pit2_isr;  //设置中断函数
	LPLD_PIT_Init(pit_init_struct2);

	LPLD_PIT_EnableIrq(pit_init_struct2);
}

void pit2_isr(void)
{
	Sys->PULSE = 1;
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