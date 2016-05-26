#include "time.h"

SystemTypeDef Sys_struct={&pit2_init, 19000, //周期
						1, //脉冲标志位
						0,  //运行时间
						{PIT3, &pit3_init, &pit3_disinit, 0}
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

PIT_InitTypeDef pit_init_struct3;
void pit3_init(void)
{
	pit_init_struct3.PIT_Pitx = Sys->timer.PIT_Pitx;
	pit_init_struct3.PIT_PeriodUs = 500;     //定时周期10毫秒
	pit_init_struct3.PIT_Isr = pit3_isr;  //设置中断函数
	LPLD_PIT_Init(pit_init_struct3);
	LPLD_PIT_EnableIrq(pit_init_struct3);
}
void pit3_isr(void)
{
	printf("计时器(%d s)溢出...\n", pit_init_struct3.PIT_PeriodMs);
}
void pit3_disinit(void)
{
	Sys->timer.runtime = (int)(((float)((PIT->CHANNEL[Sys->timer.PIT_Pitx].LDVAL)-(PIT->CHANNEL[Sys->timer.PIT_Pitx].CVAL)+1)/g_bus_clock)*1000000.0f);
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