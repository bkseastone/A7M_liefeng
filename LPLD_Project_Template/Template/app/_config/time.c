#include "time.h"
#include "servoPID.h"
#include "motor.h"
#include "OV7725_eagle.h"
#include "main.h"

uint32 sudu=400;

extern OvTypeDef			*Ov7725;
extern motorPIDTypeDef		        *motor_PID;
extern MotorTypeDef			*MotorB;
extern ServomotorTypeDef	        *ServoMotor;
extern WeizhiPIDTypeDef         	*Weizhi_PID;


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
	pit_init_struct2.PIT_Pitx = PIT2;
	pit_init_struct2.PIT_PeriodUs = Sys->PeriodUs;     //定时周期1ms
	pit_init_struct2.PIT_Isr = pit2_isr;  //设置中断函数
	LPLD_PIT_Init(pit_init_struct2);

	LPLD_PIT_EnableIrq(pit_init_struct2);
}

void pit2_isr(void)
{
  if(Ov7725->LOCK == 1){
		return;
	}
	else{
		Ov7725->LOCK = 1;
	}
	//dianjiPID
//         motor_PID->nowe=(500-(float)(MotorB->Velosity));
         motor_PID->nowe=((float)MotorB->Target_Velosity-(float)(MotorB->Velosity));
//         printf("e=%d,\n",(int)(motor_PID->nowe));
         LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, motor_PID->cal());
        //duojiPID 
 Weizhi_PID->nowe = (int32)(0 - (((Ov7725->pos.deflection>0)?1:(-1))*GAIN_deflection*Ov7725->pos.deflection*Ov7725->pos.deflection + \
				(((Ov7725->pos.location_bias)>0)?1:(-1))*GAIN_location_bias*Ov7725->pos.location_bias*Ov7725->pos.location_bias));         
         LPLD_FTM_PWM_ChangeDuty(ServoMotor->FTM_Ftmx, ServoMotor->chn, angle_to_period(Weizhi_PID->cal()));
	Ov7725->LOCK = 0;
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