#include "encoder.h"
#include "motor.h"
extern MotorTypeDef*	MotorF;
extern MotorTypeDef*	MotorB;
myic ic2_A10={&icFeedback_init, FTM2, FTM_Ch0, PTA10, &ic2_isr};  
myic ic2_A11={&icTest_init, FTM2, FTM_Ch1, PTA11, &ic3_isr};
myic*	Encoder_Feedback = &ic2_A10;
myic*	Encoder_Test = &ic2_A11;

/*	Inout Capture
	Note that the maximum frequency for the channel input signal to be
detected correctly is system clock divided by 4, which is required to 
meet Nyquist criteria for signal sampling.
*/
FTM_InitTypeDef ic_init_struct1;
#pragma optimize=size
void icFeedback_init(void)
{
	//配置FTM模块的输入捕获参数
	ic_init_struct1.FTM_Ftmx = Encoder_Feedback->FTM_Ftmx;      //使能FTM1通道
	ic_init_struct1.FTM_Mode = FTM_MODE_IC;       //使能输入捕获模式
	ic_init_struct1.FTM_ClkDiv = FTM_CLK_DIV128;  //计数器频率为总线时钟的128分频
	ic_init_struct1.FTM_Isr = Encoder_Feedback->isr;     //设置中断函数
	//初始化FTM; 使能输入捕获对应通道,上升沿捕获进入中断; 使能FTM中断
	LPLD_FTM_Init(ic_init_struct1);
	LPLD_FTM_IC_Enable(Encoder_Feedback->FTM_Ftmx, Encoder_Feedback->chn, Encoder_Feedback->pin, CAPTURE_RI);
	LPLD_FTM_EnableIrq(ic_init_struct1);
}

FTM_InitTypeDef ic_init_struct2;
#pragma optimize=size
void icTest_init(void)
{
	//配置FTM模块的输入捕获参数
	ic_init_struct2.FTM_Ftmx = Encoder_Test->FTM_Ftmx;      //使能FTM1通道
	ic_init_struct2.FTM_Mode = FTM_MODE_IC;       //使能输入捕获模式
	ic_init_struct2.FTM_ClkDiv = FTM_CLK_DIV128;  //计数器频率为总线时钟的128分频
	ic_init_struct2.FTM_Isr = Encoder_Test->isr;     //设置中断函数
	//初始化FTM; 使能输入捕获对应通道,上升沿捕获进入中断; 使能FTM中断
	LPLD_FTM_Init(ic_init_struct2);
	LPLD_FTM_IC_Enable(Encoder_Test->FTM_Ftmx, Encoder_Test->chn, Encoder_Test->pin, CAPTURE_RI);
	LPLD_FTM_EnableIrq(ic_init_struct2);
}

#pragma optimize=speed
void ic2_isr(void)
{
	uint32 cnt;
	//判断是否为FTM1的Ch0通道产生捕获中断
	if(LPLD_FTM_IsCHnF(FTM2, FTM_Ch0))
	{
		//获取FTM1的Ch0通道计数值
		cnt=LPLD_FTM_GetChVal(FTM2, FTM_Ch0);   
		//根据总线频率、分频系数、计数值计算脉冲频率
		//脉冲频率=(总线频率/输入捕获分频)/计数值
		MotorF->Velosity=(g_bus_clock/LPLD_FTM_GetClkDiv(FTM2))/cnt; 
		//清空FTM1 COUNTER
		LPLD_FTM_ClearCounter(FTM2);
		//清除输入中断标志
		LPLD_FTM_ClearCHnF(FTM2, FTM_Ch0); 
	}
}
#pragma optimize=speed
void ic3_isr(void)
{
	//判断是否为FTM1的Ch0通道产生捕获中断
	if(LPLD_FTM_IsCHnF(FTM2, FTM_Ch1))
	{
		//获取FTM1的Ch0通道计数值
		//根据总线频率、分频系数、计数值计算脉冲频率
		//脉冲频率=(总线频率/输入捕获分频)/计数值
		MotorB->Velosity=20*((g_bus_clock/LPLD_FTM_GetClkDiv(FTM2))/LPLD_FTM_GetChVal(FTM2, FTM_Ch1))/500; 
		//清空FTM1 COUNTER
		LPLD_FTM_ClearCounter(FTM2);
		//清除输入中断标志
		LPLD_FTM_ClearCHnF(FTM2, FTM_Ch1); 
	}
}
