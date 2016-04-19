#include "Capture.h"
/*	Inout Capture
	Note that the maximum frequency for the channel input signal to be
detected correctly is system clock divided by 4, which is required to 
meet Nyquist criteria for signal sampling.
*/
FTM_InitTypeDef ic_init_struct;
extern uint32 Freq2;
extern uint32 Freq3;
/*
 * 初始化FTM0的输入捕获功能
 *
 */
#pragma optimize=size
status ic_init(myic* ic)
{
	//配置FTM模块的输入捕获参数
	ic_init_struct.FTM_Ftmx = ic->FTM_Ftmx;      //使能FTM1通道
	ic_init_struct.FTM_Mode = FTM_MODE_IC;       //使能输入捕获模式
	ic_init_struct.FTM_ClkDiv = FTM_CLK_DIV128;  //计数器频率为总线时钟的128分频
	ic_init_struct.FTM_Isr = ic->isr;     //设置中断函数
	//初始化FTM; 使能输入捕获对应通道,上升沿捕获进入中断; 使能FTM0中断
	if( LPLD_FTM_Init(ic_init_struct) && \
		LPLD_FTM_IC_Enable(ic->FTM_Ftmx, ic->chn, ic->pin, CAPTURE_RI) && \
		LPLD_FTM_EnableIrq(ic_init_struct) )
	{
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}

/*
 * 输入捕获中断
 *
 */
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
		Freq2=(g_bus_clock/LPLD_FTM_GetClkDiv(FTM2))/cnt; 
		//清空FTM1 COUNTER
		LPLD_FTM_ClearCounter(FTM2);
		//清除输入中断标志
		LPLD_FTM_ClearCHnF(FTM2, FTM_Ch0); 
	}
}
#pragma optimize=speed
void ic3_isr(void)
{
	uint32 cnt;
	//判断是否为FTM1的Ch0通道产生捕获中断
	if(LPLD_FTM_IsCHnF(FTM2, FTM_Ch1))
	{
		//获取FTM1的Ch0通道计数值
		cnt=LPLD_FTM_GetChVal(FTM2, FTM_Ch1);   
		//根据总线频率、分频系数、计数值计算脉冲频率
		//脉冲频率=(总线频率/输入捕获分频)/计数值
		Freq3=(g_bus_clock/LPLD_FTM_GetClkDiv(FTM2))/cnt; 
		//清空FTM1 COUNTER
		LPLD_FTM_ClearCounter(FTM2);
		//清除输入中断标志
		LPLD_FTM_ClearCHnF(FTM2, FTM_Ch1); 
	}
}
