#include "SUP_check.h"
#include "motor.h"
#include "OV7725_eagle.h"
#include "time.h"

extern MotorTypeDef			*MotorB;
extern OvTypeDef			*Ov7725;
extern SystemTypeDef		*Sys;
PhotocellTypeDef StartEndLine_struct = {0,0};
PhotocellTypeDef* StartEndLine = &StartEndLine_struct;
#pragma optimize=size
void init_photocellB2B3(void)
{
	init_led();
	// 配置 PTC4 为GPIO功能,输入,内部上拉，上升沿产生中断
	GPIO_InitTypeDef gpio_init_struct;
	gpio_init_struct.GPIO_PTx = PTB;
	gpio_init_struct.GPIO_Pins = GPIO_Pin2|GPIO_Pin3;
	gpio_init_struct.GPIO_Dir = DIR_INPUT; //输入
	gpio_init_struct.GPIO_PinControl = IRQC_ET; //边沿中断
	gpio_init_struct.GPIO_Isr = photocell_isr; //中断函数
	LPLD_GPIO_Init(gpio_init_struct);
	LPLD_GPIO_EnableIrq(gpio_init_struct);
	StartEndLine->lineL = 0;
	StartEndLine->lineR = 0;
	StartEndLine->start_end = 0;
	StartEndLine->perioud = 0;
}

void photocell_isr(void)
{
	if(StartEndLine->lineL != 16){
		if(LPLD_GPIO_IsPinxExt(PORTB, GPIO_Pin3))
		{
			if(PTBn_I(3)==1)
			{
				StartEndLine->lineL = 1;
				if(StartEndLine->perioud==0){
					StartEndLine->perioud = Sys->RunTime;
				}
	//			printf("photocell_L HERE!\r\n");
			}
			else if(PTBn_I(3)==0)
			{
	//			printf("photocell_L NOP!\r\n");
			}

			if((StartEndLine->lineL+StartEndLine->lineR)>=2){
				StartEndLine->lineL = 0;
				StartEndLine->lineR = 0;
				StartEndLine->perioud = 0;
				StartEndLine->start_end = !StartEndLine->start_end;
				if(StartEndLine->start_end==0){
					Ov7725->LOCK = 1;
					StartEndLine->lineL = 16;
					LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, 0);
					PTD8_O = 0;
				}
			}
		}
		else if(LPLD_GPIO_IsPinxExt(PORTB, GPIO_Pin2))
		{
			if(PTBn_I(2)==0)
			{
				StartEndLine->lineR = 1;
				if(StartEndLine->perioud==0){
					StartEndLine->perioud = Sys->RunTime;
				}
	//			printf("photocell_L HERE!\r\n");
			}
			if((StartEndLine->lineL+StartEndLine->lineR)>=2){
				StartEndLine->lineL = 0;
				StartEndLine->lineR = 0;
				StartEndLine->perioud = 0;
				StartEndLine->start_end = !StartEndLine->start_end;
				if(StartEndLine->start_end==0){
					Ov7725->LOCK = 1;
					StartEndLine->lineL = 16;
					LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, 0);
					PTD8_O = 0;
				}
			}
		}
	}

}

void init_led(void)
{
	GPIO_InitTypeDef gpio_init_struct;

	gpio_init_struct.GPIO_PTx = PTD;
	gpio_init_struct.GPIO_Pins = GPIO_Pin8;
	gpio_init_struct.GPIO_Dir = DIR_OUTPUT;
	gpio_init_struct.GPIO_Output = OUTPUT_H;
	gpio_init_struct.GPIO_PinControl = IRQC_DIS;
	LPLD_GPIO_Init(gpio_init_struct);
}