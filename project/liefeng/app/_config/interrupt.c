#include "interrupt.h"
#include "time.h"

void portc4_isr(void)
{
	if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin4))
	{
		delay();
		if(PTCn_I(4)==0)
		{
			printf("Button1-PTB6 Interrupt!\r\n");
			LPLD_GPIO_Toggle_b(PTC, 0);
		}
	}
}

void uart5_isr(void)
{
	int8 recv;
	recv = LPLD_UART_GetChar(UART5);
	LPLD_UART_PutChar(UART5, recv);
}

void pit0_isr(void)
{
	PIT_InitTypeDef pit0_init_struct;
	pit0_init_struct.PIT_Pitx = PIT0;
	
	LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch0, angle_to_period(90));
	
	LPLD_PIT_Deinit(pit0_init_struct);
}

void pdb_isr(void)
{
	printf("pdb_isr OK!!\n");
}

void adc0_isr(void)
{
	int16 result;
	uint8 ab = LPLD_ADC_GetSC1nCOCO(ADC0);//判断转换完成的是A组还是B组
	result = LPLD_ADC_GetResult(ADC0, ab);//获取采样结果
	printf("ADC0_R[%d]=%d\r\n", ab, result);//打印输出结果
	LPLD_ADC_EnableConversion(ADC0, DAD1, 0, TRUE);
}

void adc1_isr(void)
{
	int16 result;
	uint8 ab = LPLD_ADC_GetSC1nCOCO(ADC1);//判断转换完成的是A组还是B组
	result = LPLD_ADC_GetResult(ADC1, ab);//获取采样结果
	printf("ADC1_R[%d]=%d\r\n", ab, result);//打印输出结果
	LPLD_ADC_EnableConversion(ADC1, DAD1, 0, TRUE);
}
