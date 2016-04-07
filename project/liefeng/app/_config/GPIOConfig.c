#include "GPIOConfig.h"
#include "interrupt.h"

void init_gpio_led_on()
{
	GPIO_InitTypeDef gpio_init_struct;
	
	gpio_init_struct.GPIO_PTx = PTA;
	gpio_init_struct.GPIO_Pins = GPIO_Pin17;
	gpio_init_struct.GPIO_Dir = DIR_OUTPUT;
	gpio_init_struct.GPIO_Output = OUTPUT_L;
	gpio_init_struct.GPIO_PinControl = IRQC_DIS;
	LPLD_GPIO_Init(gpio_init_struct);

	gpio_init_struct.GPIO_PTx = PTC;
	gpio_init_struct.GPIO_Pins = GPIO_Pin0;
	LPLD_GPIO_Init(gpio_init_struct);

	gpio_init_struct.GPIO_PTx = PTD;
	gpio_init_struct.GPIO_Pins = GPIO_Pin15;
	LPLD_GPIO_Init(gpio_init_struct);

	gpio_init_struct.GPIO_PTx = PTE;
	gpio_init_struct.GPIO_Pins = GPIO_Pin26;
	LPLD_GPIO_Init(gpio_init_struct);
}

/*
void init_gpio_monitorC4()
{
	GPIO_InitTypeDef gpio_init_struct;
	gpio_init_struct.GPIO_PTx = PTC; //PORTB
	gpio_init_struct.GPIO_Pins = GPIO_Pin4; //引脚6、7
	gpio_init_struct.GPIO_Dir = DIR_INPUT; //输入
	gpio_init_struct.GPIO_PinControl = INPUT_PULL_UP|IRQC_DIS; //内部上拉|不产生中断
	LPLD_GPIO_Init(gpio_init_struct);
}*/
void init_gpio_monitorC4()
{
	// 配置 PTC4 为GPIO功能,输入,内部上拉，上升沿产生中断
	GPIO_InitTypeDef gpio_init_struct;
	gpio_init_struct.GPIO_PTx = PTC; //PORTB
	gpio_init_struct.GPIO_Pins = GPIO_Pin4; //引脚6、7
	gpio_init_struct.GPIO_Dir = DIR_INPUT; //输入
	gpio_init_struct.GPIO_PinControl = INPUT_PULL_UP|IRQC_FA; //内部上拉|上升沿中断
	gpio_init_struct.GPIO_Isr = portc4_isr; //中断函数
	LPLD_GPIO_Init(gpio_init_struct);
	//使能中断
	LPLD_GPIO_EnableIrq(gpio_init_struct);
}
