#include "main.h"
void main (void)
{	
	//init_gpio_led_on();
	//init_gpio_monitorC4();//按键中断
	//uart4_bluetooth_init();
	//if(ftm_pwm_init(&Pwm0_A4, 54, 3000)==SUCCESS);
	
	//if(ic_init(&ic2_A11)==SUCCESS);
	//adc_init(&Adc0_0P1, 1000000);
	
	
	ov7725_dma_start();
	
	while(1)
	{
		
		if(Is_DispPhoto==1)
		{
			OV_display();
		}
	} 
}
