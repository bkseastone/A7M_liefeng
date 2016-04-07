#include "main.h"
void main (void)
{	
	//init_gpio_led_on();
	//init_gpio_monitorC4();//按键中断
	
	if(ftm_pwm_init(&Pwm0_C1, 50, 3000)==SUCCESS);
	
	if(adc_init(&Adc0_0P1, 1000000)==OK);
	
	if(ic_init(&ic2_B18)==SUCCESS);
	
	ov7620_dma_start();
	
	
	while(1)
	{
		if(Is_DispPhoto==1) OV_display();
		printf("Freq2 = %dHz. \nFreq3 = %dHz. \n", Freq2, Freq3);
	} 
}
