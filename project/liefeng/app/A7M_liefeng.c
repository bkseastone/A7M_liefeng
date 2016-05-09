/*
     ___   _____       ___  ___        _       _   _____   _____   _____   __   _   _____  
    /   | |___  |     /   |/   |      | |     | | | ____| |  ___| | ____| |  \ | | /  ___| 
   / /| |     / /    / /|   /| |      | |     | | | |__   | |__   | |__   |   \| | | |     
  / / | |    / /    / / |__/ | |      | |     | | |  __|  |  __|  |  __|  | |\   | | |  _  
 / /  | |   / /    / /       | |      | |___  | | | |___  | |     | |___  | | \  | | |_| | 
/_/   |_|  /_/    /_/        |_|      |_____| |_| |_____| |_|     |_____| |_|  \_| \_____/ 

*
*/
#include "main.h"
//#define DEBUG_PRINT

PIDTypeDef* Motor_PID = &motor_PID;
mypwm* Motor_Init = &Pwm1_A8;
myic* Feedback_encoder_Init = &ic2_A11;
void main (void)
{	
	//init_gpio_led_on();
	//init_gpio_monitorC4();//按键中断
	//ov7725_dma_start();
	//if(ftm_pwm_init(&Pwm1_A8, 54, 3000)==SUCCESS);
	//if(ic_init(&ic2_A11)==SUCCESS);
	//adc_init(&Adc0_0P1, 1000000);
	ftm_pwm_init(Motor_Init, 50000, 0);
	ic_init(Feedback_encoder_Init);
	velosity = 20*Freq3/500;
	printf("%d cm/s\n", velosity);
	while(1)
	{
		velosity = 20*Freq3/500;
		Motor_PID->nowe = TARGET_VELOSITY - velosity;
		if(velosity < TARGET_VELOSITY-1)
		{
			duty = duty+PID_cal(Motor_PID);
			LPLD_FTM_PWM_ChangeDuty(Motor_Init->FTM_Ftmx, Motor_Init->chn, duty);
		}
		else if(velosity > TARGET_VELOSITY+1)
		{
			duty = duty+PID_cal(Motor_PID);
			LPLD_FTM_PWM_ChangeDuty(Motor_Init->FTM_Ftmx, Motor_Init->chn, duty);
		}
		else
		{
			printf("%d cm/s\n", velosity);
		}
	} 
}