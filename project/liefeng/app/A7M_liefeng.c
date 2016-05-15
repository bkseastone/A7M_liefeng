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
PIDTypeDef* Motor_PID = &motor_PID;
mypwm*	MotorB = &Pwm1_A8;
mypwm*	MotorF = &Pwm1_A9;
mypwm*	ServoMotor_Init = &Pwm0_C1;
myic*	Feedback_encoder_Init = &ic2_A10;
myic*	Test_encoder_Init = &ic2_A11;
myadc*	Battery_Init = &Adc0_0P1;
void main (void)
{	
	//init_gpio_led_on();
	//init_gpio_monitorC4();//按键中断
	//ov7725_dma_start(); //摄像头
	//ftm_pwm_init(&Pwm1_A8); //电机
	//ic_init(&ic2_A11); //编码器
	//adc_init(Battery_Init, 1000000); //电池
	//ftm_pwm_init(ServoMotor_Init); //舵机
	//LPLD_FTM_PWM_ChangeDuty(Servomotor_Init->FTM_Ftmx, Servomotor_Init->chn, angle_to_period(15)); //舵机
	
	// 测试编码器
	ftm_pwm_init(MotorB);
	ic_init(Test_encoder_Init);
#if defined(DEBUG_PRINT)
	printf("%d cm/s\n", MotorB->Velosity);
#endif
	while(1)
	{
		Motor_PID->nowe = TARGET_VELOSITY - MotorB->Velosity;
		if(MotorB->Duty>=5000) MotorB->Duty = 5000;
		if(MotorB->Velosity < TARGET_VELOSITY-1)
		{
			MotorB->Duty = MotorB->Duty+PID_cal(Motor_PID);
			LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, MotorB->Duty);
		}
		else if(MotorB->Velosity > TARGET_VELOSITY+1)
		{
			MotorB->Duty = MotorB->Duty+PID_cal(Motor_PID);
			LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, MotorB->Duty);
		}
#if defined(DEBUG_PRINT)
		printf("%d cm/s\n", MotorB->Velosity);
#endif
	} 
}